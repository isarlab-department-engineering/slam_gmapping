#!/usr/bin/env python3
"""Odometria con deriva artificiale (versione didattica di GMapping).

In simulazione l'odometria del robot e' quasi perfetta, e la SLAM ha poco da
correggere. Questo nodo legge l'odometria vera (TF odom -> base_footprint),
ne integra gli spostamenti aggiungendo errori sistematici e casuali, e pubblica
un nuovo frame "odom_noisy" tale che la catena

    odom_noisy -> odom -> base_footprint

dia la posa secondo l'odometria disturbata. GMapping usa odom_noisy come frame
dell'odometria, quindi vede un robot la cui odometria deriva come quella reale.

Errori (parametri):
  drift_trans_scale    errore di scala sulla distanza (0.03 = il robot crede di
                       aver percorso il 3% in piu')
  drift_rot_scale      errore di scala sulle rotazioni (0.05 = +5%)
  drift_rot_per_meter  deriva sistematica dell'orientamento, in rad per metro
                       percorso (fa "curvare" la traiettoria stimata)
  drift_trans_noise    rumore casuale sulla traslazione, in m per radice di metro
  drift_rot_noise      rumore casuale sull'orientamento, in rad per radice di metro
  drift_seed           seme casuale (0 = diverso a ogni avvio)

Pubblica anche le traiettorie vera e disturbata (nav_msgs/Path, frame odom),
per confrontarle in RViz2.
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def compose(a, b):
    """Composizione di pose 2D (x, y, theta): a seguito da b."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at),
            math.remainder(at + bt, 2.0 * math.pi))


def inverse(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), -at)


class OdomDrift(Node):
    def __init__(self):
        super().__init__('odom_drift')
        self.odom_frame = self.declare_parameter('drift_odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('drift_base_frame', 'base_footprint').value
        self.noisy_frame = self.declare_parameter('drift_noisy_frame', 'odom_noisy').value
        self.trans_scale = self.declare_parameter('drift_trans_scale', 0.03).value
        self.rot_scale = self.declare_parameter('drift_rot_scale', 0.05).value
        self.rot_per_meter = self.declare_parameter('drift_rot_per_meter', 0.02).value
        self.trans_noise = self.declare_parameter('drift_trans_noise', 0.01).value
        self.rot_noise = self.declare_parameter('drift_rot_noise', 0.01).value
        seed = self.declare_parameter('drift_seed', 7).value
        rate = self.declare_parameter('drift_rate', 50.0).value
        self.rng = random.Random(seed if seed > 0 else None)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.path_true_pub = self.create_publisher(Path, 'odom_drift/path_true', 10)
        self.path_noisy_pub = self.create_publisher(Path, 'odom_drift/path_noisy', 10)

        self.prev_true = None     # ultima posa vera (x, y, theta)
        self.noisy = None         # posa secondo l'odometria disturbata
        self.last_stamp = None
        self.path_true = Path()
        self.path_noisy = Path()
        self.path_true.header.frame_id = self.odom_frame
        self.path_noisy.header.frame_id = self.odom_frame
        self.dist_since_path = 0.0
        self.total_dist = 0.0
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f'Deriva odometria attiva: {self.noisy_frame} -> {self.odom_frame}; '
            f'scala trasl. {self.trans_scale:+.3f}, scala rot. {self.rot_scale:+.3f}, '
            f'deriva {self.rot_per_meter:+.3f} rad/m, rumore {self.trans_noise:.3f} m/sqrt(m), '
            f'{self.rot_noise:.3f} rad/sqrt(m)')

    def step(self):
        try:
            t = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:
            return   # TF non ancora disponibile
        stamp = t.header.stamp
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        true = (t.transform.translation.x, t.transform.translation.y, yaw)

        if self.prev_true is None:
            # all'avvio odometria vera e disturbata coincidono
            self.prev_true = true
            self.noisy = true
        elif self.last_stamp is None or (stamp.sec, stamp.nanosec) != (self.last_stamp.sec, self.last_stamp.nanosec):
            # spostamento nel frame del robot dall'ultima lettura
            dx, dy, dth = compose(inverse(self.prev_true), true)
            ds = math.hypot(dx, dy)
            # errori sistematici
            dx *= 1.0 + self.trans_scale
            dy *= 1.0 + self.trans_scale
            dth = dth * (1.0 + self.rot_scale) + self.rot_per_meter * ds
            # errori casuali, con varianza proporzionale alla distanza percorsa
            if ds > 0.0:
                sq = math.sqrt(ds)
                dx += self.rng.gauss(0.0, self.trans_noise * sq)
                dth += self.rng.gauss(0.0, self.rot_noise * sq)
            self.noisy = compose(self.noisy, (dx, dy, dth))
            self.prev_true = true
            self.total_dist += ds
            self.dist_since_path += ds
            if self.dist_since_path > 0.1:
                self.dist_since_path = 0.0
                self.append_path(self.path_true, true, stamp)
                self.append_path(self.path_noisy, self.noisy, stamp)
        self.last_stamp = stamp

        # odom_noisy -> odom = noisy * true^-1, cosi' odom_noisy -> base = noisy
        off = compose(self.noisy, inverse(true))
        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.noisy_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = off[0]
        out.transform.translation.y = off[1]
        out.transform.rotation.z = math.sin(off[2] / 2.0)
        out.transform.rotation.w = math.cos(off[2] / 2.0)
        self.broadcaster.sendTransform(out)

    def append_path(self, path, pose, stamp):
        ps = PoseStamped()
        ps.header.frame_id = self.odom_frame
        ps.header.stamp = stamp
        ps.pose.position.x = pose[0]
        ps.pose.position.y = pose[1]
        ps.pose.position.z = 0.02
        ps.pose.orientation.z = math.sin(pose[2] / 2.0)
        ps.pose.orientation.w = math.cos(pose[2] / 2.0)
        path.poses.append(ps)
        path.header.stamp = stamp
        (self.path_true_pub if path is self.path_true else self.path_noisy_pub).publish(path)


def main():
    rclpy.init()
    node = OdomDrift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
