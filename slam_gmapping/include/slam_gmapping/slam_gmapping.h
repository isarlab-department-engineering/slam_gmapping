/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

//
// Created by shivesh on 29/10/18.
//

#ifndef SLAM_GMAPPING_DIDACTIC_SLAM_GMAPPING_H_
#define SLAM_GMAPPING_DIDACTIC_SLAM_GMAPPING_H_

#include <mutex>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <set>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <map>
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"

// --- Versione didattica ---
// Estende il filtro di GMapping senza modificarne la libreria: la classe base chiama
// questi tre metodi esattamente nei tre momenti del ciclo del filtro a particelle.
// Qui ci limitiamo a "fotografare" lo stato delle particelle in ciascun momento;
// la pubblicazione su ROS avviene nel nodo, al termine di ogni scansione.
// Mappa di una singola particella, gia' convertita nel formato di una OccupancyGrid
// (-1 sconosciuto, 0 libero, 100 occupato), senza dipendenze da ROS.
struct ParticleMapSnapshot {
    int size_x = 0;
    int size_y = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::vector<int8_t> data;
};

class DidacticGridSlamProcessor : public GMapping::GridSlamProcessor {
public:
    // Impostati dal nodo
    double occ_threshold = 0.25;       // soglia di occupazione, come per /map
    bool capture_maps = true;          // false: non produce le mappe (meno calcolo)
    // true: mappa completa (spazio libero e ostacoli), ricostruita lungo la traiettoria
    //       della particella come fa il nodo per /map;
    // false: mappa "interna" della particella, quella che GMapping usa per lo scan
    //       matching, che per efficienza contiene solo i punti colpiti dal laser.
    bool full_maps = true;
    // Strumento per ricostruire le mappe complete (configurato dal nodo con i
    // parametri del laser e generateMap = true) e limiti iniziali della mappa
    GMapping::ScanMatcher map_matcher;
    bool map_matcher_ready = false;
    double map_xmin = -10.0, map_ymin = -10.0, map_xmax = 10.0, map_ymax = 10.0, map_delta = 0.05;

    // Da chiamare prima di ogni processScan(): azzera le informazioni del ciclo
    void resetCycle() {
        scanmatched = false;
        resampled = false;
        didactic_ms = 0.0;
    }

    // 1) Dopo il modello di moto (a ogni scansione)
    void onOdometryUpdate() override {
        copyPoses(predicted);
    }

    // 2) Dopo lo scan matching (solo negli aggiornamenti, ogni linearUpdate metri / angularUpdate rad)
    //
    // Qui ogni particella ha appena ricevuto il punteggio dello scan matching, sommato
    // al proprio peso (logaritmico). La sua mappa NON contiene ancora la scansione
    // corrente: e' esattamente la mappa su cui la particella e' stata giudicata.
    // La scansione viene integrata nelle mappe subito dopo, insieme al ricampionamento.
    void onScanmatchUpdate() override {
        scanmatched = true;
        copyPoses(matched);

        const auto& particles = getParticles();
        const size_t n = particles.size();
        if (n == 0)
            return;

        // Pesi normalizzati, con la stessa formula che GMapping usa in normalize()
        // per calcolare Neff: i pesi logaritmici vengono "addolciti" dividendo per
        // obsSigmaGain * N, cosi' una singola scansione non decide tutto da sola.
        const double gain = 1.0 / (getobsSigmaGain() * static_cast<double>(n));
        double lmax = -std::numeric_limits<double>::max();
        for (const auto& p : particles)
            lmax = std::max(lmax, p.weight);
        normalized_weights.clear();
        double wsum = 0.0;
        for (const auto& p : particles) {
            normalized_weights.push_back(std::exp(gain * (p.weight - lmax)));
            wsum += normalized_weights.back();
        }
        for (auto& w : normalized_weights)
            w /= wsum;

        // Particella con peso attuale piu' alto e piu' basso (peso dall'ultimo
        // ricampionamento: e' quello che decide Neff e il ricampionamento)
        best_index = static_cast<int>(
            std::max_element(normalized_weights.begin(), normalized_weights.end()) -
            normalized_weights.begin());
        worst_index = static_cast<int>(
            std::min_element(normalized_weights.begin(), normalized_weights.end()) -
            normalized_weights.begin());
        // Particella usata dal nodo per la mappa /map: quella con il peso CUMULATIVO
        // (somma su tutta la traiettoria) piu' alto
        cumulative_best_index = getBestParticleIndex();

        copyTrajectory(particles[best_index], best_trajectory);
        copyTrajectory(particles[worst_index], worst_trajectory);
        if (capture_maps) {
            const auto t0 = std::chrono::steady_clock::now();
            if (full_maps && map_matcher_ready) {
                buildFullMap(particles[best_index], best_map);
                buildFullMap(particles[worst_index], worst_map);
            } else {
                copyMap(particles[best_index].map, best_map);
                copyMap(particles[worst_index].map, worst_map);
            }
            // Tempo speso per le mappe didattiche: va escluso dal tempo del filtro
            didactic_ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        }
    }

    // 3) Al ricampionamento (solo se Neff < resampleThreshold * numero di particelle).
    // Qui le particelle non sono ancora state sostituite: getIndexes() dice quali
    // particelle verranno copiate (con ripetizioni) nella nuova generazione.
    void onResampleUpdate() override {
        resampled = true;
        neff_before_resample = getneff();
        resampled_poses.clear();
        std::set<unsigned int> unique;
        for (unsigned int idx : getIndexes()) {
            resampled_poses.push_back(getParticles()[idx].pose);
            unique.insert(idx);
        }
        unique_after_resample = static_cast<unsigned int>(unique.size());
    }

    std::vector<double> normalized_weights;
    int best_index = 0;
    int worst_index = 0;
    int cumulative_best_index = 0;
    std::vector<GMapping::OrientedPoint> best_trajectory;
    std::vector<GMapping::OrientedPoint> worst_trajectory;
    ParticleMapSnapshot best_map;
    ParticleMapSnapshot worst_map;

    std::vector<GMapping::OrientedPoint> predicted;
    std::vector<GMapping::OrientedPoint> matched;
    std::vector<GMapping::OrientedPoint> resampled_poses;
    bool scanmatched = false;
    bool resampled = false;
    double neff_before_resample = 0.0;
    double didactic_ms = 0.0;          // tempo (ms) speso nel ciclo per le aggiunte didattiche
    unsigned int unique_after_resample = 0;

    // Converte una mappa di GMapping con la stessa regola usata per /map
    void copyMap(const GMapping::ScanMatcherMap& map, ParticleMapSnapshot& out) const {
        out.size_x = map.getMapSizeX();
        out.size_y = map.getMapSizeY();
        out.resolution = map.getResolution();
        const GMapping::Point origin = map.map2world(0, 0);
        out.origin_x = origin.x;
        out.origin_y = origin.y;
        out.data.assign(static_cast<size_t>(out.size_x) * out.size_y, -1);
        for (int y = 0; y < out.size_y; y++) {
            for (int x = 0; x < out.size_x; x++) {
                const double occ = map.cell(x, y);   // accesso const: le celle mai viste valgono -1
                int8_t v = -1;
                if (occ >= 0.0)
                    v = (occ > occ_threshold) ? 100 : 0;
                out.data[static_cast<size_t>(y) * out.size_x + x] = v;
            }
        }
    }

private:
    void copyPoses(std::vector<GMapping::OrientedPoint>& out) const {
        out.clear();
        for (const auto& p : getParticles())
            out.push_back(p.pose);
    }

    // Traiettoria ipotizzata dalla particella: si risale l'albero delle traiettorie
    // dal nodo della particella fino alla radice, poi si aggiunge la posa attuale.
    static void copyTrajectory(const Particle& p, std::vector<GMapping::OrientedPoint>& out) {
        out.clear();
        for (const TNode* node = p.node; node != nullptr; node = node->parent)
            out.push_back(node->pose);
        std::reverse(out.begin(), out.end());
        out.push_back(p.pose);
    }

    // Ricostruisce la mappa completa di una particella registrando, lungo la sua
    // traiettoria, le scansioni memorizzate nei nodi dell'albero (come updateMap()
    // del nodo per /map). Va fatto qui: dopo il ricampionamento i nodi delle
    // particelle eliminate vengono cancellati.
    void buildFullMap(const Particle& p, ParticleMapSnapshot& out) {
        GMapping::Point center((map_xmin + map_xmax) / 2.0, (map_ymin + map_ymax) / 2.0);
        GMapping::ScanMatcherMap smap(center, map_xmin, map_ymin, map_xmax, map_ymax, map_delta);
        for (const TNode* node = p.node; node != nullptr; node = node->parent) {
            if (!node->reading)
                continue;
            map_matcher.invalidateActiveArea();
            map_matcher.computeActiveArea(smap, node->pose, &((*node->reading)[0]));
            map_matcher.registerScan(smap, node->pose, &((*node->reading)[0]));
        }
        copyMap(smap, out);
    }


};

class SlamGmapping : public rclcpp::Node{
public:
    SlamGmapping();
    ~SlamGmapping() override;

    void init();
    void startLiveSlam();
    void publishTransform();
    void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    void publishLoop(double transform_publish_period);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;

    // --- Versione didattica ---
    // Pose di tutte le particelle (posa del robot, frame map), a ogni scansione
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    // Istantanee delle tre fasi, pubblicate a ogni aggiornamento del filtro
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr predicted_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr matched_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr resampled_pub_;
    // Numeri di ogni aggiornamento
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr neff_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr neff_threshold_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resampled_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr num_particles_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr unique_particles_pub_;
    // Riepilogo testuale visualizzato in RViz2 sopra il robot
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr info_pub_;
    // Mappe e traiettorie della particella con peso piu' alto e piu' basso
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_best_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_worst_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_best_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_worst_pub_;
    // Pesi di tutte le particelle (sfere colorate) e valori estremi
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr weights_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr weight_max_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr weight_min_pub_;
    bool publish_particle_maps_;

    // --- Passo 5 ---
    // Scan matching: correzioni delle particelle e scansione proiettata
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scanmatch_pub_;
    // Albero delle traiettorie e coalescenza
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr coalescence_pub_;
    // Ellisse di incertezza e deviazioni standard della nuvola
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uncertainty_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr std_xy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr std_theta_pub_;
    // Discordanze tra la mappa della particella migliore e della peggiore
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr disagreement_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr disagreement_cells_pub_;
    // Tempi di calcolo
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr update_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr didactic_time_pub_;

    std::vector<double> last_ranges_;   // ultima scansione, nell'ordine usato da GMapping

    // --- Passo 6 --- odometria con deriva e mappa costruita con la sola odometria
    bool odom_drift_;                   // true: usa il frame odom_noisy del nodo odom_drift
    bool publish_odometry_map_;
    GMapping::ScanMatcherMap* odo_map_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_odometry_pub_;
    void updateOdometryMap(const GMapping::OrientedPoint& odom_pose,
                           const builtin_interfaces::msg::Time& stamp);
    double last_update_ms_;             // tempo del filtro nell'ultimo aggiornamento
    bool publish_tree_;
    double ellipse_sigma_;

    void publishScanMatch(const builtin_interfaces::msg::Time& stamp) const;
    int publishTree(const builtin_interfaces::msg::Time& stamp) const;
    void publishUncertainty(const std::vector<GMapping::OrientedPoint>& poses,
                            const builtin_interfaces::msg::Time& stamp) const;
    int publishDisagreement(const builtin_interfaces::msg::Time& stamp) const;

    void publishParticleMap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub,
                            const ParticleMapSnapshot& snap,
                            const builtin_interfaces::msg::Time& stamp,
                            int8_t occupied_value = 100) const;
    // true: nella mappa della particella migliore le celle occupate valgono 120 invece
    // di 100, valore che RViz2 (schema "map") disegna in verde
    bool map_best_green_;
    void publishTrajectory(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub,
                           const std::vector<GMapping::OrientedPoint>& poses,
                           const builtin_interfaces::msg::Time& stamp) const;
    void publishWeights(const builtin_interfaces::msg::Time& stamp) const;

    // Trasformazione fissa base_frame -> "laser centrato" usato internamente da GMapping.
    // Serve a convertire le pose delle particelle (che GMapping esprime come pose del
    // laser centrato) in pose del robot, con la freccia orientata come il robot.
    tf2::Transform base_to_centered_laser_;
    // Altezza a cui disegnare le particelle (per vederle sopra il modello del robot)
    double particles_z_;
    // Numero di aggiornamenti del filtro (scan matching) eseguiti
    int update_count_;

    geometry_msgs::msg::Pose toRobotPose(const GMapping::OrientedPoint& p) const;
    void publishPoses(const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& pub,
                      const std::vector<GMapping::OrientedPoint>& poses,
                      const builtin_interfaces::msg::Time& stamp) const;
    void publishUpdateInfo(const builtin_interfaces::msg::Time& stamp);

    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan >> scan_filter_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;

    DidacticGridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    geometry_msgs::msg::PoseStamped centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    nav_msgs::msg::OccupancyGrid map_;

    tf2::Duration map_update_interval_;
    tf2::Transform map_to_odom_;
    std::mutex map_to_odom_mutex_;
    std::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    std::shared_ptr<std::thread> transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t);
    bool initMapper(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    bool addScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;
};

#endif //SLAM_GMAPPING_DIDACTIC_SLAM_GMAPPING_H_
