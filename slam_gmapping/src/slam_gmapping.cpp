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
/* Modified by: Charles DuHadway */

//
// Created by shivesh on 29/10/18.
//

#include <cmath>
#include <vector>
#include <cstdio>
#include <chrono>
#include <string>
#include "std_msgs/msg/color_rgba.hpp"
#include "slam_gmapping/slam_gmapping.h"

#include "tf2_ros/create_timer_ros.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using std::placeholders::_1;

SlamGmapping::SlamGmapping():
    Node("slam_gmapping"),
    scan_filter_sub_(nullptr),
    scan_filter_(nullptr),
    laser_count_(0),
    transform_thread_(nullptr)
{
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
     auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    map_to_odom_.setIdentity();
    base_to_centered_laser_.setIdentity();
    seed_ = static_cast<unsigned long>(time(nullptr));
    init();
    startLiveSlam();
}

void SlamGmapping::init() {
    gsp_ = new DidacticGridSlamProcessor();
    particles_z_ = 0.25;
    update_count_ = 0;
    publish_particle_maps_ = true;
    map_best_green_ = true;
    last_update_ms_ = 0.0;
    odom_drift_ = false;
    publish_odometry_map_ = true;
    publish_tree_ = true;
    ellipse_sigma_ = 2.0;

    gsp_laser_ = nullptr;
    gsp_odom_ = nullptr;
    got_first_scan_ = false;
    got_map_ = false;

    throttle_scans_ = 1;
    base_frame_ = "base_link";
    map_frame_ = "map";
    odom_frame_ = "odom";
    transform_publish_period_ = 0.05;

    map_update_interval_ = tf2::durationFromSec(0.5);
    maxUrange_ = 80.0;  maxRange_ = 0.0;
    minimum_score_ = 0;
    sigma_ = 0.05;
    kernelSize_ = 1;
    lstep_ = 0.05;
    astep_ = 0.05;
    iterations_ = 5;
    lsigma_ = 0.075;
    ogain_ = 3.0;
    lskip_ = 0;
    srr_ = 0.1;
    srt_ = 0.2;
    str_ = 0.1;
    stt_ = 0.2;
    linearUpdate_ = 1.0;
    angularUpdate_ = 0.5;
    temporalUpdate_ = 1.0;
    resampleThreshold_ = 0.5;
    particles_ = 30;
    xmin_ = -10.0;
    ymin_ = -10.0;
    xmax_ = 10.0;
    ymax_ = 10.0;
    delta_ = 0.05;
    occ_thresh_ = 0.25;
    llsamplerange_ = 0.01;
    llsamplestep_ = 0.01;
    lasamplerange_ = 0.005;
    lasamplestep_ = 0.005;
    tf_delay_ = transform_publish_period_;

    // --- Versione didattica ---
    // I valori qui sopra restano i default (identici alla versione originale), ma ora
    // si possono cambiare dall'esterno come parametri ROS, ad esempio con un file YAML.
    // Vengono letti solo all'avvio: per cambiarli occorre riavviare il nodo.
    particles_          = declare_parameter<int>("particles", particles_);
    resampleThreshold_  = declare_parameter<double>("resampleThreshold", resampleThreshold_);
    // Rumore del modello di moto (odometria):
    //   srr: traslazione dovuta a traslazione   srt: rotazione dovuta a traslazione
    //   str: traslazione dovuta a rotazione     stt: rotazione dovuta a rotazione
    srr_                = declare_parameter<double>("srr", srr_);
    srt_                = declare_parameter<double>("srt", srt_);
    str_                = declare_parameter<double>("str", str_);
    stt_                = declare_parameter<double>("stt", stt_);
    // Quando eseguire scan matching e ricampionamento:
    // dopo linearUpdate metri, angularUpdate radianti o temporalUpdate secondi (<0 = mai)
    linearUpdate_       = declare_parameter<double>("linearUpdate", linearUpdate_);
    angularUpdate_      = declare_parameter<double>("angularUpdate", angularUpdate_);
    temporalUpdate_     = declare_parameter<double>("temporalUpdate", temporalUpdate_);
    // Scan matching
    minimum_score_      = declare_parameter<double>("minimumScore", minimum_score_);
    // Guadagno delle osservazioni (obsSigmaGain): divide i pesi logaritmici prima di
    // normalizzarli. Valori piu' bassi rendono i pesi piu' "netti" (una scansione conta
    // di piu'), valori piu' alti li appiattiscono.
    ogain_              = declare_parameter<double>("ogain", ogain_);
    lstep_              = declare_parameter<double>("lstep", lstep_);
    astep_              = declare_parameter<double>("astep", astep_);
    iterations_         = declare_parameter<int>("iterations", iterations_);
    // Mappa
    delta_              = declare_parameter<double>("delta", delta_);
    occ_thresh_         = declare_parameter<double>("occ_thresh", occ_thresh_);
    throttle_scans_     = declare_parameter<int>("throttle_scans", throttle_scans_);
    // Seme del generatore casuale: 0 = diverso a ogni avvio; un valore fisso rende
    // le esecuzioni ripetibili (utile per confrontare parametri a parita' di rumore)
    const int seed      = declare_parameter<int>("seed", 0);
    // Altezza (m) a cui disegnare le particelle in RViz2: il TurtleBot3 Burger e' alto
    // circa 0.19 m, quindi con 0.25 le frecce restano visibili sopra il modello
    particles_z_        = declare_parameter<double>("particles_z", particles_z_);
    // Pubblica le mappe della particella migliore e peggiore (false = meno calcolo)
    publish_particle_maps_ = declare_parameter<bool>("publish_particle_maps", publish_particle_maps_);
    gsp_->capture_maps = publish_particle_maps_;
    // true: mappe complete; false: mappe interne (solo punti colpiti dal laser)
    gsp_->full_maps = declare_parameter<bool>("particle_maps_full", true);
    // Ostacoli della mappa della particella migliore in verde (solo per la visualizzazione)
    map_best_green_ = declare_parameter<bool>("map_best_green", map_best_green_);
    // Albero delle traiettorie di tutte le particelle (passo 5)
    publish_tree_ = declare_parameter<bool>("publish_tree", publish_tree_);
    // Semiassi dell'ellisse di incertezza, in deviazioni standard (passo 5)
    ellipse_sigma_ = declare_parameter<double>("ellipse_sigma", ellipse_sigma_);
    // Odometria con deriva artificiale (passo 6): se attiva, GMapping legge l'odometria
    // dal frame "odom_noisy" pubblicato dal nodo odom_drift (avviato dal launch file)
    odom_drift_ = declare_parameter<bool>("odom_drift", odom_drift_);
    if (odom_drift_) {
        odom_frame_ = "odom_noisy";
        RCLCPP_INFO(get_logger(), "Deriva dell'odometria attiva: odometria letta dal frame %s",
                    odom_frame_.c_str());
    }
    // Mappa costruita con la sola odometria, senza correzioni (passo 6)
    publish_odometry_map_ = declare_parameter<bool>("publish_odometry_map", publish_odometry_map_);
    gsp_->occ_threshold = occ_thresh_;
    if (seed > 0)
        seed_ = static_cast<unsigned long>(seed);
    if (throttle_scans_ < 1)
        throttle_scans_ = 1;

    RCLCPP_INFO(get_logger(),
        "Parametri: particles=%d resampleThreshold=%.2f srr=%.2f srt=%.2f str=%.2f stt=%.2f "
        "linearUpdate=%.2f angularUpdate=%.2f temporalUpdate=%.2f ogain=%.2f",
        particles_, resampleThreshold_, srr_, srt_, str_, stt_,
        linearUpdate_, angularUpdate_, temporalUpdate_, ogain_);
}

void SlamGmapping::startLiveSlam() {
    entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::SystemDefaultsQoS());
    sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS());
    sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::SystemDefaultsQoS());
    // --- Versione didattica ---
    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "gmapping/particles", rclcpp::SystemDefaultsQoS());
    predicted_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "gmapping/update/step1_predicted", rclcpp::SystemDefaultsQoS());
    matched_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "gmapping/update/step2_matched", rclcpp::SystemDefaultsQoS());
    resampled_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "gmapping/update/step3_resampled", rclcpp::SystemDefaultsQoS());
    neff_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/neff", rclcpp::SystemDefaultsQoS());
    neff_threshold_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/neff_threshold", rclcpp::SystemDefaultsQoS());
    resampled_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "gmapping/resampled", rclcpp::SystemDefaultsQoS());
    num_particles_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "gmapping/num_particles", rclcpp::SystemDefaultsQoS());
    unique_particles_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "gmapping/unique_particles", rclcpp::SystemDefaultsQoS());
    info_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "gmapping/info", rclcpp::SystemDefaultsQoS());
    map_best_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "gmapping/map_best_particle", rclcpp::SystemDefaultsQoS());
    map_worst_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "gmapping/map_worst_particle", rclcpp::SystemDefaultsQoS());
    path_best_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "gmapping/trajectory_best_particle", rclcpp::SystemDefaultsQoS());
    path_worst_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "gmapping/trajectory_worst_particle", rclcpp::SystemDefaultsQoS());
    weights_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "gmapping/particle_weights", rclcpp::SystemDefaultsQoS());
    weight_max_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/weight_max", rclcpp::SystemDefaultsQoS());
    weight_min_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/weight_min", rclcpp::SystemDefaultsQoS());
    // --- Passo 5 ---
    scanmatch_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "gmapping/scanmatch", rclcpp::SystemDefaultsQoS());
    tree_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "gmapping/trajectory_tree", rclcpp::SystemDefaultsQoS());
    coalescence_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "gmapping/coalescence_depth", rclcpp::SystemDefaultsQoS());
    uncertainty_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "gmapping/uncertainty", rclcpp::SystemDefaultsQoS());
    std_xy_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/std_xy", rclcpp::SystemDefaultsQoS());
    std_theta_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/std_theta_deg", rclcpp::SystemDefaultsQoS());
    disagreement_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "gmapping/map_disagreement", rclcpp::SystemDefaultsQoS());
    disagreement_cells_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "gmapping/map_disagreement_cells", rclcpp::SystemDefaultsQoS());
    update_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/update_time_ms", rclcpp::SystemDefaultsQoS());
    didactic_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "gmapping/didactic_time_ms", rclcpp::SystemDefaultsQoS());
    map_odometry_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "gmapping/map_odometry", rclcpp::SystemDefaultsQoS());
    scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
            (node_, "scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
//    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//        "scan", rclcpp::SensorDataQoS(),
//        std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
            (*scan_filter_sub_, *buffer_, odom_frame_, 10, node_);
    scan_filter_->registerCallback(std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&SlamGmapping::publishLoop, this, transform_publish_period_));
}

void SlamGmapping::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    rclcpp::Rate r(1.0 / transform_publish_period);
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}

SlamGmapping::~SlamGmapping()
{
    if(transform_thread_){
        transform_thread_->join();
    }

    delete gsp_;
    delete gsp_laser_;
    delete gsp_odom_;
    delete odo_map_;
}

bool SlamGmapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.header.stamp = t;
    // Get the laser's pose that is centered
    geometry_msgs::msg::PoseStamped odom_pose;
    try
    {
        buffer_->transform(centered_laser_pose_, odom_pose, odom_frame_, tf2::durationFromSec(1.0));
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }

    double yaw = tf2::getYaw(odom_pose.pose.orientation);

    gmap_pose = GMapping::OrientedPoint(odom_pose.pose.position.x,
                                        odom_pose.pose.position.y,
                                        yaw);
    return true;
}

bool SlamGmapping::initMapper(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    if (scan->ranges.size() < 2)
        return false;
    laser_frame_ = scan->header.frame_id;
    // Get the laser's pose, relative to base.
    geometry_msgs::msg::PoseStamped ident;
    geometry_msgs::msg::PoseStamped laser_pose;

    try{
        ident.header.frame_id = laser_frame_;
        ident.header.stamp = scan->header.stamp;
        tf2::Transform transform;
        transform.setIdentity();
        tf2::toMsg(transform, ident.pose);
        buffer_->transform(ident, laser_pose, base_frame_);
    }
    catch (tf2::TransformException& e){
        RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)", e.what());
        return false;
    }

    // create a point 1m above the laser position and transform it into the laser-frame
    geometry_msgs::msg::PointStamped up;
    up.header.stamp = scan->header.stamp;
    up.header.frame_id = base_frame_;
    up.point.x = up.point.y = 0;
    up.point.z = 1 + laser_pose.pose.position.z;
    try
    {
        buffer_->transform(up, up, laser_frame_);
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    if (fabs(fabs(up.point.z) - 1) > 0.001)
    {
        RCLCPP_INFO(this->get_logger(),
                "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.point.z);
        return false;
    }

    gsp_laser_beam_count_ = static_cast<unsigned int>(scan->ranges.size());

    double angle_center = (scan->angle_min + scan->angle_max)/2;

    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = get_clock()->now();
    tf2::Quaternion q;

    if (up.point.z > 0)
    {
        do_reverse_range_ = scan->angle_min > scan->angle_max;
        q.setEuler(angle_center, 0, 0);
        RCLCPP_INFO(this->get_logger(),"Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan->angle_min < scan->angle_max;
        q.setEuler(-angle_center, 0, M_PI);
        RCLCPP_INFO(this->get_logger(), "Laser is mounted upside down.");
    }

    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;

    centered_laser_pose_.pose.orientation.w = q.getW();
    centered_laser_pose_.pose.orientation.x = q.getX();
    centered_laser_pose_.pose.orientation.y = q.getY();
    centered_laser_pose_.pose.orientation.z = q.getZ();

    // --- Versione didattica ---
    // GMapping lavora con la posa di un "laser centrato" (il laser ruotato in modo che
    // i raggi siano simmetrici attorno all'asse x). Per il TurtleBot3 il laser copre
    // 0..2*pi, quindi il laser centrato e' ruotato di pi rispetto al robot: senza questa
    // correzione le frecce delle particelle punterebbero all'indietro.
    {
        tf2::Transform base_to_laser;
        tf2::fromMsg(laser_pose.pose, base_to_laser);
        tf2::Transform laser_to_centered(q, tf2::Vector3(0.0, 0.0, 0.0));
        base_to_centered_laser_ = base_to_laser * laser_to_centered;
    }

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(scan->ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan->angle_min - scan->angle_max)/2;
    for(unsigned int i=0; i<scan->ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan->angle_increment);
    }

    RCLCPP_DEBUG(this->get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
            scan->angle_min, scan->angle_max, scan->angle_increment);
    RCLCPP_DEBUG(this->get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f",
            laser_angles_.front(), laser_angles_.back(), std::fabs(scan->angle_increment));

    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // setting maxRange and maxUrange here so we can set a reasonable default
    maxRange_ = scan->range_max - 0.01;
    maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan->angle_increment), gmap_pose,
                                           0.0, maxRange_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, static_cast<unsigned int>(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(static_cast<unsigned int>(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // --- Versione didattica ---
    // Strumento per ricostruire le mappe complete della particella migliore e peggiore,
    // configurato come quello usato in updateMap() per /map
    gsp_->map_matcher.setLaserParameters(gsp_laser_beam_count_, &(laser_angles_[0]),
                                         gsp_laser_->getPose());
    gsp_->map_matcher.setlaserMaxRange(maxRange_);
    gsp_->map_matcher.setusableRange(maxUrange_);
    gsp_->map_matcher.setgenerateMap(true);
    gsp_->map_xmin = xmin_;
    gsp_->map_ymin = ymin_;
    gsp_->map_xmax = xmax_;
    gsp_->map_ymax = ymax_;
    gsp_->map_delta = delta_;
    gsp_->map_matcher_ready = true;
    // Mappa costruita con la sola odometria (passo 6)
    {
        GMapping::Point center((xmin_ + xmax_) / 2.0, (ymin_ + ymax_) / 2.0);
        odo_map_ = new GMapping::ScanMatcherMap(center, xmin_, ymin_, xmax_, ymax_, delta_);
    }

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, static_cast<unsigned int>(seed_));

    RCLCPP_INFO(this->get_logger(), "Initialization complete");

    return true;
}

bool SlamGmapping::addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose) {
    if (!getOdomPose(gmap_pose, scan->header.stamp))
        return false;

    const size_t n_in = scan->ranges.size();
    if (n_in < 2 || gsp_laser_beam_count_ < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Empty or too short scan, skipping");
        return false;
    }

    // The number of beams may change between scans: resample to the size used at init.
    // NaN/inf/short readings are replaced with range_max, which the mapper treats as "no hit".
    std::vector<double> ranges_double(gsp_laser_beam_count_);
    for (unsigned int i = 0; i < gsp_laser_beam_count_; i++) {
        size_t src = (n_in == gsp_laser_beam_count_)
                     ? i
                     : static_cast<size_t>(std::lround(static_cast<double>(i) * (n_in - 1) / (gsp_laser_beam_count_ - 1)));
        // If the angle increment is negative, we have to invert the order of the readings.
        if (do_reverse_range_)
            src = n_in - 1 - src;
        const float r = scan->ranges[src];
        if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
            ranges_double[i] = static_cast<double>(scan->range_max);
        else
            ranges_double[i] = static_cast<double>(r);
    }

    // --- Versione didattica --- conserva la scansione per proiettarla sulla mappa
    last_ranges_ = ranges_double;

    GMapping::RangeReading reading(gsp_laser_beam_count_,
                                   ranges_double.data(),
                                   gsp_laser_,
                                   scan->header.stamp.sec);

    reading.setPose(gmap_pose);

    RCLCPP_DEBUG(this->get_logger(), "processing scan");

    return gsp_->processScan(reading);
}


void SlamGmapping::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    tf2::TimePoint last_map_update = tf2::TimePointZero;

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;

    // processScan() applica il modello di moto a tutte le particelle a ogni scansione,
    // ma esegue scan matching e (eventuale) ricampionamento solo quando il robot ha
    // percorso abbastanza strada: 'processed' e' vero solo in quel caso.
    gsp_->resetCycle();
    const auto t_start = std::chrono::steady_clock::now();
    const bool processed = addScan(scan, odom_pose);
    // Tempo del filtro: tutta l'elaborazione della scansione meno le aggiunte didattiche
    if (processed && gsp_->scanmatched) {
        const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();
        last_update_ms_ = std::max(0.0, total_ms - gsp_->didactic_ms);
    }

    // --- Versione didattica ---
    if (got_first_scan_) {
        // Particelle "dal vivo", a ogni scansione: si vede la loro dispersione dovuta
        // al solo modello di moto tra un aggiornamento e il successivo.
        std::vector<GMapping::OrientedPoint> current;
        for (const auto& p : gsp_->getParticles())
            current.push_back(p.pose);
        publishPoses(particles_pub_, current, scan->header.stamp);
        // Ellisse di incertezza della nuvola, anch'essa a ogni scansione
        publishUncertainty(current, scan->header.stamp);

        // Se in questa scansione c'e' stato un aggiornamento completo del filtro,
        // pubblica le istantanee delle tre fasi e i numeri.
        if (processed && gsp_->scanmatched)
            publishUpdateInfo(scan->header.stamp);
        // Mappa con la sola odometria: la scansione viene registrata nella posa
        // indicata dall'odometria, senza scan matching (agli stessi istanti degli
        // aggiornamenti del filtro, piu' la prima scansione)
        if (processed)
            updateOdometryMap(odom_pose, scan->header.stamp);
    }

    if(processed)
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        tf2::Quaternion q;
        q.setRPY(0, 0, mpose.theta);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        q.setRPY(0, 0, odom_pose.theta);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        tf2::TimePoint timestamp = tf2_ros::fromMsg(scan->header.stamp);
        if(!got_map_ || (timestamp - last_map_update) > map_update_interval_)
        {
            updateMap(scan);
            last_map_update = tf2_ros::fromMsg(scan->header.stamp);
        }
    }
}

// --- Versione didattica ---
// Converte la posa di una particella (posa del "laser centrato" secondo GMapping)
// nella posa del robot nel frame della mappa, sollevata di particles_z_.
geometry_msgs::msg::Pose SlamGmapping::toRobotPose(const GMapping::OrientedPoint& p) const
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.theta);
    tf2::Transform map_to_centered_laser(q, tf2::Vector3(p.x, p.y, 0.0));

    // map->base = map->laser_centrato * laser_centrato->base
    tf2::Transform map_to_base = map_to_centered_laser * base_to_centered_laser_.inverse();

    geometry_msgs::msg::Pose pose;
    pose.position.x = map_to_base.getOrigin().x();
    pose.position.y = map_to_base.getOrigin().y();
    pose.position.z = particles_z_;
    tf2::Quaternion q_base;
    q_base.setRPY(0.0, 0.0, tf2::getYaw(map_to_base.getRotation()));
    pose.orientation = tf2::toMsg(q_base);
    return pose;
}

void SlamGmapping::publishPoses(const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& pub,
                                const std::vector<GMapping::OrientedPoint>& poses,
                                const builtin_interfaces::msg::Time& stamp) const
{
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    for (const auto& p : poses)
        msg.poses.push_back(toRobotPose(p));
    pub->publish(msg);
}

// Pubblica tutto cio' che riguarda un aggiornamento del filtro (scan matching avvenuto).
void SlamGmapping::publishUpdateInfo(const builtin_interfaces::msg::Time& stamp)
{
    update_count_++;
    const int n = static_cast<int>(gsp_->getParticles().size());

    // Neff usato per decidere se ricampionare. Se il ricampionamento c'e' stato,
    // dopo di esso i pesi vengono azzerati e Neff torna uguale a n: per questo
    // usiamo il valore salvato subito prima.
    const double neff = gsp_->resampled ? gsp_->neff_before_resample : gsp_->getneff();
    const double threshold = resampleThreshold_ * n;
    const int unique = gsp_->resampled ? static_cast<int>(gsp_->unique_after_resample) : n;

    // 1) prima dello scan matching, 2) dopo lo scan matching, 3) dopo il ricampionamento.
    // Se non c'e' stato ricampionamento si pubblica un insieme vuoto, cosi' in RViz2
    // le frecce della fase 3 spariscono invece di restare quelle di un ciclo precedente.
    publishPoses(predicted_pub_, gsp_->predicted, stamp);
    publishPoses(matched_pub_, gsp_->matched, stamp);
    publishPoses(resampled_pub_,
                 gsp_->resampled ? gsp_->resampled_poses : std::vector<GMapping::OrientedPoint>(),
                 stamp);

    std_msgs::msg::Float64 f;
    f.data = neff;
    neff_pub_->publish(f);
    f.data = threshold;
    neff_threshold_pub_->publish(f);

    std_msgs::msg::Bool b;
    b.data = gsp_->resampled;
    resampled_flag_pub_->publish(b);

    std_msgs::msg::Int32 i;
    i.data = n;
    num_particles_pub_->publish(i);
    i.data = unique;
    unique_particles_pub_->publish(i);

    // Pesi, mappe e traiettorie della particella migliore e peggiore
    const double w_max = gsp_->normalized_weights.empty() ? 0.0 : gsp_->normalized_weights[gsp_->best_index];
    const double w_min = gsp_->normalized_weights.empty() ? 0.0 : gsp_->normalized_weights[gsp_->worst_index];
    f.data = w_max;
    weight_max_pub_->publish(f);
    f.data = w_min;
    weight_min_pub_->publish(f);
    publishWeights(stamp);
    publishTrajectory(path_best_pub_, gsp_->best_trajectory, stamp);
    publishTrajectory(path_worst_pub_, gsp_->worst_trajectory, stamp);
    if (publish_particle_maps_) {
        publishParticleMap(map_best_pub_, gsp_->best_map, stamp,
                           map_best_green_ ? static_cast<int8_t>(120) : static_cast<int8_t>(100));
        publishParticleMap(map_worst_pub_, gsp_->worst_map, stamp);
    }
    const bool same_best = (gsp_->best_index == gsp_->cumulative_best_index);

    // --- Passo 5 ---
    publishScanMatch(stamp);
    const int coalescence = publishTree(stamp);
    const int disagreement = publishDisagreement(stamp);
    i.data = coalescence;
    coalescence_pub_->publish(i);
    f.data = last_update_ms_;
    update_time_pub_->publish(f);
    f.data = gsp_->didactic_ms;
    didactic_time_pub_->publish(f);
    char extra[256];
    snprintf(extra, sizeof(extra),
             "\nCoalescenza: %d aggiornamenti fa\nDiscordanze mappe: %d celle\n"
             "Tempo filtro %.0f ms (+%.0f ms didattica)",
             coalescence, disagreement, last_update_ms_, gsp_->didactic_ms);

    // Testo riassuntivo, sopra la particella migliore
    char text[512];
    if (gsp_->resampled) {
        snprintf(text, sizeof(text),
                 "Aggiornamento %d\nParticelle: %d\nNeff: %.1f (soglia %.1f)\n"
                 "RICAMPIONAMENTO: distinte %d / %d\n"
                 "Peso max %.3f (n.%d)  min %.4f (n.%d)\n"
                 "/map: particella n.%d (%s)",
                 update_count_, n, neff, threshold, unique, n,
                 w_max, gsp_->best_index, w_min, gsp_->worst_index,
                 gsp_->cumulative_best_index, same_best ? "= peso max" : "peso cumulativo");
    } else {
        snprintf(text, sizeof(text),
                 "Aggiornamento %d\nParticelle: %d\nNeff: %.1f (soglia %.1f)\n"
                 "nessun ricampionamento\n"
                 "Peso max %.3f (n.%d)  min %.4f (n.%d)\n"
                 "/map: particella n.%d (%s)",
                 update_count_, n, neff, threshold,
                 w_max, gsp_->best_index, w_min, gsp_->worst_index,
                 gsp_->cumulative_best_index, same_best ? "= peso max" : "peso cumulativo");
    }

    std::string full_text = std::string(text) + extra;

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = "gmapping_info";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toRobotPose(gsp_->getParticles()[gsp_->getBestParticleIndex()].pose);
    m.pose.position.z = particles_z_ + 0.5;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.10;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m.text = full_text;
    info_pub_->publish(m);

    RCLCPP_INFO(get_logger(),
                "[Aggiornamento %d] Neff=%.1f soglia=%.1f -> %s (distinte %d/%d) | "
                "peso max %.3f (n.%d) min %.4f (n.%d) | /map da particella n.%d | "
                "coalescenza %d | discordanze %d | filtro %.0f ms (+%.0f didattica)",
                update_count_, neff, threshold,
                gsp_->resampled ? "RICAMPIONAMENTO" : "nessun ricampionamento", unique, n,
                w_max, gsp_->best_index, w_min, gsp_->worst_index, gsp_->cumulative_best_index,
                coalescence, disagreement, last_update_ms_, gsp_->didactic_ms);
}

// --- Passo 6 ---
// Mappa costruita con la sola odometria: ogni scansione viene registrata nella posa
// fornita dall'odometria (eventualmente con deriva), come se fosse esatta. Si
// confronta con /map per vedere che cosa corregge la SLAM.
void SlamGmapping::updateOdometryMap(const GMapping::OrientedPoint& odom_pose,
                                     const builtin_interfaces::msg::Time& stamp)
{
    if (!publish_odometry_map_ || !odo_map_ || last_ranges_.empty() || !gsp_->map_matcher_ready)
        return;
    gsp_->map_matcher.invalidateActiveArea();
    gsp_->map_matcher.computeActiveArea(*odo_map_, odom_pose, last_ranges_.data());
    gsp_->map_matcher.registerScan(*odo_map_, odom_pose, last_ranges_.data());
    ParticleMapSnapshot snap;
    gsp_->copyMap(*odo_map_, snap);
    publishParticleMap(map_odometry_pub_, snap, stamp);
}

// --- Passo 5 ---
// Scan matching: per ogni particella un segmento dalla posa prevista (rosso) a quella
// corretta (verde); per la particella con peso massimo, la scansione corrente
// proiettata dalla posa prevista (rosso) e da quella corretta (verde).
void SlamGmapping::publishScanMatch(const builtin_interfaces::msg::Time& stamp) const
{
    const auto& pred = gsp_->predicted;
    const auto& match = gsp_->matched;
    if (pred.empty() || pred.size() != match.size())
        return;

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = stamp;
    clear.ns = "scanmatch_clear";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    std_msgs::msg::ColorRGBA red, green;
    red.r = 1.0f; red.g = 0.0f; red.b = 0.0f; red.a = 1.0f;
    green.r = 0.0f; green.g = 0.8f; green.b = 0.0f; green.a = 1.0f;

    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = map_frame_;
    lines.header.stamp = stamp;
    lines.ns = "correzioni";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.012;
    for (size_t k = 0; k < pred.size(); k++) {
        lines.points.push_back(toRobotPose(pred[k]).position);
        lines.colors.push_back(red);
        lines.points.push_back(toRobotPose(match[k]).position);
        lines.colors.push_back(green);
    }
    arr.markers.push_back(lines);

    // Punti finali dei raggi laser visti dalla posa p (posa del laser centrato).
    // Disegnati a pochi cm dal pavimento, sopra la mappa.
    auto scanPoints = [this](const GMapping::OrientedPoint& p, int id, const char* ns,
                             const std_msgs::msg::ColorRGBA& color,
                             const builtin_interfaces::msg::Time& st) {
        visualization_msgs::msg::Marker pts;
        pts.header.frame_id = map_frame_;
        pts.header.stamp = st;
        pts.ns = ns;
        pts.id = id;
        pts.type = visualization_msgs::msg::Marker::POINTS;
        pts.action = visualization_msgs::msg::Marker::ADD;
        pts.pose.orientation.w = 1.0;
        pts.scale.x = 0.04;
        pts.scale.y = 0.04;
        pts.color = color;
        const size_t nb = std::min(last_ranges_.size(), laser_angles_.size());
        for (size_t b = 0; b < nb; b++) {
            const double r = last_ranges_[b];
            if (r >= maxUrange_)
                continue;   // nessun ostacolo entro la portata utile
            geometry_msgs::msg::Point q;
            q.x = p.x + r * std::cos(p.theta + laser_angles_[b]);
            q.y = p.y + r * std::sin(p.theta + laser_angles_[b]);
            q.z = 0.03;
            pts.points.push_back(q);
        }
        return pts;
    };
    const int b = gsp_->best_index;
    if (b >= 0 && static_cast<size_t>(b) < pred.size()) {
        arr.markers.push_back(scanPoints(pred[b], 0, "scansione_prevista", red, stamp));
        arr.markers.push_back(scanPoints(match[b], 0, "scansione_corretta", green, stamp));
    }
    scanmatch_pub_->publish(arr);
}

// Albero delle traiettorie di tutte le particelle. Ogni ramo e' colorato in base a
// quante particelle attuali discendono da esso: blu scuro e spesso il tronco comune a
// tutte, azzurro chiaro i rami di poche particelle.
// Restituisce la profondita' di coalescenza: quanti aggiornamenti indietro bisogna
// risalire perche' tutte le traiettorie confluiscano in un unico antenato comune.
int SlamGmapping::publishTree(const builtin_interfaces::msg::Time& stamp) const
{
    using TNode = GMapping::GridSlamProcessor::TNode;
    const auto& particles = gsp_->getParticles();
    const int n = static_cast<int>(particles.size());

    // Coalescenza: si risale l'albero a passi uguali da tutte le particelle
    std::set<const TNode*> level;
    for (const auto& p : particles)
        if (p.node) level.insert(p.node);
    int depth = 0;
    while (level.size() > 1) {
        std::set<const TNode*> up;
        for (const TNode* node : level)
            if (node->parent) up.insert(node->parent);
        if (up.empty())
            break;
        level.swap(up);
        depth++;
    }

    if (!publish_tree_)
        return depth;

    // Numero di particelle attuali che discendono da ogni nodo
    std::map<const TNode*, int> descendants;
    for (const auto& p : particles)
        for (const TNode* node = p.node; node != nullptr; node = node->parent)
            descendants[node]++;

    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = map_frame_;
    lines.header.stamp = stamp;
    lines.ns = "albero";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.015;
    for (const auto& entry : descendants) {
        const TNode* node = entry.first;
        if (!node->parent)
            continue;
        const double t = (n > 0) ? static_cast<double>(entry.second) / n : 0.0;
        std_msgs::msg::ColorRGBA c;
        c.r = static_cast<float>(0.6 * (1.0 - t));
        c.g = static_cast<float>(0.4 + 0.5 * (1.0 - t));
        c.b = 1.0f;
        c.a = static_cast<float>(0.5 + 0.5 * t);
        geometry_msgs::msg::Point a = toRobotPose(node->pose).position;
        geometry_msgs::msg::Point b = toRobotPose(node->parent->pose).position;
        a.z = b.z = 0.04;
        lines.points.push_back(a);
        lines.colors.push_back(c);
        lines.points.push_back(b);
        lines.colors.push_back(c);
    }
    if (!lines.points.empty())
        tree_pub_->publish(lines);
    return depth;
}

// Ellisse di incertezza: media e covarianza delle posizioni delle particelle.
// I semiassi valgono ellipse_sigma_ deviazioni standard lungo le direzioni principali.
void SlamGmapping::publishUncertainty(const std::vector<GMapping::OrientedPoint>& poses,
                                      const builtin_interfaces::msg::Time& stamp) const
{
    const size_t n = poses.size();
    if (n == 0)
        return;
    std::vector<geometry_msgs::msg::Pose> rp;
    double mx = 0.0, my = 0.0, ms = 0.0, mc = 0.0;
    for (const auto& p : poses) {
        rp.push_back(toRobotPose(p));
        mx += rp.back().position.x;
        my += rp.back().position.y;
        const double yaw = tf2::getYaw(rp.back().orientation);
        ms += std::sin(yaw);
        mc += std::cos(yaw);
    }
    mx /= n; my /= n;
    const double mean_yaw = std::atan2(ms, mc);
    double cxx = 0.0, cxy = 0.0, cyy = 0.0, ctt = 0.0;
    for (const auto& pose : rp) {
        const double dx = pose.position.x - mx;
        const double dy = pose.position.y - my;
        const double dt = std::remainder(tf2::getYaw(pose.orientation) - mean_yaw, 2.0 * M_PI);
        cxx += dx * dx; cxy += dx * dy; cyy += dy * dy; ctt += dt * dt;
    }
    cxx /= n; cxy /= n; cyy /= n; ctt /= n;

    // Autovalori e orientazione della matrice di covarianza 2x2
    const double tr2 = 0.5 * (cxx + cyy);
    const double disc = std::sqrt(0.25 * (cxx - cyy) * (cxx - cyy) + cxy * cxy);
    const double l1 = std::max(tr2 + disc, 0.0);
    const double l2 = std::max(tr2 - disc, 0.0);
    const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "incertezza";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = mx;
    m.pose.position.y = my;
    m.pose.position.z = particles_z_ - 0.02;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    m.pose.orientation = tf2::toMsg(q);
    // dimensione minima per non avere un marker di scala nulla
    m.scale.x = std::max(2.0 * ellipse_sigma_ * std::sqrt(l1), 0.01);
    m.scale.y = std::max(2.0 * ellipse_sigma_ * std::sqrt(l2), 0.01);
    m.scale.z = 0.005;
    m.color.r = 0.2f;
    m.color.g = 0.4f;
    m.color.b = 1.0f;
    m.color.a = 0.35f;
    uncertainty_pub_->publish(m);

    std_msgs::msg::Float64 f;
    f.data = std::sqrt(cxx + cyy);
    std_xy_pub_->publish(f);
    f.data = std::sqrt(ctt) * 180.0 / M_PI;
    std_theta_pub_->publish(f);
}

// Mappa delle discordanze tra la particella con peso massimo e quella con peso minimo:
// 120 (verde) = occupato solo per la migliore, 100 (viola) = occupato solo per la
// peggiore, 0 (trasparente con lo schema costmap) = accordo o cella sconosciuta.
// Restituisce il numero di celle discordanti.
int SlamGmapping::publishDisagreement(const builtin_interfaces::msg::Time& stamp) const
{
    const auto& b = gsp_->best_map;
    const auto& w = gsp_->worst_map;
    if (!publish_particle_maps_ || b.data.empty() || b.data.size() != w.data.size() ||
        b.size_x != w.size_x)
        return 0;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame_;
    grid.info.map_load_time = stamp;
    grid.info.resolution = static_cast<float>(b.resolution);
    grid.info.width = static_cast<uint32_t>(b.size_x);
    grid.info.height = static_cast<uint32_t>(b.size_y);
    grid.info.origin.position.x = b.origin_x;
    grid.info.origin.position.y = b.origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(b.data.size(), 0);
    int count = 0;
    for (size_t k = 0; k < b.data.size(); k++) {
        if (b.data[k] == 100 && w.data[k] == 0) {
            grid.data[k] = 120;
            count++;
        } else if (w.data[k] == 100 && b.data[k] == 0) {
            grid.data[k] = 100;
            count++;
        }
    }
    disagreement_pub_->publish(grid);
    std_msgs::msg::Int32 c;
    c.data = count;
    disagreement_cells_pub_->publish(c);
    return count;
}

// Pubblica la mappa di una singola particella come OccupancyGrid nel frame della mappa
// occupied_value: valore delle celle occupate. Lo standard e' 100; RViz2 disegna i valori
// tra 101 e 127 (fuori standard) in verde, e lo usiamo solo per distinguere le mappe.
void SlamGmapping::publishParticleMap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub,
                                      const ParticleMapSnapshot& snap,
                                      const builtin_interfaces::msg::Time& stamp,
                                      int8_t occupied_value) const
{
    if (snap.data.empty())
        return;
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame_;
    grid.info.map_load_time = stamp;
    grid.info.resolution = static_cast<float>(snap.resolution);
    grid.info.width = static_cast<uint32_t>(snap.size_x);
    grid.info.height = static_cast<uint32_t>(snap.size_y);
    grid.info.origin.position.x = snap.origin_x;
    grid.info.origin.position.y = snap.origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data = snap.data;
    if (occupied_value != 100)
        std::replace(grid.data.begin(), grid.data.end(), static_cast<int8_t>(100), occupied_value);
    pub->publish(grid);
}

// Pubblica la traiettoria ipotizzata da una particella (pose del robot, frame map)
void SlamGmapping::publishTrajectory(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub,
                                     const std::vector<GMapping::OrientedPoint>& poses,
                                     const builtin_interfaces::msg::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    for (const auto& p : poses) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose = toRobotPose(p);
        path.poses.push_back(ps);
    }
    pub->publish(path);
}

// Una sfera per particella, nella posa dopo lo scan matching: dimensione e colore
// (dal rosso al verde) proporzionali al peso, relativo al peso massimo.
void SlamGmapping::publishWeights(const builtin_interfaces::msg::Time& stamp) const
{
    const auto& w = gsp_->normalized_weights;
    const auto& poses = gsp_->matched;
    if (w.empty() || w.size() != poses.size())
        return;
    const double w_max = *std::max_element(w.begin(), w.end());

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = stamp;
    // ns diverso da quello delle sfere: RViz2 rifiuta un MarkerArray che contiene
    // due marker con la stessa coppia (ns, id), anche se uno dei due e' DELETEALL
    clear.ns = "gmapping_weights_clear";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    for (size_t i = 0; i < w.size(); i++) {
        const double t = (w_max > 0.0) ? w[i] / w_max : 0.0;   // 0 = peso minimo relativo, 1 = massimo
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = stamp;
        m.ns = "gmapping_weights";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = toRobotPose(poses[i]);
        m.pose.position.z = particles_z_ + 0.03;
        const double d = 0.03 + 0.12 * t;
        m.scale.x = d;
        m.scale.y = d;
        m.scale.z = d;
        m.color.r = static_cast<float>(1.0 - t);
        m.color.g = static_cast<float>(t);
        m.color.b = 0.0f;
        m.color.a = 0.9f;
        arr.markers.push_back(m);
    }
    weights_pub_->publish(arr);
}

double SlamGmapping::computePoseEntropy()
{
    double weight_total=0.0;
    for (const auto &it : gsp_->getParticles()) {
        weight_total += it.weight;
    }
    double entropy = 0.0;
    for (const auto &it : gsp_->getParticles()) {
        if(it.weight/weight_total > 0.0)
            entropy += it.weight/weight_total * log(it.weight/weight_total);
    }
    return -entropy;
}

void SlamGmapping::updateMap(const sensor_msgs::msg::LaserScan::ConstSharedPtr /*scan*/)
{
    RCLCPP_DEBUG(this->get_logger(), "Update map");
    map_mutex_.lock();
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(gsp_laser_beam_count_, &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];
    std_msgs::msg::Float64 entropy;
    entropy.data = computePoseEntropy();
    if(entropy.data > 0.0)
        entropy_publisher_->publish(entropy);

    if(!got_map_) {
        map_.info.resolution = static_cast<nav_msgs::msg::MapMetaData::_resolution_type>(delta_);
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
    }

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    RCLCPP_DEBUG(this->get_logger(), "Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        RCLCPP_DEBUG(this->get_logger(), "  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            RCLCPP_DEBUG(this->get_logger(), "Reading is NULL");
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    // the map may have expanded, so resize ros message as well
    if(map_.info.width != (unsigned int) smap.getMapSizeX() || map_.info.height != (unsigned int) smap.getMapSizeY()) {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        RCLCPP_DEBUG(this->get_logger(), "map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        map_.info.width = static_cast<nav_msgs::msg::MapMetaData::_width_type>(smap.getMapSizeX());
        map_.info.height = static_cast<nav_msgs::msg::MapMetaData::_height_type>(smap.getMapSizeY());
        map_.info.origin.position.x = xmin_;
        map_.info.origin.position.y = ymin_;
        map_.data.resize(map_.info.width * map_.info.height);

        RCLCPP_DEBUG(this->get_logger(), "map origin: (%f, %f)", map_.info.origin.position.x, map_.info.origin.position.y);
    }

    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);
            if(occ < 0)
                map_.data[MAP_IDX(map_.info.width, x, y)] = -1;
            else if(occ > occ_thresh_)
            {
                //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.data[MAP_IDX(map_.info.width, x, y)] = 100;
            }
            else
                map_.data[MAP_IDX(map_.info.width, x, y)] = 0;
        }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.header.stamp = get_clock()->now();
    map_.header.frame_id = map_frame_;

    sst_->publish(map_);
    sstm_->publish(map_.info);
    map_mutex_.unlock();
}

void SlamGmapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration(
            static_cast<int32_t>(static_cast<rcl_duration_value_t>(tf_delay_)), 0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = odom_frame_;
    try {
        transform.transform = tf2::toMsg(map_to_odom_);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        RCLCPP_INFO(this->get_logger(), te.what());
    }
    map_to_odom_mutex_.unlock();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto slam_gmapping_node = std::make_shared<SlamGmapping>();
    rclcpp::spin(slam_gmapping_node);
    return(0);
}
