/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  linear_velocity_stddev = declare_parameter<double>("linear_velocity_stddev", 0.05);
  angular_velocity_stddev = declare_parameter<double>("angular_velocity_stddev", 0.05);
  max_scan_gap = declare_parameter<double>("max_scan_gap", 0.5);
  max_linear_speed = declare_parameter<double>("max_linear_speed", 1.5);
  max_angular_speed = declare_parameter<double>("max_angular_speed", 3.0);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);

  for (double value : {freq, linear_velocity_stddev, angular_velocity_stddev,
                       max_scan_gap, max_linear_speed, max_angular_speed})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("RF2O rates, limits and standard deviations must be positive");

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  
  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (!GT_pose_initialized || new_scan->ranges.size() < 80 ||
      !std::isfinite(new_scan->angle_min) || !std::isfinite(new_scan->angle_max) ||
      !std::isfinite(new_scan->angle_increment) || new_scan->angle_increment == 0.0 ||
      !(new_scan->range_max > new_scan->range_min)) return;

  auto scan = *new_scan;
  if (scan.angle_increment < 0.0) {
    std::reverse(scan.ranges.begin(), scan.ranges.end());
    std::swap(scan.angle_min, scan.angle_max);
    scan.angle_increment = -scan.angle_increment;
  }
  unsigned int valid = 0;
  for (auto &range : scan.ranges) {
    if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max)
      range = 0.0f;
    else ++valid;
  }
  if (valid < std::max<size_t>(80, scan.ranges.size() / 5)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Too few usable LiDAR returns");
    return;
  }
  const rclcpp::Time stamp(scan.header.stamp);
  const double age = (get_clock()->now() - stamp).seconds();
  if (age < -0.05 || age > max_scan_gap) return;
  const double center = 0.5 * (scan.angle_min + scan.angle_max);
  const double fov = scan.angle_max - scan.angle_min;
  if (fov <= 0.0 || std::abs(fov - scan.angle_increment * (scan.ranges.size() - 1)) > 0.01)
    return;
  if (!rf2o_ref.first_laser_scan) {
    // LD19 revolutions contain slightly different numbers of samples. Match the
    // established angular grid instead of continually discarding scan pairs.
    // Nearest measured rays preserve invalid sectors; large geometry changes
    // still reinitialize below.
    const auto count = scan.ranges.size();
    if (count != rf2o_ref.width &&
        std::abs(static_cast<long>(count) - static_cast<long>(rf2o_ref.width)) <= 8 &&
        std::abs(center - scan_center) <= 1e-5 && std::abs(fov - scan_fov) <= 1e-5 &&
        scan.header.frame_id == scan_frame) {
      std::vector<float> resampled(rf2o_ref.width);
      for (size_t i = 0; i < resampled.size(); ++i) {
        const size_t source = std::lround(static_cast<double>(i) * (count - 1) /
                                          (resampled.size() - 1));
        resampled[i] = scan.ranges[source];
      }
      scan.ranges.swap(resampled);
      scan.angle_increment = fov / (scan.ranges.size() - 1);
    }
    const double dt = (stamp - rf2o_ref.last_odom_time).seconds();
    if (dt <= 0.0) return;
    if (dt > max_scan_gap || scan.ranges.size() != rf2o_ref.width ||
        std::abs(center - scan_center) > 1e-5 || std::abs(fov - scan_fov) > 1e-5 ||
        scan.header.frame_id != scan_frame) {
      // Start a new scan pair at the last accepted pose after an outage/geometry change.
      rf2o_ref.first_laser_scan = true;
      new_scan_available = false;
    }
  }
  last_scan = scan;
  rf2o_ref.current_scan_time = stamp;
  if (rf2o_ref.first_laser_scan) {
    scan_center = center;
    scan_fov = fov;
    scan_frame = scan.header.frame_id;
    if (!setLaserPoseFromTf()) return;
    rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
    rf2o_ref.first_laser_scan = false;
    new_scan_available = false;
    return;
  }
  new_scan_available = true;
}


/** 
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{  
  bool retrieved = false;  
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // RF2O samples angles in [-FOV/2, FOV/2]. Rotate that virtual scan frame
  // into the actual LaserScan frame (LD19 uses [0, 2*pi]). No output sign hacks.
  Pose3d centered_scan = Pose3d::Identity();
  centered_scan.linear() = rf2o::matrixYaw(scan_center);
  rf2o_ref.setLaserPose(laser_tf * centered_scan);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready 
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    new_scan_available = false;
    const double dt = (rf2o_ref.current_scan_time - rf2o_ref.last_odom_time).seconds();
    if (dt <= 0.0 || dt > max_scan_gap) return;
    const Pose3d previous = rf2o_ref.getPose();
    const bool solved = rf2o_ref.odometryCalculation(last_scan);
    const Pose3d delta = previous.inverse() * rf2o_ref.getPose();
    const double speed = delta.translation().head<2>().norm() / dt;
    const double turn_rate = std::abs(rf2o::getYaw(delta.rotation())) / dt;
    if (!solved || !rf2o_ref.getPose().matrix().allFinite() ||
        !rf2o_ref.getIncrementCovariance().allFinite() ||
        !std::isfinite(speed) || !std::isfinite(turn_rate) ||
        speed > max_linear_speed || turn_rate > max_angular_speed) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected invalid RF2O solution; rebasing scan pair");
      rf2o_ref.first_laser_scan = true;
      return;
    }
    // Use base-frame displacement, including lateral motion and laser lever arm.
    rf2o_ref.lin_speed = delta.translation().x() / dt;
    rf2o_ref.ang_speed = rf2o::getYaw(delta.rotation()) / dt;
    publish();
    initial_robot_pose.pose.pose.position.x = rf2o_ref.getPose().translation().x();
    initial_robot_pose.pose.pose.position.y = rf2o_ref.getPose().translation().y();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.getPose().rotation()));
    initial_robot_pose.pose.pose.orientation = tf2::toMsg(q);
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
//    RCLCPP_WARN(get_logger(), "Waiting for laser_scans....");
      if (!warning_issued)
      {
          RCLCPP_WARN(get_logger(), "Waiting for laser_scans....");
          warning_issued = true;
      }
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = rf2o_ref.lateral_speed;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  for (int index : {0, 7, 14, 21, 28, 35}) {
    odom.pose.covariance[index] = 1e6;
    odom.twist.covariance[index] = 1e6;
  }
  odom.twist.covariance[0] = linear_velocity_stddev * linear_velocity_stddev;
  odom.twist.covariance[7] = linear_velocity_stddev * linear_velocity_stddev;
  odom.twist.covariance[35] = angular_velocity_stddev * angular_velocity_stddev;
  odom_pub->publish(odom);
  
  // Velocity uncertainty floors are provisional; not a measured accuracy claim.
  // Pose is intentionally not fused by the EKF (it is integrated from this same source).
  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok()){ 
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }

  return 0;
}
