#include "gicp_localization/gicp_localization_node.hpp"
#include "gicp_localization/pose_math.hpp"
#include "gicp_localization/prior_map.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace gicp_localization {
namespace {
std::string expandUser(const std::string& path) {
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) return std::string(home) + path.substr(1);
  }
  return path;
}
}  // namespace

GicpLocalizationNode::GicpLocalizationNode() : rclcpp::Node("gicp_localization") {
  // ---- 参数 ----
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
  base_frame_ = declare_parameter<std::string>("base_frame", "body");
  const std::string cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
  const std::string prior_map_path = expandUser(
      declare_parameter<std::string>("prior_map_path", "~/result/GlobalMap.pcd"));
  fitness_threshold_ = declare_parameter<double>("fitness_threshold", 0.8);
  min_scan_points_ = declare_parameter<int>("min_scan_points", 100);
  const double localization_freq = declare_parameter<double>("localization_freq", 0.5);
  const double tf_pub_freq = declare_parameter<double>("tf_pub_freq", 50.0);

  GicpParams gp;
  gp.map_voxel_size = declare_parameter<double>("map_voxel_size", 0.4);
  gp.scan_voxel_size = declare_parameter<double>("scan_voxel_size", 0.1);
  gp.max_corr_dist = declare_parameter<double>("gicp_max_corr_dist", 1.0);
  gp.num_neighbors = declare_parameter<int>("gicp_num_neighbors", 20);
  gp.num_threads = declare_parameter<int>("gicp_num_threads", 4);
  gp.max_iterations = declare_parameter<int>("gicp_max_iterations", 20);

  const auto init = declare_parameter<std::vector<double>>(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  if (init.size() == 6) {
    T_map_odom_ = poseParamToIsometry(init[0], init[1], init[2], init[3], init[4], init[5]);
  }

  // ---- 加载先验地图（失败致命）----
  prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/prior_map", rclcpp::QoS(1).transient_local());
  aligner_ = std::make_unique<GicpAligner>(gp);
  try {
    auto pts = loadPriorMapPcd(prior_map_path);
    aligner_->setMap(pts);
    RCLCPP_INFO(get_logger(), "已加载先验地图 %s (%zu 点)", prior_map_path.c_str(), pts.size());
    // 发布先验地图(latched, map 系)供 RViz 叠加查看
    pcl::PointCloud<pcl::PointXYZ> mapcloud;
    mapcloud.reserve(pts.size());
    for (const auto& p : pts) mapcloud.emplace_back(p.x(), p.y(), p.z());
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(mapcloud, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = now();
    prior_map_pub_->publish(map_msg);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "先验地图加载失败：%s", e.what());
    throw;
  }

  diag_ = make_diagnostics(this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ---- callback group ----
  slow_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fast_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions fast_opts;
  fast_opts.callback_group = fast_group_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&GicpLocalizationNode::cloudCb, this, std::placeholders::_1), fast_opts);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      std::bind(&GicpLocalizationNode::odomCb, this, std::placeholders::_1), fast_opts);
  init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      std::bind(&GicpLocalizationNode::initialPoseCb, this, std::placeholders::_1), fast_opts);

  loc_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization", rclcpp::QoS(50));

  gicp_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / localization_freq),
      std::bind(&GicpLocalizationNode::gicpTimerCb, this), slow_group_);
  tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tf_pub_freq),
      std::bind(&GicpLocalizationNode::tfTimerCb, this), fast_group_);
}

void GicpLocalizationNode::cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  std::vector<Eigen::Vector4f> pts;
  pts.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    // 丢弃非有限点(同 loadPriorMapPcd):防 small_gicp voxelgrid_sampling 越界刷屏。
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    pts.emplace_back(p.x, p.y, p.z, 1.0f);
  }
  std::lock_guard<std::mutex> lk(mtx_);
  latest_scan_ = std::move(pts);
}

void GicpLocalizationNode::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Isometry3d T;
  tf2::fromMsg(msg->pose.pose, T);
  std::lock_guard<std::mutex> lk(mtx_);
  latest_odom_ = T;
}

void GicpLocalizationNode::initialPoseCb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  Eigen::Isometry3d T_map_base;
  tf2::fromMsg(msg->pose.pose, T_map_base);
  std::lock_guard<std::mutex> lk(mtx_);
  pending_init_base_ = T_map_base;
  RCLCPP_INFO(get_logger(), "收到 /initialpose，下一周期强制重配");
}

void GicpLocalizationNode::gicpTimerCb() {
  if (!aligner_->hasMap()) return;

  std::vector<Eigen::Vector4f> scan;
  Eigen::Isometry3d seed;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!latest_scan_) return;
    scan = *latest_scan_;
    if (pending_init_base_) {
      Eigen::Isometry3d odom = latest_odom_.value_or(Eigen::Isometry3d::Identity());
      seed = mapToOdomFromBase(*pending_init_base_, odom);
      pending_init_base_.reset();
    } else {
      seed = T_map_odom_;
    }
  }

  if (static_cast<int>(scan.size()) < min_scan_points_) {
    RCLCPP_WARN(get_logger(), "扫描点过少(%zu)，跳过本周期", scan.size());
    return;
  }

  AlignOutcome out = aligner_->align(scan, seed);
  const bool ok = accept(out.fitness, fitness_threshold_);

  GicpMetrics m;
  m.fitness = out.fitness;
  m.mean_residual = out.mean_residual;
  m.num_inliers = out.num_inliers;
  m.num_source = out.num_source;
  m.iterations = out.iterations;
  m.converged = out.converged;
  m.accepted = ok;
  std::size_t rejected_snapshot = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto d = correctionDelta(T_map_odom_, out.T_map_odom);
    m.correction_delta_trans = d.translation;
    m.correction_delta_rot = d.rotation;
    if (ok) {
      T_map_odom_ = out.T_map_odom;
    } else {
      rejected_snapshot = ++rejected_;
    }
  }
  if (!ok) {
    RCLCPP_WARN(get_logger(), "配准被拒 fitness=%.3f converged=%d (累计 %zu)",
                out.fitness, out.converged, rejected_snapshot);
  }
  diag_->report(m);
}

void GicpLocalizationNode::tfTimerCb() {
  Eigen::Isometry3d T_mo;
  std::optional<Eigen::Isometry3d> odom;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    T_mo = T_map_odom_;
    odom = latest_odom_;
  }
  const auto stamp = now();

  geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T_mo);
  tf.header.stamp = stamp;
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = odom_frame_;
  tf_broadcaster_->sendTransform(tf);

  if (odom) {
    Eigen::Isometry3d T_mb = composeMapToBase(T_mo, *odom);
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = map_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose = tf2::toMsg(T_mb);
    loc_pub_->publish(o);
  }
}

}  // namespace gicp_localization
