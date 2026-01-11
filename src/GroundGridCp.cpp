/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <groundgrid/GroundGridNodeCp.hpp>

namespace groundgrid
{
GroundGridNodeCp::GroundGridNodeCp()
: Node("groundgrid_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Set parameters
  load_parameters();
  grid_->setConfig(config_);
  segmentation_.setConfig(config_);
  segmentation_.init(grid_->mDimension, grid_->mResolution);

  auto qos = rclcpp::SensorDataQoS();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odometry/filtered_map", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr msg) { map_ptr_ = grid_->update(msg); });

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensors/velodyne_points", qos,
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { handle_cloud(msg); });

  cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/groundgrid/segmented_cloud", rclcpp::QoS(10));
  gridmap_pub_ =
    create_publisher<grid_map_msgs::msg::GridMap>("/groundgrid/grid_map", rclcpp::QoS(1));
}

void GroundGridNodeCp::load_parameters()
{
  config_->point_count_cell_variance_threshold =
    declare_parameter<int>("point_count_cell_variance_threshold", 10);
  config_->max_ring = declare_parameter<int>("max_ring", 1024);
  config_->groundpatch_detection_minimum_threshold =
    declare_parameter<double>("groundpatch_detection_minimum_threshold", 0.01);
  config_->distance_factor = declare_parameter<double>("distance_factor", 0.0001);
  config_->minimum_distance_factor = declare_parameter<double>("minimum_distance_factor", 0.0005);
  config_->miminum_point_height_threshold =
    declare_parameter<double>("miminum_point_height_threshold", 0.3);
  config_->minimum_point_height_obstacle_threshold =
    declare_parameter<double>("minimum_point_height_obstacle_threshold", 0.1);
  config_->outlier_tolerance = declare_parameter<double>("outlier_tolerance", 0.1);
  config_->ground_patch_detection_minimum_point_count_threshold =
    declare_parameter<double>("ground_patch_detection_minimum_point_count_threshold", 0.25);
  config_->patch_size_change_distance =
    declare_parameter<double>("patch_size_change_distance", 20.0);
  config_->occupied_cells_decrease_factor =
    declare_parameter<double>("occupied_cells_decrease_factor", 5.0);
  config_->occupied_cells_point_count_factor =
    declare_parameter<double>("occupied_cells_point_count_factor", 20.0);
  config_->min_outlier_detection_ground_confidence =
    declare_parameter<double>("min_outlier_detection_ground_confidence", 1.25);
  config_->thread_count = declare_parameter<int>("thread_count", 8);
}

void GroundGridNodeCp::handle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Map not initialized yet, this means the node hasn't received any odom message so far.
  if (!map_ptr_) return;
  // TODO(HTR): create parameter for global frame

  pcl::PointCloud<PCLPoint>::Ptr cloud(new pcl::PointCloud<PCLPoint>);
  pcl::fromROSMsg(*msg, *cloud);

  geometry_msgs::msg::TransformStamped map2base_tf, cloud2origin_tf;
  try {
    map2base_tf =
      tf_buffer_.lookupTransform("map", "base_link", msg->header.stamp, tf2::durationFromSec(0.0));
    cloud2origin_tf = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "Received point cloud but transforms are not available: %s", ex.what());
    return;
  }

  geometry_msgs::msg::PointStamped origin;
  origin.header = msg->header;
  origin.header.frame_id = msg->header.frame_id;
  origin.point.x = origin.point.y = origin.point.z = 0.0;
  tf2::doTransform(origin, origin, cloud2origin_tf);

  // transform cloud to map frame if needed
  if (msg->header.frame_id != "map") {
    geometry_msgs::msg::TransformStamped map2cloud_tf;
    pcl::PointCloud<PCLPoint>::Ptr transformed(new pcl::PointCloud<PCLPoint>);
    transformed->points.reserve(cloud->points.size());
    try {
      tf_buffer_.canTransform(
        "map", msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.0));
      map2cloud_tf = tf_buffer_
                       .lookupTransform(
                         "map", msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.0))
                       .transform.translation;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to get map transform for point cloud transformation: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PointStamped ps_in;
    ps_in.header = msg->header;
    ps_in.header.frame_id = "map";

    for (const auto & p : cloud->points) {
      ps_in.point.x = p.x;
      ps_in.point.y = p.y;
      ps_in.point.z = p.z;
      tf2::doTransform(ps_in, ps_in, map2cloud_tf);
      PCLPoint & tp = transformed->points.emplace_back(p);
      tp.x = ps_in.point.x;
      tp.y = ps_in.point.y;
      tp.z = ps_in.point.z;
    }
    cloud = transformed;
  }

  PCLPoint origin_point;
  origin_point.x = origin.point.x;
  origin_point.y = origin.point.y;
  origin_point.z = origin.point.z;

  sensor_msgs::msg::PointCloud2 point_cloud_out;
  pcl::toROSMsg(
    *segmentation_.filter_cloud(cloud, origin_point, map2base_tf, *map_ptr_), point_cloud_out);

  point_cloud_out.header = msg->header;
  point_cloud_out.header.frame_id = "map";
  cloud_pub_->publish(point_cloud_out);

  grid_map_msgs::msg::GridMap grid_msg;
  grid_map::GridMapRosConverter::toMessage(*map_ptr_, grid_msg);
  grid_msg.info.header.stamp = msg->header.stamp;
  gridmap_pub_->publish(grid_msg);
}

}  // namespace groundgrid