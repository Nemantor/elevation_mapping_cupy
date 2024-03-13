//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

// STL
#include <iostream>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

// ROS
#include <geometry_msgs/msg/polygon_stamped.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



// Grid Map
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <elevation_map_msgs/srv/check_safety.hpp>
#include <elevation_map_msgs/srv/initialize.hpp>
#include <elevation_map_msgs/msg/channel_info.hpp>

#include "elevation_mapping_cupy/elevation_mapping_wrapper_ros2.hpp"

namespace py = pybind11;

namespace elevation_mapping_cupy {

class ElevationMappingNode : public rclcpp::Node{
 public:
  ElevationMappingNode();
  using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using ColMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;


  // Subscriber and Synchronizer for Pointcloud messages
  using PointCloudSubscriber = message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;
  using PointCloudSubscriberPtr = std::shared_ptr<PointCloudSubscriber>;
  using PointCloudPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, elevation_map_msgs::msg::ChannelInfo>;
  using PointCloudSync = message_filters::Synchronizer<PointCloudPolicy>;
  using PointCloudSyncPtr = std::shared_ptr<PointCloudSync>;


  

 private:
  void readParameters();
  void setupMapPublishers();
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2& cloud);
  void inputPointCloud(const sensor_msgs::msg::PointCloud2& cloud, std::vector<std::string>& channels);


  void publishAsPointCloud(const grid_map::GridMap& map) const;
  bool getSubmap(grid_map_msgs::srv::GetGridMap::Request& request, grid_map_msgs::srv::GetGridMap::Response& response);
  bool checkSafety(elevation_map_msgs::srv::CheckSafety::Request& request, elevation_map_msgs::srv::CheckSafety::Response& response);
  bool initializeMap(elevation_map_msgs::srv::Initialize::Request& request, elevation_map_msgs::srv::Initialize::Response& response);
  bool clearMap(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response);
  bool clearMapWithInitializer(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response);
  bool setPublishPoint(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response);
  void updatePose();
  void updateVariance();
  void updateTime();
  void updateGridMap();
  void publishNormalAsArrow(const grid_map::GridMap& map) const;
  void initializeWithTF();
  void publishMapToOdom(double error);
  void publishStatistics();
  void publishMapOfIndex();

  visualization_msgs::msg::Marker vectorToArrowMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const int id) const;

  // define the subscriber handles
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudSub_;
  // define the timer handles
  rclcpp::TimerBase::SharedPtr updatePoseTimer_;
  rclcpp::TimerBase::SharedPtr updateVarianceTimer_;
  rclcpp::TimerBase::SharedPtr updateTimeTimer_;
  rclcpp::TimerBase::SharedPtr updateGridMapTimer_;
  rclcpp::TimerBase::SharedPtr publishMapTimer_;

  // define the publishers construct
  std::pair<rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr, std::map<std::string, std::vector<std::string>>> publishers_;
  // define a simple publisher


  std::vector<PointCloudSyncPtr> pointCloudSyncs_;
  std::vector<rclcpp::Publisher<grid_map_msgs::msg::GridMap>> mapPubs_;
  std::shared_ptr<tf2_ros::TransformListener> transformListener_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  ElevationMappingWrapper map_;
  std::string mapFrameId_;
  std::string correctedMapFrameId_;
  std::string baseFrameId_;

  // map topics info
  std::vector<std::vector<std::string>> map_topics_;
  std::vector<std::vector<std::string>> map_layers_;
  std::vector<std::vector<std::string>> map_basic_layers_;
  std::set<std::string> map_layers_all_;
  std::set<std::string> map_layers_sync_;
  std::vector<double> map_fps_;
  std::set<double> map_fps_unique_;
  std::map<std::string, std::vector<std::string>> channels_;

  std::vector<std::string> initialize_frame_id_;
  std::vector<double> initialize_tf_offset_;
  std::string initializeMethod_;

  Eigen::Vector3d lowpassPosition_;
  Eigen::Vector4d lowpassOrientation_;

  std::mutex mapMutex_;  // protects gridMap_
  grid_map::GridMap gridMap_;
  std::atomic_bool isGridmapUpdated_;  // needs to be atomic (read is not protected by mapMutex_)

  std::mutex errorMutex_;  // protects positionError_, and orientationError_
  double positionError_;
  double orientationError_;

  double positionAlpha_;
  double orientationAlpha_;


  std::atomic_bool enablePointCloudPublishing_;
  bool enableNormalArrowPublishing_;
  bool enableDriftCorrectedTFPublishing_;
  bool useInitializerAtStart_;
  double initializeTfGridSize_;
  std::atomic_int pointCloudProcessCounter_;
};

}  // namespace elevation_mapping_cupy
