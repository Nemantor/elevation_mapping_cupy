#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>    
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("display_submap_node");

  // create the service client
  rclcpp::Client<grid_map_msgs::srv::GetGridMap>::SharedPtr client =
  node->create_client<grid_map_msgs::srv::GetGridMap>("get_submap");

  //create a submap publisher
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr submap_publisher;
  submap_publisher = node->create_publisher<grid_map_msgs::msg::GridMap>("submap_livox_sensor", 10);

  // create a tf buffer and tf listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  while(true) {

    rclcpp::sleep_for(100ms);

  auto request = std::make_shared<grid_map_msgs::srv::GetGridMap::Request>();

  std::string requestedFrameId = "livox_sensor";
  std::string mapFrameId_ = "camera_init";
  geometry_msgs::msg::TransformStamped transformStamped;
  const auto& timestamp = node->now();
  try {
      transformStamped = tfBuffer_->lookupTransform(mapFrameId_, requestedFrameId, tf2::TimePointZero);

  } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(node->get_logger(), "Could not get transform from %s to %s: %s",
                      mapFrameId_.c_str(), requestedFrameId.c_str(), ex.what());
  }

  double robot_x = transformStamped.transform.translation.x;
  double robot_y = transformStamped.transform.translation.y;

  request->frame_id = "camera_init";
  request->length_x = 2.0;
  request->length_y = 2.0;
  request->position_x = robot_x;
  request->position_y = robot_y;



  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Submap received");
    std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response_ptr = result.get();
    auto pose = response_ptr->map.info.pose;
    auto length_x = response_ptr->map.info.length_x;
    auto length_y = response_ptr->map.info.length_y;
    auto position_x = response_ptr->map.info.pose.position.x;
    auto position_y = response_ptr->map.info.pose.position.y;
    if (response_ptr->map.layers.size() == 0) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No layers in the submap");
      continue;
    } else {
      auto layer_name = response_ptr->map.layers[0];
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Submap layer name: [%s]", layer_name);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Submap length: [%f, %f]", length_x, length_y);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Submap position: [%f, %f]", position_x, position_y);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Submap pose: [%f, %f, %f]", pose.position.x, pose.position.y, pose.position.z);
      response_ptr->map.header.frame_id = "camera_init";
      submap_publisher->publish(response_ptr->map);

    }



    // // obtained submap is in livox_sensor frame we have transformed it to camera_init frame for visualization
    // grid_map::GridMap submap;
    // grid_map::GridMapRosConverter::fromMessage(response_ptr->map, submap);

    // // create the required eigen isometry transformation
    // Eigen::Isometry3d transformationOdomToMap;
    // std::string requestedFrameId = "livox_sensor";
    // std::string mapFrameId_ = "camera_init";




    // auto submap_converted = submap.getTransformedMap(transformationOdomToMap, "elevation", mapFrameId_,2.0);
    // std::unique_ptr<grid_map_msgs::msg::GridMap> submap_msg;
    // submap_msg = grid_map::GridMapRosConverter::toMessage(submap_converted);



  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call service get_submap");
  }
  }

  rclcpp::shutdown();
  return 0;
}