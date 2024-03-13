//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "elevation_mapping_cupy/elevation_mapping_ros2.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);


  py::scoped_interpreter guard{};  // start the interpreter and keep it alive
  auto mapNode = std::make_shared<elevation_mapping_cupy::ElevationMappingNode>();

  // Spin
  rclcpp::spin(mapNode);
  rclcpp::shutdown();
  py::gil_scoped_release release;

  return 0;
}