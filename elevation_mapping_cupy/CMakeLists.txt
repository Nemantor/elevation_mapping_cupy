# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.8)
project(elevation_mapping_cupy)


find_package(ament_cmake REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(elevation_map_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)


if(PYTHONLIBS_FOUND)
  message(STATUS "Using Python Libraries at: " ${PYTHON_LIBRARIES})
  message(STATUS "Using Python include directories at: " ${PYTHON_INCLUDE_DIRS})
else()
  message(WARNING "Could not find Python Libraries")
endif()





include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(elevation_mapping_ros
    src/elevation_mapping_wrapper.cpp
    src/elevation_mapping_ros.cpp)

ament_target_dependencies(elevation_mapping_ros
  "rclcpp"
  "rclpy"
  "tf2"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
  "elevation_map_msgs"
  "grid_map_msgs"
  "grid_map_ros"
  "image_transport"
  "pcl_ros"
  "pybind11"
  "Eigen3"
  "OpenCV"
  "std_srvs"
)

target_link_libraries(elevation_mapping_ros ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(elevation_mapping_node src/elevation_mapping_node.cpp)
# target_link_libraries(elevation_mapping_node elevation_mapping_ros)


install(
  TARGETS elevation_mapping_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

  ament_package()