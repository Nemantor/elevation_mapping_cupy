
#include "elevation_mapping_cupy/elevation_mapping_ros2.hpp"

#include "pybind11/eigen.h"

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/projection_matrix.h>

#include <elevation_map_msgs/msg/statistics.hpp>


namespace elevation_mapping_cupy {




void ElevationMappingNode::inputPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                                            const std::vector<std::string>& channels) {

    auto start = this->now();
    auto* pcl_pc = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2ConstPtr cloudPtr(pcl_pc);
    pcl_conversions::toPCL(cloud, *pcl_pc);

    //get channels
    auto fields = cloud.fields;
    uint array_dim = channels.size();

    RowMatrixXd points = RowMatrixXd(pcl_pc->width * pcl_pc->height, array_dim);

    for (unsigned int i = 0; i < pcl_pc->width * pcl_pc->height; ++i) {
        for (unsigned int j = 0; j < channels.size(); ++j) {
        float temp;
        uint point_idx = i * pcl_pc->point_step + pcl_pc->fields[j].offset;
        memcpy(&temp, &pcl_pc->data[point_idx], sizeof(float));
        points(i, j) = static_cast<double>(temp);
        }
    }

    // get pose of sensor in map frame
    tf2::Transform transformTf;
    std::string sensorFrameId = cloud.header.frame_id;
    auto timeStamp = cloud.header.stamp;
    Eigen::Affine3d transformationSensorToMap;

    try {
        // implement here the transorm between the lidar and the map
        //
        //
    } catch(tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s",
                     mapFrameId_.c_str(), sensorFrameId.c_str(), ex.what());
        return;
    }

    double positionError = 0.0;
    double orientationError = 0.0;
    {
        std::lock_guard<std::mutex> lock(errorMutex_);
        positionError = positionError_;
        orientationError = orientationError_;
    }

    map_.input(points, channels, transformationSensorToMap.rotation(), transformationSensorToMap.translation(),
                positionError, orientationError);

    if(enableDriftCorrectedTFPublishing_) {
        publishMapToOdom(map_.get_additive_mean_error());
    }
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Input pointcloud took " << (this->now() - start).seconds() << " seconds.");
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 100, "positionError: " << positionError << " orientationError: " << orientationError);
                                                
}










}  // namespace elevation_mapping_cupy