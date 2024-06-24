
#include "elevation_mapping_cupy/elevation_mapping_ros2.hpp"

#include "pybind11/eigen.h"

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/projection_matrix.h>

#include <elevation_map_msgs/msg/statistics.hpp>
#include <chrono>
using namespace std::chrono_literals;


namespace elevation_mapping_cupy {


ElevationMappingNode::ElevationMappingNode() : Node("elevation_mapping_node") {
    double recordableFps, updateVarianceFps, timeInterval, updatePoseFps, updateGridMapFps, publishStatisticsFps;

    //declare parameters
    declare_parameter("point_cloud_topic", "livox/pcl");
    declare_parameter("initialize_frame_id", "base_link");
    declare_parameter("pose_topic", "pose");
    declare_parameter("map_frame", "World");
    declare_parameter("base_frame", "base_link");
    declare_parameter("initialize_method", "linear");
    declare_parameter("position_lowpass_alpha", 0.2);
    declare_parameter("orientation_lowpass_alpha", 0.2);
    declare_parameter("update_variance_fps", 10.0);
    declare_parameter("time_interval", 0.02);
    declare_parameter("update_pose_fps", 50.0);
    declare_parameter("initialize_tf_grid_size", 10.0);
    declare_parameter("map_acquire_fps", 10.0);
    declare_parameter("enable_point_cloud_publishing", false);
    declare_parameter("enable_drift_corrected_tf_publishing", false);
    declare_parameter("use_initializer_at_start", true);
    declare_parameter("enable_visibility_cleanup", false);
    // declare_parameter("initialize_frame_id",def_init_tf );


    RCLCPP_INFO(this->get_logger(), "ElevationMappingNode started.");

    //first create a pose tf2_listener for the robot pose and the required buffer for it
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transformListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    //initilaize the variable can be used with default constructor but I don't trust myself enough
    lowpassPosition_ = Eigen::Vector3d::Zero();
    lowpassOrientation_ = Eigen::Vector4d(0, 0, 0, 1);
    positionError_ = 0.0;
    orientationError_ = 0.0;
    positionAlpha_ = 0.1;
    orientationAlpha_ = 0.1;
    enablePointCloudPublishing_ = false;
    isGridmapUpdated_ = false;

    std::string map_fram;
    std::string pose_topic;
    std::string point_cloud_topic;



    //get parameters


    pose_topic = this->get_parameter("pose_topic").as_string();
    mapFrameId_ = this->get_parameter("map_frame").as_string();
    baseFrameId_ = this->get_parameter("base_frame").as_string();
    initializeMethod_ = this->get_parameter("initialize_method").as_string();
    positionAlpha_ = this->get_parameter("position_lowpass_alpha").as_double();
    orientationAlpha_ = this->get_parameter("orientation_lowpass_alpha").as_double();
    updateVarianceFps= this->get_parameter("update_variance_fps").as_double();
    timeInterval = this->get_parameter("time_interval").as_double();
    updatePoseFps = this->get_parameter("update_pose_fps").as_double();
    initializeTfGridSize_ = this->get_parameter("initialize_tf_grid_size").as_double();
    updateGridMapFps = this->get_parameter("map_acquire_fps").as_double();
    enablePointCloudPublishing_ = this->get_parameter("enable_point_cloud_publishing").as_bool();
    enableDriftCorrectedTFPublishing_ = this->get_parameter("enable_drift_corrected_tf_publishing").as_bool();
    useInitializerAtStart_ = this->get_parameter("use_initializer_at_start").as_bool();
    point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    this->get_parameter("enable_visibility_cleanup").as_bool();
    // initialize_frame_id_ = this->get_parameter("initialize_frame_id").as_string_array();



    //intialize the cupy map wrapper
    RCLCPP_INFO(this->get_logger(), "Initializing map.");
    auto nh = std::make_shared<rclcpp::Node>("elevation_mapping_wrapper");
    nh->declare_parameter("plugin_config_file", "/home/eongan/focus/cupy_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/plugin_config.yaml");
    nh->declare_parameter("weight_file", "/home/eongan/focus/cupy_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/weights.dat");
    nh->declare_parameter("max_height_range", static_cast<float>(1.0));
    nh->declare_parameter("time_interval", static_cast<float>(0.02));
    nh->declare_parameter("resolution", static_cast<float>(0.2));
    nh->declare_parameter("map_legth", static_cast<float>(5.0));
    nh->declare_parameter("sensor_noise_factor", static_cast<float>(0.01));
    nh->declare_parameter("enable_visibility_cleanup", static_cast<bool>(false));
    timeInterval = nh->get_parameter("time_interval").as_double();

    map_.initialize(nh);


    //intialize the pointcloud subscriber
    pointcloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic,rclcpp::SensorDataQoS() ,std::bind(&ElevationMappingNode::pointcloudCallback, this, std::placeholders::_1));

    pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("elevation_map_pointcloud", 1);
    publishers_ = std::make_pair(this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map_raw", 1), std::map<std::string, std::vector<std::string>>());
    publishers_.second["layers"] = {"elevation"};
    publishers_.second["basic_layers"] = {"elevation"};
    
    //initialize the layers and publishers
    
    map_layers_.push_back({"elevation"});
    map_basic_layers_.push_back({"elevation"});

    //create the map publishers timers
    map_layers_all_.insert("elevation");


    //Now we create the timers
    updatePoseTimer_ = this->create_wall_timer(std::chrono::milliseconds(uint(1 / (updatePoseFps + 0.0001) * 1000)), std::bind(&ElevationMappingNode::updatePose, this));
    updateVarianceTimer_ = this->create_wall_timer(std::chrono::milliseconds(uint(1 / (updateVarianceFps + 0.0001) * 1000)), std::bind(&ElevationMappingNode::updateVariance, this));
    updateGridMapTimer_ = this->create_wall_timer(std::chrono::milliseconds(uint(1 / (updateGridMapFps + 0.0001) * 1000)), std::bind(&ElevationMappingNode::updateGridMap, this));
    updateTimeTimer_ = this->create_wall_timer(std::chrono::milliseconds(uint(timeInterval * 1000)), std::bind(&ElevationMappingNode::updateTime, this));
    publishMapTimer_ = this->create_wall_timer(100ms, std::bind(&ElevationMappingNode::publishMapOfIndex, this));

    //Now we create the services can also implemented as a topic
    //Only one service for now, for getting the submap around the robot
    getSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>("get_submap", std::bind(&ElevationMappingNode::getSubmap, this, std::placeholders::_1, std::placeholders::_2));


    RCLCPP_INFO(this->get_logger(), "ElevationMappingNode finish initialization.");
}


void ElevationMappingNode::publishMapOfIndex() {
    int index = 0;
    if(!isGridmapUpdated_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Gridmap not updated yet. not publishing");
        return;
    }
    grid_map_msgs::msg::GridMap msg;
    // pointcloud2 msg;
    sensor_msgs::msg::PointCloud2 msg_pc;
    std::vector<std::string> layers;
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        for (const auto& layer : map_layers_[index]) {
            bool is_layer_in_all = map_layers_all_.find(layer) != map_layers_all_.end();
            if (is_layer_in_all && gridMap_.exists(layer)) {
                layers.push_back(layer);
            } else if (map_.exists_layer(layer)){
                RCLCPP_INFO_STREAM(this->get_logger(), "Layer " << layer << " does not exist in the grid map, but adding.");
                ElevationMappingWrapper::RowMatrixXf map_data;
                map_.get_layer_data(layer, map_data);
                gridMap_.add(layer, map_data);
                layers.push_back(layer);
            }
        }
        if (layers.empty()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "No layers to publish.");
            return;
        }
        msg = * grid_map::GridMapRosConverter::toMessage(gridMap_, layers);
        msg.header.frame_id = mapFrameId_;
        grid_map::GridMapRosConverter::toPointCloud(gridMap_, "elevation", msg_pc);
        msg_pc.header.frame_id = mapFrameId_;
        //  create a pointcloud message
        sensor_msgs::msg::PointCloud2 reduced_pc;
        
        // reduce the number of points published
        for (auto& point : msg_pc.data) {
            // accept only one in 10 points
            if (rand() % 10 == 0) {
                reduced_pc.data.push_back(point);
            }

        }
        reduced_pc.header.frame_id = mapFrameId_;
        reduced_pc.header.stamp = this->now();
    }
    msg.basic_layers = map_basic_layers_[index];
    publishers_.first->publish(msg);
    if( display_pub_count > 20){
        display_pub_count = 0;
        RCLCPP_INFO_STREAM(this->get_logger(), "[elevatiob map]Publishing pointcloud");
        pointCloudPub_->publish(reduced_pc);
    } else {
        display_pub_count++;
    
    }
}


void ElevationMappingNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
    auto fields = cloud.fields;
    std::vector<std::string> channels;
    for (const auto& field : fields) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Channel: " << field.name);
        channels.push_back(field.name);
    }
    inputPointCloud(cloud, channels);
}

//this function inputs a pcl2 cloud to the elevation mapping in the sensor frame
void ElevationMappingNode::inputPointCloud(const sensor_msgs::msg::PointCloud2& cloud, std::vector<std::string>& channels) {
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
    // the frame of the map will be referenced to the initualisation pose of fast_lio

    tf2::Transform transformTf;
    std::string sensorFrameId = "sensor_frame";
    auto timeStamp = cloud.header.stamp;
    Eigen::Affine3d transformationSensorToMap;

    try {
        auto t = tfBuffer_->lookupTransform(mapFrameId_, sensorFrameId, tf2::TimePointZero);
        //convert tf2 to eigen
        auto temp = tf2::transformToEigen(t);
        transformationSensorToMap = temp.affine();
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

    // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Input pointcloud took " << (this->now() - start).seconds() << " seconds.");
    // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 100, "positionError: " << positionError << " orientationError: " << orientationError);
                                                
}


//this function updates the base frame of the robot with the tf2 message
void ElevationMappingNode::updatePose() {
    geometry_msgs::msg::TransformStamped transformStamped;
    const auto& timestamp = this->now();
    Eigen::Affine3d transformationBaseToMap;
    try {
        transformStamped = tfBuffer_->lookupTransform(mapFrameId_, baseFrameId_, tf2::TimePointZero);
        auto temp = tf2::transformToEigen(transformStamped);
        transformationBaseToMap = temp.affine();        
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s",
                     mapFrameId_.c_str(), baseFrameId_.c_str(), ex.what());
        return;
    }

    //This is to check if the robot is moving, in the original code reference to origin have to be checked
    Eigen::Vector3d position(transformStamped.transform.translation.x,
                            transformStamped.transform.translation.y,
                            transformStamped.transform.translation.z);

    map_.move_to(position, transformationBaseToMap.rotation().transpose());
    Eigen::Vector3d position3 = position;
    Eigen::Vector4d orientation(transformStamped.transform.rotation.x,
                                transformStamped.transform.rotation.y,
                                transformStamped.transform.rotation.z,
                                transformStamped.transform.rotation.w);
    lowpassPosition_ = positionAlpha_ * position3 + (1.0 - positionAlpha_) * lowpassPosition_;
    lowpassOrientation_ = orientationAlpha_ * orientation + (1.0 - orientationAlpha_) * lowpassOrientation_;
    {
        std::lock_guard<std::mutex> lock(errorMutex_);
        positionError_ = (position3 - lowpassPosition_).norm();
        orientationError_ = (orientation - lowpassOrientation_).norm();
    }

    if(useInitializerAtStart_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "clearing map with initializer");
        initializeWithTF();
        useInitializerAtStart_ = false;
    }


}



void ElevationMappingNode::initializeWithTF() {
    RCLCPP_INFO(this->get_logger(), "Initializing map with TF.");
    std::vector<Eigen::Vector3d> points;
    const auto& timestamp = this->now();
    int i = 0;
    Eigen::Vector3d p;
    for (const auto& frame_id : initialize_frame_id_) {
        Eigen::Affine3d transformationBaseToMap;
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer_->lookupTransform(mapFrameId_, frame_id, tf2::TimePointZero);
            auto temp = tf2::transformToEigen(transformStamped);
            transformationBaseToMap = temp.affine();               
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s",
                         mapFrameId_.c_str(), frame_id.c_str(), ex.what());
            return;
        }
        p = transformationBaseToMap.translation();
        RCLCPP_INFO_STREAM(this->get_logger(), "wheel_point" << p);
        p.z() += -0.15;
        points.push_back(p);
        i++;
    }
    if(points.size() < 3) {
        points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, initializeTfGridSize_, -0.66));
        points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, initializeTfGridSize_, -0.66));
        points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, -initializeTfGridSize_, -0.66));
        points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, -initializeTfGridSize_, -0.66));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Intializing with points using" << initializeMethod_);
    map_.initializeWithPoints(points, initializeMethod_);
}

void ElevationMappingNode::publishAsPointCloud(const grid_map::GridMap& map) const {
    sensor_msgs::msg::PointCloud2 msg;
    for (const auto& layer : map.getLayers()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Channels available: " << layer);

    }
    grid_map::GridMapRosConverter::toPointCloud(map, "elevation", msg);
    }

bool ElevationMappingNode::getSubmap(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request_ptr, std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response_ptr) {
    auto request = *request_ptr;
    std::string requestedFrameId = request.frame_id;
    Eigen::Isometry3d transformationOdomToMap;
    grid_map::Position requestedSubmapposition(request.position_x, request.position_y);

    //hpefully we always will be making requests in the map frame
    if(requestedFrameId != mapFrameId_) {
        geometry_msgs::msg::TransformStamped transformStamped;
        const auto& timestamp = this->now();
        try {
            transformStamped = tfBuffer_->lookupTransform(requestedFrameId, mapFrameId_, tf2::TimePointZero);
            auto temp = tf2::transformToEigen(transformStamped);
            transformationOdomToMap = temp.affine(); 

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s",
                         mapFrameId_.c_str(), requestedFrameId.c_str(), ex.what());
            return false;
        }
        Eigen::Vector3d p(request.position_x, request.position_y, 0);
        Eigen::Vector3d mapP = transformationOdomToMap.inverse() * p;
        requestedSubmapposition.x() = mapP.x();
        requestedSubmapposition.y() = mapP.y();
    }

    grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
    RCLCPP_INFO_STREAM(this->get_logger(), "Requested submap at position " << requestedSubmapposition.transpose() << " with length " << requestedSubmapLength.transpose() << " in frame " << requestedFrameId << ".");
    bool isSuccess = false;
    grid_map::Index index;
    grid_map::GridMap submap;
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        //the new gridmap does not have indicies
        submap = gridMap_.getSubmap(requestedSubmapposition, requestedSubmapLength, isSuccess);
    }
    const auto& length = submap.getLength();
    if(requestedFrameId != mapFrameId_) {
       RCLCPP_INFO_STREAM(this->get_logger(), "Transforming submap to frame " << requestedFrameId << ".");
       submap = submap.getTransformedMap(transformationOdomToMap, "elevation",requestedFrameId);
    }
    if (request.layers.empty()) {
        //it uses a newfangled allacator have to look closely for now not working
        auto created_msg = grid_map::GridMapRosConverter::toMessage(submap);
        response_ptr->map = *created_msg;
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "NOT IMPLEMETED YOU ARE FUCKED Requested layers: ");
        return false;
    }
    return isSuccess;
}

void ElevationMappingNode::updateGridMap() {
    std::vector<std::string> layers(map_layers_all_.begin(), map_layers_all_.end());
    std::lock_guard<std::mutex> lock(mapMutex_);
    map_.get_grid_map(gridMap_, layers);
    gridMap_.setTimestamp(this->now().nanoseconds());

    if (enablePointCloudPublishing_) {
        publishAsPointCloud(gridMap_);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "Grifmap updated hopefully");
    isGridmapUpdated_ = true;
}

bool ElevationMappingNode::initializeMap(elevation_map_msgs::srv::Initialize::Request& request, elevation_map_msgs::srv::Initialize::Response& response) {
    if (request.type == request.POINTS) {
        std::vector<Eigen::Vector3d> points;
        for(const auto& point : request.points) {
            const auto& pointFrameId = point.header.frame_id;
            const auto& timeStamp = point.header.stamp;
            const auto& pvector = Eigen::Vector3d(point.point.x, point.point.y, point.point.z);
            if(mapFrameId_ != pointFrameId) {
                geometry_msgs::msg::TransformStamped transformStamped;
                Eigen::Affine3d transformationBaseToMap;
                try {
                    transformStamped = tfBuffer_->lookupTransform(mapFrameId_, pointFrameId, tf2::TimePointZero);
                    auto temp = tf2::transformToEigen(transformStamped);
                    transformationBaseToMap = temp.affine(); 
                
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s",
                                 mapFrameId_.c_str(), pointFrameId.c_str(), ex.what());
                    return false;
                }
                const auto transformed_p = transformationBaseToMap * pvector;
                points.push_back(transformed_p);
            } else {
                points.push_back(pvector);
            }

        }
        std::string method;
        switch (request.method) {
        case request.NEAREST:
            method = "nearest";
            break;
        case request.LINEAR:
            method = "linear";
            break;
        case request.CUBIC:
            method = "cubic";
            break;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Intializing with points using" << method);
        map_.initializeWithPoints(points, method);
    }
    response.success = true;
    return true;
}


void ElevationMappingNode::updateVariance() {
    map_.update_variance();
    // RCLCPP_INFO_STREAM(this->get_logger(), "Variance updated.");

}

void ElevationMappingNode::updateTime() {
    map_.update_time();
    // RCLCPP_INFO_STREAM(this->get_logger(), "Time updated.");
}

}  // namespace elevation_mapping_cupy