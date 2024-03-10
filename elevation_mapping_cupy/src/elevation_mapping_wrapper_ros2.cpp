#include "elevation_mapping_cupy/elevation_mapping_wrapper_ros2.hpp"

// Pybind
#include <pybind11/eigen.h>

// PCL
#include <pcl/common/projection_matrix.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <utility>


namespace elevation_mapping_cupy {

ElevationMappingWrapper::ElevationMappingWrapper() {}


// this function creates the handles with the python library
void ElevationMappingWrapper::initialize(rclcpp::Node::SharedPtr nh) {

    auto threading = py::module::import("threading");
    py::gil_scoped_acquire acquire;

    auto sys = py::module::import("sys");
    auto path = sys.attr("path");
    std::string module_path = ament_index_cpp::get_package_share_directory("elevation_mapping_cupy");
    module_path += "/script";
    path.attr("insert")(0, module_path);

    auto elevation_mapping = py::module::import("elevation_mapping_cupy.elevation_mapping");
    auto parameter = py::module::import("elevation_mapping_cupy.parameter");
    param_ = parameter.attr("Parameter")();
    setParameters(nh);
    map_ = elevation_mapping.attr("ElevationMapping")(param_);
}

// this function sets the parameters for the elevation mapping in python
void ElevationMappingWrapper::setParameters(rclcpp::Node::SharedPtr nh) {
    py::list paramNames = param_.attr("get_names")();
    py::list paramTypes = param_.attr("get_types")();
    py::gil_scoped_acquire acquire;

    for(int i = 0; i < paramNames.size(); i++) {
        std::string type = py::cast<std::string>(paramTypes[i]);
        std::string name = py::cast<std::string>(paramNames[i]);
        if (type == "float") {
            float param;
            if (nh->get_parameter(name, param)) {
                param_.attr("set_value")(name, param);
            }
            } else if (type == "str") {
            std::string param;
            if (nh->get_parameter(name, param)) {
                param_.attr("set_value")(name, param);
            }
            } else if (type == "bool") {
            bool param;
            if (nh->get_parameter(name, param)) {
                param_.attr("set_value")(name, param);
            }
            } else if (type == "int") {
            int param;
            if (nh->get_parameter(name, param)) {
                param_.attr("set_value")(name, param);
            }
            }

    }
    //publishers
    // name = "elevation map raw"
    // layer = {'elevation', 'variance'}
    // basic_layers = {'elevation'}
    // fps = 5.0

    //subscribers
    // name = "livoxMID360"
    // topic_name = "livox/pcl"
    // data_type = "pointcloud"


    // subscribers
    const char* const name = "livoxMID360";
    py::dict sub_dict;
    sub_dict[name] = py::dict();
    sub_dict[name]["topic_name"] = "livox/pcl";
    sub_dict[name]["data_type"] = "pointcloud";
    param_.attr("subscriber_cfg") = sub_dict;


    RCLCPP_INFO_STREAM(nh->get_logger(), "No pointcloud_channel_fusions parameter found. Using default values");
    RCLCPP_INFO_STREAM(nh->get_logger(), "No image_channel_fusions parameter found. Using default values.");

    param_.attr("update")();
    resolution_ = py::cast<float>(param_.attr("get_value")("resolution"));
    map_length_ = py::cast<float>(param_.attr("get_value")("true_map_length"));
    map_n_ = py::cast<int>(param_.attr("get_value")("true_cell_n"));

    // no idea what these are for we wil see
    enable_normal_color_ = false;
    enable_normal_ = false;      


}


void ElevationMappingWrapper::input(const RowMatrixXd& points, const std::vector<std::string>& channels, const RowMatrixXd& R,
                                    const Eigen::VectorXd& t, const double positionNoise, const double orientationNoise) {
  py::gil_scoped_acquire acquire;
  map_.attr("input_pointcloud")(Eigen::Ref<const RowMatrixXd>(points), channels, Eigen::Ref<const RowMatrixXd>(R),
                     Eigen::Ref<const Eigen::VectorXd>(t), positionNoise, orientationNoise);
}


void ElevationMappingWrapper::move_to(const Eigen::VectorXd& p, const RowMatrixXd& R) {
  py::gil_scoped_acquire acquire;
  map_.attr("move_to")(Eigen::Ref<const Eigen::VectorXd>(p), Eigen::Ref<const RowMatrixXd>(R));
}

void ElevationMappingWrapper::clear() {
  py::gil_scoped_acquire acquire;
  map_.attr("clear")();
}

double ElevationMappingWrapper::get_additive_mean_error() {
  py::gil_scoped_acquire acquire;
  return map_.attr("get_additive_mean_error")().cast<double>();
}


bool ElevationMappingWrapper::exists_layer(const std::string& layerName) {
  py::gil_scoped_acquire acquire;
  return py::cast<bool>(map_.attr("exists_layer")(layerName));
}

void ElevationMappingWrapper::get_layer_data(const std::string& layerName, RowMatrixXf& map) {
  py::gil_scoped_acquire acquire;
  map = RowMatrixXf(map_n_, map_n_);
  map_.attr("get_map_with_name_ref")(layerName, Eigen::Ref<RowMatrixXf>(map));
}

void ElevationMappingWrapper::get_grid_map(grid_map::GridMap& gridMap, const std::vector<std::string>& requestLayerNames) {

    std::vector<std::string> basicLayerNames;
    std::vector<std::string> layerNames = requestLayerNames;
    std::vector<int> selection;

    for (const auto& layerName : layerNames) {
        if(layerName == "elevation") {
            basicLayerNames.push_back("elevation");
        }
    }

    RowMatrixXd pos(1, 3);
    py::gil_scoped_acquire acquire;
    map_.attr("get_position")(Eigen::Ref<RowMatrixXd>(pos));
    grid_map::Position position(pos(0, 0), pos(0, 1));
    grid_map::Length length(map_length_, map_length_);
    gridMap.setGeometry(length, resolution_, position);
    std::vector<Eigen::MatrixXf> maps;

    for (const auto& layerName : layerNames) {
        bool exists = map_.attr("exists_layer")(layerName).cast<bool>();
        if (exists) {
        RowMatrixXf map(map_n_, map_n_);
        map_.attr("get_map_with_name_ref")(layerName, Eigen::Ref<RowMatrixXf>(map));
        gridMap.add(layerName, map);
        }
    }
    if (enable_normal_color_) {
        RowMatrixXf normal_x(map_n_, map_n_);
        RowMatrixXf normal_y(map_n_, map_n_);
        RowMatrixXf normal_z(map_n_, map_n_);
        map_.attr("get_normal_ref")(Eigen::Ref<RowMatrixXf>(normal_x), Eigen::Ref<RowMatrixXf>(normal_y), Eigen::Ref<RowMatrixXf>(normal_z));
        gridMap.add("normal_x", normal_x);
        gridMap.add("normal_y", normal_y);
        gridMap.add("normal_z", normal_z);
    }
    gridMap.setBasicLayers(basicLayerNames);
    if (enable_normal_color_) {
        addNormalColorLayer(gridMap);

    }
}


void ElevationMappingWrapper::initializeWithPoints(std::vector<Eigen::Vector3d>& points, std::string method) {
  RowMatrixXd points_m(points.size(), 3);
  int i = 0;
  for (auto& p : points) {
    points_m(i, 0) = p.x();
    points_m(i, 1) = p.y();
    points_m(i, 2) = p.z();
    i++;
  }
  py::gil_scoped_acquire acquire;
  map_.attr("initialize_map")(Eigen::Ref<const RowMatrixXd>(points_m), method);
}

void ElevationMappingWrapper::addNormalColorLayer(grid_map::GridMap& map) {
  const auto& normalX = map["normal_x"];
  const auto& normalY = map["normal_y"];
  const auto& normalZ = map["normal_z"];

  map.add("color");
  auto& color = map["color"];

  // X: -1 to +1 : Red: 0 to 255
  // Y: -1 to +1 : Green: 0 to 255
  // Z:  0 to  1 : Blue: 128 to 255

  // For each cell in map.
  for (size_t i = 0; i < color.size(); ++i) {
    const Eigen::Vector3f colorVector((normalX(i) + 1.0) / 2.0, (normalY(i) + 1.0) / 2.0, (normalZ(i)));
    Eigen::Vector3i intColorVector = (colorVector * 255.0).cast<int>();
    grid_map::colorVectorToValue(intColorVector, color(i));
  }
}

void ElevationMappingWrapper::update_variance() {
  py::gil_scoped_acquire acquire;
  map_.attr("update_variance")();
}

void ElevationMappingWrapper::update_time() {
  py::gil_scoped_acquire acquire;
  map_.attr("update_time")();
}

}  // namespace elevation_mapping_cupy