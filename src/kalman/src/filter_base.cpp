#include "ros/ros.h"
//#include <Eigen/Dense>
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/Quaternion.h"
//#include "tf2_ros/buffer.h"
//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/transform_broadcaster.h"
#include "kalman/filter_base.hpp"
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/node/node.h"


FilterBase::FilterBase()
{
    // Reading params to setup filter and sensor properties
    FilterBase::ReadParams();
    return;
};

void FilterBase::ReadParams(){
    
    YAML::Node paramList = YAML::LoadFile("/home/darshit/slam-workspace/Code/robot-navigation/src/kalman/config/filter_params.yaml");
    // Extract the sensor list first
    m_sensorList = paramList["sensor_list"].as<std::vector<std::string>>();
    
    // Load all the sensor properties into one YAML node.
    auto sensor_properties_list = paramList["sensor_properties"];
    
    for(std::string sensor_iterator : m_sensorList)
    {
        auto sensor_properties = sensor_properties_list[sensor_iterator];
        sensor sensorInstance;
        sensorInstance.sensorName = sensor_iterator;
        sensorInstance.sensorInputVector = sensor_properties["measurements"].as<std::vector<bool>>();
        
        std::vector<double> vectorizedMeasurementNoise;
        vectorizedMeasurementNoise = sensor_properties["measurement_covariance"].as<std::vector<double>>(); 
        sensorInstance.measurementNoiseMatrix.resize(15,15);
        sensorInstance.measurementNoiseMatrix = Eigen::Map<Eigen::Matrix<double,15,15>>(vectorizedMeasurementNoise.data());  
    };

    
};

FilterBase::~FilterBase()
{
    
};

int main(){
    std::cout<<"Starting filter base"<<std::endl;
    FilterBase filterBase;
}
