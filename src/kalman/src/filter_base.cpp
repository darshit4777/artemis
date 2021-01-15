#include "ros/ros.h"
#include "kalman/filter_base.hpp"
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/node/node.h"
#include "boost/bind.hpp"

FilterBase::FilterBase()
{
    // Reading params to setup filter and sensor properties
    YAML::Node paramList = FilterBase::ReadParams();
    
    // Create vector of sensors with assigned parameters
    FilterBase::AssignSensorParams(paramList);
    // Create measurement matrix.
    // We use the for each function 
    std::for_each(m_sensorVector.begin(),m_sensorVector.end(),boost::bind(&FilterBase::CreateMeasurementMatrix,this,_1));
    std::cout<<"Filter Base has completed initialization"<<std::endl;
    return;
};

YAML::Node FilterBase::ReadParams(){
    ROS_INFO("Reading params to setup filter");
    YAML::Node paramList = YAML::LoadFile("/home/darshit/artemis-workspace/Code/artemis/src/kalman/config/filter_params.yaml");
    return paramList;
};

void FilterBase::AssignSensorParams(YAML::Node& paramList)
{
    // Extract the sensor list first
    m_sensorList = paramList["sensor_list"].as<std::vector<std::string>>();
    
    // Load all the sensor properties into one YAML node.
    auto sensor_properties_list = paramList["sensor_properties"];
    
    std::vector<double> vectorizedMotionNoise;
    vectorizedMotionNoise = paramList["process_noise_covariance"].as<std::vector<double>>();
    m_motionNoiseCovarianceMatrix.resize(15,15);
    m_motionNoiseCovarianceMatrix = Eigen::Map<Eigen::Matrix<double,15,15>>(vectorizedMotionNoise.data());

    std::vector<double> vectorizedInitialCovariance;
    
    // TODO : add initial covariance as a part of the filter belief. For this it is necessary to add filter 
    // belief to the base class.
    
    // TODO : implement a check for all sensors available in the sensor list 
    // to be present in the sensor_properties_list. To raise an exception if 
    // not present

    for(std::string sensor_iterator : m_sensorList)
    {
        auto sensor_properties = sensor_properties_list[sensor_iterator];
        // Create an instance of the sensor struct.
        Sensor sensorInstance;
        // Assign the sensor name
        sensorInstance.sensorName = sensor_iterator;
        // Assign the sensor type
        sensorInstance.sensorType = sensor_properties["sensor_type"].as<std::string>();
        // Assign the sensor input vector
        sensorInstance.sensorInputVector = sensor_properties["measurements"].as<std::vector<bool>>();
        
        // Assign the sensor noise covariance matrix
        std::vector<double> vectorizedMeasurementNoise;
        vectorizedMeasurementNoise = sensor_properties["measurement_covariance"].as<std::vector<double>>(); 
        sensorInstance.measurementNoiseMatrix.resize(15,15);
        sensorInstance.measurementNoiseMatrix = Eigen::Map<Eigen::Matrix<double,15,15>>(vectorizedMeasurementNoise.data());  

        // Add the sensor to the list of available sensors
        m_sensorVector.push_back(sensorInstance);  
    };
    return;
}

void FilterBase::CreateMeasurementMatrix(FilterBase::Sensor &sensor)
{
    // For the argument sensor, we will create the measurement matrix
    Eigen::MatrixXd measurementMatrix;
    int stateSize = 15;
    int measurementSize = std::count(sensor.sensorInputVector.begin(),sensor.sensorInputVector.end(),true);
    measurementMatrix.resize(measurementSize,stateSize);
    measurementMatrix.setZero();

    std::vector<bool> copyVector = sensor.sensorInputVector;
    
    // Algorithm - for each 1 found in the copy vector - create a row vector with one at that position
    // also convert the 1 in the copy vector to 0 and run again. 


    for(int i = 0; i < measurementSize; i++)
    {
        std::vector<bool>::iterator itr = std::find(copyVector.begin(),copyVector.end(), true);
        
        // Get the position of the index where the true exists
        int indexPosition = std::distance(copyVector.begin(),itr);
        Eigen::MatrixXd measurementVector;
        measurementVector.resize(1,stateSize);
        measurementVector.setZero();
        measurementVector(indexPosition) = 1.0;

        // Adding tte measurement vector to the measurement matrix
        measurementMatrix.row(i) = measurementVector;

        // Setting the value at the generated index to zero.
        copyVector[indexPosition] = false;
    };

    // Assign the measurement matrix to the correct sensor
    sensor.measurementMatrix = measurementMatrix;
    return;
};

FilterBase::~FilterBase()
{
    
};

void FilterBase::Sensor::UpdateMeasurements(std::vector<double> measurement)
{
    // This function will be used to update the internal measurement vector
    // To make things simpler we ask for measurements to be provided in a standard vector
    
    // Currently we support a full state vector only with a state size of 15 states.
    assert(measurement.size() == 15);
    this->measurementVector.resize(15,1);
    this->measurementVector.setZero();

    // Map the data of the std::vector to the Eigen Matrix
    this->measurementVector = Eigen::Map<Eigen::Matrix<double,15,1>>(measurement.data());

    // Multiply the given measurements with the activation vector to create the actual measurement
    this->measurementMatrix = this->measurementMatrix * this->measurementVector;

    // Set the time of update
    this->updateTime = std::clock();
    return;
}

//int main(){
//    std::cout<<"Starting filter base"<<std::endl;
//    FilterBase filterBase;
//}
