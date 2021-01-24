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
    std::for_each(m_sensorVector.begin(),m_sensorVector.end(),boost::bind(&FilterBase::CreateSensorModelMatrix,this,_1));
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

    // Initialize the initial belief as well.
    std::vector<double> vectorizedInitialCovariance;
    vectorizedInitialCovariance = paramList["initial_covariance"].as<std::vector<double>>();
    
    m_filterBelief.beliefCovariance = Eigen::Map<Eigen::Matrix<double,15,15>>(vectorizedInitialCovariance.data());
    m_filterBelief.beliefVector.resize(15,1);
    m_filterBelief.beliefVector.setZero();
    
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

        // Add the sensor to the list of available sensors
        m_sensorVector.push_back(sensorInstance);  
    };
    return;
}

void FilterBase::CreateSensorModelMatrix(FilterBase::Sensor &sensor)
{
    // For the argument sensor, we will create the measurement matrix
    Eigen::MatrixXd sensorModelMatrix;
    int stateSize = 15;
    int measurementSize = std::count(sensor.sensorInputVector.begin(),sensor.sensorInputVector.end(),true);
    sensorModelMatrix.resize(measurementSize,stateSize);
    sensorModelMatrix.setZero();

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
        sensorModelMatrix.row(i) = measurementVector;

        // Setting the value at the generated index to zero.
        copyVector[indexPosition] = false;
    };

    // Assign the measurement matrix to the correct sensor
    sensor.sensorModelMatrix = sensorModelMatrix;
    return;
};

FilterBase::~FilterBase()
{
    
};

void FilterBase::Sensor::UpdateMeasurements(FilterBase::Sensor::measurement measurement)
{
    // This function will be used to update the internal measurement vector
    // To make things simpler we ask for measurements to be provided in a standard vector

    // Multiply the given measurements with the sensor model matrix to create the actual measurement
    this->measurementVector = this->sensorModelMatrix * measurement.measurementVector;
    // Assign the measurement covariance
    this->measurementCovarianceMatrix = this->sensorModelMatrix * measurement.measurementCovariance * this->sensorModelMatrix.transpose();
    // Set the time of update
    this->updateTime = std::clock();
    return;
}

//int main(){
//    std::cout<<"Starting filter base"<<std::endl;
//    FilterBase filterBase;
//}
