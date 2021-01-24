#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "yaml-cpp/node/node.h"
#include <chrono>
#include <ctime>

class FilterBase
{
private:
    // ros::NodeHandle _nh;
public:
    std::vector<double> m_stateVector; //< State Vector
    std::vector<double> m_controlVector; //< Control Vector

    // TODO : Problems to be Solved :
    // 1 - How to create activation for various measurements - Done
    // 2 - Should we be caring about sparse matrices and their multiplication?
    // 3 - How to create support for multiple sensors.
    // 4 - How to solve the problem of two sensors measuring the same quantity.

    std::vector<unsigned int> m_odomActivationVector; //< Activation vector used for odometry measurements
    std::vector<unsigned int> m_imuActivationVector;  //< Activation vector used for imu measurements


    // Creating a struct to store the information from each sensor
    class Sensor
    {   
        public:
        std::string sensorType;
        std::string sensorName;
        std::vector<bool> sensorInputVector;
        Eigen::MatrixXd measurementCovarianceMatrix;
        Eigen::MatrixXd sensorModelMatrix;
        Eigen::MatrixXd measurementVector;
        std::clock_t updateTime;

        struct measurement{
            Eigen::VectorXd measurementVector;
            Eigen::MatrixXd measurementCovariance;
        };    

        

    };
    void UpdateMeasurements(FilterBase::Sensor::measurement measurement,FilterBase::Sensor* sensor);
    
    struct belief{
        Eigen::VectorXd beliefVector;
        Eigen::MatrixXd beliefCovariance;
    };
    
    belief m_filterBelief;
    
    std::vector<std::string> m_sensorList;

    std::vector<FilterBase::Sensor> m_sensorVector; //< Vector which holds all sensor information.


    // # TODO : Create a method that takes in the A matrix , B Matrix and C matrix as specified in a rosparam.
    // Until then we can continue to operate with a single state space model    

    Eigen::MatrixXd m_stateMatrix;  //< State Matrix A 
    Eigen::MatrixXd m_controlMatrix; //< Control Matrix B 

    // # TODO : Create a method that takes in and assigns values to the noise matrices as specified
    // in rosparams.
    Eigen::MatrixXd m_motionNoiseCovarianceMatrix; //< Motion noise covariance matrix R
    belief m_inputBelief;
    belief m_outputBelief;

    FilterBase();
    ~FilterBase();

    /** 
     * @brief Reads a yaml config file and assigns values to the sensor vector member
     * @return YAML Node which can be further pruned to extract params. 
     * If unsuccesful in reading params then exceptions will be raised
     */
    YAML::Node ReadParams();
    /**
     * @brief uses the params read from the param file, creates sensors and assigns
     * parameters to them.
     * @return void.
     */
    void AssignSensorParams(YAML::Node& paramList);

    /**
     * @brief Uses the activation vector provded to create the C matrix.
     */
    void CreateSensorModelMatrix(FilterBase::Sensor& sensor);

    /**
     * @brief Creates the state matrix
     */
    void CreateStateMatrix();

    /**
     * @brief Creates the control matrix
     */
    void CreateControlMatrix(); 

    /**
     * @brief Updates the state and the control vectors
    */

    void UpdateFilterState(nav_msgs::Odometry odom, sensor_msgs::Imu imu);

    /**
     * @brief Executes a single prediction step
     * @return void, Makes changes to class variables  
    */

    virtual void ExecutePredictionStep() = 0;

    /**
     * @brief Executes a single update step for a given sensor
     * @return void, Makes changes to the belief variables
    */

    virtual void ExecuteSingleUpdateStep(FilterBase::Sensor& sensor) = 0;
    
    /**
     * @brief Execute update for all available sensors;
     * @return void, Makes changes to the belief variables
    */

    virtual void ExecuteUpdateStep() = 0;

};
