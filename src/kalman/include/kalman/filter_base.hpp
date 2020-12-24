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
class FilterBase
{
private:
    /* data */
public:
    std::vector<double> m_stateVector; //< State Vector
    std::vector<double> m_controlVector; //< Control Vector
    
    // TODO : Create a method that takes in specific values from each sensor to be used for the filter

    // TODO : Problems to be Solved :
    // 1 - How to create activation for various measurements - Done
    // 2 - Should we be caring about sparse matrices and their multiplication?
    // 3 - How to create support for multiple sensors.
    // 4 - How to solve the problem of two sensors measuring the same quantity.

    std::vector<unsigned int> m_odomActivationVector; //< Activation vector used for odometry measurements
    std::vector<unsigned int> m_imuActivationVector;  //< Activation vector used for imu measurements


    // Creating a struct to store the information from each sensor
    struct sensor
    {
        std::vector<int> sensorInputVector;
        Eigen::MatrixXd measurementNoiseMatrix;
    };

    std::vector<sensor> sensorVector; //< Vector which holds all sensor information.


    // # TODO : Create a method that takes in the A matrix , B Matrix and C matrix as specified in a rosparam.
    // Until then we can continue to operate with a single state space model    

    Eigen::MatrixXd m_stateMatrix;  //< State Matrix A 
    Eigen::MatrixXd m_controlMatrix; //< Control Matrix B 
    Eigen::MatrixXd m_measurementMatrix; //< Measurement Matrix C

    // # TODO : Create a method that takes in and assigns values to the noise matrices as specified
    // in rosparams.
    Eigen::MatrixXd m_motionNoiseCovarianceMatrix; //< Motion noise covariance matrix R
    Eigen::MatrixXd m_measurementNoiseCovarianceMatrix; //< measurement noise covariance matrix Q
    
    Eigen::MatrixXd m_KalmanGain; //< Matrix to hold the Kalman Gain

    struct belief{
        std::vector<double> meanVector;
        Eigen::MatrixXd covarianceMatrix;
    };
    belief m_inputBelief;
    belief m_outputBelief;

    FilterBase();
    ~FilterBase();

    /** 
     * @brief Read the rosparams
     * @return Void. if unsuccesful in reading params then exceptions will be raised
     */
    void ReadParams();

    /**
     * @brief Uses the activation vector provded to create the C matrix.
     */
    void CreateMeasurementMatrix();

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


};
