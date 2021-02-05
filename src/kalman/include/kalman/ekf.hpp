#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "filter_base.hpp"

class ExtendedKalmanFilter : public FilterBase
{
    /**
     * The model used for  Extended Kalman Filter is a modified form of the linearized
     * kinematics for a Differential Drive Robot.
     * An additional assumption right now, is that we only consider linear 
     * and angular velocities as control inputs.
     * The resulting prediction model looks as follows 
     * 
     * X[t+1] = x[t] + Vx * dt 
     *          y[t] + Vy * dt
     *          z[t] + Vz * dt 
     * 
     * and onwards. The difference between the KF and EKF models is that the EKF 
     * model makes use of a non-linear Jacobian matrix G. 
     * 
     * dt will be measured as the time interval between two prediction calls
    */
    
    private:


    public:
    Eigen::MatrixXd m_jacobianMatrixG;
    Eigen::MatrixXd m_controlInputMatrixB;
    Eigen::VectorXd m_controlInputVector;

    ros::Time currentMessageTime;
    ros::Time previousMessageTime;

    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();

    /**
     * @brief Executes a single prediction step
     * @return void, Makes changes to class variables  
    */

    void ExecutePredictionStep() override;

    /**
     * @brief Executes a single update step for a given sensor
     * @return void, Makes changes to the belief variables
    */

    void ExecuteSingleUpdateStep(FilterBase::Sensor& sensor) override;

    /**
     * @brief Execute update for all available sensors;
     * @return void, Makes changes to the belief variables
    */

    void ExecuteUpdateStep() override;

    /**
     * @brief Helper function that returns the current value of belief. 
     * To be used by processes outside the class definition that need to make 
     * use of the belief.
     * @return belief struct consisting of belief vector and belief covariance.
    */

    belief GetBelief();
    
    /**
     * @brief Function which updates the non linear jacobian matrix G on the basis of updated 
     * state and control input variables
     * @return void, updates the member m_jacobianMatrixG
    */
    void UpdateJacobianMatrix(double dt);
};



