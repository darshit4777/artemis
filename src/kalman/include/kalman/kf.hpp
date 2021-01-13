#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "filter_base.hpp"

class KalmanFilter : public FilterBase
{
    /**
     * The model used for Kalman Filter is a modified form of the linearized
     * kinematics for a Differential Drive Robot.
     * An additional assumption right now, is that we only consider linear 
     * and angular velocities as control inputs.
     * The resulting prediction model looks as follows 
     * 
     * X[t+1] = [ 1 0 0 ][x]    + [1 0] [Vlinear] *dt
     *          [ 0 1 0 ][y]      [0 0] [Vangular]
     *          [ 0 0 1 ][theta]  [0 1]
     * 
     * dt will be measured as the time interval between two prediction calls
    */
    
    private:


    public:
    
    struct belief{
        Eigen::VectorXd beliefVector;
        Eigen::MatrixXd beliefCovariance;
    };

    belief m_filterBelief;
    Eigen::MatrixXd m_jacobianMatrixA;
    Eigen::MatrixXd m_controlInputMatrixB;

    ros::Time currentMessageTime;
    ros::Time previousMessageTime;

    /**
     * @brief Executes a single prediction step
     * @return void, Makes changes to class variables  
    */

    void ExecutePredictionStep() override;

    /**
     * @brief Executes a single update step for a given sensor
     * @return void, Makes changes to the belief variables
    */

    void ExecuteSingleUpdateStep(FilterBase::Sensor &sensor) override;

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
};



