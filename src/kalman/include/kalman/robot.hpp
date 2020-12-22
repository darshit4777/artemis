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

class Robot
{
private:
    // Publisher for filtered odometry
    ros::Publisher _filteredOdometryPublisher;
    ros::NodeHandle _nodehandle;
    
    // Subscribers for various state variables
    ros::Subscriber _odometrySubscriber;
    ros::Subscriber _imuSubscriber;
    
    // Transform buffer and broadcaster - to Publish the robot to map transform.
    
    //tf2_ros::Buffer _transformBuffer;
    //tf2_ros::TransformListener _transformListener;
    static tf2_ros::TransformBroadcaster _transfromBroadcaster;
    
    // A steady timer to run the filter at a specific frequency.
    // ros::SteadyTimer filterTimer;
    
public:
    
    std::string m_robotFrame; //< Robot base link frame of reference
    std::string m_mapFrame;   //< World / map frame of reference.
    
    struct topicNames{
        std::string m_odometryTopic;
        std::string m_imuTopic;
        std::string m_gpsTopic;
        std::string m_filteredOdometryTopic;
    } m_topicNames;

    nav_msgs::Odometry m_robotRawOdometry;
    nav_msgs::Odometry m_robotFilteredOdometry;

    sensor_msgs::Imu m_robotRawImu;

    bool m_odometryReceived;
    bool m_imuReceived;
    // TODO: Add a filter KF or EKF here 

    /**
     * @brief Class constructor. Initializes the class and assigns the nodehandle.
     * Sets the rosparams and starts the filter timer
     */
    Robot(const ros::NodeHandle *nh);
    ~Robot();

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void ReadRosparams();
    void PublishTransform();
    
};



