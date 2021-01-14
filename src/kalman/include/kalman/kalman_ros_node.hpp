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
#include "kalman/kf.hpp"

class Robot
{
private:
    // Publisher for filtered odometry
    ros::Publisher _filteredOdometryPublisher;
    ros::NodeHandle _nodehandle;
    
    // Transform buffer and broadcaster - to Publish the robot to map transform.
    tf2_ros::TransformBroadcaster _transfromBroadcaster;
    
public:
    
    std::string m_robotFrame; //< Robot base link frame of reference
    std::string m_mapFrame;   //< World / map frame of reference.
    
    //std::set<std::pair<std::string,std::string>> m_sensorSet;
    std::map<std::string,std::string> m_sensorSet;
    std::vector<ros::Subscriber> subscriberVector;
    
    nav_msgs::Odometry m_robotRawOdometry;
    nav_msgs::Odometry m_robotFilteredOdometry;

    ros::Timer filterPublishTimer;
    
    KalmanFilter m_kalmanFilter;
    /**
     * @brief Class constructor. Initializes the class and assigns the nodehandle.
     * Sets the rosparams and starts the filter timer
     */
    Robot(const ros::NodeHandle *nh);
    ~Robot();

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg, FilterBase::Sensor &sensor);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg,FilterBase::Sensor &sensor);
    void ReadRosparams();
    void PublishFilteredBelief();
    nav_msgs::Odometry ConvertBeliefToOdometry(KalmanFilter::belief &belief);
    void CreateMeasurementFromOdometry(const nav_msgs::Odometry);
    void CreateMeasurementFromImu(const sensor_msgs::Imu);

    private:
    void CreateSubscribers(FilterBase::Sensor sensor);
    void CreateSensorTopicPair(std::string sensorName);
    
    // TODO : Prepare a template function for this ?
    std::vector<double> PrepareOdometryMeasurement(const nav_msgs::Odometry &odomMsg);
    std::vector<double> PrepareImuMeasurement(const sensor_msgs::Imu &imuMsg);

    
};



