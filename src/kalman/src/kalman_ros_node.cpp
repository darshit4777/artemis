#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "kalman/kalman_ros_node.hpp"
//#include "kalman/filter_base.hpp"

Robot::Robot(const ros::NodeHandle *nh)
{
    // Class constructor.
    _nodehandle = *nh;
    
    ReadRosparams();
    CreateSubscribers(m_filterBase.m_sensorVector[0]);

    // Assigning the publisher
    _filteredOdometryPublisher = 
    _nodehandle.advertise<nav_msgs::Odometry>("/filtered_odometry_topic",10);


    return;
};

void Robot::ReadRosparams()
{
    /**
     * Read rosparams
     */
    
    // Create a sensor name-topic pair for each sensor
    std::for_each(m_filterBase.m_sensorList.begin(),m_filterBase.m_sensorList.end(),
    boost::bind(&Robot::CreateSensorTopicPair,this,_1));
    


    // Parameters for robot frame and map frame. Sometimes these may take names
    // Apart from the default base_link and map - best to parameterize it.
    _nodehandle.param("robot_frame",m_robotFrame,std::string("base_link"));
    _nodehandle.param("map_frame",m_mapFrame,std::string("map"));

};

void Robot::OdometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // Callback for odometry subscriber
    
    //m_robotRawOdometry = *msg;
    //if(!m_odometryReceived)
    //{
    //    m_odometryReceived = true;
    //    ROS_INFO("Odometry received !!");
    //};
    return;
};

void Robot::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Callback for odometry subscriber
    //m_robotRawImu = *msg;
    //if(!m_imuReceived)
    //{
    //    m_imuReceived = true;
    //    ROS_INFO("IMU messages received !!");
    //};
    return;
};

void Robot::PublishTransform()
{
    geometry_msgs::TransformStamped robotTransform;
    // Assigning the correct frame ids for the transform
    robotTransform.child_frame_id = m_robotFrame;
    robotTransform.header.frame_id = m_mapFrame;

    // The timestamp for the transform is chosen as the timestamp when the filtered message was created.
    robotTransform.header.stamp = m_robotFilteredOdometry.header.stamp;

    // Transferring contents of the odometry message to transform message.
    robotTransform.transform.translation.x = m_robotFilteredOdometry.pose.pose.position.x;
    robotTransform.transform.translation.y = m_robotFilteredOdometry.pose.pose.position.y;
    robotTransform.transform.translation.z = m_robotFilteredOdometry.pose.pose.position.z;
    robotTransform.transform.rotation.x = m_robotFilteredOdometry.pose.pose.orientation.x;
    robotTransform.transform.rotation.y = m_robotFilteredOdometry.pose.pose.orientation.y;
    robotTransform.transform.rotation.z = m_robotFilteredOdometry.pose.pose.orientation.z;
    robotTransform.transform.rotation.w = m_robotFilteredOdometry.pose.pose.orientation.w;

    // Send it !    
    this->_transfromBroadcaster.sendTransform(robotTransform);
    return;
};

void Robot::CreateSubscribers(FilterBase::Sensor sensor)
{
    // This function will create subscribers on the basis of sensor name, type and topic name provided.
    ROS_INFO("Create Subs has been called");
    ros::Subscriber subscriberInstance;
    
    // Find the relevant topic of the sensor from the sensor name-topic pair map
    //std::set<std::pair<std::string,std::string>>::iterator itr = std::find(m_sensorSet.begin(),
    std::string sensorName = sensor.sensorName;
    std::cout<<m_sensorSet[sensorName]<<std::endl;
    

    // Algorithm - find the ros topic name that the sensor holds from the rosparams
    // match the name of the sensor and then according to sensor type - create a 
    // subscriber with a appropriate callback.
    return;
};

void Robot::CreateSensorTopicPair(std::string sensorName)
{
    std::string sensorTopicName;
    std::string paramName = "sensor_properties/" + sensorName +"/sensor_topic";
    _nodehandle.getParam(paramName,sensorTopicName);
    std::pair<std::string,std::string> sensorNameTopicPair;
    sensorNameTopicPair.first = sensorName;
    sensorNameTopicPair.second = sensorTopicName;

    m_sensorSet.insert(sensorNameTopicPair);
};

Robot::~Robot(){};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"kalman_ros_node");
    ros::NodeHandle nh;
    Robot kalman(&nh);
    ros::spin();
    return 0;
}