#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "kalman/robot.hpp"

Robot::Robot(const ros::NodeHandle *nh)
{
    // Class constructor.
    _nodehandle = *nh;
    
    // Setting initial values for message recieved checks;
    m_odometryReceived = false;
    m_imuReceived = false;

    // Assigning the subscribers
    _odometrySubscriber = _nodehandle.subscribe("/odometry_topic",10,&Robot::OdometryCallback,this);
    _imuSubscriber = _nodehandle.subscribe("/imu_topic",20,&Robot::ImuCallback,this);

    // Assigning the publisher
    _filteredOdometryPublisher = _nodehandle.advertise<nav_msgs::Odometry>("/filtered_odometry_topic",10);

    return;
};

void Robot::ReadRosparams()
{
    /**
     * Read rosparams
     */
    _nodehandle.param("odom_topic",m_topicNames.m_imuTopic,std::string("/odom"));
    _nodehandle.param("imu_topic",m_topicNames.m_imuTopic,std::string("/imu/data"));
    _nodehandle.param("filtered_odom_topic",m_topicNames.m_filteredOdometryTopic,std::string("/filtered_odom"));
    _nodehandle.param("robot_frame",m_robotFrame,std::string("base_link"));
    _nodehandle.param("map_frame",m_mapFrame,std::string("map"));

};

void Robot::OdometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // Callback for odometry subscriber
    m_robotRawOdometry = *msg;
    if(!m_odometryReceived)
    {
        m_odometryReceived = true;
        ROS_INFO("Odometry received !!");
    };
    return;
};

void Robot::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Callback for odometry subscriber
    m_robotRawImu = *msg;
    if(!m_imuReceived)
    {
        m_imuReceived = true;
        ROS_INFO("IMU messages received !!");
    };
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
    _transfromBroadcaster.sendTransform(robotTransform);
    return;
};

