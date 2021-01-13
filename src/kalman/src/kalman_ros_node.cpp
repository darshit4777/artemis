#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "kalman/kalman_ros_node.hpp"
#include "tf/tf.h"

Robot::Robot(const ros::NodeHandle *nh)
{
    /**
     * The class constructor will assign the nodehandle, read rosparams and pair
     * sensors with topic names. The create subscribers function, will create 
     * appropriate subscribers on the basis of sensor type and topic name.
     */


    _nodehandle = *nh; 
    ReadRosparams();
    
    // Create subscribers and assign callbacks for each sensor type.
    std::for_each(m_kalmanFilter.m_sensorVector.begin(),m_kalmanFilter.m_sensorVector.end(),
    boost::bind(&Robot::CreateSubscribers,this,_1));    

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
    std::for_each(m_kalmanFilter.m_sensorList.begin(),m_kalmanFilter.m_sensorList.end(),
    boost::bind(&Robot::CreateSensorTopicPair,this,_1));
    


    // Parameters for robot frame and map frame. Sometimes these may take names
    // Apart from the default base_link and map - best to parameterize it.
    _nodehandle.param("robot_frame",m_robotFrame,std::string("base_link"));
    _nodehandle.param("map_frame",m_mapFrame,std::string("map"));

};

void Robot::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg,FilterBase::Sensor &sensor)
{
    /**
     * General Callback for odometry subscriber
     * Odometry message contents has specific positions in the measurement vector
     * The measurement vector is defined as 
     * [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, x_theta, y_theta, z_theta, x_omega, y_omega, z_omega]
     * An odometry message gives the following information 
     * x, y, z positions
     * x, y, z velocities
     * x, y, z orientations (theta)
     * x, y, z angular velocities (omega)
     * Therefore these measurements will be put in the correct positions of a measurement vector.
     * 
     * The prepare odometry measurement function carries out all the necessary assignments
     */
    nav_msgs::Odometry odomMsg = *msg;
    sensor.UpdateMeasurements(Robot::PrepareOdometryMeasurement(odomMsg));
    return;
};

std::vector<double> Robot::PrepareOdometryMeasurement(const nav_msgs::Odometry &odomMsg){
    std::vector<double> measurementVector;
    // Positions
    measurementVector[0] = odomMsg.pose.pose.position.x;
    measurementVector[1] = odomMsg.pose.pose.position.y;
    measurementVector[2] = odomMsg.pose.pose.position.z;

    // Velocities
    measurementVector[3] = odomMsg.twist.twist.linear.x;
    measurementVector[4] = odomMsg.twist.twist.linear.y;
    measurementVector[5] = odomMsg.twist.twist.linear.z;

    // Accelerations
    measurementVector[6] = 0.0;
    measurementVector[7] = 0.0;
    measurementVector[8] = 0.0;

    // Orientations
    // TODO : Currently we make the assumption of 2D motion only. This means, 
    // Pitch and roll will be ignored. Yaw will be given most importance.
    Eigen::Quaternion<double> q;
    q.x() = odomMsg.pose.pose.orientation.x;
    q.y() = odomMsg.pose.pose.orientation.y;
    q.z() = odomMsg.pose.pose.orientation.z;
    q.w() = odomMsg.pose.pose.orientation.w; 
    Eigen::AngleAxis<double> angleAxis(q); 
    
    // This is probably a bad way of doing things 
    measurementVector[9] = 0.0;
    measurementVector[10] = 0.0;
    measurementVector[11] = angleAxis.angle();
    
    // Angular Velocities
    measurementVector[12] = odomMsg.twist.twist.angular.x;
    measurementVector[13] = odomMsg.twist.twist.angular.y;
    measurementVector[14] = odomMsg.twist.twist.angular.z;

    return measurementVector;
}

void Robot::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg,FilterBase::Sensor &sensor)
{
    /**
     * General Callback for odometry subscriber
     * IMU message contents has specific positions in the measurement vector
     * The measurement vector is defined as 
     * [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, x_theta, y_theta, z_theta, x_omega, y_omega, z_omega]
     * An odometry message gives the following information 
     * x ,y, z accelerations
     * x, y, z orientations (theta)
     * x, y, z angular velocities (omega)
     * Therefore these measurements will be put in the correct positions of a measurement vector.
     * 
     * The prepare imu measurement function carries out all the necessary assignments
     */

    // General Callback for IMU subscriber
    sensor_msgs::Imu imuMsg = *msg;
    sensor.UpdateMeasurements(Robot::PrepareImuMeasurement(imuMsg));
    return;
};

std::vector<double> Robot::PrepareImuMeasurement(const sensor_msgs::Imu &imuMsg)
{
    // Prepare a measurement using IMU data
    std::vector<double> measurementVector;
    // Positions
    measurementVector[0] = 0.0;
    measurementVector[1] = 0.0;
    measurementVector[2] = 0.0;

    // Velocities
    measurementVector[3] = 0.0;
    measurementVector[4] = 0.0;
    measurementVector[5] = 0.0;

    // AccelerationsimuMsg.linear_acceleration.x
    measurementVector[6] = imuMsg.linear_acceleration.x;
    measurementVector[7] = imuMsg.linear_acceleration.y;
    measurementVector[8] = imuMsg.linear_acceleration.z;

    // Orientations
    // TODO : Currently we make the assumption of 2D motion only. This means, 
    // Pitch and roll will be ignored. Yaw will be given most importance.
    Eigen::Quaternion<double> q;
    q.x() = imuMsg.orientation.x;
    q.y() = imuMsg.orientation.y;
    q.z() = imuMsg.orientation.z;
    q.w() = imuMsg.orientation.w; 
    Eigen::AngleAxis<double> angleAxis(q); 
    
    // This is probably a bad way of doing things 
    measurementVector[9] = 0.0;
    measurementVector[10] = 0.0;
    measurementVector[11] = angleAxis.angle();
    
    // Angular Velocities
    measurementVector[12] = imuMsg.angular_velocity.x;
    measurementVector[13] = imuMsg.angular_velocity.y;
    measurementVector[14] = imuMsg.angular_velocity.z;

    return measurementVector;

}

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
    ros::Subscriber subscriberInstance;
    
    // Find the relevant topic of the sensor from the sensor name-topic pair map
    std::string subscriberTopicName = m_sensorSet[sensor.sensorName];
    std::string sensorType = sensor.sensorType;

    if(sensorType == "odometry"){
        /**
         * For odometry sensors we create a generalized odometry subsciber
         */
        subscriberInstance = _nodehandle.subscribe<nav_msgs::Odometry>(subscriberTopicName,20,
        boost::bind(&Robot::OdometryCallback,this,_1,sensor));
    }
    else if (sensorType == "imu"){
        /**
         * For inertial sensors we create a genearlized subscriber
         */
        subscriberInstance = _nodehandle.subscribe<sensor_msgs::Imu>(subscriberTopicName,20,
        boost::bind(&Robot::ImuCallback,this,_1,sensor));
    };
    subscriberVector.push_back(subscriberInstance);
    std::string logString = "Subscriber created for " + sensor.sensorName + " listening to topic " + subscriberTopicName;
    ROS_INFO_STREAM(logString);

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