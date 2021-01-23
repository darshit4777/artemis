#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "kalman/kalman_ros_node.hpp"
#include "tf/tf.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/yaml.h"

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
    _nodehandle.advertise<nav_msgs::Odometry>("/filtered_odometry",10);

    odomRecieved = false;
    imuRecieved = false;
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
    odomRecieved = true;
    nav_msgs::Odometry odomMsg = *msg;
    sensor.UpdateMeasurements(Robot::PrepareOdometryMeasurement(odomMsg));
    return;
};

FilterBase::Sensor::measurement Robot::PrepareOdometryMeasurement(const nav_msgs::Odometry &odomMsg){
    Eigen::VectorXd measurementVector;
    measurementVector.resize(15,1);
    // Positions
    measurementVector[0] = odomMsg.pose.pose.position.x;
    measurementVector[1] = odomMsg.pose.pose.position.y;
    measurementVector[2] = odomMsg.pose.pose.position.z;

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
    measurementVector[3] = 0.0;
    measurementVector[4] = 0.0;
    measurementVector[5] = angleAxis.angle();
        
    // Velocities
    measurementVector[6] = odomMsg.twist.twist.linear.x;
    measurementVector[7] = odomMsg.twist.twist.linear.y;
    measurementVector[8] = odomMsg.twist.twist.linear.z;

    // Angular Velocities
    measurementVector[9] = odomMsg.twist.twist.angular.x;
    measurementVector[10] = odomMsg.twist.twist.angular.y;
    measurementVector[11] = odomMsg.twist.twist.angular.z;
    
    // Accelerations
    measurementVector[12] = 0.0;
    measurementVector[13] = 0.0;
    measurementVector[14] = 0.0;

    Eigen::MatrixXd measurementCovariance;
    measurementCovariance.resize(12,12);
    
    std::vector<double> poseCovarianceVectorized;
    std::vector<double> twistCovarianceVectorized;
    for (auto i = odomMsg.pose.covariance.end(); i != odomMsg.pose.covariance.begin(); --i)
    {
        poseCovarianceVectorized.push_back(*i);
    };
    for (auto i = odomMsg.twist.covariance.end(); i != odomMsg.twist.covariance.begin(); --i)
    {
        twistCovarianceVectorized.push_back(*i);
    };

    measurementCovariance.block<6,6>(0,0) = Eigen::Map<Eigen::Matrix<double,6,6>>(poseCovarianceVectorized.data());
    measurementCovariance.block<6,6>(6,6) = Eigen::Map<Eigen::Matrix<double,6,6>>(twistCovarianceVectorized.data());
    FilterBase::Sensor::measurement odomMeasurement;
    odomMeasurement.measurementVector = measurementVector;
    odomMeasurement.measurementCovariance = measurementCovariance;
    return odomMeasurement;
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
    imuRecieved = true;
    sensor.UpdateMeasurements(Robot::PrepareImuMeasurement(imuMsg));
    return;
};

FilterBase::Sensor::measurement Robot::PrepareImuMeasurement(const sensor_msgs::Imu &imuMsg)
{
    // Prepare a measurement using IMU data
    Eigen::VectorXd measurementVector;
    measurementVector.resize(15,1);
    // Positions
    measurementVector[0] = 0.0;
    measurementVector[1] = 0.0;
    measurementVector[2] = 0.0;

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
    measurementVector[3] = 0.0;
    measurementVector[4] = 0.0;
    measurementVector[5] = angleAxis.angle();

    // Velocities
    measurementVector[6] = 0.0;
    measurementVector[7] = 0.0;
    measurementVector[8] = 0.0;

    // Angular Velocities
    measurementVector[9] = imuMsg.angular_velocity.x;
    measurementVector[10] = imuMsg.angular_velocity.y;
    measurementVector[11] = imuMsg.angular_velocity.z;
    
    
    // AccelerationsimuMsg.linear_acceleration.x
    measurementVector[12] = imuMsg.linear_acceleration.x;
    measurementVector[13] = imuMsg.linear_acceleration.y;
    measurementVector[14] = imuMsg.linear_acceleration.z;

    
    Eigen::MatrixXd measurementCovariance;
    measurementCovariance.resize(9,9);
    std::vector<double> angularVelocityCovarianceVectorized;
    std::vector<double> linearAccelerationCovarianceVectorized;
    std::vector<double> orientationCovarianceVectorized;
    
    for (auto i = imuMsg.angular_velocity_covariance.end(); i != imuMsg.angular_velocity_covariance.begin(); --i)
    {
        angularVelocityCovarianceVectorized.push_back(*i);
    };
    for (auto i = imuMsg.linear_acceleration_covariance.end(); i != imuMsg.linear_acceleration_covariance.begin(); --i)
    {
        linearAccelerationCovarianceVectorized.push_back(*i);
    };
    for (auto i = imuMsg.orientation_covariance.end(); i != imuMsg.orientation_covariance.begin(); --i)
    {
        orientationCovarianceVectorized.push_back(*i);
    };

    measurementCovariance.block<3,3>(0,0) = Eigen::Map<Eigen::Matrix<double,6,6>>(orientationCovarianceVectorized.data());
    measurementCovariance.block<3,3>(3,3) = Eigen::Map<Eigen::Matrix<double,6,6>>(angularVelocityCovarianceVectorized.data());
    measurementCovariance.block<3,3>(6,6) = Eigen::Map<Eigen::Matrix<double,6,6>>(linearAccelerationCovarianceVectorized.data());

    
    


    FilterBase::Sensor::measurement measurement;
    measurement.measurementVector = measurementVector;
    return measurement;

}

nav_msgs::Odometry Robot::ConvertBeliefToOdometry(KalmanFilter::belief &belief)
{
    nav_msgs::Odometry filteredOdom;
    filteredOdom.child_frame_id = "base_link";
    filteredOdom.header.frame_id = "map";
    
    // Assign positions
    filteredOdom.pose.pose.position.x = belief.beliefVector[0];
    filteredOdom.pose.pose.position.y = belief.beliefVector[1];
    filteredOdom.pose.pose.position.z = belief.beliefVector[2];

    // Assign orientation
    double yaw, pitch, roll = 0.0;
    yaw = belief.beliefVector[3];
    pitch = belief.beliefVector[4];
    roll = belief.beliefVector[5];

    // TODO : Use estimates of yaw pitch and roll to compute a quaternion

    double qW = cos(yaw/2);
    double qZ = sin(yaw/2);

    filteredOdom.pose.pose.orientation.x = 0.0;
    filteredOdom.pose.pose.orientation.y = 0.0;
    filteredOdom.pose.pose.orientation.z = qZ;
    filteredOdom.pose.pose.orientation.w = qW;
    
    // Assign velocity
    filteredOdom.twist.twist.linear.x = belief.beliefVector[6];
    filteredOdom.twist.twist.linear.y = belief.beliefVector[7];
    filteredOdom.twist.twist.linear.z = belief.beliefVector[8];

    filteredOdom.twist.twist.angular.x = belief.beliefVector[9];
    filteredOdom.twist.twist.angular.y = belief.beliefVector[10];
    filteredOdom.twist.twist.angular.z = belief.beliefVector[11];

    // Timestamp
    filteredOdom.header.stamp = ros::Time::now();

    return filteredOdom;
};


void Robot::PublishFilteredBelief()
{
    /**
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
    */
    KalmanFilter::belief filteredBelief = m_kalmanFilter.GetBelief();
    nav_msgs::Odometry odomMsg = ConvertBeliefToOdometry(filteredBelief);
    _filteredOdometryPublisher.publish(odomMsg);
    return;
};

void Robot::CreateSubscribers(FilterBase::Sensor sensor)
{
    // This function will create subscribers on the basis of sensor name, type and topic name provided.
    ros::Subscriber subscriberInstance;
    
    // Find the relevant topic of the sensor from the sensor name-topic pair map
    std::string subscriberTopicName = m_sensorSet[sensor.sensorName];
    ROS_INFO_STREAM(subscriberTopicName);
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
    // TODO : Fix the path so that we have to refer to the config folder only
    YAML::Node paramList = YAML::LoadFile("/home/darshit/artemis-workspace/Code/artemis/src/kalman/config/filter_params.yaml");
    YAML::Node sensor_properties = paramList["sensor_properties"];
    YAML::Node sensor = sensor_properties[sensorName];
    sensorTopicName = sensor["sensor_topic"].as<std::string>();

    std::pair<std::string,std::string> sensorNameTopicPair;
    sensorNameTopicPair.first = sensorName;
    sensorNameTopicPair.second = sensorTopicName;
    ROS_INFO_STREAM(sensorNameTopicPair.first);
    ROS_INFO_STREAM(sensorNameTopicPair.second);
    m_sensorSet.insert(sensorNameTopicPair);
};

Robot::~Robot(){};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"kalman_ros_node");
    ros::NodeHandle nh;
    Robot kalman(&nh);
    
    boost::shared_ptr<nav_msgs::Odometry const> odomCheck;
    odomCheck = ros::topic::waitForMessage<nav_msgs::Odometry>("/husky_velocity_controller/odom");
    ROS_INFO("Recieved odometry message");    
    boost::shared_ptr<sensor_msgs::Imu const> imuCheck;
    imuCheck = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu/data");
    ROS_INFO("Recieved IMU message");    

    while(ros::ok())
    {
        
        if (kalman.imuRecieved && kalman.odomRecieved)
        {
            kalman.m_kalmanFilter.ExecutePredictionStep();
            kalman.m_kalmanFilter.ExecuteUpdateStep();
            kalman.PublishFilteredBelief();
        }
        
        ros::spinOnce();
    }
    
    return 0;
}