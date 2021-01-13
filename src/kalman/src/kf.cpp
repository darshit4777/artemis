#include "kalman/kf.hpp"
#include "boost/bind.hpp"

KalmanFilter::KalmanFilter()
{
    // Initialize the Belief

    // Setting initial belief vector to zero
    m_filterBelief.beliefVector.resize(15,1);
    m_filterBelief.beliefVector.setZero();
    
    // Setting initial belief covariance to an identity matrix with value 
    // diagonal elements as 0.1
    m_filterBelief.beliefCovariance.resize(15,15);
    m_filterBelief.beliefCovariance.setIdentity();
    m_filterBelief.beliefCovariance = m_filterBelief.beliefCovariance * 0.1;
    
    // Setting the jacobian matrix to identity for Kalman Filter
    m_jacobianMatrixA.resize(15,15);
    m_jacobianMatrixA.setIdentity();

    // Setting the control input matrix for a Kalman Filter
    m_controlInputMatrixB.resize(3,2);
    m_controlInputMatrixB << 1 , 0 ,
                             0 , 0 ,
                             0 , 1 ;      
    currentMessageTime = ros::Time::now();
    previousMessageTime = ros::Time::now();                                               
    return;
};

void KalmanFilter::ExecutePredictionStep()
{   
    // Calculate the time interval between two prediction steps
    currentMessageTime = ros::Time::now();
    ros::Duration deltaTime = currentMessageTime - previousMessageTime;
    double dt = deltaTime.toSec();

    // Set the control inputs - linear and angular velocity
    
    double x_vel, y_vel, angularVelocity;
    x_vel = m_filterBelief.beliefVector[3];
    y_vel = m_filterBelief.beliefVector[4];
    angularVelocity = m_filterBelief.beliefVector[14];
    double linearVelocity = pow(pow(x_vel,2) + pow(y_vel,2),0.5);
    
    Eigen::Vector2d controlInputs;
    controlInputs << linearVelocity, angularVelocity;
    
    // Prediction Step 
    // Calculate belief vector 
    m_filterBelief.beliefVector = m_jacobianMatrixA * m_filterBelief.beliefVector + m_controlInputMatrixB * controlInputs * dt;

    // Calculate the belief covariance
    m_filterBelief.beliefCovariance = m_jacobianMatrixA * m_filterBelief.beliefCovariance * m_jacobianMatrixA.transpose() + m_motionNoiseCovarianceMatrix;

    return;

    // TODO : Put a mutex over the belief vector - it is likely going to run into scenarios where it will be read and written to 
    // at the same time.

};

void KalmanFilter::ExecuteUpdateStep()
{   
    // Executes the update step for each sensor
    std:for_each(m_sensorVector.begin(),m_sensorVector.end(),boost::bind(&KalmanFilter::ExecuteSingleUpdateStep,this,_1));
    return;
};

void KalmanFilter::ExecuteSingleUpdateStep(FilterBase::Sensor &sensor)
{
    /**
     * Here we execute the update equation for a single sensor
    */

   // Calculating Kalman Gain
   auto inverseTerm = sensor.measurementMatrix * m_filterBelief.beliefCovariance * sensor.measurementMatrix + sensor.measurementNoiseMatrix;
   auto kalmanGain = m_filterBelief.beliefCovariance * sensor.measurementMatrix.transpose() * (inverseTerm.inverse());

   // Calculating updated belief vector
   m_filterBelief.beliefVector = m_filterBelief.beliefVector + kalmanGain * (sensor.measurementVector - m_filterBelief.beliefCovariance * m_filterBelief.beliefVector);

   // Calculating updated covariance matrix
   m_filterBelief.beliefCovariance = m_filterBelief.beliefCovariance - kalmanGain * sensor.measurementMatrix * m_filterBelief.beliefCovariance;

    return;
};

KalmanFilter::belief KalmanFilter::GetBelief()
{
    belief copyBelief(m_filterBelief); // Use a copy constructor assignment.
    return copyBelief;
}


KalmanFilter::~KalmanFilter()
{
    return;
}

