#include "kalman/kf.hpp"


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

    Eigen::Vector2d controlInputs;
    double x_vel, y_vel, ang_vel;
    x_vel = m_filterBelief.beliefVector[3];
    y_vel = m_filterBelief.beliefVector[4];
    ang_vel = m_filterBelief.beliefVector[14];
    double linearVelocity = pow(pow(x_vel,2) + pow(y_vel,2),0.5);
    controlInputs << linearVelocity, ang_vel;
    
    // Prediction Step 
    m_filterBelief.beliefVector = m_jacobianMatrixA * m_filterBelief.beliefVector + m_controlInputMatrixB * controlInputs * dt;

    return;

    // TODO : Put a mutex over the belief vector - it is likely going to run into scenarios where it will be read and written to 
    // at the same time.

}

KalmanFilter::~KalmanFilter()
{
    return;
}

