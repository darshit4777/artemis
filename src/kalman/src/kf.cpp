#include "kalman/kf.hpp"
#include "boost/bind.hpp"

KalmanFilter::KalmanFilter()
{
    
    // Setting the jacobian matrix to identity for Kalman Filter
    m_jacobianMatrixA.resize(15,15);
    m_jacobianMatrixA.setIdentity();

    // Setting the control input vector for a Kalman Filter 
    // All measurements which are first and higher order derivatives are considered as control inputs
    m_controlInputVector.resize(9,1);
    // Setting the control input matrix for a Kalman Filter
    m_controlInputMatrixB.resize(15,9);
    m_controlInputMatrixB.setZero();
    Eigen::MatrixXd identity9;
    identity9.resize(9,9);
    identity9.setIdentity();
    m_controlInputMatrixB.block<9,9>(0,0) = identity9; 
    
    currentMessageTime = ros::Time::now();
    previousMessageTime = ros::Time::now();                                               
    return;
};

void KalmanFilter::UpdateControlInputVector()
{   

    m_controlInputVector = m_filterBelief.beliefVector.block<9,1>(6,0);   
    return;
}

void KalmanFilter::ExecutePredictionStep()
{   
    // Calculate the time interval between two prediction steps
    currentMessageTime = ros::Time::now();
    ros::Duration deltaTime = currentMessageTime - previousMessageTime;
    std::cout<<currentMessageTime<<std::endl;
    std::cout<<previousMessageTime<<std::endl;
    previousMessageTime = currentMessageTime;
    double dt = deltaTime.toSec();
    std::cout<<"The elapsed time is "<<std::endl;
    std::cout<<dt<<std::endl;
    // Update the control input vector
    UpdateControlInputVector();
    
    // Prediction Step 
    // Calculate belief vector 
    std::cout<<"The control input vector is "<<std::endl;
    std::cout<<m_controlInputVector<<std::endl;
    

    std::cout<<"The control addition is"<<std::endl;
    auto controlAddition = m_controlInputMatrixB * m_controlInputVector * dt;
    std::cout<<controlAddition<<std::endl;
    m_filterBelief.beliefVector = m_jacobianMatrixA * m_filterBelief.beliefVector + m_controlInputMatrixB * m_controlInputVector * dt;

    // Calculate the belief covariance
    m_filterBelief.beliefCovariance = m_jacobianMatrixA * m_filterBelief.beliefCovariance * m_jacobianMatrixA.transpose() + m_motionNoiseCovarianceMatrix;
    std::cout<<"Belief vector after prediction"<<std::endl;
    std::cout<<m_filterBelief.beliefVector<<std::endl;

    std::cout<<"Belief covariance after prediction"<<std::endl;
    std::cout<<m_filterBelief.beliefCovariance<<std::endl;

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

   // Check if the sensor information is stale. If so, we don't use the sensor information.

   std::clock_t currentTime = std::clock();
   double timeFromLastUpdate = (currentTime - sensor.updateTime) / (double) CLOCKS_PER_SEC;
   if( timeFromLastUpdate > 0.5)
   {
       std::cout<<"[WARNING] Information from "<<sensor.sensorName<<" sensor is stale and won't be fused"<<std::endl;
       return;
   }
   // Calculating Kalman Gain
   if(sensor.measurementCovarianceMatrix.size() == 0)

   {
       std::cout<<"[WARNING] Sensor covariance for "<<sensor.sensorName<<" sensor is not filled out. skipping update "<< std::endl; 
       return;
   }

   auto adjustedBeliefCovariance = sensor.sensorModelMatrix * m_filterBelief.beliefCovariance * sensor.sensorModelMatrix.transpose();
   auto inverseTerm = adjustedBeliefCovariance + sensor.measurementCovarianceMatrix;
   auto kalmanGain = m_filterBelief.beliefCovariance * sensor.sensorModelMatrix.transpose() * (inverseTerm.inverse());
   // Calculating updated belief vector
   m_filterBelief.beliefVector = m_filterBelief.beliefVector + kalmanGain * (sensor.measurementVector - sensor.sensorModelMatrix * m_filterBelief.beliefVector);

   // Calculating updated covariance matrix
   m_filterBelief.beliefCovariance = m_filterBelief.beliefCovariance - kalmanGain * sensor.sensorModelMatrix * m_filterBelief.beliefCovariance;
   std::cout<<"After measurements from "<<sensor.sensorName<<std::endl;
   std::cout<<m_filterBelief.beliefVector<<std::endl;
   std::cout<<m_filterBelief.beliefCovariance<<std::endl;
   
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

/**
 * Acceleration covariance seems to drop a lot after imu integration - check imu covariances
 * Control addition seems to be incorrect! It's almost as if multiplication with dt results in an increase in value
 */