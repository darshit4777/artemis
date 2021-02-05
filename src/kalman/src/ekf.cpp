#include "kalman/ekf.hpp"
#include "boost/bind.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    
    // Setting the jacobian matrix to identity for Kalman Filter
    m_jacobianMatrixG.resize(15,15);
    m_jacobianMatrixG.setIdentity(); 
    
    currentMessageTime = ros::Time::now();
    previousMessageTime = ros::Time::now();                                               
    return;
};

void ExtendedKalmanFilter::UpdateJacobianMatrix(double dt)
{   
    // Resetting the matrix
    m_jacobianMatrixG.setIdentity();
    // Theta serves as the yaw - extracted for convenience
    double theta = m_filterBelief.beliefVector[5];
    // Updating Position Rows
    /// X position
    m_jacobianMatrixG.row(0)[6] = cos(theta) * dt;
    m_jacobianMatrixG.row(0)[7] = -sin(theta) * dt;

    /// Y position
    m_jacobianMatrixG.row(1)[6] = sin(theta) * dt;
    m_jacobianMatrixG.row(1)[7] = cos(theta) * dt;   

    /// Z position
    m_jacobianMatrixG.row(2)[8] = dt;

    /// Roll
    m_jacobianMatrixG.row(3)[9] = dt;
    /// Pitch
    m_jacobianMatrixG.row(4)[10] = dt;
    /// Yaw
    m_jacobianMatrixG.row(5)[11] = dt;

    // Updating Velocity Rows
    /// X Velocity
    m_jacobianMatrixG.row(6)[12] = dt;
    /// Y Velocity
    m_jacobianMatrixG.row(7)[13] = dt;
    /// Z Velocity
    m_jacobianMatrixG.row(8)[14] = dt;
       
    return;
}

void ExtendedKalmanFilter::ExecutePredictionStep()
{   
    // Calculate the time interval between two prediction steps
    currentMessageTime = ros::Time::now();
    ros::Duration deltaTime = currentMessageTime - previousMessageTime;
    //std::cout<<currentMessageTime<<std::endl;
    //std::cout<<previousMessageTime<<std::endl;
    previousMessageTime = currentMessageTime;
    double dt = deltaTime.toSec();
    if (dt > 5.0)
    {
        dt = 1.0;
    }

    UpdateJacobianMatrix(dt);
    
    // Prediction Step 
    m_filterBelief.beliefVector = m_jacobianMatrixG * m_filterBelief.beliefVector;

    // Calculate the belief covariance
    m_filterBelief.beliefCovariance = m_jacobianMatrixG * m_filterBelief.beliefCovariance * m_jacobianMatrixG.transpose() + m_motionNoiseCovarianceMatrix;
    //std::cout<<"Belief vector after prediction"<<std::endl;
    //std::cout<<m_filterBelief.beliefVector<<std::endl;

    //std::cout<<"Belief covariance after prediction"<<std::endl;
    //std::cout<<m_filterBelief.beliefCovariance<<std::endl;

    return;

    // TODO : Put a mutex over the belief vector - it is likely going to run into scenarios where it will be read and written to 
    // at the same time.

};

void ExtendedKalmanFilter::ExecuteUpdateStep()
{   
    // Executes the update step for each sensor
    std:for_each(m_sensorVector.begin(),m_sensorVector.end(),boost::bind(&ExtendedKalmanFilter::ExecuteSingleUpdateStep,this,_1));
    return;
};

void ExtendedKalmanFilter::ExecuteSingleUpdateStep(FilterBase::Sensor &sensor)
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
   //std::cout<<"After measurements from "<<sensor.sensorName<<std::endl;
   //std::cout<<m_filterBelief.beliefVector<<std::endl;
   //std::cout<<m_filterBelief.beliefCovariance<<std::endl;
   
   return;
};

ExtendedKalmanFilter::belief ExtendedKalmanFilter::GetBelief()
{
    belief copyBelief(m_filterBelief); // Use a copy constructor assignment.
    return copyBelief;
}


ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
    return;
}

/**
 * Acceleration covariance seems to drop a lot after imu integration - check imu covariances
 * High imu acceleration noise combined with low covariances causes velocity to blow up quickly
 * Covariance increases rapidly - No effect of measurement on covariance ?
 */