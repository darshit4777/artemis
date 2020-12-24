#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "kalman/filter_base.hpp"


FilterBase::FilterBase()
{
    // Reading Rosparams to setup filter and sensor properties
    FilterBase::ReadParams();

    return;
};

bool FilterBase::ReadParams()
FilterBase::~FilterBase()
{
    
}
