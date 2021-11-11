#ifndef ODOMTYPES_H
#define ODOMTYPES_H

// #include <mutex>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/se2.h"

#include "Converter.h"

namespace ORB_SLAM3
{
namespace ODOM
{
class Preintegrated
{
public:
    Preintegrated(Vector3d& noise):meas(Eigen::Vector3d::Zero()), cov(Eigen::Matrix3d::Identity()),
                                    odom_x_noise(noise[0]), odom_y_noise(noise[1]), odom_theta_noise(noise[2]) {};
    
    Preintegrated(Preintegrated* mOdomPre);
    
    void IntegrateNewMeasurement(g2o::SE2 &currOdom, g2o::SE2 &lastOdom);

    void MergePrevious(Preintegrated* mOdomPre);

public:

    // Measurement and covariance
    Eigen::Vector3d meas;
    Eigen::Matrix3d cov;

    //Odometry Noise
    double odom_x_noise;
    double odom_y_noise;
    double odom_theta_noise;

}; // class Preintegrated

}
}
#endif