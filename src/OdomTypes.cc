#include "OdomTypes.h"

namespace ORB_SLAM3
{
namespace ODOM
{
const float eps = 1e-4;
Preintegrated::Preintegrated(Preintegrated* mOdomPre): meas(mOdomPre->meas), cov(mOdomPre->cov)
{
    
}

void Preintegrated::MergePrevious(Preintegrated* pPre)
{
    if(pPre==this)
        return;
    // std::unique_lock<std::mutex> lock1(mMutex);

    meas.head<2>() += Eigen::Rotation2Dd(pPre->meas[2]).toRotationMatrix() * pPre->meas.head<2>();      // update new d_trans        
    meas[2] +=pPre->meas[2];                                                                            // update new d_phi
    cov *= pPre->cov;                                                                                   // upadate cov

}

void Preintegrated::IntegrateNewMeasurement(g2o::SE2 &currOdom, g2o::SE2 &lastOdom)
{
    g2o::SE2 odok = currOdom - lastOdom;
    
    // Check zero movement
    if(fabs(fabs(odok.rotation().angle())-0.001) < eps)
    {    
        odok.setRotation(Eigen::Rotation2Dd(0.0));
    } 
    Vector2d odork(odok.translation().x(), odok.translation().y());                              
    Matrix2d Phi_ik = Rotation2Dd(meas[2]).toRotationMatrix();                                    

    meas.head<2>() += Phi_ik * odork;                                                           // Update del trans rely only on prev deltaRot
    meas[2] += odok.rotation().angle();                                                         // Update del rot

    Matrix3d Ak = Matrix3d::Identity();
    Matrix3d Bk = Matrix3d::Identity();

    Ak.block<2,1>(0,2) = Phi_ik * Vector2d(-odork[1], odork[0]);
    Bk.block<2,2>(0,0) = Phi_ik;

    Matrix3d Sigmak = cov;
    Matrix3d Sigma_vk = Matrix3d::Identity();

    Sigma_vk(0,0) = (odom_x_noise * odom_x_noise);
    Sigma_vk(1,1) = (odom_y_noise * odom_y_noise);
    Sigma_vk(2,2) = (odom_theta_noise * odom_theta_noise);

    Matrix3d Sigma_k_1 = Ak * Sigmak * Ak.transpose() + Bk * Sigma_vk * Bk.transpose();
    cov = Sigma_k_1;

}
}
}
