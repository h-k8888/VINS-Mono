#include "utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    //R0将ng1旋转到[0,0,1]，也就相当于旋转坐标系把z轴旋转到了ng1
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();

    //旋转的过程中可能改变了yaw角，再把yaw旋转回原来的位置。
    //这个是沿z轴的旋转，因此不改变g沿z轴的方向。只是坐标系的x,y轴改变了。
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
