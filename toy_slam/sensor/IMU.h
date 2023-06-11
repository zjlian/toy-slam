#pragma once

#include <Eigen/Dense>

namespace toy::sensor
{

    struct IMU
    {
        struct LinearAcceleration
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct Orientation
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
        };

        double time = 0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;
    };

    /// 获取旋转矩阵
    Eigen::Matrix3f GetOrientationMatrix(const IMU &val);

} // namespace toy::sensor