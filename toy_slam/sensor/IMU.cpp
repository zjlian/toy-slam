#include "IMU.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace toy::sensor
{

    /// 获取旋转矩阵
    Eigen::Matrix3f GetOrientationMatrix(const IMU &val)
    {
        Eigen::Quaterniond q(val.orientation.w, val.orientation.x, val.orientation.y, val.orientation.z);
        Eigen::Matrix3f result = q.matrix().cast<float>();
        return result;
    }

} // namespace toy::sensor