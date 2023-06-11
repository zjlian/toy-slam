#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>

namespace toy::sensor
{

    /// 点云
    struct PointCloud
    {
        using PointType = pcl::PointXYZ;
        using CloudType = pcl::PointCloud<PointType>;

        double time = 0.0;
        std::shared_ptr<CloudType> cloud_ptr{std::make_shared<CloudType>()};
    };

} // namespace toy::sensor