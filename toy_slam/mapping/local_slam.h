#pragma once

#include "toy_slam/sensor/PointCloud.h"

#include <deque>

#include <Eigen/Dense>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace toy::mapping
{
    struct Frame
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        sensor::PointCloud cloud_data;
    };

    class LocalSlam
    {
    public:
        LocalSlam();

        /// 用点云更新地图
        Eigen::Matrix4f Update(const sensor::PointCloud &data);
        /// 设置初始位姿
        bool SetInitPose(const Eigen::Matrix4f &pose);
        /// 设置预估位姿
        bool SetPredictPose(const Eigen::Matrix4f &pose);

        std::shared_ptr<sensor::PointCloud::CloudType> GetNewLocalMap();
        std::shared_ptr<sensor::PointCloud::CloudType> GetNewGlobalMap();
        std::shared_ptr<sensor::PointCloud::CloudType> GetCurrentScan();

    private:
        void CreateNewFrame(const Frame &new_frame);

    private:
        using PointType = sensor::PointCloud::PointType;
        pcl::VoxelGrid<PointType> cloud_filter_;
        pcl::VoxelGrid<PointType> local_map_filter_;
        pcl::VoxelGrid<PointType> display_filter_;
        std::unique_ptr<pcl::NormalDistributionsTransform<PointType, PointType>> ndt_;

        std::deque<Frame> local_map_frames_;
        std::deque<Frame> global_map_frames_;

        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;
        std::shared_ptr<sensor::PointCloud::CloudType> local_map_;
        std::shared_ptr<sensor::PointCloud::CloudType> global_map_;
        std::shared_ptr<sensor::PointCloud::CloudType> result_cloud_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_{Eigen::Matrix4f::Identity()};
        Eigen::Matrix4f predict_pose_{Eigen::Matrix4f::Identity()};
    };

} // namespace toy::mapping