#include "map_builder.h"
#include "toy_slam/sensor/GNSS.h"
#include "toy_slam/sensor/IMU.h"
#include "toy_slam/sensor/PointCloud.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>

namespace toy::mapping
{

    /// 更新雷达和imu的外参
    void MapBuilder::SetLidarToIMU(const Eigen::Matrix4f &val)
    {
        lidar_to_imu_ = val;
    }

    /// 前端更新一次
    void MapBuilder::Tick()
    {
        if (!lidar_to_imu_.has_value())
        {
            spdlog::warn("雷达和IMU的外参未设置");
            return;
        }

        sensor_->ParseData(pointcloud_buff_);
        sensor_->ParseData(imu_buff_);
        sensor_->ParseData(gnss_buff_);
        spdlog::debug("获取传感器数据，当前累计 pointcloud: {}, imu: {}, gnss: {}",
                      pointcloud_buff_.size(),
                      imu_buff_.size(),
                      gnss_buff_.size());

        while ((pointcloud_buff_.size() + imu_buff_.size() + gnss_buff_.size()) > 0)
        {
            auto cloud_data = pointcloud_buff_.front();
            auto imu_data = imu_buff_.front();
            auto gnss_data = gnss_buff_.front();

            if (!init_time_.has_value())
            {
                init_time_ = cloud_data->time;
            }
            auto run_time = cloud_data->time - init_time_.value();

            // 计算当前点云和IMU的时间差，抛弃时间差超过 0.05秒的数据
            auto cloud_imu_diff_time = cloud_data->time - imu_data->time;
            if (cloud_imu_diff_time < -0.05)
            {
                pointcloud_buff_.pop_front();
            }
            else if (cloud_imu_diff_time > 0.05)
            {
                imu_buff_.pop_front();
                gnss_buff_.pop_front();
            }
            else
            {
                pointcloud_buff_.pop_front();
                imu_buff_.pop_front();
                gnss_buff_.pop_front();

                // 更新 GNSS，用作真值比较
                Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
                static std::once_flag once_flag;
                std::call_once(once_flag, [&] {
                    sensor::InitGNSSOriginPosition(*gnss_data);
                });
                sensor::UpdateGNSSXYZ(*gnss_data);
                odometry_matrix(0, 3) = gnss_data->local_E;
                odometry_matrix(1, 3) = gnss_data->local_N;
                odometry_matrix(2, 3) = gnss_data->local_U;
                odometry_matrix.block<3, 3>(0, 0) = sensor::GetOrientationMatrix(*imu_data);
                odometry_matrix *= *lidar_to_imu_;
                SetLastGnss(odometry_matrix);

                // 用 gnss 数据初始化 slam pose
                static std::once_flag initi_local_slam_pose;
                std::call_once(initi_local_slam_pose, [&] {
                    local_slam_->SetInitPose(odometry_matrix);
                });

                // 更新前端
                local_slam_->SetPredictPose(odometry_matrix); // 这里的 gnss 作为预估值并没有实际用于点云匹配
                auto new_pose = local_slam_->Update(*cloud_data);
                SetLastPose(new_pose);
            }
        }
    }

    /// 获取最新的 GNSS Pose
    Eigen::Matrix4f MapBuilder::GetLastGnss()
    {
        return gnss_pose_;
    }

    /// 获取最新的机器人 Pose
    Eigen::Matrix4f MapBuilder::GetLastPose()
    {
        return slam_pose_;
    }

    /// 更新 GNSS Pose 查询值
    void MapBuilder::SetLastGnss(const Eigen::Matrix4f &matrix)
    {
        gnss_pose_ = matrix;
    }

    /// 更新机器人 Pose 查询值
    void MapBuilder::SetLastPose(const Eigen::Matrix4f &matrix)
    {
        slam_pose_ = matrix;
    }

    /// 获取完整的地图
    std::shared_ptr<sensor::PointCloud::CloudType> MapBuilder::GetLastGlobalMap()
    {
        return local_slam_->GetNewGlobalMap();
    }

    /// 获取当前的局部地图
    std::shared_ptr<sensor::PointCloud::CloudType> MapBuilder::GetLastLocalMap()
    {
        return local_slam_->GetNewLocalMap();
    }

    /// 获取最新的扫描帧
    std::shared_ptr<sensor::PointCloud::CloudType> MapBuilder::GetLastLaserScan()
    {
        return local_slam_->GetCurrentScan();
    }

} // namespace toy::mapping