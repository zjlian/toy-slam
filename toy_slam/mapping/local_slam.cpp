#include "local_slam.h"
#include "toy_slam/sensor/PointCloud.h"
#include <cassert>
#include <math.h>
#include <memory>
#include <pcl/filters/filter.h>
#include <pcl/registration/ndt.h>
#include <vector>

namespace toy::mapping
{

    LocalSlam::LocalSlam()
        : ndt_(std::make_unique<pcl::NormalDistributionsTransform<PointType, PointType>>()),
          local_map_(std::make_unique<sensor::PointCloud::CloudType>()),
          global_map_(std::make_unique<sensor::PointCloud::CloudType>()),
          result_cloud_(std::make_unique<sensor::PointCloud::CloudType>())
    {
        cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
        local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
        display_filter_.setLeafSize(0.5, 0.5, 0.5);

        ndt_->setResolution(1.0);
        ndt_->setStepSize(0.1);
        ndt_->setTransformationEpsilon(0.01);
        ndt_->setMaximumIterations(30);
    }

    Eigen::Matrix4f LocalSlam::Update(const sensor::PointCloud &data)
    {
        current_frame_.cloud_data.time = data.time;
        std::vector<int> indices;
        // 过滤无效点，结果存到 current_frame_ 里
        pcl::removeNaNFromPointCloud(*data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

        // 素体滤波
        auto filtered_cloud_ptr = std::make_shared<sensor::PointCloud::CloudType>();
        cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
        cloud_filter_.filter(*filtered_cloud_ptr);

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static auto last_pose = init_pose_;
        static auto predict_pose = init_pose_;
        static auto last_key_frame_pose = init_pose_;

        // 如果没有历史的地图数据，直接创建一帧新地图
        if (local_map_frames_.empty())
        {
            current_frame_.pose = init_pose_;
            CreateNewFrame(current_frame_);
            return current_frame_.pose;
        }

        // 使用新点云与历史地图做匹配
        ndt_->setInputSource(filtered_cloud_ptr);
        ndt_->align(*result_cloud_, predict_pose);
        current_frame_.pose = ndt_->getFinalTransformation();

        // 使用两帧点云的距离预估下一帧点云的位置
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        // 检查匹配结果，是否满足生成关键帧的要求，位移距离超过 2 米
        if ((current_frame_.pose.col(3).head(3) - last_key_frame_pose.col(3).head(3)).lpNorm<1>() > 2.0)
        {
            CreateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        return current_frame_.pose;
    }

    bool LocalSlam::SetInitPose(const Eigen::Matrix4f &pose)
    {
        init_pose_ = pose;
        return true;
    }

    bool LocalSlam::SetPredictPose(const Eigen::Matrix4f &pose)
    {
        predict_pose_ = pose;
        return true;
    }

    void LocalSlam::CreateNewFrame(const Frame &new_frame)
    {
        auto key_frame = std::move(new_frame);
        assert(new_frame.cloud_data.cloud_ptr->empty());

        // 更新局部地图
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > 20)
        {
            local_map_frames_.pop_front();
        }
        local_map_ = std::make_unique<sensor::PointCloud::CloudType>();
        auto transformed_cloud_ptr = std::make_shared<sensor::PointCloud::CloudType>();
        // 遍历所有 local map 关键帧，转换位姿合并点云
        for (const auto &f : local_map_frames_)
        {
            pcl::transformPointCloud(*f.cloud_data.cloud_ptr, *transformed_cloud_ptr, f.pose);
            *local_map_ += *transformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        // 更新下一次 ndt 匹配使用的初始点云
        if (local_map_frames_.size() < 10)
        {
            ndt_->setInputTarget(local_map_);
        }
        else
        {
            // 点云较多，先进行素体滤波
            sensor::PointCloud filtered_local_map;
            local_map_filter_.setInputCloud(local_map_);
            local_map_filter_.filter(*filtered_local_map.cloud_ptr);
            ndt_->setInputTarget(filtered_local_map.cloud_ptr);
        }

        // 更新 global map
        global_map_frames_.push_back(key_frame);
        if (global_map_frames_.size() % 100 == 0)
        {
            global_map_ = std::make_unique<sensor::PointCloud::CloudType>();
            for (const auto &f : global_map_frames_)
            {
                pcl::transformPointCloud(*f.cloud_data.cloud_ptr, *transformed_cloud_ptr, f.pose);
                *global_map_ += *transformed_cloud_ptr;
            }
            has_new_global_map_ = true;
        }
    }

    std::shared_ptr<sensor::PointCloud::CloudType> LocalSlam::GetNewLocalMap()
    {
        if (has_new_local_map_)
        {
            auto result = std::make_shared<sensor::PointCloud::CloudType>();
            display_filter_.setInputCloud(local_map_);
            display_filter_.filter(*result);
            return result;
        }
        return nullptr;
    }
    std::shared_ptr<sensor::PointCloud::CloudType> LocalSlam::GetNewGlobalMap()
    {
        if (has_new_global_map_)
        {
            auto result = std::make_shared<sensor::PointCloud::CloudType>();
            display_filter_.setInputCloud(global_map_);
            display_filter_.filter(*result);
            return result;
        }
        return nullptr;
    }
    std::shared_ptr<sensor::PointCloud::CloudType> LocalSlam::GetCurrentScan()
    {
        auto result = std::make_shared<sensor::PointCloud::CloudType>();
        display_filter_.setInputCloud(result_cloud_);
        display_filter_.filter(*result);
        return result;
    }

} // namespace toy::mapping