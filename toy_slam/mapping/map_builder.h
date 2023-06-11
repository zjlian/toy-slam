#pragma once

#include "toy_slam/mapping/local_slam.h"
#include "toy_slam/mapping/sensor_queue.h"
#include "toy_slam/sensor/GNSS.h"
#include "toy_slam/sensor/IMU.h"
#include "toy_slam/sensor/PointCloud.h"

#include <functional>
#include <future>
#include <memory>
#include <thread>

#include "ros/init.h"
#include "ros/rate.h"
#include <pcl/common/transforms.h>

namespace toy::mapping
{

    class MapBuilder
    {
    public:
        MapBuilder() = default;

        auto GetSensorQueue()
        {
            return sensor_;
        }

        void Test(Eigen::Matrix4f lidar_to_imu, std::function<void(sensor::PointCloud &)> pc_pub, std::function<void(Eigen::Matrix4f &)> gnss_pub)
        {
            auto result = std::async([=, this] {
                std::deque<std::unique_ptr<sensor::PointCloud>> cloud_data_buff;
                std::deque<std::unique_ptr<sensor::IMU>> imu_data_buff;
                std::deque<std::unique_ptr<sensor::GNSS>> gnss_data_buff;
                bool transform_received = false;
                bool gnss_origin_position_inited = false;

                ros::Rate rate(100);
                while (ros::ok())
                {
                    rate.sleep();
                    ros::spinOnce();
                    sensor_->ParseData(cloud_data_buff);
                    sensor_->ParseData(imu_data_buff);
                    sensor_->ParseData(gnss_data_buff);

                    while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0)
                    {
                        auto cloud = *cloud_data_buff.front();
                        auto imu = *imu_data_buff.front();
                        auto gnss = *gnss_data_buff.front();

                        double d_time = cloud.time - imu.time;
                        if (d_time < -0.05)
                        {
                            cloud_data_buff.pop_front();
                        }
                        else if (d_time > 0.05)
                        {
                            imu_data_buff.pop_front();
                            gnss_data_buff.pop_front();
                        }
                        else
                        {
                            imu_data_buff.pop_front();
                            gnss_data_buff.pop_front();
                            cloud_data_buff.pop_front();

                            Eigen::Matrix4f odometry_matrix;

                            if (!gnss_origin_position_inited)
                            {
                                sensor::InitGNSSOriginPosition(gnss);
                                gnss_origin_position_inited = true;
                            }

                            sensor::UpdateGNSSXYZ(gnss);
                            odometry_matrix(0, 3) = gnss.local_E;
                            odometry_matrix(1, 3) = gnss.local_N;
                            odometry_matrix(2, 3) = gnss.local_U;
                            odometry_matrix.block<3, 3>(0, 0) = sensor::GetOrientationMatrix(imu);
                            odometry_matrix *= lidar_to_imu;

                            pcl::transformPointCloud(*cloud.cloud_ptr, *cloud.cloud_ptr, odometry_matrix);
                            pc_pub(cloud);
                            gnss_pub(odometry_matrix);
                        }
                    }
                }
            });
            result.wait();
        }

    private:
        std::shared_ptr<SensorQueue> sensor_{std::make_shared<SensorQueue>()};
        std::shared_ptr<LocalSlam> local_slam_{std::make_shared<LocalSlam>()};
    };

} // namespace toy::mapping