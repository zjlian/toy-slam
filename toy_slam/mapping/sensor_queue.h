#pragma once

#include "toy_slam/sensor/GNSS.h"
#include "toy_slam/sensor/IMU.h"
#include "toy_slam/sensor/PointCloud.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <type_traits>

namespace toy::mapping
{

    class SensorQueue
    {
    public:
        template <typename T>
        void AddData(std::unique_ptr<T> data)
        {
            std::lock_guard<std::mutex> lock{mutex_};
            if constexpr (std::is_same_v<T, sensor::GNSS>)
            {
                gnss_.push_back(std::move(data));
            }
            else if constexpr (std::is_same_v<T, sensor::IMU>)
            {
                imu_.push_back(std::move(data));
            }
            else if constexpr (std::is_same_v<T, sensor::PointCloud>)
            {
                pointcloud_.push_back(std::move(data));
            }
            else
            {
                assert(false && "不支持的传感器类型");
            }
        }

        template <typename T>
        void ParseData(std::deque<T> &result)
        {
            auto move_to_back = [](auto &target, auto &source) {
                std::move(
                    std::make_move_iterator(source.begin()),
                    std::make_move_iterator(source.end()),
                    std::back_inserter(target));
            };

            std::lock_guard<std::mutex> lock{mutex_};
            if constexpr (std::is_same_v<typename T::element_type, sensor::GNSS>)
            {
                move_to_back(result, gnss_);
                gnss_.clear();
            }
            else if constexpr (std::is_same_v<typename T::element_type, sensor::IMU>)
            {
                move_to_back(result, imu_);
                imu_.clear();
            }
            else if constexpr (std::is_same_v<typename T::element_type, sensor::PointCloud>)
            {
                move_to_back(result, pointcloud_);
                pointcloud_.clear();
            }
            else
            {
                assert(false && "不支持的传感器类型");
            }
        }

    private:
        std::mutex mutex_;
        std::deque<std::unique_ptr<sensor::GNSS>> gnss_;
        std::deque<std::unique_ptr<sensor::IMU>> imu_;
        std::deque<std::unique_ptr<sensor::PointCloud>> pointcloud_;
    };

} // namespace toy::mapping