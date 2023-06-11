#pragma once

#include <deque>

#include "GeographicLib/LocalCartesian.hpp"

namespace toy::sensor
{

    struct GNSS
    {
        inline static double origin_longitude = 0;
        inline static double origin_latitude = 0;
        inline static double origin_altitude = 0;

        double time = 0.0;
        double longitude = 0.0; // 经度
        double latitude = 0.0;  // 纬度
        double altitude = 0.0;  // 海拔
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        int status = 0;
        int service = 0;

        inline static GeographicLib::LocalCartesian _geo_converter_{};
        inline static bool _origin_position_inited_ = false;
    };

    /// 初始化 GNSS 的原点坐标
    void InitGNSSOriginPosition(const GNSS &val);

    /// 更新 GNSS 坐标
    void UpdateGNSSXYZ(GNSS &val);

    /// 同步 GNSS 数据
    bool SyncGNSSData(std::deque<GNSS> &unsynced, std::deque<GNSS> &synced, double sync_time);

} // namespace toy::sensor