#include "GNSS.h"
#include <cassert>

namespace toy::sensor
{
    /// 初始化 GNSS 的原点坐标
    void InitGNSSOriginPosition(const GNSS &val)
    {
        GNSS::_geo_converter_.Reset(val.latitude, val.longitude, val.altitude);

        GNSS::origin_longitude = val.longitude;
        GNSS::origin_latitude = val.latitude;
        GNSS::origin_altitude = val.altitude;

        GNSS::_origin_position_inited_ = true;
    }

    /// 更新 GNSS 坐标
    void UpdateGNSSXYZ(GNSS &val)
    {
        assert(GNSS::_origin_position_inited_ && "GeoConverter must set origin position");
        GNSS::_geo_converter_.Forward(
            val.latitude, val.longitude, val.altitude,
            val.local_E, val.local_N, val.local_U);
    }

    /// 同步 GNSS 数据
    bool SyncGNSSData(std::deque<GNSS> &unsynced, std::deque<GNSS> &synced, double sync_time)
    {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值

        // while (unsynced.size() >= 2)
        // {
        //     if (unsynced.front().time > sync_time)
        //     {
        //         return false;
        //     }
        // }
        return false;
    }

} // namespace toy::sensor