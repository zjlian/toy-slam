#include "ros/publisher.h"
#include "ros/time.h"
#include "toy_slam/mapping/map_builder.h"
#include "toy_slam/sensor/GNSS.h"
#include "toy_slam/sensor/IMU.h"
#include "toy_slam/sensor/PointCloud.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/exceptions.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <spdlog/spdlog.h>

using namespace toy;

class TFListener
{
public:
    TFListener(ros::NodeHandle &nh, std::string_view base_frame_id, std::string_view child_frame_id)
        : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id)
    {
    }

    bool LookupData(Eigen::Matrix4f &transform_matrix)
    {
        try
        {
            tf::StampedTransform transform;
            listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
            TransformToMatrix(transform, transform_matrix);
            return true;
        }
        catch (tf::TransformException &ex)
        {
            return false;
        }
    }

private:
    bool TransformToMatrix(tf::StampedTransform transform, Eigen::Matrix4f &transform_matrix)
    {
        Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

        // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        return true;
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};

class Node
{
public:
    Node(ros::NodeHandle &nh)
        : nh_(nh)
    {
        SubscriberSensor();
        InitPublisher();
        Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
        while (!lidar_to_imu_ptr->LookupData(lidar_to_imu) && ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        map_builder_ = std::make_unique<mapping::MapBuilder>();
        spdlog::info("启动测试");
        auto gnss_pub = [this](Eigen::Matrix4f &transform_matrix) {
            nav_msgs::Odometry odom;
            odom.header.frame_id = "map";
            odom.child_frame_id = "lidar";
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = transform_matrix(0, 3);
            odom.pose.pose.position.y = transform_matrix(1, 3);
            odom.pose.pose.position.z = transform_matrix(2, 3);

            Eigen::Quaternionf q;
            q = transform_matrix.block<3, 3>(0, 0);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            pub_.at("odom").publish(odom);
        };

        auto pc_pub = [this](sensor::PointCloud &pc) {
            auto output = std::make_shared<sensor_msgs::PointCloud2>();
            pcl::toROSMsg(*pc.cloud_ptr, *output);
            output->header.stamp = ros::Time::now();
            output->header.frame_id = "/map";
            pub_.at("map").publish(*output);
        };
        map_builder_->Test(lidar_to_imu, pc_pub, gnss_pub);
    }

private:
    void SubscriberSensor()
    {
        sub_["pointcloud"] = nh_.subscribe("/kitti/velo/pointcloud", 1000, &Node::HandlePointCloud, this);
        sub_["imu"] = nh_.subscribe("/kitti/oxts/imu", 1000, &Node::HandleIMU, this);
        sub_["gnss"] = nh_.subscribe("/kitti/oxts/gps/fix", 1000, &Node::HandleGNSS, this);

        lidar_to_imu_ptr = std::make_unique<TFListener>(nh_, "velo_link", "imu_link");
    }

    void InitPublisher()
    {
        pub_["map"] = nh_.advertise<sensor_msgs::PointCloud2>("current_scan", 100);
        pub_["scan"] = pub_["map"];
        pub_["local"] = nh_.advertise<sensor_msgs::PointCloud2>("local_map", 100);
        pub_["global"] = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 100);
        pub_["odom"] = nh_.advertise<nav_msgs::Odometry>("lidar_odom", 100);
        pub_["gnss"] = nh_.advertise<nav_msgs::Odometry>("gnss", 100);
    }

    void HandlePointCloud(sensor_msgs::PointCloud2::ConstPtr ptr)
    {
        spdlog::debug("收到 PointCloud {}", ptr->header.seq);
        auto data = std::make_unique<sensor::PointCloud>();
        data->time = ptr->header.stamp.toSec();
        pcl::fromROSMsg(*ptr, *(data->cloud_ptr));
        map_builder_->GetSensorQueue()->AddData(std::move(data));
    }

    void HandleGNSS(sensor_msgs::NavSatFix::ConstPtr ptr)
    {
        spdlog::debug("收到 gnss {}", ptr->header.seq);
        auto data = std::make_unique<sensor::GNSS>();
        data->time = ptr->header.stamp.toSec();
        data->latitude = ptr->latitude;
        data->longitude = ptr->longitude;
        data->altitude = ptr->altitude;
        data->status = ptr->status.status;
        data->service = ptr->status.service;
        map_builder_->GetSensorQueue()->AddData(std::move(data));
    }

    void HandleIMU(sensor_msgs::Imu::ConstPtr ptr)
    {
        spdlog::debug("收到 imu {}", ptr->header.seq);
        auto data = std::make_unique<sensor::IMU>();
        data->time = ptr->header.stamp.toSec();

        data->linear_acceleration.x = ptr->linear_acceleration.x;
        data->linear_acceleration.y = ptr->linear_acceleration.y;
        data->linear_acceleration.z = ptr->linear_acceleration.z;

        data->angular_velocity.x = ptr->angular_velocity.x;
        data->angular_velocity.y = ptr->angular_velocity.y;
        data->angular_velocity.z = ptr->angular_velocity.z;

        data->orientation.x = ptr->orientation.x;
        data->orientation.y = ptr->orientation.y;
        data->orientation.z = ptr->orientation.z;
        data->orientation.w = ptr->orientation.w;

        map_builder_->GetSensorQueue()->AddData(std::move(data));
    }

private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Subscriber> sub_;
    std::map<std::string, ros::Publisher> pub_;
    std::unique_ptr<TFListener> lidar_to_imu_ptr;

    std::unique_ptr<mapping::MapBuilder> map_builder_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "toy_slam");
    ros::NodeHandle nh;

    Node node{nh};

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
