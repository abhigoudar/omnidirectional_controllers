#pragma once

#include <chrono>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <omnidirectional_controllers/types.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace omnidirectional_controllers {
    class PositionController{
        public:
        //
        PositionController(const std::string&);
        //
        ~PositionController();
        //
        void runController();
        //
        private:
        //
        YAML::Node controller_config;
        YAML::Node robot_config;
        //
        rclcpp::Node::SharedPtr node;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sp_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_sp_pub;
        //
        void initialize();
        void loadConfig();
        void poseSetpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr);
        void update();
        //
        nav_msgs::msg::Odometry latest_odom_msg;
        geometry_msgs::msg::PoseStamped latest_pose_sp;
        geometry_msgs::msg::Twist twist_sp;
        int64_t loop_dur_us;
        PGains gains;
        //
    };
}