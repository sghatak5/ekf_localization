#ifndef LOCALIZATION_NODE_H_
#define LOCALIZATION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ekf.h"

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    ExtendedKalmanFilter ekf_;
    rclcpp::Time lastImuTime_;
    bool originSet_;
    double refLat_, refLon_, refAlt_;
    Eigen::Vector4d imuOrientation;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void timerCallback();
};


#endif
