#include "ekf_localization/localization_node.h"
#include <cmath>

LocalizationNode::LocalizationNode()
    : Node("localization_node"),
    ekf_(Eigen::VectorXd::Zero(10), //initState
        Eigen::MatrixXd::Identity(10, 10), //initCovariance
        Eigen::MatrixXd::Identity(6, 6), //mesasurement noise
        Eigen::MatrixXd::Identity(10, 10)), //process noise
    lastImuTime_(this->get_clock()->now()),
    originSet_(false),
    refLat_(0.0), refLon_(0.0), refAlt_(0.0)
    {
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
        /*gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&LocalizationNode::gpsCallback, this, std::placeholders::_1));*/
        posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", 10);

        this->declare_parameter("gravity", 9.81);
        //this->declare_parameter("use_sim_time", true);
        RCLCPP_INFO(this->get_logger(), "Localization Node has been initialized");
    }

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    float dt = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - 
                (lastImuTime_.seconds() + lastImuTime_.nanoseconds() * 1e-9);

    lastImuTime_ = msg->header.stamp;


    Eigen::Vector3d imuAngularVelocity = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d imuLinearAcceleration = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    ekf_.predict(dt, imuLinearAcceleration, imuAngularVelocity, this->get_parameter("gravity").as_double());

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "map";
    Eigen::VectorXf state = ekf_.getState().cast<float>();
    pose_msg.pose.position.x = state[0];
    pose_msg.pose.position.y = state[1];
    pose_msg.pose.position.z = state[2];
    pose_msg.pose.orientation.w = state[6];
    pose_msg.pose.orientation.x = state[7];
    pose_msg.pose.orientation.y = state[8];
    pose_msg.pose.orientation.z = state[9];
    posePub_->publish(pose_msg);
}