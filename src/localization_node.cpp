#include "ekf_localization/localization_node.h"
#include "ekf_localization/utilities/wgs84ToNed.h"
#include <cmath>

LocalizationNode::LocalizationNode()
    : Node("localization_node"),
    ekf_(
        (Eigen::VectorXd(10) << 0, 0, 0.5, 0, 0, 0, 1, 0, 0, 0).finished(), //initState
        (Eigen::VectorXd(10) << 1, 1, 1, 0.1, 0.1, 0.1, 1, 0.1, 0.1, 0.1).finished().asDiagonal(), //initCovariance
        (Eigen::VectorXd(10) << 0.3, 0.3, 0.3, 0.01, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6).finished().asDiagonal(), //mesasurement noise
        (Eigen::VectorXd(10) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-8, 1e-8, 1e-8, 1e-8).finished().asDiagonal()), //process noise
    lastImuTime_(this->get_clock()->now()),
    originSet_(false),
    refLat_(47.47894999999999), refLon_(19.057784999999996), refAlt_(0.054999155923724174)
    {
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
        gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "navsat", 10, std::bind(&LocalizationNode::gpsCallback, this, std::placeholders::_1));
        posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "drone/pose", 10);
        velPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "drone/velocity", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&LocalizationNode::timerCallback, this));

        this->declare_parameter("gravity", 9.81);
        //this->declare_parameter("use_sim_time", true);
        

        RCLCPP_INFO(this->get_logger(), "Localization Node has been initialized");

        // Eigen::VectorXd initialState = ekf_.getState();
        // RCLCPP_INFO(this->get_logger(), "EKF initialized - State: x=%f, y=%f, z=%f", 
        //             initialState[0], initialState[1], initialState[2]);
        // RCLCPP_INFO(this->get_logger(), "Localization Node has been initialized");
    }

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    double currTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double lastTime = lastImuTime_.seconds() + lastImuTime_.nanoseconds() * 1e-9;
    float dt = currTime - lastTime;
    if (dt <= 0) dt = 0.001;

    lastImuTime_ = msg->header.stamp;


    Eigen::Vector3d imuAngularVelocity = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d imuLinearAcceleration = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    this->imuOrientation = Eigen::Vector4d(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //RCLCPP_INFO(this->get_logger(), "Z: %f",msg->linear_acceleration.z);
    ekf_.predict(dt, imuLinearAcceleration, imuAngularVelocity, -this->get_parameter("gravity").as_double());
}

void LocalizationNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    WGS84toNED converter(this->refLat_, this->refLon_, this->refAlt_);

    if (!this->originSet_){
        this->originSet_ = true;
        Eigen::VectorXd resetState(10);
        Eigen::Vector3d NED = converter.convertToNed(msg->latitude, msg->longitude, msg->altitude);
        resetState << NED[0], NED[1], NED[2], 0, 0, 0, 1, 0, 0, 0; // Position from GPS, zero velocity, identity quaternion
        ekf_.setState(resetState);
        originSet_ = true;
        RCLCPP_INFO(this->get_logger(), "EKF reset with first GPS - State: x=%f, y=%f, z=%f", 
                    NED[0], NED[1], NED[2]);
    }

    if (originSet_){
    Eigen::Vector3d NED = converter.convertToNed(msg->latitude, msg->longitude, msg->altitude);
    // RCLCPP_INFO(this->get_logger(), "GPS - Lat: %f, Lon: %f, Alt: %f, NED: x=%f, y=%f, z=%f", 
    //                 msg->latitude, msg->longitude, msg->altitude, NED[0], NED[1], NED[2]);
    Eigen::VectorXd measurement(10);
    
    measurement << NED[0], NED[1], NED[2], 0, 0, 0,
            this->imuOrientation[0], this->imuOrientation[1], this->imuOrientation[2], this->imuOrientation[3];

    ekf_.update(measurement);
    // Eigen::VectorXd updatedState = ekf_.getState();
    // RCLCPP_INFO(this->get_logger(), "After update - State: x=%f, y=%f, z=%f", 
    //                 updatedState[0], updatedState[1], updatedState[2]);
    }
}

void LocalizationNode::timerCallback(){

    // auto now = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Current clock time: %.6f (sec), %09ld (nsec)", 
    //             now.seconds(), now.nanoseconds());
    // RCLCPP_INFO(this->get_logger(), "Localization Node is Publishing Pose"); 

    RCLCPP_INFO(this->get_logger(), "Localization Node is Publishing Pose");
    auto poseMsg = geometry_msgs::msg::PoseStamped();
    poseMsg.header.stamp = this->get_clock()->now();
    poseMsg.header.frame_id = "map";

    auto velMsg = geometry_msgs::msg::TwistStamped();
    velMsg.header.stamp = this->get_clock()->now();
    velMsg.header.frame_id = "map";


    Eigen::VectorXf state = ekf_.getState().cast<float>();

    poseMsg.pose.position.x = state[0];
    poseMsg.pose.position.y = state[1];
    poseMsg.pose.position.z = state[2];
    poseMsg.pose.orientation.w = state[6];
    poseMsg.pose.orientation.x = state[7];
    poseMsg.pose.orientation.y = state[8];
    poseMsg.pose.orientation.z = state[9];

    velMsg.twist.linear.x = state[3];
    velMsg.twist.linear.y = state[4];
    velMsg.twist.linear.z = state[5];

    posePub_->publish(poseMsg);
    velPub_->publish(velMsg);
}
