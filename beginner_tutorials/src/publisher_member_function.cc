/**
 * @file publisher_member_function.cc
 * @brief C++ ROS2 node
 * Copyright (c)
 * All rights reserved
 *
 * This file is part of the ENPM700 Assignments. Redistribution and use in
 * source and binary forms, with or without modification, are permitted
 * exclusively under the terms of the Apache-2.0 license.
 */
#include <beginner_tutorial_interfaces/srv/string.hpp>
#include <chrono>
#include <csignal>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief Minimal Publisher class to demonstrate basic publish and service
 * server
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher
   */
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0), str_("Anirudh Swarankar ") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating topic /topic with queue "
        << 10
        << " & service /change_msg");
    RCLCPP_WARN_STREAM(this->get_logger(), "Setting msg to Anirudh Swarankar");

    // Param handling
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set publish frequency";
    this->declare_parameter("freq", 10.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_callback_ = param_subscriber_->add_parameter_callback(
        "freq",
        std::bind(&MinimalPublisher::param_cb, this, std::placeholders::_1));
    // Publisher object
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int) (1000 / param.as_double())),
        std::bind(&MinimalPublisher::timer_callback, this));
    // Service server
    service_ = this->create_service<beginner_tutorial_interfaces::srv::String>(
        "/change_msg", std::bind(&MinimalPublisher::change_msg_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_tf();
  }

 private:

  void pub_tf() {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";
    t.transform.translation.x = 10.0;
    t.transform.translation.y = -10.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 30);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
  /**
   * @brief Callback function executed when the param is set externally via cli
   * or other modules
   * @param param parameter
   */
  void param_cb(const rclcpp::Parameter &param) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter "
        << param.get_name().c_str()
        << " of type "
        << param.get_type_name().c_str()
        << " -> " << param.as_double());

    auto period = std::chrono::milliseconds((int) (1000 / param.as_double()));
    // replacing timer with new frequency
    timer_ = this->create_wall_timer(
        period, std::bind(&MinimalPublisher::timer_callback, this));
  }

  /**
   * @brief Service server callback. In request body a string is received
   * @param req Request
   * @param resp Response
   */
  void change_msg_callback(
      const std::shared_ptr<beginner_tutorial_interfaces::srv::String_Request>
      req,
      std::shared_ptr<beginner_tutorial_interfaces::srv::String_Response>
      resp) {
    try {
      str_ = req->data;
      resp->response = true;
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Changed to " << str_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed setting string to" << req->data);
      resp->response = false;
    }
  }

  /**
   * @brief Timer function to publish message periodically
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = str_ + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publish: " << message.data.c_str());
    publisher_->publish(message);
  }

  /**
   * @brief Timer object
   */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Publisher object
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /**
   * @brief Timer object
   */
  rclcpp::Service<beginner_tutorial_interfaces::srv::String>::SharedPtr
      service_;
  size_t count_;
  std::string str_;
  /**
   * @brief Param event handler object
   */
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  /**
   * @brief Param callback handler object
   */
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_;
  /**
   * @brief Tf2_ros object
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, [](int sig) {
    RCLCPP_FATAL(rclcpp::get_logger("talker"),
                 "Ctrl+C pressed, shutting down node");
    rclcpp::shutdown();
  });
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  return 0;
}
