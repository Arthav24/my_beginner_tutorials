/**
 * @file publisher_member_function.cc
 * @brief C++ ROS2 node
 * Copyright (c)
 * All rights reserved
 *
 * This file is part of the ENPM700 Assignments. Redistribution and use in source and
 * binary forms, with or without modification, are permitted exclusively
 * under the terms of the Apache-2.0 license.
*/
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <beginner_tutorial_interfaces/srv/string.hpp>
#include <string>
#include <csignal>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  MinimalPublisher() : Node("minimal_publisher"), count_(0), str_("Anirudh Swarankar ") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating topic /topic with queue " << 10 << " & service /change_msg");
    RCLCPP_WARN_STREAM(this->get_logger(), "Setting msg to Anirudh Swarankar");

    // Param handling
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set publish frequency";
    this->declare_parameter("freq", 10.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_callback_ = param_subscriber_->add_parameter_callback("freq",
                                                                std::bind(&MinimalPublisher::param_cb,
                                                                          this,
                                                                          std::placeholders::_1));
    // Publisher object
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int) (1000 / param.as_double())),
        std::bind(&MinimalPublisher::timer_callback, this));
    // Service server
    service_ = this->create_service<beginner_tutorial_interfaces::srv::String>("/change_msg",
                                                                               std::bind(&MinimalPublisher::change_msg_callback,
                                                                                         this,
                                                                                         std::placeholders::_1,
                                                                                         std::placeholders::_2));
  }

 private:
  /**
   * @brief Callback function executed when the param is set externally via cli or other modules
   * @param param parameter
   */
  void param_cb(const rclcpp::Parameter &param) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received an update to parameter " << param.get_name().c_str() << " of type "
                                                          << param.get_type_name().c_str() << " -> "
                                                          << param.as_double());

    auto period = std::chrono::milliseconds((int) (1000 / param.as_double()));
    // replacing timer with new frequency
    timer_ = this->create_wall_timer(period, std::bind(&MinimalPublisher::timer_callback, this));
  }

  /**
   * @brief Service server callback. In request body a string is received
   * @param req Request
   * @param resp Response
   */
  void change_msg_callback(const std::shared_ptr<beginner_tutorial_interfaces::srv::String_Request> req,
                           std::shared_ptr<beginner_tutorial_interfaces::srv::String_Response> resp) {
    try {
      str_ = req->data;
      resp->response = true;
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Changed to " << str_);
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed setting string to" << req->data);
      resp->response = false;
    }
  }
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = str_ + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publish: " << message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorial_interfaces::srv::String>::SharedPtr service_;
  size_t count_;
  std::string str_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, [](int sig) {
    RCLCPP_FATAL(rclcpp::get_logger("talker"), "Ctrl+C pressed, shutting down node");
    rclcpp::shutdown();

  });
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  return 0;
}
