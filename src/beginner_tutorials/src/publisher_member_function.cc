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
#include <string>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  MinimalPublisher() : Node("minimal_publisher"), count_(0), str_("Anirudh Swarankar ") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating topic /topic with queue " << 10 << " & service /change_msg");
    RCLCPP_WARN_STREAM(this->get_logger(), "Setting msg to Anirudh Swarankar");
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    service_ = this->create_service<beginner_tutorial_interfaces::srv::String>("/change_msg",
                                                                               std::bind(&MinimalPublisher::change_msg_callback,
                                                                                         this,
                                                                                         std::placeholders::_1,
                                                                                         std::placeholders::_2));
  }

 private:
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
