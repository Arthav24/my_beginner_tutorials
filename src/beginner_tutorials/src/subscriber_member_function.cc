/**
* @file subscriber_member_function.cc
* @brief C++ ROS2 node
* @author Anirudh Swarankar
* Copyright (c)
* All rights reserved
*
* This file is part of the ENPM700 Assignments. Redistribution and use in source and
* binary forms, with or without modification, are permitted exclusively
* under the terms of the Apache-2.0 license.
*/
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
* @brief Minimal Subscriber class it inherits rclcpp node
*/
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief Topic callback for topic /topic
   * @param msg msg
   */
  void topic_callback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
