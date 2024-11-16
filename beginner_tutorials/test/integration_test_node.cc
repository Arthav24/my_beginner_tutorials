//
// Created by arthavnuc on 11/15/24.
//
/**
 * @file integration test node.cc
 * @brief C++ ROS2 node
 * Copyright (c)
 * All rights reserved
 *
 * This file is part of the ENPM700 Assignments. Redistribution and use in
 * source and binary forms, with or without modification, are permitted
 * exclusively under the terms of the Apache-2.0 license.
 */
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>

auto Logger = rclcpp::get_logger(""); // create an initial Logger
class TestFixture {
 public:
  TestFixture() {
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode");
    Logger = testerNode->get_logger();
  }

  ~TestFixture() {
  }

 protected:
  rclcpp::Node::SharedPtr testerNode;
};

TEST_CASE_METHOD (TestFixture, "Talker node publishes messages", "[integration][talker]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
  std::string received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = testerNode->create_subscription<std_msgs::msg::String>(
      "/topic",
      10,
      [&received_message, &message_received, this](const std_msgs::msg::String::SharedPtr msg) {
        received_message = msg->data;
        message_received = true;
        RCLCPP_INFO_STREAM(testerNode->get_logger(), "RCVD -->" << received_message);
      });

// Add the subscription node to the executor
  executor->
      add_node(testerNode);


// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(2)) {
    executor->
        spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->
      cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(message_received); // Ensure a message was received
  REQUIRE(received_message.find("Anirudh Swarankar")
              != std::string::npos); // Check if the received message matches expectation

}