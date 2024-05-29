/*
 * Package:   pid_controller
 * Filename:  PidControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include "pid_controller/PidControllerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "navigator_msgs/srv/retrievestatus.hpp"
#include "diagnostic_msgs/msg/diagnosticstatus.hpp"
#include "diagnostic_msgs/msg/keyvalue.hpp"

#include <memory>
#include <utility>
#include <time.h>

using namespace Voltron::PidController;

const float max_steering_angle_radians = 0.35;

PidControllerNode::PidControllerNode() : rclcpp::Node("pid_controller") {
  this->statusBool = NULL;
  this->callback_count = -1;
  this->callback_arr[3] = {false};
  this->declare_parameter("KP");
  this->declare_parameter("KI");
  this->declare_parameter("KD");
  this->declare_parameter("time_delta_cap_seconds");
  float KP = this->get_parameter("KP").as_double();
  float KI = this->get_parameter("KI").as_double();
  float KD = this->get_parameter("KD").as_double();
  float time_delta_cap_seconds = this->get_parameter("time_delta_cap_seconds").as_double();
  this->controller = std::make_unique
    <PidController>(KP, KI, KD, time_delta_cap_seconds);
  this->steering_control_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("output", 8);
  this->diagnostic_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>
    ("node_statuses", 10);
  this->command_subscription = this->create_subscription
    <std_msgs::msg::Float32>("target", 8,
    std::bind(& PidControllerNode::update_target, this, std::placeholders::_1));
  this->measurement_subscription = this->create_subscription
    <std_msgs::msg::Float32>("measurement", 8,
    std::bind(& PidControllerNode::update_measurement, this, std::placeholders::_1));
  this->last_update_time = this->clock.now();
  this->service = this->create_service<navigator_msgs::srv::RetrieveStatus>('retrieve_status',
  std::bind(& PidControllerNode::retrieve_status, this, std::placeholders::_1, std::placeholders::_2));
}

PidControllerNode::~PidControllerNode() {}

void PidControllerNode::update_target(
  const std_msgs::msg::Float32::SharedPtr command) {
  float target = command->data;
  if(target > max_steering_angle_radians) target = max_steering_angle_radians;
  if(target < (-1 * max_steering_angle_radians)) target = (-1 * max_steering_angle_radians);
  this->controller->set_target(target);
  this->recalculate_output();
  this->callback_count++;
  this->callback_arr[this->callback_count] = true;
}

void PidControllerNode::update_measurement(
  const std_msgs::msg::Float32::SharedPtr measurement) {
  this->controller->set_measurement(measurement->data);
  this->recalculate_output();
  this->callback_count++;
  this->callback_arr[this->callback_count] = true;
}

void PidControllerNode::recalculate_output() {
  clock_t::time_point now = this->clock.now();
  clock_t::duration time_delta = now - this->last_update_time;
  this->last_update_time = now;
  float time_delta_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(time_delta)
    .count(); // Convert to seconds float
  float steering_power = this->controller->compute(time_delta_seconds);
  auto message = std_msgs::msg::Float32();
  message.data = steering_power;
  this->steering_control_publisher->publish(message);
  this->callback_count++;
  this->callback_arr[this->callback_count] = true;
}

void PidControllerNode::retrieve_status(const std::shared_ptr<navigator_msgs::srv::RetrieveStatus::Response> response) {
  this->status = diagnostic_msgs::DiagnosticStatus node_status;
  this->stamp = diagnostic_msgs::KeyValue stamp_ID;

  this->status.name = 'Pid Controller';

  this->stamp_ID.key = 'stamp ID';
  this->stamp_ID.value = this->clock.now();

  this->status.values.push_back(this->stamp_ID);

  for (int i = 0; i < this->callback_count; i++) {
    if (this->callback_arr[i] == true) {
      this->statusBool = true;
    }
    else if (this->callback_arr[i] == false) {
      this->statusBool = false;
      break;
    }
  }

  if (this->statusBool = true) {
    this->status.level = diagnostic_msgs::DiagnosticStatus::OK;
    this->status.message = 'Node is FUNCTIONING properly!';
  }
  else if (this->statusBool = false) {
    this->status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    this->status.message = 'Node has encountered an ERROR!';
  }
  else if (this->statusBool = NULL) {
    this->status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    this->status.message = 'Status unknown...';
  }

  response->status = this->status;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request from Guardian to Pid Controller...");
  this->diagnostic_publisher->publish(this->status)
}
