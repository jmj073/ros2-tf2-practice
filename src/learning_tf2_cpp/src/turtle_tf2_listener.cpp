// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "turtle2");
    forward_speed_ = this->declare_parameter<double>("forward_speed", 0.5);
    rotation_rate_ = this->declare_parameter<double>("rotation_rate", 1.0);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(source_frame_ + "/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      500ms, std::bind(&FrameListener::on_timer, this));
  }

private: // member function
  void on_timer()
  {
    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        publish_transform();
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");
        turtle_spawned_ = true;
      }
    } else {
      // Check if the service is ready
      if (spawner_->service_is_ready()) {
        send_spawn_request();
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }

  void publish_transform()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    const auto& fromFrameRel = target_frame_;
    const auto& toFrameRel = source_frame_;

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Twist msg;

    msg.angular.z = rotation_rate_ * atan2(
      t.transform.translation.y,
      t.transform.translation.x);

    msg.linear.x = forward_speed_ * sqrt(
      pow(t.transform.translation.x, 2) +
      pow(t.transform.translation.y, 2));

    publisher_->publish(msg);
  }

  void send_spawn_request()
  {
    // Initialize request with turtle name and coordinates
    // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = source_frame_;
    request->x = 4.0;
    request->y = 2.0;
    request->theta = 0.0;

    // Call request
    using ServiceResponseFuture =
      rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        if (result->name == source_frame_) {
          turtle_spawning_service_ready_ = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
        }
      };

    auto result = spawner_->async_send_request(request, response_received_callback);
  }

private: // member variable
  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string source_frame_;
  double forward_speed_ = 0.5;
  double rotation_rate_ = 1.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
