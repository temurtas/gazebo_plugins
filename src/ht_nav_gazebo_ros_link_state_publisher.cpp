// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/ht_nav_gazebo_ros_link_state_publisher.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class HTNavGazeboRosLinkStatePublisherPrivate
{
public:
  /// Indicates which link
  enum
  {
    FRONT_RIGHT, // Front right wheel
    FRONT_LEFT,  // Front left wheel
    REAR_RIGHT,  // Rear right wheel
    REAR_LEFT,   // Rear left wheel
    IMU_LINK     // Steering wheel
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);
  void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link);
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Joint state publisher.
  std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 5> publishers;

  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr front_right_wheel_state_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr front_left_wheel_state_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rear_right_wheel_state_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rear_left_wheel_state_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr imu_link_state_pub_;

  // publishers[0] = front_right_wheel_state_pub_;
  // publishers[1] = front_left_wheel_state_pub_;
  // publishers[2] = rear_right_wheel_state_pub_;
  // publishers[3] = rear_left_wheel_state_pub_;
  // publishers[4] = imu_link_state_pub_;

  /// Joints being tracked.
  std::vector<gazebo::physics::LinkPtr> links_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  int data_counter_ = 0;

};

HTNavGazeboRosLinkStatePublisher::HTNavGazeboRosLinkStatePublisher()
: impl_(std::make_unique<HTNavGazeboRosLinkStatePublisherPrivate>())
{
}

HTNavGazeboRosLinkStatePublisher::~HTNavGazeboRosLinkStatePublisher()
{
}

void HTNavGazeboRosLinkStatePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  impl_->links_.resize(5);

  auto front_right_wheel =
    sdf->Get<std::string>("front_right_wheel", "front_right_wheel").first;
  impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_RIGHT] =
    model->GetLink(front_right_wheel);
  if (!impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_RIGHT]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Front Right Wheel [%s] not found.", front_right_wheel.c_str());
    impl_->ros_node_.reset();
  }

  auto front_left_wheel =
    sdf->Get<std::string>("front_left_wheel", "front_left_wheel").first;
  impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_LEFT] =
    model->GetLink(front_left_wheel);
  if (!impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_LEFT]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Front Left Wheel [%s] not found.", front_left_wheel.c_str());
    impl_->ros_node_.reset();
  }

  auto rear_right_wheel =
    sdf->Get<std::string>("rear_right_wheel", "rear_right_wheel").first;
  impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::REAR_RIGHT] =
    model->GetLink(rear_right_wheel);
  if (!impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::REAR_RIGHT]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Rear Right Wheel [%s] not found.", rear_right_wheel.c_str());
    impl_->ros_node_.reset();
  }

  auto rear_left_wheel =
    sdf->Get<std::string>("rear_left_wheel", "rear_left_wheel").first;
  impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::REAR_LEFT] =
    model->GetLink(rear_left_wheel);
  if (!impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::REAR_LEFT]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Rear Left Wheel [%s] not found.", rear_left_wheel.c_str());
    impl_->ros_node_.reset();
  }

  auto imu_link =
    sdf->Get<std::string>("imu_link", "imu_link").first;
  impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::IMU_LINK] =
    model->GetLink(imu_link);
  if (!impl_->links_[HTNavGazeboRosLinkStatePublisherPrivate::IMU_LINK]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "IMU Link [%s] not found.", imu_link.c_str());
    impl_->ros_node_.reset();
  }

  // Update rate
  double update_rate = 100.0;
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f", update_rate);
  } else {
    update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->last_update_time_ = model->GetWorld()->SimTime();

  // Link state publishers
  // impl_->front_right_wheel_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
  //   "front_right_link_states", qos.get_publisher_qos("front_right_link_states", rclcpp::QoS(1000)));
  // impl_->front_left_wheel_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
  //     "front_left_link_states", qos.get_publisher_qos("front_left_link_states", rclcpp::QoS(1000)));
  // impl_->rear_right_wheel_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
  //     "rear_right_link_states", qos.get_publisher_qos("rear_right_link_states", rclcpp::QoS(1000)));
  // impl_->rear_left_wheel_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
  //     "rear_left_link_states", qos.get_publisher_qos("rear_left_link_states", rclcpp::QoS(1000)));
  // impl_->imu_link_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
  //     "imu_link_states", qos.get_publisher_qos("imu_link_states", rclcpp::QoS(1000)));

    // FRONT_RIGHT, // Front right wheel  // front_right_link_states
    // FRONT_LEFT,  // Front left wheel   // front_left_link_states
    // REAR_RIGHT,  // Rear right wheel   // rear_right_link_states
    // REAR_LEFT,   // Rear left wheel    // rear_left_link_states
    // IMU_LINK     // Steering wheel     // imu_link_states
    
  impl_->publishers[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_RIGHT] = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "front_right_link_states", qos.get_publisher_qos("front_right_link_states", rclcpp::QoS(1000)));
  impl_->publishers[HTNavGazeboRosLinkStatePublisherPrivate::FRONT_LEFT] = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "front_left_link_states", qos.get_publisher_qos("front_left_link_states", rclcpp::QoS(1000)));
  impl_->publishers[HTNavGazeboRosLinkStatePublisherPrivate::REAR_RIGHT] = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "rear_right_link_states", qos.get_publisher_qos("rear_right_link_states", rclcpp::QoS(1000)));
  impl_->publishers[HTNavGazeboRosLinkStatePublisherPrivate::REAR_LEFT] = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "rear_left_link_states", qos.get_publisher_qos("rear_left_link_states", rclcpp::QoS(1000)));
  impl_->publishers[HTNavGazeboRosLinkStatePublisherPrivate::IMU_LINK] = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "imu_link_states", qos.get_publisher_qos("imu_link_states", rclcpp::QoS(1000)));    

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&HTNavGazeboRosLinkStatePublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void HTNavGazeboRosLinkStatePublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  int i = 0;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosLinkStatePublisherPrivate::OnUpdate");
#endif
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Populate message
  sensor_msgs::msg::JointState link_state;

  link_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  // link_state.name.resize(3);
  // link_state.position.resize(3);
  // link_state.velocity.resize(3);
  // link_state.effort.resize(3);

  // ignition::math::Vector3d velocity;
  // ignition::math::Pose3d position;
  // ignition::math::Quaternion quaternion;
  // double euler[3]; 

  gazebo::physics::LinkPtr link;

    // FRONT_RIGHT, // Front right wheel
    // FRONT_LEFT,  // Front left wheel
    // REAR_RIGHT,  // Rear right wheel
    // REAR_LEFT,   // Rear left wheel
    // IMU_LINK     // Steering wheel

  for (i = 0; i < (int)(links_.size()); i++)
  {
    link = links_[i];
    LinkStateSolver(&link_state, link);
    // Publish
    publishers[i]->publish(link_state);
  }
  // velocity = link->WorldLinearVel();        
  // position = link->WorldCoGPose();
  // quaternion = position.Rot();
  // euler[0] = quaternion.Roll();
  // euler[1] = quaternion.Pitch();
  // euler[2] = quaternion.Yaw();
  
  // link_state.name[0] = link->GetName();
  // link_state.name[1] = link->GetName();
  // link_state.name[2] = link->GetName();
  // link_state.position[0] = position.X();
  // link_state.position[1] = position.Y();
  // link_state.position[2] = position.Z();
  // link_state.velocity[0] = velocity.X();
  // link_state.velocity[1] = velocity.Y();
  // link_state.velocity[2] = velocity.Z();
  // link_state.effort[0] =  quaternion.Roll();
  // link_state.effort[1] =  quaternion.Pitch();
  // link_state.effort[2] =  quaternion.Yaw();

  // front_right_wheel_state_pub_;
  // front_left_wheel_state_pub_;
  // rear_right_wheel_state_pub_;
  // rear_left_wheel_state_pub_;
  // imu_link_state_pub_;

  // // Publish
  // publishers[0]->publish(link_state);

  if (data_counter_ % 100 == 0)
  {
  double yaw = link_state.effort[2] * 180.0 / 3.14;
  RCLCPP_INFO(
    ros_node_->get_logger(), "Yaw Angle [%f]", yaw );
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif


#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Update time
  last_update_time_ = current_time;
  data_counter_ += 1;
}


void HTNavGazeboRosLinkStatePublisherPrivate::LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link)
{
  link_state->name.resize(3);
  link_state->position.resize(3);
  link_state->velocity.resize(3);
  link_state->effort.resize(3);

  ignition::math::Vector3d velocity;
  ignition::math::Pose3d position;
  // ignition::math::Quaternion quaternion;
  // double euler[3]; 

  velocity = link->WorldLinearVel();        
  position = link->WorldCoGPose();
  // quaternion = position.Rot();
  // euler[0] = quaternion.Roll();
  // euler[1] = quaternion.Pitch();
  // euler[2] = quaternion.Yaw();
  
  link_state->name[0] = link->GetName();
  link_state->name[1] = link->GetName();
  link_state->name[2] = link->GetName();
  link_state->position[0] = position.X();
  link_state->position[1] = position.Y();
  link_state->position[2] = position.Z();
  link_state->velocity[0] = velocity.X();
  link_state->velocity[1] = velocity.Y();
  link_state->velocity[2] = velocity.Z();
  link_state->effort[0] =  position.Rot().Roll();
  link_state->effort[1] =  position.Rot().Pitch();
  link_state->effort[2] =  position.Rot().Yaw();

}


GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboRosLinkStatePublisher)
}  // namespace gazebo_plugins
