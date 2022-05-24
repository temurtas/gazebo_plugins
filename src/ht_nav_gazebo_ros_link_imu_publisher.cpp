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
#include <gazebo_plugins/ht_nav_gazebo_ros_link_imu_publisher.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class HTNavGazeboRosLinkIMUPublisherPrivate
{
public:
  /// Indicates which link
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);
  void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link);
  void Euler2Cnb(double c_nb[3][3], double euler_in[3]);
  void MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]);
  // void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link, double sim_time);
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  // sensor_msgs::msg::Imu::SharedPtr msg_;
  // ignition::math::Vector3d accelerometer_data{0, 0, 0};
  // ignition::math::Vector3d gyroscope_data{0, 0, 0};

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr imu_meas_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

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

HTNavGazeboRosLinkIMUPublisher::HTNavGazeboRosLinkIMUPublisher()
: impl_(std::make_unique<HTNavGazeboRosLinkIMUPublisherPrivate>())
{
}

HTNavGazeboRosLinkIMUPublisher::~HTNavGazeboRosLinkIMUPublisher()
{
}

void HTNavGazeboRosLinkIMUPublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  impl_->links_.resize(5);

  auto imu_link =
    sdf->Get<std::string>("imu_link", "imu_link").first;
  impl_->links_[0] =
    model->GetLink(imu_link);
  if (!impl_->links_[0]) {
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

  // IMU Link IMU publisher
  impl_->imu_meas_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "imu_data_link_body", qos.get_publisher_qos("imu_data_link_body", rclcpp::QoS(1000)));

  // impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    // "imu_data_body", qos.get_publisher_qos("imu_data_body", rclcpp::SensorDataQoS().best_effort()));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&HTNavGazeboRosLinkIMUPublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void HTNavGazeboRosLinkIMUPublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  // int i = 0;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosLinkIMUPublisherPrivate::OnUpdate");
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

  link = links_[0];
  LinkStateSolver(&link_state, link);

  sensor_msgs::msg::JointState imu_measurement;
  imu_measurement.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

  link = links_[0];

  imu_measurement.name.resize(3);
  imu_measurement.position.resize(3);
  imu_measurement.velocity.resize(3);
  imu_measurement.effort.resize(3);

  ignition::math::Vector3d ang_acc;
  ignition::math::Vector3d lin_acc;
  ignition::math::Vector3d ang_vel;
  ignition::math::Vector3d velocity;
  ignition::math::Pose3d position;
  
  velocity = link->WorldLinearVel(); 
  position = link->WorldPose();
  ang_vel  = link->WorldAngularVel();
  lin_acc  = link->WorldLinearAccel();
  ang_acc  = link->WorldAngularAccel();

  double C_nb[3][3], euler_in[3];
  double lin_acc_ref[3], ang_vel_ref[3];
  double lin_acc_body[3], ang_vel_body[3];

  euler_in[0] = -link_state.effort[1]; 
  euler_in[1] = -link_state.effort[0]; 
  euler_in[2] = -link_state.effort[2]; 

  ang_vel_ref[0] = -ang_vel.Y(); 
  ang_vel_ref[1] = -ang_vel.X(); 
  ang_vel_ref[2] = -ang_vel.Z(); 

  lin_acc_ref[0] = -lin_acc.Y(); 
  lin_acc_ref[1] = -lin_acc.X(); 
  lin_acc_ref[2] = -lin_acc.Z() - 9.80; 

  Euler2Cnb(C_nb, euler_in);
  MatrixVectorMult(ang_vel_body, C_nb, ang_vel_ref);
  MatrixVectorMult(lin_acc_body, C_nb, lin_acc_ref);

  // Fill message with latest sensor data

  // accelerometer_data.X() = -lin_acc_body[1];
  // accelerometer_data.Y() = -lin_acc_body[0];
  // accelerometer_data.Z() = -lin_acc_body[2];

  // gyroscope_data.X() = -ang_vel_body[1];
  // gyroscope_data.Y() = -ang_vel_body[0];
  // gyroscope_data.Z() = -ang_vel_body[2];

  // ignition::math::Quaterniond quaternion1;
  // geometry_msgs::msg::Quaternion quaternion2;

  // quaternion1 = position.Rot();

  // quaternion2.w = quaternion1.W();
  // quaternion2.x = quaternion1.X();
  // quaternion2.y = quaternion1.Y();
  // quaternion2.z = quaternion1.Z();

//   msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
//     current_time);
//   msg_->orientation = quaternion2;
// //     gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(quaternion2);
    
  // (void)accelerometer_data;
  // (void)gyroscope_data;
  // msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(ang_vel_body);
  // msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(lin_acc_body);

  // pub_->publish(*msg_);

  // (void)msg_;

  imu_measurement.name[0]     = link->GetName();
  imu_measurement.name[1]     = link->GetName();
  imu_measurement.name[2]     = link->GetName();
  imu_measurement.position[0] = ang_vel_body[0];
  imu_measurement.position[1] = ang_vel_body[1];
  imu_measurement.position[2] = ang_vel_body[2];
  imu_measurement.velocity[0] = lin_acc_body[0];
  imu_measurement.velocity[1] = lin_acc_body[1];
  imu_measurement.velocity[2] = lin_acc_body[2];
  imu_measurement.effort[0]   = ang_acc.X();
  imu_measurement.effort[1]   = ang_acc.Y();
  imu_measurement.effort[2]   = ang_acc.Z();
  
  imu_meas_pub_->publish(imu_measurement);

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

  // if (data_counter_ % 100 == 0)
  // {
  // double yaw = link_state.effort[2] * 180.0 / 3.14;
  // RCLCPP_INFO(
  //   ros_node_->get_logger(), "Yaw Angle [%f]", yaw );
  // }

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

void HTNavGazeboRosLinkIMUPublisherPrivate::LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link)
{
  link_state->name.resize(3);
  link_state->position.resize(3);
  link_state->velocity.resize(3);
  link_state->effort.resize(3);

  ignition::math::Vector3d ang_vel;
  ignition::math::Vector3d velocity;
  ignition::math::Pose3d position;

  velocity = link->WorldLinearVel();        
  position = link->WorldPose();

  
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

void HTNavGazeboRosLinkIMUPublisherPrivate::Euler2Cnb(double c_nb[3][3], double euler_in[3])
{
    c_nb[0][0] = cos(euler_in[1]) * cos(euler_in[2]);
    c_nb[0][1] = cos(euler_in[1]) * sin(euler_in[2]);
    c_nb[0][2] = -sin(euler_in[1]);
    c_nb[1][0] = -cos(euler_in[0]) * sin(euler_in[2]) + sin(euler_in[0]) * sin(euler_in[1]) * cos(euler_in[2]);
    c_nb[1][1] = cos(euler_in[0]) * cos(euler_in[2]) + sin(euler_in[0]) * sin(euler_in[1]) * sin(euler_in[2]);
    c_nb[1][2] = sin(euler_in[0]) * cos(euler_in[1]);
    c_nb[2][0] = sin(euler_in[0]) * sin(euler_in[2]) + cos(euler_in[0]) * sin(euler_in[1]) * cos(euler_in[2]);
    c_nb[2][1] = -sin (euler_in[0]) * cos(euler_in[2]) + cos(euler_in[0]) * sin(euler_in[1]) * sin(euler_in[2]);
    c_nb[2][2] = cos(euler_in[0]) * cos(euler_in[1]);
}


void HTNavGazeboRosLinkIMUPublisherPrivate::MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]) {
	for (int i = 0; i < 3; i++)
	{
		vector_res[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			vector_res[i] = vector_res[i] + matrix1[i][j]*vector2[j];
		}
	}
	return;
}


GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboRosLinkIMUPublisher)
}  // namespace gazebo_plugins
