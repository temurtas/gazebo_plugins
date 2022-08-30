// Copyright 2019 Open Source Robotics Foundation
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

/*
 * \desc Bumper controller
 * \author Nate Koenig
 * \date 09 Sept. 2008
 */

#include <gazebo/plugins/ContactPlugin.hh>

#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_plugins/ht_nav_gazebo_ros_contact_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <gazebo_plugins/ht_nav_config.hpp>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <memory>
#include <string>

namespace gazebo_plugins
{
class HTNavGazeboRosContactSensorPrivate
{
public:
  /// Callback to be called when sensor updates.
  void OnUpdate();
  void Euler2Cnb(double c_nb[3][3], double euler_in[3]);
  void MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]);
 
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Contact mesage publisher.
  rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr pub_{nullptr};

  /// Pointer to sensor
  gazebo::sensors::ContactSensorPtr parent_sensor_;

  /// Frame name, to be used by TF.
  std::string frame_name_;

  /// Connects to pre-render events.
  gazebo::event::ConnectionPtr update_connection_;

  int counter_ = 0;
  FILE *fptr;

  int init_flag_ = 0;
  int data_counter_ = 0;
};

HTNavGazeboRosContactSensor::HTNavGazeboRosContactSensor()
: impl_(std::make_unique<HTNavGazeboRosContactSensorPrivate>())
{
}

HTNavGazeboRosContactSensor::~HTNavGazeboRosContactSensor()
{
  impl_->ros_node_.reset();
}

void HTNavGazeboRosContactSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
  if (!impl_->parent_sensor_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Contact sensor parent is not of type ContactSensor. Aborting");
    impl_->ros_node_.reset();
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Contact state publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ContactsState>(
    "ht_bumper_states", qos.get_publisher_qos("ht_bumper_states", rclcpp::SensorDataQoS().reliable()));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing contact states to [%s]",
    impl_->pub_->get_topic_name());

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "It works at start");

  // Get tf frame for output
  impl_->frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;

  impl_->update_connection_ = impl_->parent_sensor_->ConnectUpdated(
    std::bind(&HTNavGazeboRosContactSensorPrivate::OnUpdate, impl_.get()));

  impl_->parent_sensor_->SetActive(true);
}

void HTNavGazeboRosContactSensorPrivate::OnUpdate()
{
  counter_ = counter_ +1;

  if (init_flag_ == 0)
  {

    fptr = fopen(base_path"contact_forces.txt", "w");
    if (fptr == NULL)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Could not open file !");
      return;
    }

    init_flag_ = 1;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosContactSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill message");
#endif
  gazebo::msgs::Contacts contacts;
  contacts = parent_sensor_->Contacts();

  auto contact_state_msg = gazebo_ros::Convert<gazebo_msgs::msg::ContactsState>(contacts);
  contact_state_msg.header.frame_id = frame_name_;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("publish");
#endif

  // if (counter_ % 100 == 0)
  // {
  //   RCLCPP_INFO(ros_node_->get_logger(), "Contacts Size [%d]", contacts.contact_size());
  // }
  

for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // std::cout << "Collision between[" << contacts.contact(i).collision1()
    //           << "] and [" << contacts.contact(i).collision2() << "]\n";
    // if (counter_ % 100 == 0)
    // {
    //   RCLCPP_INFO(ros_node_->get_logger(), "Pos Size [%d]", contacts.contact(i).position_size());
    // }
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
    // RCLCPP_INFO(
    // ros_node_->get_logger(), "Subscribed to [%s]", contacts.contact(i).position(j).x());
    // RCLCPP_INFO(ros_node_->get_logger(), "Pos X [%lf]", contacts.contact(i).position(j).x());
    // RCLCPP_INFO(ros_node_->get_logger(), "Pos Y [%lf]", contacts.contact(i).position(j).y());
    // RCLCPP_INFO(ros_node_->get_logger(), "Pos Z [%lf]", contacts.contact(i).position(j).z());

    // RCLCPP_INFO(ros_node_->get_logger(), "Normal X [%lf]", contacts.contact(i).normal(j).x());
    // RCLCPP_INFO(ros_node_->get_logger(), "Normal Y [%lf]", contacts.contact(i).normal(j).y());
    // RCLCPP_INFO(ros_node_->get_logger(), "Normal Z [%lf]", contacts.contact(i).normal(j).z());

    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 name [%s]", contacts.contact(i).wrench(j).body_1_name());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 name [%s]", contacts.contact(i).wrench(j).body_2_name());

    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 force x [%lf]", contacts.contact(i).wrench(j).body_1_wrench().force().x());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 force y [%lf]", contacts.contact(i).wrench(j).body_1_wrench().force().y());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 force z [%lf]", contacts.contact(i).wrench(j).body_1_wrench().force().z());

    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 force x [%lf]", contacts.contact(i).wrench(j).body_2_wrench().force().x());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 force y [%lf]", contacts.contact(i).wrench(j).body_2_wrench().force().y());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 force z [%lf]", contacts.contact(i).wrench(j).body_2_wrench().force().z());
    
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 torque x [%lf]", contacts.contact(i).wrench(j).body_1_wrench().torque().x());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 torque y [%lf]", contacts.contact(i).wrench(j).body_1_wrench().torque().y());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_1 torque z [%lf]", contacts.contact(i).wrench(j).body_1_wrench().torque().z());

    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 torque x [%lf]", contacts.contact(i).wrench(j).body_2_wrench().torque().x());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 torque y [%lf]", contacts.contact(i).wrench(j).body_2_wrench().torque().y());
    // RCLCPP_INFO(ros_node_->get_logger(), "wrench body_2 torque z [%lf]", contacts.contact(i).wrench(j).body_2_wrench().torque().z());

    double sim_time;
    double body_1_force[3], body_2_force[3], body_1_torque[3], body_2_torque[3];

    body_1_force[0] = contacts.contact(i).wrench(j).body_1_wrench().force().x();
    body_1_force[1] = contacts.contact(i).wrench(j).body_1_wrench().force().y();
    body_1_force[2] = contacts.contact(i).wrench(j).body_1_wrench().force().z();

    body_2_force[0] = contacts.contact(i).wrench(j).body_2_wrench().force().x();
    body_2_force[1] = contacts.contact(i).wrench(j).body_2_wrench().force().y();
    body_2_force[2] = contacts.contact(i).wrench(j).body_2_wrench().force().z();

    body_1_torque[0] = contacts.contact(i).wrench(j).body_1_wrench().torque().x();
    body_1_torque[1] = contacts.contact(i).wrench(j).body_1_wrench().torque().y();
    body_1_torque[2] = contacts.contact(i).wrench(j).body_1_wrench().torque().z();

    body_2_torque[0] = contacts.contact(i).wrench(j).body_2_wrench().torque().x();
    body_2_torque[1] = contacts.contact(i).wrench(j).body_2_wrench().torque().y();
    body_2_torque[2] = contacts.contact(i).wrench(j).body_2_wrench().torque().z();

    int time_sec, time_nsec;
    int ix = 0;
    time_sec = contacts.contact(i).time().sec();
    time_nsec = contacts.contact(i).time().nsec();
    sim_time = time_sec + time_nsec*1e-9;

    fprintf(fptr,"%lf\t", sim_time );    // sim_time

    for (ix = 0; ix < 3; ix++){
      fprintf(fptr,"%lf\t", body_1_force[ix] );   
    }
    for (ix = 0; ix < 3; ix++){
      fprintf(fptr,"%lf\t", body_2_force[i] );   
    }
    for (ix = 0; ix < 3; ix++){
      fprintf(fptr,"%lf\t", body_1_torque[i] );   
    }
    for (ix = 0; ix < 3; ix++){
      fprintf(fptr,"%lf\t", body_2_torque[i] );   
    }

    fprintf(fptr,"\n");


      // std::cout << j << "  Position:"
      //           << contacts.contact(i).position(j).x() << " "
      //           << contacts.contact(i).position(j).y() << " "
      //           << contacts.contact(i).position(j).z() << "\n";
      // std::cout << "   Normal:"
      //           << contacts.contact(i).normal(j).x() << " "
      //           << contacts.contact(i).normal(j).y() << " "
      //           << contacts.contact(i).normal(j).z() << "\n";
      // std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
  }
  pub_->publish(contact_state_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}


void HTNavGazeboRosContactSensorPrivate::Euler2Cnb(double c_nb[3][3], double euler_in[3])
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


void HTNavGazeboRosContactSensorPrivate::MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]) {
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

GZ_REGISTER_SENSOR_PLUGIN(HTNavGazeboRosContactSensor)
}  // namespace gazebo_plugins
