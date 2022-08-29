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
#include <gazebo_plugins/ht_nav_config.hpp>

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

  // ignition::math::Vector3d accelerometer_data{0, 0, 0};
  // ignition::math::Vector3d gyroscope_data{0, 0, 0};

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr imu_meas_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ideal_pub_;

  /// Joints being tracked.
  std::vector<gazebo::physics::LinkPtr> links_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  double GaussianKernel(double mu, double sigma);

  double gaussian_noise_ = 0.0;
  double acc_bias_std_ = 0.0;
  double gyro_drift_std_ = 0.0;
  double acc_sf_std_ = 0.0;
  double gyro_sf_std_ =0.0;
  double acc_rw_std_ = 0.0;
  double gyro_rw_std_ = 0.0;

  double pub_freq_ = 0.0;
  double delta_t_ = 0.0;

  std::vector<double> acc_bias_ ;
  std::vector<double> gyro_drift_ ;
  std::vector<double> acc_sf_ ;
  std::vector<double> gyro_sf_ ;
  std::vector<double> acc_rw_ ;
  std::vector<double> gyro_rw_ ;
  double acc_rw_coeff_ = 0.0;
  double gyro_rw_coeff_ = 0.0;


  ignition::math::Vector3d accelerometer_data{0, 0, 0};
  ignition::math::Vector3d gyroscope_data{0, 0, 0};

  ignition::math::Vector3d accelerometer_data_err{0, 0, 0};
  ignition::math::Vector3d gyroscope_data_err{0, 0, 0};

  FILE *fptr;

  int init_flag_ = 0;
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

/* *************** NOISE Parameters**************************************************** */

  if (sdf->HasElement("updateRateHZ"))
  {
    impl_->pub_freq_ =  sdf->Get<double>("updateRateHZ");
    impl_->delta_t_ = 1 / impl_->pub_freq_;
  }
  else
  {
    impl_->pub_freq_ = 100.0;
    impl_->delta_t_ = 1 / impl_->pub_freq_;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <updateRateHZ>, set to default: 100.0 ");
  }

  if (sdf->HasElement("gaussianNoise"))
  {
    impl_->gaussian_noise_ =  sdf->Get<double>("gaussianNoise");
  }
  else
  {
    impl_->gaussian_noise_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <gaussianNoise>, set to default: 0.0 ");
  }

  if (sdf->HasElement("AccBiasStd"))
  {
    impl_->acc_bias_std_ =  sdf->Get<double>("AccBiasStd");
  }
  else
  {
    impl_->acc_bias_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccBiasStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("GyroBiasStd"))
  {
    impl_->gyro_drift_std_ =  sdf->Get<double>("GyroBiasStd");
  }
  else
  {
    impl_->gyro_drift_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <GyroBiasStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("AccSFStd"))
  {
    impl_->acc_sf_std_ =  sdf->Get<double>("AccSFStd");
  }
  else
  {
    impl_->acc_sf_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccSFStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("GyroSFStd"))
  {
    impl_->gyro_sf_std_ =  sdf->Get<double>("GyroSFStd");
  }
  else
  {
    impl_->gyro_sf_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <GyroBiasStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("AccRWStd"))
  {
    impl_->acc_rw_std_ =  sdf->Get<double>("AccRWStd");
  }
  else
  {
    impl_->acc_rw_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccRWStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("GyroRWStd"))
  {
    impl_->gyro_rw_std_ =  sdf->Get<double>("GyroRWStd");
  }
  else
  {
    impl_->gyro_rw_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <GyroRWStd>, set to default: 0.0 ");
  }

  impl_->acc_bias_.assign(3, 0.0);
  impl_->gyro_drift_.assign(3, 0.0);
  impl_->acc_sf_.assign(3, 0.0);
  impl_->gyro_sf_.assign(3, 0.0);
  impl_->acc_rw_.assign(3, 0.0);
  impl_->gyro_rw_.assign(3, 0.0);

/* ********************************************************************************** */


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

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_data_body", qos.get_publisher_qos("imu_data_body", rclcpp::SensorDataQoS().best_effort()));

  impl_->ideal_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_data_body_ideal", qos.get_publisher_qos("imu_data_body_ideal", rclcpp::SensorDataQoS().best_effort()));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&HTNavGazeboRosLinkIMUPublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void HTNavGazeboRosLinkIMUPublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  int i = 0;

  if (init_flag_ == 0)
  {

    fptr = fopen(base_path"imu_link_data_errors_added.txt", "w");
    if (fptr == NULL)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Could not open file !");
      return;
    }

    for (i = 0; i < 3; i++)
    {
    acc_bias_[i] = GaussianKernel(0, acc_bias_std_) * 9.81 * 1e-3;              // mg --> m/s^2
    gyro_drift_[i] = GaussianKernel(0, gyro_drift_std_) / 57.295779 / 3600.0;   // deg/hr --> rad/s
    acc_sf_[i] = GaussianKernel(0, acc_sf_std_) * 1e-6;                         // ppm -> unitless
    gyro_sf_[i] = GaussianKernel(0, gyro_sf_std_) * 1e-6;                       // ppm -> unitless
    }   


    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", acc_bias_[i] / 9.81 *1e3 ); // m/s^2 --> mg
    }
    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", gyro_drift_[i] * 57.295779 * 3600.0 ); // rad/s --> deg/hr 
    }
    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", acc_sf_[i] *1e6 );     // unitless --> ppm
    }
    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", gyro_sf_[i] *1e6 );    // unitless --> ppm
    }

    fprintf(fptr,"\n");

    acc_rw_coeff_ = GaussianKernel(0, acc_rw_std_) * 0.3048 / 60.0 * sqrt(delta_t_) ; // ft/s/rt-hr --> m/s^2  
    gyro_rw_coeff_ = GaussianKernel(0, gyro_rw_std_) / 57.295779 / 3600.0 / 60.0 * sqrt(delta_t_); // deg/rt-hr --> rad/s

    init_flag_ = 1;
  }

  for (i = 0; i < 3; i++)
  {
    acc_rw_[i] = GaussianKernel(0, acc_rw_coeff_);
    gyro_rw_[i] = GaussianKernel(0, gyro_rw_coeff_);
  }
  
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
  double lin_acc_err[3], ang_vel_err[3];

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

  // Create the IMU message package 
  sensor_msgs::msg::Imu msg;
  sensor_msgs::msg::Imu ideal_msg;

  // Tag IMU messages with time stamp of Gazebo Sim Time
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    current_time);

  ideal_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    current_time);

  // Fill message with latest sensor data

  // accelerometer_data.X() = -lin_acc_body[1];
  // accelerometer_data.Y() = -lin_acc_body[0];
  // accelerometer_data.Z() = -lin_acc_body[2];

  // gyroscope_data.X() = -ang_vel_body[1];
  // gyroscope_data.Y() = -ang_vel_body[0];
  // gyroscope_data.Z() = -ang_vel_body[2];

  // geometry_msgs::msg::Quaternion quaternion2;
  // quaternion2.w = quaternion1.W();
  // quaternion2.x = quaternion1.X();
  // quaternion2.y = quaternion1.Y();
  // quaternion2.z = quaternion1.Z();

  // msg_->orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(quaternion2);
  // msg.angular_velocity    = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(accelerometer_data);
  // msg.linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(gyroscope_data);


  ignition::math::Quaterniond quaternion1;
  quaternion1 = position.Rot();

  for (i = 0; i < 3; i++)
  {
    lin_acc_err[i]  = lin_acc_body[i]  + acc_sf_[i]  * lin_acc_body[i]  +  acc_bias_[i]   + acc_rw_[i];
    ang_vel_err[i] = ang_vel_body[i] + gyro_sf_[i] * ang_vel_body[i] +  gyro_drift_[i] + gyro_rw_[i];
  }

  // Fill IMU message 
  msg.linear_acceleration.x = -lin_acc_err[1];
  msg.linear_acceleration.y = -lin_acc_err[0];
  msg.linear_acceleration.z = -lin_acc_err[2];

  msg.angular_velocity.x = -ang_vel_err[1];
  msg.angular_velocity.y = -ang_vel_err[0];
  msg.angular_velocity.z = -ang_vel_err[2];

  msg.orientation.w = quaternion1.W();
  msg.orientation.x = quaternion1.X();
  msg.orientation.y = quaternion1.Y();
  msg.orientation.z = quaternion1.Z();

  // Fill Ideal IMU message 
  ideal_msg.linear_acceleration.x = -lin_acc_body[1];
  ideal_msg.linear_acceleration.y = -lin_acc_body[0];
  ideal_msg.linear_acceleration.z = -lin_acc_body[2];
  // Convert to Gazebo-World Body Frame
  ideal_msg.angular_velocity.x = -ang_vel_body[1];
  ideal_msg.angular_velocity.y = -ang_vel_body[0];
  ideal_msg.angular_velocity.z = -ang_vel_body[2];

  ideal_msg.orientation.w = quaternion1.W();
  ideal_msg.orientation.x = quaternion1.X();
  ideal_msg.orientation.y = quaternion1.Y();
  ideal_msg.orientation.z = quaternion1.Z();

  // Publish IMU Message
  pub_->publish(msg);
  ideal_pub_->publish(ideal_msg);

  // Fill IMU Link IMU Data (Old Method)
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


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double HTNavGazeboRosLinkIMUPublisherPrivate::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = ignition::math::Rand::DblUniform();

  // normalized uniform random variable
  double V = ignition::math::Rand::DblUniform();

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}


GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboRosLinkIMUPublisher)
}  // namespace gazebo_plugins
