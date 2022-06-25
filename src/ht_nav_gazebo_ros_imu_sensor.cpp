// Copyright 2018 Open Source Robotics Foundation, Inc.
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


#include <builtin_interfaces/msg/time.hpp>
#include <ignition/math/Rand.hh>
#include <gazebo_plugins/ht_nav_gazebo_ros_imu_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/imu.hpp>
#include <gazebo_plugins/ht_nav_config.hpp>

#include <iostream>
#include <memory>
#include <string>
#include "math.h"

namespace gazebo_plugins
{

class HTNavGazeboRosImuSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for imu message
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ideal_pub_;
  /// IMU message modified each update
  sensor_msgs::msg::Imu::SharedPtr msg_;
  sensor_msgs::msg::Imu::SharedPtr ideal_msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest imu data to ROS
  void OnUpdate();

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

  std::vector<double> acc_data_;
  std::vector<double> gyro_data_;
  std::vector<double> acc_data_err_;
  std::vector<double> gyro_data_err_;

  FILE *fptr;

  int init_flag_ = 0;
  int data_counter_ = 0;

};

HTNavGazeboRosImuSensor::HTNavGazeboRosImuSensor()
: impl_(std::make_unique<HTNavGazeboRosImuSensorPrivate>())
{
}

HTNavGazeboRosImuSensor::~HTNavGazeboRosImuSensor()
{
}

void HTNavGazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an imu sensor. Exiting.");
    return;
  }

  bool initial_orientation_as_reference = false;
  if (!_sdf->HasElement("initial_orientation_as_reference")) {
    RCLCPP_INFO_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> is unset, using default value of false "
      "to comply with REP 145 (world as orientation reference)");
  } else {
    initial_orientation_as_reference = _sdf->Get<bool>("initial_orientation_as_reference");
  }
/* *************** NOISE Parameters**************************************************** */

  if (_sdf->HasElement("updateRateHZ"))
  {
    impl_->pub_freq_ =  _sdf->Get<double>("updateRateHZ");
    impl_->delta_t_ = 1 / impl_->pub_freq_;
  }
  else
  {
    impl_->pub_freq_ = 100.0;
    impl_->delta_t_ = 1 / impl_->pub_freq_;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <updateRateHZ>, set to default: 100.0 ");
  }

  if (_sdf->HasElement("gaussianNoise"))
  {
    impl_->gaussian_noise_ =  _sdf->Get<double>("gaussianNoise");
  }
  else
  {
    impl_->gaussian_noise_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <gaussianNoise>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("AccBiasStd"))
  {
    impl_->acc_bias_std_ =  _sdf->Get<double>("AccBiasStd");
  }
  else
  {
    impl_->acc_bias_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccBiasStd>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("GyroBiasStd"))
  {
    impl_->gyro_drift_std_ =  _sdf->Get<double>("GyroBiasStd");
  }
  else
  {
    impl_->gyro_drift_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <GyroBiasStd>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("AccSFStd"))
  {
    impl_->acc_sf_std_ =  _sdf->Get<double>("AccSFStd");
  }
  else
  {
    impl_->acc_sf_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccSFStd>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("GyroSFStd"))
  {
    impl_->gyro_sf_std_ =  _sdf->Get<double>("GyroSFStd");
  }
  else
  {
    impl_->gyro_sf_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <GyroBiasStd>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("AccRWStd"))
  {
    impl_->acc_rw_std_ =  _sdf->Get<double>("AccRWStd");
  }
  else
  {
    impl_->acc_rw_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <AccRWStd>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("GyroRWStd"))
  {
    impl_->gyro_rw_std_ =  _sdf->Get<double>("GyroRWStd");
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

  impl_->acc_data_.assign(3, 0.0);
  impl_->gyro_data_.assign(3, 0.0);
  impl_->acc_data_err_.assign(3, 0.0);
  impl_->gyro_data_err_.assign(3, 0.0);

/* ********************************************************************************** */

  if (initial_orientation_as_reference) {
    RCLCPP_WARN_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> set to true, this behavior is deprecated "
      "as it does not comply with REP 145.");
  } else {
    // This complies with REP 145
    impl_->sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().best_effort()));

  impl_->ideal_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/out_ideal", qos.get_publisher_qos("~/out_ideal", rclcpp::SensorDataQoS().best_effort()));

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  auto ideal_msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);
  ideal_msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  // TODO(anyone): covariance for IMU's orientation once this is added to gazebo
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->msg_ = msg;

  ideal_msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  ideal_msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  ideal_msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  ideal_msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  ideal_msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  ideal_msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->ideal_msg_ = ideal_msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&HTNavGazeboRosImuSensorPrivate::OnUpdate, impl_.get() ));

  // impl_->sensor_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
  //   std::bind(&HTNavGazeboRosImuSensorPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}



void HTNavGazeboRosImuSensorPrivate::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosImuSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

  int i = 0;

  // if (data_counter_ % 100 == 0)
  // {
  //   RCLCPP_INFO(
  //     ros_node_->get_logger(), "Acc Bias std [%f]", acc_bias_std_ );

  //   RCLCPP_INFO(
  //     ros_node_->get_logger(), "Init flag [%d]", init_flag_ );
  // }

  if (init_flag_ == 0)
  {

    fptr = fopen(base_path"imu_errors_added.txt", "w");
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

  // if (data_counter_ % 100 == 0)
  // {
  //   RCLCPP_INFO(
  //     ros_node_->get_logger(), "Acc Bias X [%f]", acc_bias_[0] );
  // }

  accelerometer_data = sensor_->LinearAcceleration();
  gyroscope_data = sensor_->AngularVelocity();

  acc_data_[0] = accelerometer_data.X();
  acc_data_[1] = accelerometer_data.Y();
  acc_data_[2] = accelerometer_data.Z();

  gyro_data_[0] = gyroscope_data.X();
  gyro_data_[1] = gyroscope_data.Y();
  gyro_data_[2] = gyroscope_data.Z();

  // if (data_counter_ % 100 == 0)
  // {
  //   RCLCPP_INFO(
  //     ros_node_->get_logger(), "Acc Bias [%f %f %f]", acc_bias_[0], acc_bias_[1], acc_bias_[2] );
  // }

  for (i = 0; i < 3; i++)
  {
    acc_data_err_[i]  = acc_data_[i]  + acc_sf_[i]  * acc_data_[i]  +  acc_bias_[i]   + acc_rw_[i];
    gyro_data_err_[i] = gyro_data_[i] + gyro_sf_[i] * gyro_data_[i] +  gyro_drift_[i] + gyro_rw_[i];
  }

  accelerometer_data_err.X() = acc_data_err_[0];
  accelerometer_data_err.Y() = acc_data_err_[1];
  accelerometer_data_err.Z() = acc_data_err_[2];

  gyroscope_data_err.X() = gyro_data_err_[0];
  gyroscope_data_err.Y() = gyro_data_err_[1];
  gyroscope_data_err.Z() = gyro_data_err_[2];
  
  // Fill message with latest sensor data
  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->orientation =
     gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());
  // msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
  //   sensor_->AngularVelocity());
  // msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
  //   sensor_->LinearAcceleration());

  ideal_msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  ideal_msg_->orientation =
     gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());
  
  ideal_msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    gyroscope_data);
  ideal_msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    accelerometer_data);


  msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    gyroscope_data_err);
  msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    accelerometer_data_err);

  data_counter_ += 1;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("Publish");
#endif
  // Publish message
  pub_->publish(*msg_);
  ideal_pub_->publish(*ideal_msg_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double HTNavGazeboRosImuSensorPrivate::GaussianKernel(double mu, double sigma)
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



GZ_REGISTER_SENSOR_PLUGIN(HTNavGazeboRosImuSensor)

}  // namespace gazebo_plugins