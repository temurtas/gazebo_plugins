// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <math.h>
#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_plugins/ht_nav_gazebo_ros_gps_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gazebo_plugins/ht_nav_config.hpp>

#include <iostream>
#include <memory>
#include <string>
#include "math.h"

#define tau_i 0.01	// sampling period
#define tau_s 0.01  // If IMU data is diff not acc, use it to convert
#define m_acc 1 
#define R_0 6378137				//WGS84 Equatorial radius in meters
#define e  0.0818191908425		//WGS84 eccentricity
#define w_ie 7.292115E-5		//Earth rotation rate(rad / s)
#define R_P 6356752.31425		//WGS84 Polar radius in meters
#define RAD2DEG 57.2957795130823
#define DEG2RAD 0.0174532925199433

namespace gazebo_plugins
{

class HTNavGazeboRosGpsSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for gps message
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ideal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  /// GPS message modified each update
  sensor_msgs::msg::NavSatFix::SharedPtr msg_;
  /// GPS sensor this plugin is attached to
  gazebo::sensors::GpsSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  /// Publish latest gps data to ROS
  void OnUpdate();

  double RN_calculator(double R_N_input);
  double RE_calculator(double R_E_input);
  double GaussianKernel(double mu, double sigma);
  std::vector<double> pos_bias_ ;
  std::vector<double> pos_drift_ ;
  std::vector<double> pos_current_drift_ ;
  std::vector<double> pos_err_ ;

  double pos_drift_freq_ = 0.0;
  double pos_bias_std_ = 0.0;
  double pos_drift_std_ = 0.0;
  double gaussian_noise_ = 0.0;
  double delta_t_ = 0.0;
  double pub_freq_ = 0.0;

  FILE *fptr;

  int init_flag_ = 0;
  int data_counter_ = 0;

};

HTNavGazeboRosGpsSensor::HTNavGazeboRosGpsSensor()
: impl_(std::make_unique<HTNavGazeboRosGpsSensorPrivate>())
{
}

HTNavGazeboRosGpsSensor::~HTNavGazeboRosGpsSensor()
{
}

void HTNavGazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a GPS sensor. Exiting.");
    return;
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

  if (_sdf->HasElement("posDriftFreq"))
  {
    impl_->pos_drift_freq_ =  _sdf->Get<double>("posDriftFreq");
  }
  else
  {
    impl_->pos_drift_freq_ = 1500.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <posDriftFreq>, set to default: 1500.0 ");
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

  if (_sdf->HasElement("posBiasStd"))
  {
    impl_->pos_bias_std_ =  _sdf->Get<double>("posBiasStd");
  }
  else
  {
    impl_->pos_bias_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <posBias>, set to default: 0.0 ");
  }

  if (_sdf->HasElement("posDriftStd"))
  {
    impl_->pos_drift_std_=  _sdf->Get<double>("posDriftStd");
  }
  else
  {
    impl_->pos_drift_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <posDrift>, set to default: 0.0 ");
  }

  impl_->pos_bias_.assign(3, 0.0);
  impl_->pos_drift_.assign(3, 0.0);
  impl_->pos_current_drift_.assign(3, 0.0);
  impl_->pos_err_.assign(3, 0.0);

  // impl_->ideal_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
  //   "~/ideal_out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().best_effort()));

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().best_effort()));


  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->position_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_LATITUDE_NOISE_METERS));
  msg->position_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_LONGITUDE_NOISE_METERS));
  msg->position_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_ALTITUDE_NOISE_METERS));
  msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  impl_->msg_ = msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&HTNavGazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
}

void HTNavGazeboRosGpsSensorPrivate::OnUpdate()
{
  int i = 0;
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosGpsSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
  #endif
  // Fill message with latest sensor data

  // msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
  //   sensor_->LastUpdateTime());
  // // msg_->latitude = sensor_->Latitude().Degree();
  // // msg_->longitude = sensor_->Longitude().Degree();
  // // msg_->altitude = sensor_->Altitude();

  // msg_->latitude = sensor_->Latitude().Degree();
  // msg_->longitude = sensor_->Longitude().Degree();
  // msg_->altitude = sensor_->Altitude();

  // // Publish message
  // ideal_pub_->publish(*msg_);

  if (init_flag_ == 0)
  {

    fptr = fopen(base_path"gps_errors_added.txt", "w");
    if (fptr == NULL)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Could not open file !");
      return;
    }
    
    for (i = 0; i < 3; i++)
    {
      pos_bias_[i] = GaussianKernel(0, pos_bias_std_);              
    }   

    for (i = 0; i < 3; i++)
    {
      pos_drift_[i] = GaussianKernel(0, pos_drift_std_);              
    }   

    for (i = 0; i < 3; i++)
    {
      pos_current_drift_[i] = pos_drift_[i];            
    }   

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_bias_[i]); 
    }

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_drift_[i]); 
    }

    fprintf(fptr,"\n");

    init_flag_ = 1;
  }else{
    // double temp_var[3];
    double temp_var2[3];
    for (i = 0; i < 3; i++)
    {
      // temp_var[i] = exp(- 1/pos_drift_freq_ * delta_t_ );
      temp_var2[i] = GaussianKernel(0, sqrt(2 / pos_drift_freq_) * pos_drift_[i] ) * delta_t_ ;
      pos_current_drift_[i] = pos_current_drift_[i] * exp(- 1/pos_drift_freq_ * delta_t_ ) + temp_var2[i]  ;              
    } 

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", temp_var2[i]); 
    }

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_current_drift_[i]); 
    }

    fprintf(fptr,"\n");  
  }

  for (i = 0; i < 3; i++)
  {
    pos_err_[i] = pos_bias_[i] + pos_current_drift_[i];
  }

  // for (i = 0; i < 3; i++)
  // {
  //   pos_err_[i] = GaussianKernel(0, gaussian_noise_);
  // }

  double RN = RN_calculator(sensor_->Latitude().Degree() * DEG2RAD);
  double RE = RE_calculator(sensor_->Latitude().Degree() * DEG2RAD);

  

  double lat_err = RAD2DEG*(pos_err_[0] / RN);
  double lon_err = RAD2DEG*(pos_err_[1] / RE / cos(sensor_->Latitude().Degree() * DEG2RAD) );

  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->latitude  = sensor_->Latitude().Degree() + lat_err;
  msg_->longitude = sensor_->Longitude().Degree() + lon_err;
  msg_->altitude  = sensor_->Altitude() + pos_err_[2];

  //ignition::math::Vector3d gps_vel;
  //gps_vel = sensor_->VelocityENU();
  // // gps_vel[0] = sensor_->VelocityNorth(); 
  // // gps_vel[1] = sensor_->VelocityEast();
  // // gps_vel[2] = - sensor_->VelocityUp();

  // Publish message
  pub_->publish(*msg_);

  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
  #endif

  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double HTNavGazeboRosGpsSensorPrivate::GaussianKernel(double mu, double sigma)
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


double HTNavGazeboRosGpsSensorPrivate::RN_calculator(double R_N_input) {
	double R_N_result = R_0 * (1 - pow(e, 2)) / sqrt(pow((1 - pow(e, 2) * pow(sin(R_N_input), 2)), 3));
	return R_N_result;
}

double HTNavGazeboRosGpsSensorPrivate::RE_calculator(double R_E_input) {
	double R_E_result = R_0 / sqrt( (1 - pow(e, 2) * pow(sin(R_E_input), 2)) );
	return R_E_result;
}

GZ_REGISTER_SENSOR_PLUGIN(HTNavGazeboRosGpsSensor)

}  // namespace gazebo_plugins
