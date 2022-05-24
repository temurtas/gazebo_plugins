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
#include <gazebo_plugins/ht_nav_gazebo_ros_link_gps_publisher.hpp>
#include <gazebo_plugins/ht_nav_config.hpp>

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
class HTNavGazeboRosLinkGPSPublisherPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);
  // void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link);
  void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link, std::vector<double> pos_err, std::vector<double> vel_err); 
 // void LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link, double sim_time);
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr imu_link_pub;

  /// Joints being tracked.
  std::vector<gazebo::physics::LinkPtr> links_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  double GaussianKernel(double mu, double sigma);
  std::vector<double> pos_bias_ ;
  std::vector<double> pos_drift_ ;
  std::vector<double> pos_current_drift_ ;
  std::vector<double> pos_err_ ;
  std::vector<double> vel_err_ ;

  double vel_err_std_ = 0.0;
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

HTNavGazeboRosLinkGPSPublisher::HTNavGazeboRosLinkGPSPublisher()
: impl_(std::make_unique<HTNavGazeboRosLinkGPSPublisherPrivate>())
{
}

HTNavGazeboRosLinkGPSPublisher::~HTNavGazeboRosLinkGPSPublisher()
{
}

void HTNavGazeboRosLinkGPSPublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  impl_->links_.resize(1);

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

 if (sdf->HasElement("update_rate"))
  {
    impl_->pub_freq_ =  sdf->Get<double>("update_rate");
    impl_->delta_t_ = 1 / impl_->pub_freq_;
  }
  else
  {
    impl_->pub_freq_ = 100.0;
    impl_->delta_t_ = 1 / impl_->pub_freq_;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <update_rate>, set to default: 100.0 ");
  }

  if (sdf->HasElement("posDriftFreq"))
  {
    impl_->pos_drift_freq_ =  sdf->Get<double>("posDriftFreq");
  }
  else
  {
    impl_->pos_drift_freq_ = 1500.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <posDriftFreq>, set to default: 1500.0 ");
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

  if (sdf->HasElement("posBiasStd"))
  {
    impl_->pos_bias_std_ =  sdf->Get<double>("posBiasStd");
  }
  else
  {
    impl_->pos_bias_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <posBiasStd>, set to default: 0.0 ");
  }

  if (sdf->HasElement("velErrStd"))
  {
    impl_->vel_err_std_ =  sdf->Get<double>("velErrStd");
  }
  else
  {
    impl_->vel_err_std_ = 0.0;
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "missing <velErrStd>, set to default: 0.0 ");
  }

  

  if (sdf->HasElement("posDriftStd"))
  {
    impl_->pos_drift_std_=  sdf->Get<double>("posDriftStd");
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
  impl_->vel_err_.assign(3, 0.0);


/* *************** NOISE Parameters**************************************************** */

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

    // Joint state publisher
  impl_->imu_link_pub = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "link_gps_data", qos.get_publisher_qos("link_gps_data", rclcpp::QoS(1000)));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&HTNavGazeboRosLinkGPSPublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void HTNavGazeboRosLinkGPSPublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
    int i = 0;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("HTNavGazeboRosLinkGPSPublisherPrivate::OnUpdate");
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


  if (init_flag_ == 0)
  {

    fptr = fopen(base_path"link_gps_errors_added.txt", "w");
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

    for (i = 0; i < 3; i++)
    {
      vel_err_[i] = GaussianKernel(0, vel_err_std_);
    }   

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_bias_[i]); 
    }

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_drift_[i]); 
    }

    fprintf(fptr,"%lf\t", vel_err_std_); 

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", vel_err_[i]); 
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

    for (i = 0; i < 3; i++)
    {
      vel_err_[i] = GaussianKernel(0, vel_err_std_);
    }  

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", temp_var2[i]); 
    }

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", pos_current_drift_[i]); 
    }

    fprintf(fptr,"%lf\t", vel_err_std_); 

    for (i = 0; i < 3; i++){
      fprintf(fptr,"%lf\t", vel_err_[i]); 
    }

    fprintf(fptr,"\n");  
  }

  for (i = 0; i < 3; i++)
  {
    pos_err_[i] = pos_bias_[i] + pos_current_drift_[i];
  }


  // Populate message
  sensor_msgs::msg::JointState link_state;

  link_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

  gazebo::physics::LinkPtr link;

  // double sim_time = current_time.Double();
  link = links_[0];
  LinkStateSolver(&link_state, link, pos_err_, vel_err_);
  // Publish
  imu_link_pub->publish(link_state);

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

void HTNavGazeboRosLinkGPSPublisherPrivate::LinkStateSolver(sensor_msgs::msg::JointState *link_state, gazebo::physics::LinkPtr link, std::vector<double> pos_err, std::vector<double> vel_err)
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
  // GAZEBO_WORLD_FRAME 2 NED
  link_state->position[0] = - 1 * position.Y() + pos_err[0];
  link_state->position[1] = - 1 * position.X() + pos_err[1];
  link_state->position[2] = - 1 * position.Z() + pos_err[2];
  link_state->velocity[0] = - 1 * velocity.Y() + vel_err[0];
  link_state->velocity[1] = - 1 * velocity.X() + vel_err[1];
  link_state->velocity[2] = - 1 * velocity.Z() + vel_err[2];
  link_state->effort[0]   = - 1 * position.Rot().Pitch();
  link_state->effort[1]   = - 1 * position.Rot().Roll();
  link_state->effort[2]   = - 1 * position.Rot().Yaw();

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double HTNavGazeboRosLinkGPSPublisherPrivate::GaussianKernel(double mu, double sigma)
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


GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboRosLinkGPSPublisher)
}  // namespace gazebo_plugins
