// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file as2_platform_multirotor_simulator.cpp
 *
 * MultirotorSimulatorPlatform class implementation
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp"

#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

namespace as2_platform_multirotor_simulator
{

MultirotorSimulatorPlatform::MultirotorSimulatorPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options), as2_interface_(this)
{
  RCLCPP_INFO(this->get_logger(), "Initializing MultirotorSimulatorPlatform...");

  // Read parameters
  readParams(platform_params_);

  // Timers
  RCLCPP_INFO(this->get_logger(), "Using update freq: %f", platform_params_.update_freq);
  RCLCPP_INFO(this->get_logger(), "Using control freq: %f", platform_params_.control_freq);
  RCLCPP_INFO(
    this->get_logger(), "Using inertial odometry freq: %f",
    platform_params_.inertial_odometry_freq);
  RCLCPP_INFO(this->get_logger(), "Using state pub freq: %f", platform_params_.state_freq);
  RCLCPP_INFO(
    this->get_logger(), "GPS origin: %f, %f, %f", platform_params_.latitude,
    platform_params_.longitude, platform_params_.altitude);

  simulator_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.update_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorTimerCallback, this));
  simulator_control_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.control_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorControlTimerCallback, this));
  simulator_inertial_odometry_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.inertial_odometry_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorInertialOdometryTimerCallback, this));
  simulator_state_pub_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.imu_pub_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorStateTimerCallback, this));

  gps_handler_.setOrigin(
    platform_params_.latitude, platform_params_.longitude,
    platform_params_.altitude);  // Set origin for GPS

  // Configure sensors
  configureSensors();

  // Hot-teleport service. Relative name "set_platform_state" resolves to
  // /<namespace>/set_platform_state (NOT /<namespace>/<node>/...) because
  // node-private names need the `~` prefix in rclcpp.
  set_platform_state_srv_ = this->create_service<as2_msgs::srv::SetPlatformState>(
    "set_platform_state",
    std::bind(
      &MultirotorSimulatorPlatform::handleSetPlatformState, this,
      std::placeholders::_1, std::placeholders::_2));
}

MultirotorSimulatorPlatform::~MultirotorSimulatorPlatform()
{
  // Timers
  simulator_timer_.reset();
  simulator_control_timer_.reset();
  simulator_state_pub_timer_.reset();

  // Sensors
  sensor_ground_truth_ptr_.reset();
  sensor_odom_estimate_ptr_.reset();
  sensor_imu_ptr_.reset();
  sensor_gps_ptr_.reset();
}

void MultirotorSimulatorPlatform::configureSensors()
{
  getParam("global_ref_frame", frame_id_earth_);
  getParam("base_frame", frame_id_baselink_);
  frame_id_baselink_ = as2::tf::generateTfName(this, frame_id_baselink_);

  // Get gimbal name
  std::string gimbal_name = "gimbal";
  std::string gimbal_base_name = "gimbal_base";
  getParam("gimbal.frame_id", gimbal_name);
  getParam("gimbal.base_frame_id", gimbal_base_name);

  RCLCPP_INFO(this->get_logger(), "Ground truth freq: %f", platform_params_.ground_truth_pub_freq);
  RCLCPP_INFO(this->get_logger(), "Odometry freq: %f", platform_params_.odometry_pub_freq);
  RCLCPP_INFO(this->get_logger(), "IMU freq: %f", platform_params_.imu_pub_freq);
  RCLCPP_INFO(this->get_logger(), "GPS freq: %f", platform_params_.gps_pub_freq);

  // Ground truth
  sensor_ground_truth_ptr_ = std::make_unique<as2::sensors::GroundTruth>(
    this, platform_params_.ground_truth_pub_freq);
  // Odometry
  sensor_odom_estimate_ptr_ = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(
    as2_names::topics::sensor_measurements::odom, this, platform_params_.odometry_pub_freq);
  // IMU
  sensor_imu_ptr_ = std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::Imu>>(
    as2_names::topics::sensor_measurements::imu, this, platform_params_.imu_pub_freq);
  // GPS
  sensor_gps_ptr_ = std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::NavSatFix>>(
    as2_names::topics::sensor_measurements::gps, this, platform_params_.gps_pub_freq);
  // Gimbal
  sensor_gimbal_ptr_ = std::make_unique<as2::sensors::Gimbal>(
    gimbal_name, gimbal_base_name, this, platform_params_.gimbal_pub_freq);
  gimbal_control_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
    "platform/" + gimbal_name + "/gimbal_command", 10,
    std::bind(&MultirotorSimulatorPlatform::gimbalControlCallback, this, std::placeholders::_1));

  // Direct per-motor command passthrough (RL "motor" action mode). Relative
  // names resolve under the drone namespace, matching the env's
  // /<ns>/actuator_command/motors and /<ns>/motor_speed. SensorData QoS to
  // match the env's best-effort pub/sub.
  motors_command_sub_ = this->create_subscription<actuator_msgs::msg::Actuators>(
    "actuator_command/motors", rclcpp::SensorDataQoS(),
    std::bind(&MultirotorSimulatorPlatform::motorsCommandCallback, this, std::placeholders::_1));
  motor_speed_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
    "motor_speed", rclcpp::SensorDataQoS());

  geometry_msgs::msg::Transform gimbal_transform;
  getParam("gimbal.base_transform.x", gimbal_transform.translation.x);
  getParam("gimbal.base_transform.y", gimbal_transform.translation.y);
  getParam("gimbal.base_transform.z", gimbal_transform.translation.z);
  sensor_gimbal_ptr_->setGimbalBaseTransform(gimbal_transform);
}

bool MultirotorSimulatorPlatform::ownSetArmingState(bool state)
{
  state ? simulator_.arm() : simulator_.disarm();
  RCLCPP_INFO(this->get_logger(), "Arming state set to %d.", state);
  return true;
}

bool MultirotorSimulatorPlatform::ownSetOffboardControl(bool offboard)
{
  RCLCPP_INFO(this->get_logger(), "Offboard state set to %d.", offboard);
  return true;
}

bool MultirotorSimulatorPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  if (platform_info_msg_.current_control_mode.control_mode == msg.control_mode) {
    RCLCPP_INFO(
      this->get_logger(), "Control mode already set to [%s]",
      as2::control_mode::controlModeToString(msg).c_str());
    return true;
  }

  multirotor::YawControlMode yaw_mode = multirotor::YawControlMode::ANGLE;
  if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    yaw_mode = multirotor::YawControlMode::RATE;
  }

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
    case as2_msgs::msg::ControlMode::HOVER:
      {
        simulator_.set_control_mode(multirotor::ControlMode::HOVER);
        break;
      }
    case as2_msgs::msg::ControlMode::POSITION:
      {
        simulator_.set_control_mode(multirotor::ControlMode::POSITION, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        simulator_.set_control_mode(multirotor::ControlMode::VELOCITY, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      {
        simulator_.set_control_mode(multirotor::ControlMode::TRAJECTORY, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::ACRO:
      {
        simulator_.set_control_mode(multirotor::ControlMode::ACRO);
        break;
      }
    default:
      {
        RCLCPP_ERROR(
          this->get_logger(), "Desired control mode not supported: [%s]",
          as2::control_mode::controlModeToString(msg).c_str());
        return false;
        break;
      }
  }
  RCLCPP_INFO(
    this->get_logger(), "Control mode set to [%s]", as2::control_mode::controlModeToString(
      msg).c_str());
  return true;
}

bool MultirotorSimulatorPlatform::ownSendCommand()
{
  const as2_msgs::msg::ControlMode current_control_mode = platform_info_msg_.current_control_mode;

  switch (current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
    case as2_msgs::msg::ControlMode::HOVER:
      {
        // Hovering
        break;
      }
    case as2_msgs::msg::ControlMode::POSITION:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_pose_msg_) || !as2_interface_.processCommand(
              command_twist_msg_))
          {
            return false;
          }
        }
        RCLCPP_INFO(
          this->get_logger(), "Setting position to: [%f, %f, %f]",
          command_pose_msg_.pose.position.x,
          command_pose_msg_.pose.position.y, command_pose_msg_.pose.position.z);
        Eigen::Vector3d position;
        position.x() = command_pose_msg_.pose.position.x;
        position.y() = command_pose_msg_.pose.position.y;
        position.z() = command_pose_msg_.pose.position.z;
        simulator_.set_reference_position(position);

        // Velocity limits
        Eigen::Vector3d velocity;
        velocity.x() = command_twist_msg_.twist.linear.x;
        velocity.y() = command_twist_msg_.twist.linear.y;
        velocity.z() = command_twist_msg_.twist.linear.z;
        if (velocity.norm() > 0.0) {
          const bool proportional_saturation_flag =
            simulator_.get_controller_const().get_position_controller_const().
            get_proportional_saturation_flag();
          simulator_.get_controller().get_position_controller().set_output_saturation(
            velocity,
            -velocity,
            proportional_saturation_flag);
        }
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_twist_msg_)) {
            return false;
          }
        }
        Eigen::Vector3d velocity;
        velocity.x() = command_twist_msg_.twist.linear.x;
        velocity.y() = command_twist_msg_.twist.linear.y;
        velocity.z() = command_twist_msg_.twist.linear.z;
        simulator_.set_reference_velocity(velocity);
        break;
      }
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_trajectory_msg_)) {
            return false;
          }
        }
        Eigen::Vector3d position, velocity, acceleration;
        position.x() = command_trajectory_msg_.setpoints[0].position.x;
        position.y() = command_trajectory_msg_.setpoints[0].position.y;
        position.z() = command_trajectory_msg_.setpoints[0].position.z;
        velocity.x() = command_trajectory_msg_.setpoints[0].twist.x;
        velocity.y() = command_trajectory_msg_.setpoints[0].twist.y;
        velocity.z() = command_trajectory_msg_.setpoints[0].twist.z;
        acceleration.x() = command_trajectory_msg_.setpoints[0].acceleration.x;
        acceleration.y() = command_trajectory_msg_.setpoints[0].acceleration.y;
        acceleration.z() = command_trajectory_msg_.setpoints[0].acceleration.z;

        simulator_.set_reference_trajectory(
          position, velocity, acceleration);
        break;
      }
    case as2_msgs::msg::ControlMode::ACRO:
      {
        double thrust = command_thrust_msg_.thrust;
        Eigen::Vector3d angular_velocity;
        angular_velocity.x() = command_twist_msg_.twist.angular.x;
        angular_velocity.y() = command_twist_msg_.twist.angular.y;
        angular_velocity.z() = command_twist_msg_.twist.angular.z;

        simulator_.set_reference_acro(thrust, angular_velocity);
        break;
      }
    default:
      RCLCPP_ERROR(
        this->get_logger(), "Control mode %d not supported.",
        platform_info_msg_.current_control_mode.control_mode);
      return false;
      break;
  }

  switch (current_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      {
        simulator_.set_reference_yaw_rate(command_twist_msg_.twist.angular.z);
        break;
      }
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
    default:
      {
        double roll, pitch, yaw;
        as2::frame::quaternionToEuler(command_pose_msg_.pose.orientation, roll, pitch, yaw);
        if (current_control_mode.control_mode ==
          as2_msgs::msg::ControlMode::TRAJECTORY)
        {
          yaw = command_trajectory_msg_.setpoints[0].yaw_angle;
        } else {
          as2::frame::quaternionToEuler(command_pose_msg_.pose.orientation, roll, pitch, yaw);
        }
        simulator_.set_reference_yaw_angle(yaw);
        break;
      }
  }

  return true;
}

void MultirotorSimulatorPlatform::ownStopPlatform()
{
  // Send hover to platform here
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::HOVER;
  setPlatformControlMode(control_mode_msg);
}

void MultirotorSimulatorPlatform::ownKillSwitch()
{
  // Switch off motors
  simulator_.disarm();
}

bool MultirotorSimulatorPlatform::ownTakeoff()
{
  // Send takeoff to platform here

  // Set control mode to position
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::POSITION;
  control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  control_mode_msg.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  setPlatformControlMode(control_mode_msg);

  // Set reference position to current position and 1m above
  command_pose_msg_.header.frame_id = frame_id_earth_;
  command_pose_msg_.header.stamp = this->now();
  command_pose_msg_.pose.position.x = simulator_.get_state().kinematics.position.x();
  command_pose_msg_.pose.position.y = simulator_.get_state().kinematics.position.y();
  const double takeoff_height = simulator_.get_floor_height() + 1.0;
  command_pose_msg_.pose.position.z = takeoff_height;
  command_pose_msg_.pose.orientation.w = simulator_.get_state().kinematics.orientation.w();
  command_pose_msg_.pose.orientation.x = simulator_.get_state().kinematics.orientation.x();
  command_pose_msg_.pose.orientation.y = simulator_.get_state().kinematics.orientation.y();
  command_pose_msg_.pose.orientation.z = simulator_.get_state().kinematics.orientation.z();

  // Set reference velocity to 1m/s to speed limit
  command_twist_msg_.header.frame_id = frame_id_earth_;
  command_twist_msg_.header.stamp = this->now();
  command_twist_msg_.twist.linear.x = 1.0;
  command_twist_msg_.twist.linear.y = 1.0;
  command_twist_msg_.twist.linear.z = 1.0;

  // Set references
  if (!ownSendCommand()) {
    return false;
  }

  // TODO(RPS98): Use multithread execution
  while (rclcpp::ok() &&
    std::abs(simulator_.get_state().kinematics.position.z() - takeoff_height) > 0.2)
  {
    // Spin timers
    simulatorTimerCallback();
    simulatorControlTimerCallback();
    simulatorInertialOdometryTimerCallback();
    simulatorStateTimerCallback();
  }
  return true;
}

bool MultirotorSimulatorPlatform::ownLand()
{
  // Send land to platform here

  // Set control mode to position
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::POSITION;
  control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  control_mode_msg.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  setPlatformControlMode(control_mode_msg);

  // Set reference position to current position and 1m above
  command_pose_msg_.header.frame_id = frame_id_earth_;
  command_pose_msg_.header.stamp = this->now();
  command_pose_msg_.pose.position.x = simulator_.get_state().kinematics.position.x();
  command_pose_msg_.pose.position.y = simulator_.get_state().kinematics.position.y();
  const double land_height = simulator_.get_floor_height();
  command_pose_msg_.pose.position.z = land_height;
  command_pose_msg_.pose.orientation.w = simulator_.get_state().kinematics.orientation.w();
  command_pose_msg_.pose.orientation.x = simulator_.get_state().kinematics.orientation.x();
  command_pose_msg_.pose.orientation.y = simulator_.get_state().kinematics.orientation.y();
  command_pose_msg_.pose.orientation.z = simulator_.get_state().kinematics.orientation.z();

  // Set reference velocity to 1m/s to speed limit
  command_twist_msg_.header.frame_id = frame_id_earth_;
  command_twist_msg_.header.stamp = this->now();
  command_twist_msg_.twist.linear.x = 1.0;
  command_twist_msg_.twist.linear.y = 1.0;
  command_twist_msg_.twist.linear.z = 1.0;

  // Set references
  if (!ownSendCommand()) {
    return false;
  }

  // TODO(RPS98): Use multithread execution
  while (rclcpp::ok() &&
    std::abs(simulator_.get_state().kinematics.position.z() - land_height) > 0.2)
  {
    // Call timers
    simulatorTimerCallback();
    simulatorControlTimerCallback();
    simulatorInertialOdometryTimerCallback();
    simulatorStateTimerCallback();
  }
  return true;
}

void MultirotorSimulatorPlatform::gimbalControlCallback(
  const as2_msgs::msg::GimbalControl::SharedPtr msg)
{
  double roll, pitch, yaw;
  switch (msg->control_mode) {
    case as2_msgs::msg::GimbalControl::POSITION_MODE:
      {
        roll = msg->target.vector.x;
        pitch = msg->target.vector.y;
        yaw = msg->target.vector.z;
        break;
      }
    case as2_msgs::msg::GimbalControl::SPEED_MODE:
    // TODO(RPS98): Implement speed mode
    // {
    //   as2::frame::quaternionToEuler(gimbal_desired_orientation_.quaternion, roll, pitch, yaw);
    //   double dt = 1.0 / platform_params_.gimbal_pub_freq;
    //   roll += msg->target.vector.x * dt;
    //   pitch += msg->target.vector.y * dt;
    //   yaw += msg->target.vector.z * dt;
    //   break;
    // }
    default:
      {
        RCLCPP_ERROR(
          this->get_logger(), "Gimbal control mode %d not supported.", msg->control_mode);
        break;
      }
  }
  as2::frame::eulerToQuaternion(
    roll, pitch, yaw, gimbal_desired_orientation_.quaternion);
}

void MultirotorSimulatorPlatform::motorsCommandCallback(
  const actuator_msgs::msg::Actuators::SharedPtr msg)
{
  if (msg->velocity.size() < 4) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "actuator_command/motors needs >= 4 velocities, got %zu", msg->velocity.size());
    return;
  }
  // Drive the simulator's direct-actuation mode: motor speeds (rad/s) go
  // straight to the dynamics, bypassing the inner controller. set_control_mode
  // is a no-op once already in MOTOR_W, so calling it per command is cheap.
  Eigen::Matrix<double, 4, 1> w;
  w << msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3];
  simulator_.set_control_mode(multirotor::ControlMode::MOTOR_W);
  simulator_.set_refence_motors_angular_velocity(w);
}

Eigen::Vector3d MultirotorSimulatorPlatform::readVectorParams(const std::string & param_name)
{
  Eigen::Vector3d default_value = Eigen::Vector3d::Zero();  // Default value

  try {
    std::vector<double> vec;
    this->getParam(param_name, vec);

    if (vec.size() != 3) {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter '%s' is not a vector of size 3.", param_name.c_str());
      // Print vector
      RCLCPP_ERROR(this->get_logger(), "Vector: ");
      for (auto & v : vec) {
        RCLCPP_ERROR(this->get_logger(), "%f", v);
      }
      return default_value;
    }

    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Error getting parameter %s: %s", param_name.c_str(), e.what());
    return default_value;
  }
}

void MultirotorSimulatorPlatform::readParams(PlatformParams & platform_params)
{
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");
  // Platform Parameters
  getParam("imu_pub_freq", platform_params.imu_pub_freq);
  getParam("odometry_pub_freq", platform_params.odometry_pub_freq);
  getParam("ground_truth_pub_freq", platform_params.ground_truth_pub_freq);
  getParam("gps_pub_freq", platform_params.gps_pub_freq);
  getParam("gimbal.pub_freq", platform_params.gimbal_pub_freq);

  // Get max frequency
  platform_params.state_freq = std::max(
    std::max(platform_params.imu_pub_freq, platform_params.odometry_pub_freq),
    std::max(platform_params.ground_truth_pub_freq, platform_params.gps_pub_freq));

  // GPS Origin
  getParam("gps_origin.latitude", platform_params.latitude);
  getParam("gps_origin.longitude", platform_params.longitude);
  getParam("gps_origin.altitude", platform_params.altitude);

  // Simulator params
  double floor_height = 0.0;
  getParam("use_odom_for_control", using_odom_for_control_);
  getParam("floor_height", floor_height);
  getParam("simulation.update_freq", platform_params.update_freq);
  getParam("simulation.control_freq", platform_params.control_freq);
  getParam("simulation.inertial_odometry_freq", platform_params.inertial_odometry_freq);

  // Initial pose
  Eigen::Vector3d initial_position;
  getParam("vehicle_initial_pose.x", initial_position.x());
  getParam("vehicle_initial_pose.y", initial_position.y());
  getParam("vehicle_initial_pose.z", initial_position.z());
  double roll, pitch, yaw;
  getParam("vehicle_initial_pose.yaw", yaw);
  getParam("vehicle_initial_pose.pitch", pitch);
  getParam("vehicle_initial_pose.roll", roll);
  Eigen::Quaterniond initial_orientation;
  as2::frame::eulerToQuaternion(roll, pitch, yaw, initial_orientation);

  // Dynamics params
  // Dynamics::State params
  SimulatorParams::DynamicsParams & dp = simulator_params_.dynamics_params;
  dp.state.kinematics.position = initial_position;
  dp.state.kinematics.orientation = initial_orientation;

  // Dynamics::Model params
  dp.model_params.gravity = readVectorParams("multirotor.dynamics.model.gravity");
  getParam("multirotor.dynamics.model.vehicle_mass", dp.model_params.vehicle_mass);
  dp.model_params.vehicle_inertia =
    readVectorParams("multirotor.dynamics.model.vehicle_inertia").asDiagonal();
  getParam(
    "multirotor.dynamics.model.vehicle_drag_coefficient", dp.model_params.vehicle_drag_coefficient);
  // Optional rotor-coupled linear drag (MonoRace k_x term); defaults to 0 when
  // the config omits it, so other UAV configs are unaffected.
  getParam(
    "multirotor.dynamics.model.rotor_drag_coefficient",
    dp.model_params.rotor_drag_coefficient, true);
  // Optional body-frame anisotropic quadratic drag [x,y,z]; [0,0,0] if absent.
  dp.model_params.body_quadratic_drag =
    readVectorParams("multirotor.dynamics.model.body_quadratic_drag");
  // Optional MonoRace thrust aero-droop; default 0 (disabled) if absent.
  getParam("multirotor.dynamics.model.thrust_k_angle", dp.model_params.thrust_k_angle, true);
  getParam("multirotor.dynamics.model.thrust_k_hor", dp.model_params.thrust_k_hor, true);
  getParam(
    "multirotor.dynamics.model.thrust_aero_radius", dp.model_params.thrust_aero_radius, true);
  dp.model_params.vehicle_aero_moment_coefficient =
    readVectorParams("multirotor.dynamics.model.vehicle_aero_moment_coefficient").asDiagonal();
  getParam(
    "multirotor.dynamics.model.force_process_noise_auto_correlation",
    dp.model_params.moment_process_noise_auto_correlation);
  getParam(
    "multirotor.dynamics.model.moment_process_noise_auto_correlation",
    dp.model_params.moment_process_noise_auto_correlation);

  double thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed,
    time_constant, rotational_inertia;
  getParam("multirotor.dynamics.model.motors_params.thrust_coefficient", thrust_coefficient);
  getParam("multirotor.dynamics.model.motors_params.torque_coefficient", torque_coefficient);
  getParam("multirotor.dynamics.model.motors_params.x_dist", x_dist);
  getParam("multirotor.dynamics.model.motors_params.y_dist", y_dist);
  getParam("multirotor.dynamics.model.motors_params.min_speed", min_speed);
  getParam("multirotor.dynamics.model.motors_params.max_speed", max_speed);
  getParam("multirotor.dynamics.model.motors_params.time_constant", time_constant);
  getParam("multirotor.dynamics.model.motors_params.rotational_inertia", rotational_inertia);

  // Optional ASYMMETRIC per-motor layout. If motors_x / motors_y /
  // motors_direction are all provided (each length 4) they define each motor's
  // body-frame position and spin direction independently, so the model can
  // reproduce a real (non-symmetric) quad frame — different roll/pitch torque
  // arms per motor. Otherwise fall back to the symmetric quad-X helper built
  // from x_dist / y_dist (backward compatible with existing configs). The
  // mixer and the INDI mixer-inverse are computed from these motors_params
  // below, so an asymmetric layout propagates consistently to the controller.
  std::vector<double> motors_x, motors_y, motors_direction;
  getParam("multirotor.dynamics.model.motors_params.motors_x", motors_x, true);
  getParam("multirotor.dynamics.model.motors_params.motors_y", motors_y, true);
  getParam("multirotor.dynamics.model.motors_params.motors_direction", motors_direction, true);

  if (motors_x.size() == 4 && motors_y.size() == 4 && motors_direction.size() == 4) {
    std::vector<multirotor::model::MotorParams<double>> motors;
    motors.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      multirotor::model::MotorParams<double> motor;
      motor.thrust_coefficient = thrust_coefficient;
      motor.torque_coefficient = torque_coefficient;
      motor.min_speed = min_speed;
      motor.max_speed = max_speed;
      motor.time_constant = time_constant;
      motor.rotational_inertia = rotational_inertia;
      // +1 = CW, -1 = CCW (matches Model::create_quadrotor_x_config).
      motor.motor_rotation_direction = (motors_direction[i] >= 0.0) ? 1 : -1;
      motor.pose = multirotor::model::MotorParams<double>::IsometryTypeP::Identity();
      motor.pose.translation() = Eigen::Vector3d(motors_x[i], motors_y[i], 0.0);
      motors.push_back(motor);
    }
    dp.model_params.motors_params = motors;
    RCLCPP_INFO(
      this->get_logger(),
      "Multirotor model: ASYMMETRIC per-motor layout from motors_x/y/direction.");
  } else {
    dp.model_params.motors_params = multirotor::model::Model<double, 4>::create_quadrotor_x_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);
  }

  // Controller params Indi
  SimulatorParams::ControllerParams & cp = simulator_params_.controller_params;
  cp.indi_controller_params.inertia = dp.model_params.vehicle_inertia;
  auto mixing_matrix_6D_4rotors =
    multirotor::model::Model<double, 4>::compute_mixer_matrix<6>(dp.model_params.motors_params);
  cp.indi_controller_params.mixer_matrix_inverse =
    indi_controller::compute_quadrotor_mixer_matrix_inverse(mixing_matrix_6D_4rotors);

  cp.indi_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.indi.kp");
  cp.indi_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.indi.ki");
  cp.indi_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.indi.kd");
  cp.indi_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.indi.alpha");
  cp.indi_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.indi.antiwindup_cte");
  Eigen::Vector3d angular_acceleration_limit =
    readVectorParams("multirotor.controller.indi.angular_acceleration_limit");
  cp.indi_controller_params.pid_params.upper_output_saturation = angular_acceleration_limit;
  cp.indi_controller_params.pid_params.lower_output_saturation = -angular_acceleration_limit;
  cp.indi_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Acro
  cp.acro_controller_params.gravity = dp.model_params.gravity;
  cp.acro_controller_params.vehicle_mass = dp.model_params.vehicle_mass;
  cp.acro_controller_params.kp_rot =
    readVectorParams("multirotor.controller.acro.kp_rot").asDiagonal();

  // Controller params Trajectory
  cp.trajectory_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.trajectory.kp");
  cp.trajectory_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.trajectory.ki");
  cp.trajectory_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.trajectory.kd");
  cp.trajectory_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.trajectory.alpha");
  cp.trajectory_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.trajectory.antiwindup_cte");
  Eigen::Vector3d linear_acceleration_limit =
    readVectorParams("multirotor.controller.trajectory.linear_acceleration_limit");
  cp.trajectory_controller_params.pid_params.upper_output_saturation = linear_acceleration_limit;
  cp.trajectory_controller_params.pid_params.lower_output_saturation = -linear_acceleration_limit;
  cp.trajectory_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Velocity
  cp.velocity_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.velocity.kp");
  cp.velocity_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.velocity.ki");
  cp.velocity_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.velocity.kd");
  cp.velocity_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.velocity.alpha");
  cp.velocity_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.velocity.antiwindup_cte");
  linear_acceleration_limit =
    readVectorParams("multirotor.controller.velocity.linear_acceleration_limit");
  cp.velocity_controller_params.pid_params.upper_output_saturation = linear_acceleration_limit;
  cp.velocity_controller_params.pid_params.lower_output_saturation = -linear_acceleration_limit;
  cp.velocity_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Position
  cp.position_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.position.kp");
  cp.position_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.position.ki");
  cp.position_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.position.kd");
  cp.position_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.position.alpha");
  cp.position_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.position.antiwindup_cte");
  Eigen::Vector3d inear_velocity_limit =
    readVectorParams("multirotor.controller.position.linear_velocity_limit");
  cp.position_controller_params.pid_params.upper_output_saturation = inear_velocity_limit;
  cp.position_controller_params.pid_params.lower_output_saturation = -inear_velocity_limit;
  cp.position_controller_params.pid_params.proportional_saturation_flag = true;

  // IMU params
  getParam("multirotor.imu.gyro_noise_var", simulator_params_.imu_params.gyro_noise_var);
  getParam("multirotor.imu.accel_noise_var", simulator_params_.imu_params.accel_noise_var);
  getParam(
    "multirotor.imu.gyro_bias_noise_autocorr_time",
    simulator_params_.imu_params.gyro_bias_noise_autocorr_time);
  getParam(
    "multirotor.imu.accel_bias_noise_autocorr_time",
    simulator_params_.imu_params.accel_bias_noise_autocorr_time);

  // Inertial Odometry params
  getParam("multirotor.inertial_odometry.alpha", simulator_params_.inertial_odometry_params.alpha);
  simulator_params_.inertial_odometry_params.initial_world_orientation = initial_orientation;

  simulator_ = Simulator(simulator_params_);
  simulator_.enable_floor_collision(floor_height);
  RCLCPP_INFO(this->get_logger(), "Parameters read.");
}

void MultirotorSimulatorPlatform::simulatorTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  static rclcpp::Time last_time_dynamics = current_time;
  double dt = (current_time - last_time_dynamics).seconds();
  last_time_dynamics = current_time;

  if (dt <= 0.0) {
    return;
  }

  // Update simulator
  simulator_.update_dynamics(dt);
  simulator_.update_imu(dt);
}

void MultirotorSimulatorPlatform::simulatorControlTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  static rclcpp::Time last_time_control = current_time;
  double dt = (current_time - last_time_control).seconds();
  last_time_control = current_time;

  if (dt <= 0.0) {
    return;
  }
  control_state_ = simulator_.get_state().kinematics;
  if (using_odom_for_control_) {
    control_state_.position = simulator_.get_odometry().position;
    control_state_.orientation = simulator_.get_odometry().orientation;
  }

  simulator_.update_controller(dt, control_state_);

  // Publish the actual (lagged) motor angular velocities for the RL observation
  // — the source for the "motor mode" rpm obs channels. Cheap at control_freq.
  const auto & motor_w = simulator_.get_actuation_motors_angular_velocity();
  actuator_msgs::msg::Actuators motor_msg;
  motor_msg.header.stamp = current_time;
  motor_msg.velocity = {motor_w[0], motor_w[1], motor_w[2], motor_w[3]};
  motor_speed_pub_->publish(motor_msg);
}

void MultirotorSimulatorPlatform::simulatorInertialOdometryTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  static rclcpp::Time last_time_control = current_time;
  double dt = (current_time - last_time_control).seconds();
  last_time_control = current_time;

  if (dt <= 0.0) {
    return;
  }

  simulator_.update_inertial_odometry(dt);
}

void MultirotorSimulatorPlatform::simulatorStateTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();

  // Get odometry simulator state for imu orientation
  const Kinematics odometry_kinematics = simulator_.get_odometry();

  // Get imu simulator
  Eigen::Vector3d imu_angular_velocity, imu_acceleration;
  simulator_.get_imu_measurement(imu_angular_velocity, imu_acceleration);
  geometry_msgs::msg::Vector3 imu_angular_velocity_msg;
  imu_angular_velocity_msg.x = imu_angular_velocity.x();
  imu_angular_velocity_msg.y = imu_angular_velocity.y();
  imu_angular_velocity_msg.z = imu_angular_velocity.z();
  geometry_msgs::msg::Vector3 imu_acceleration_msg;
  imu_acceleration_msg.x = imu_acceleration.x();
  imu_acceleration_msg.y = imu_acceleration.y();
  imu_acceleration_msg.z = imu_acceleration.z();

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = current_time;
  imu_msg.header.frame_id = frame_id_baselink_;
  imu_msg.angular_velocity = imu_angular_velocity_msg;
  imu_msg.linear_acceleration = imu_acceleration_msg;
  imu_msg.orientation.w = odometry_kinematics.orientation.w();
  imu_msg.orientation.x = odometry_kinematics.orientation.x();
  imu_msg.orientation.y = odometry_kinematics.orientation.y();
  imu_msg.orientation.z = odometry_kinematics.orientation.z();
  sensor_imu_ptr_->updateData(imu_msg);

  // Get odometry simulator state
  nav_msgs::msg::Odometry odometry;
  as2_interface_.convertToOdom(odometry_kinematics, odometry, current_time);
  sensor_odom_estimate_ptr_->updateData(odometry);

  // Get ground truth simulator state

  geometry_msgs::msg::PoseStamped ground_truth_pose;
  geometry_msgs::msg::TwistStamped ground_truth_twist;
  const Kinematics kinematics =
    simulator_.get_state().kinematics;
  as2_interface_.convertToGroundTruth(
    kinematics, ground_truth_pose, ground_truth_twist, current_time);

  sensor_ground_truth_ptr_->updateData(ground_truth_pose, ground_truth_twist);

  // Convert to GPS
  double lat, lon, alt;
  gps_handler_.Local2LatLon(
    ground_truth_pose.pose.position.x, ground_truth_pose.pose.position.y,
    ground_truth_pose.pose.position.z, lat, lon, alt);

  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.stamp = current_time;
  gps_msg.header.frame_id = frame_id_earth_;
  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
  gps_msg.altitude = alt;
  sensor_gps_ptr_->updateData(gps_msg);

  // Move Gimbal
  gimbal_desired_orientation_.header.stamp = current_time;
  sensor_gimbal_ptr_->updateData(gimbal_desired_orientation_);
}

void MultirotorSimulatorPlatform::handleSetPlatformState(
  const std::shared_ptr<as2_msgs::srv::SetPlatformState::Request> request,
  std::shared_ptr<as2_msgs::srv::SetPlatformState::Response> response)
{
  try {
    // Start from the current state to preserve actuator/dynamics fields that
    // aren't part of the request payload (rotor angular velocities, etc.).
    auto state = simulator_.get_state();

    // Kinematics — write the entire pose + twist atomically.
    state.kinematics.position.x() = request->pose.position.x;
    state.kinematics.position.y() = request->pose.position.y;
    state.kinematics.position.z() = request->pose.position.z;
    state.kinematics.orientation = Eigen::Quaterniond(
      request->pose.orientation.w,
      request->pose.orientation.x,
      request->pose.orientation.y,
      request->pose.orientation.z);
    state.kinematics.orientation.normalize();
    state.kinematics.linear_velocity.x() = request->linear_velocity.x;
    state.kinematics.linear_velocity.y() = request->linear_velocity.y;
    state.kinematics.linear_velocity.z() = request->linear_velocity.z;
    state.kinematics.angular_velocity.x() = request->angular_velocity.x;
    state.kinematics.angular_velocity.y() = request->angular_velocity.y;
    state.kinematics.angular_velocity.z() = request->angular_velocity.z;
    // Accelerations and net forces are derived quantities; zero them so the
    // integrator restarts cleanly from this snapshot.
    state.kinematics.linear_acceleration.setZero();
    state.kinematics.angular_acceleration.setZero();
    state.dynamics.force.setZero();
    state.dynamics.torque.setZero();

    simulator_.get_dynamics().set_state(state);

    // Re-anchor the IMU bias estimate and the inertial-odometry origin to the
    // teleport pose so downstream state estimators don't see a giant
    // discontinuity in the integrated odometry stream.
    simulator_.get_imu().reset();
    simulator_.get_inertial_odometry().set_initial_position(state.kinematics.position);
    simulator_.get_inertial_odometry().set_initial_orientation(state.kinematics.orientation);
    simulator_.get_inertial_odometry().reset();

    // Optional: switch to HOVER so the controller doesn't chase stale
    // references from before the teleport. set_control_mode rebases all
    // references to the current (just-teleported) state.
    if (request->reset_to_hover) {
      simulator_.set_control_mode(multirotor::HOVER, multirotor::ANGLE);
    }

    response->success = true;
    response->message = "ok";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("set_platform_state failed: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "set_platform_state: %s", e.what());
  }
}

}  // namespace as2_platform_multirotor_simulator
