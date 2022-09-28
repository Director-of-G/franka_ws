// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_subscriber_veloc.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <franka/robot_state.h>

namespace franka_example_controllers {

bool MySubscriberVeloc::init(hardware_interface::RobotHW* robot_hw,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MySubscriberVeloc: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hw->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "MySubscriberVeloc: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Exception getting Cartesian handle: " << e.what());
    return false;
  }

//创建subscriber===============================================================
_nh = node_handle;
_command_sub = _nh.subscribe("/haptic/position", 1, &MySubscriberVeloc::ControlCommandCallback, this);

//取joint name================================================================
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MySubscriberVeloc: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

//取model interface============================================================
auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  //取state_interface============================================================
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");
    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "MySubscriberVeloc: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "MySubscriberVeloc: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}


void MySubscriberVeloc::ControlCommandCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f,%f,%f]", msg->x,msg->y,msg->z);
  _delta_x=msg->x;
  _delta_y=msg->y;
  _delta_z=msg->z;
}

void MySubscriberVeloc::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
}

void MySubscriberVeloc::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  //计算实际末端速度
  Eigen::Matrix<double, 6, 1> dx;
  dx=jacobian*dq;
  // std::cout<<"dx is:"<<dx<<std::endl;

  //计算command 法1
  double difference = _delta_x-_delta_x_last;
  double x_saturated =_delta_x_last + std::max(std::min(difference, 0.001), -0.001);
  difference = _delta_y -_delta_y_last;
  double y_saturated =_delta_y_last + std::max(std::min(difference, 0.001), -0.001);
  difference =_delta_z -_delta_z_last;
  double z_saturated =_delta_z_last + std::max(std::min(difference, 0.001), -0.001);  

  //计算command 法2
  // double difference = _delta_x-dx[0];
  // double x_saturated =dx[0] + std::max(std::min(difference, 0.001), -0.001);
  // difference = _delta_y -dx[1];
  // double y_saturated =dx[1] + std::max(std::min(difference, 0.001), -0.001);
  // difference =_delta_z -dx[2];
  // double z_saturated =dx[2] + std::max(std::min(difference, 0.001), -0.001);  

  std::array<double, 6> command = {{x_saturated, y_saturated, z_saturated, 0.0, 0.0, 0.0}};
  // std::cout<<"dx is:"<<dx<<std::endl;

  _delta_x_last = x_saturated;
  _delta_y_last = y_saturated;
  _delta_z_last = z_saturated;

  velocity_cartesian_handle_->setCommand(command);

}

void MySubscriberVeloc::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MySubscriberVeloc,
                       controller_interface::ControllerBase)
