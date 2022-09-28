// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#include <franka_example_controllers/JointVelocityCommand.h>

#include <algorithm>
#include <Eigen/Dense>
#include <vector>

namespace franka_example_controllers {

class MyVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  void JointVelocityCallback(const JointVelocityCommand::ConstPtr& msg);
  Eigen::Matrix<double, 7, 1> saturateAndLimit(const Vector7d& dq, const Vector7d& dq_limit);

  Vector7d q_d;
  Vector7d dq_d;
  Vector7d ddq_d;
  Vector7d dq_limits;
  Vector7d ddq_limits;
  Vector7d dq_last;
  bool joint_vel_prepared;
  int expected_control_hz;
  ros::NodeHandle _nh;
  ros::Subscriber _joint_vel_cmd_sub;  // 用于接收joint velocity command
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;
  ros::Duration time_since_updated;
};

}  // namespace franka_example_controllers
