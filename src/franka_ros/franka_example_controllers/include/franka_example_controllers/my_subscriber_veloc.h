// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace franka_example_controllers {

class MySubscriberVeloc : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface,
                                               franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  void ControlCommandCallback(const geometry_msgs::Point::ConstPtr& msg);
  ros::NodeHandle _nh;
  ros::Publisher _end_pose_pub; // 用于end pose发布
  ros::Subscriber _command_sub; // 用于接收 control command

  double _delta_x_last=0;
  double _delta_y_last=0;
  double _delta_z_last=0;

  double _delta_x=0;
  double _delta_y=0;
  double _delta_z=0;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
};

}  // namespace franka_example_controllers
