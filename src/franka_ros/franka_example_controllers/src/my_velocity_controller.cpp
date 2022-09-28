// Modified from joint_velocity_example_controller.cpp
// yxj 20220422

#include <franka_example_controllers/my_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#define BASE_CONTROL_HZ 1000

namespace franka_example_controllers {
using Vector7d = Eigen::Matrix<double, 7, 1>;

Eigen::Matrix<double, 7, 1> MyVelocityController::saturateAndLimit(
    const Vector7d& dq_diff,
    const Vector7d& ddq_limit) {
    Vector7d dq_diff_limited;
    for (size_t i = 0; i < 7; i++) {
        double dqi_diff = dq_diff[i];
        double dqi_limit = ddq_limit[i];
        dq_diff_limited[i] = std::max(std::min(dqi_diff, dqi_limit), -dqi_limit);
    }
  return dq_diff_limited;
}

void MyVelocityController::JointVelocityCallback(const JointVelocityCommand::ConstPtr& msg)
{
    for (int i = 0; i < 7; i++){
        q_d[i] = msg->q_d[i];
        dq_d[i] = msg->dq_d[i];
        ddq_d[i] = msg->ddq_d[i];
    }
    joint_vel_prepared = true;
}

bool MyVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  joint_vel_prepared = false;
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  _nh = node_handle;
  expected_control_hz = 100;
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "MyVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("MyVelocityController: Could not parse joint names");
  }
//   std::vector<double> joint_vel_limits;
//   if (!node_handle.getParam("dq_limits", joint_vel_limits)) {
//     ROS_ERROR("MyVelocityController: Could not parse joint names");
//   }
//   else {
//       for (int i = 0; i < 7; i++) {
//           dq_limits[i] = joint_vel_limits[i];
//       }
//   }
  for (int i = 0; i < 7; i++){
      ddq_limits[i] = 0.0005;
  }
  for (int i = 0; i < 7; i++) {
      dq_limits[i] = 0.15;
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("MyVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "MyVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("MyVelocityController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "MyVelocityController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "MyVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  _joint_vel_cmd_sub = _nh.subscribe("/dyn_franka_joint_vel", 1, &MyVelocityController::JointVelocityCallback, this);

  return true;
}

void MyVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  time_since_updated = ros::Duration(0.0);
}

void MyVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  Vector7d difference = dq_d - dq_last;
  Vector7d dq_saturated = dq_last + saturateAndLimit(difference, dq_limits);
  for (int i = 0; i < 7; i++){
    dq_last[i] = dq_saturated[i];
    velocity_joint_handles_[i].setCommand(dq_saturated[i]);
  }
//   std::cout << dq_saturated[0] << " " << dq_saturated[1] << " " << dq_saturated[2] << " " << dq_saturated[3] << " " << dq_saturated[4] << " " << dq_saturated[5] << " " << dq_saturated[6] << std::endl;
}

void MyVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyVelocityController,
                       controller_interface::ControllerBase)
