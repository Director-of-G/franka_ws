// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

//produced by yxj 20211211
//copied from cartesian_pose_example_controller.cpp

#include <franka_example_controllers/my_subscriber.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

#include <franka_example_controllers/pseudo_inversion.h>


#include <signal.h>
#include <fstream>


namespace franka_example_controllers {

void MySubscriber::ControlCommandCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%f,%f,%f]-------------------------------------", msg->x,msg->y,msg->z);
  position_desired<<position_d_[0]+6*msg->x,position_d_[1]+6*msg->y,position_d_[2]+6*msg->z;
  if (msg->y > _orientation_mid_width) {
    orientation_d_ = Eigen::Quaterniond(rotM_right);
  } else if (msg->y < -_orientation_mid_width) {
    orientation_d_ = Eigen::Quaterniond(rotM_left);
  } else {
    orientation_d_ = Eigen::Quaterniond(rotM_left).slerp((msg->y/_orientation_mid_width/2 + 0.5), Eigen::Quaterniond(rotM_right));
  }
}

// void MySubscriber::GripperCallback(const std_msgs::Int8MultiArray& msg) {//一旦/my_gripper有消息，调用这个callback
//   std::lock_guard<std::mutex> _(subscriber_mutex_);//被调用的时候，先锁住subscriber_mutex_. lock_guard的用法就是在lock_guard生命周期结束后自动解锁
//   // double gripper_width = 2 * msg.position[0];
//   // ROS_INFO_STREAM("gripper_width"<<gripper_width<<" start_pos_grasping_ * max_width_ "<<start_pos_grasping_ * max_width_ <<" grasping_"<<grasping_);
//   if (msg.data[0]==1&& !grasping_) {//if gripper_width<0.042，并且不是正在抓取的情况下，那么就开始抓
//     // Grasp object
//     franka_gripper::GraspGoal grasp_goal;
//     grasp_goal.force = 5;
//     grasp_goal.speed = 0.3;
//     grasp_goal.epsilon.inner = grasp_epsilon_inner_;
//     grasp_goal.epsilon.outer = grasp_epsilon_outer_scaling_ * max_width_;

//     grasp_client_.sendGoal(grasp_goal);
//     ROS_INFO("I've sent the grasp_goal!!!!!");
//     if (grasp_client_.waitForResult(ros::Duration(5.0))) {
//       grasping_ = true;
//     } else {
//       ROS_INFO("my_gripper_node: GraspAction was not successful.");
//       stop_client_.sendGoal(franka_gripper::StopGoal());
//     }
//   } else if (msg.data[0]==1 && grasping_) {//if gripper_width>0.042，并且是正在抓取的情况下，那么就把爪子打开
//     // Open gripper
//     franka_gripper::MoveGoal move_goal;
//     move_goal.speed = 0.3;
//     move_goal.width = 0.07;
//     move_client_.sendGoal(move_goal);
//     ROS_INFO("I've sent the move_goal!!!!!");
//     if (move_client_.waitForResult(ros::Duration(5.0))) {
//       grasping_ = false;
//     } else {
//       ROS_ERROR("my_gripper_node: MoveAction was not successful.");
//       stop_client_.sendGoal(franka_gripper::StopGoal());
//     }
//   }
// }
// };



bool MySubscriber::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  rotM_left << 0,1,0,0,0,-1,-1,0,0;
  rotM_right << 0,1,0,0,0,1,1,0,0;

  //节点句柄
  _nh = node_handle;
  // signal(SIGINT,MySubscriber::myStaticSigintHandler);
  // Eigen::VectorXd log_t(1);//记时间
  // Eigen::MatrixXd log_q(7,1);//记关节角


  // cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  // if (cartesian_pose_interface_ == nullptr) {
  //   ROS_ERROR(
  //       "MySubscriber: Could not get Cartesian Pose "
  //       "interface from hardware");
  //   return false;
  // }
    // 用力矩控制后，cartesian_pose_handle_被替代，取笛卡尔空间位姿的句柄就不用了
  // try {
  //   cartesian_pose_handle_ =  <franka_hw::FrankaCartesianPoseHandle>(
  //       cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "MySubscriber: Exception getting Cartesian handle: " << e.what());
  //   return false;
  // }

  _my_gripper.init(std::make_shared<ros::NodeHandle>(_nh));
  // auto pnh = std::make_shared<ros::NodeHandle>("~");//创建一个节点，"~"表示私有名称
  // if (_my_gripper.init(pnh)) {
  //   ros::spin();
  // }

  //arm_id 172.16.0.2
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MySubscriber: Could not get parameter arm_id");
    return false;
  }

  //取model_interface,model_handle
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }


  //取state_interface, state_handle
  //-----------------------原先的。
  // auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  // if (state_interface == nullptr) {
  //   ROS_ERROR("MySubscriber: Could not get state interface from hardware");
  //   return false;
  // }
  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");

  //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "MySubscriber: Robot is not in the expected starting position for "
  //           "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //           "robot_ip:=<robot-ip> load_gripper:=<has-#include <dynamic_reconfigure/server.h>attached-gripper>` first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "MySubscriber: Exception getting state handle: " << e.what());
  //   return false;
  // }
  //----------------------现在的
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  //创建subscriber===============================================================
  _command_sub = _nh.subscribe("/haptic/position", 1, &MySubscriber::ControlCommandCallback, this);
  // _gripper_sub = _nh.subscribe("/haptic/button_state", 1, &MySubscriber::GripperCallback, this);//leader订阅/my_gripper这么个topic

  //取joint names===============================================================
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }
  //取joint handles===============================================================
  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MySubscriber:Fuck!! Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "MySubscriber: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void MySubscriber::starting(const ros::Time& /* time */) {

  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  elapsed_time_ = ros::Duration(0.0);
  // _new_pose = position_d_;//new_pose其实在这个版本里没用
  position_desired<<position_d_[0],position_d_[1],position_d_[2];


}

void MySubscriber::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  //===================================================================================yxj0106
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
  Eigen::Vector3d position(transform.translation());//末端位置
  Eigen::Quaterniond orientation(transform.linear());//末端姿态四元数

  // Eigen::Matrix<double, 3,7> j_pos= jacobian.block<3,7>(0,0);//jacobian_position

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position_desired-position;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation_d_.inverse() * orientation);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);


  // compute control
  // allocate variables
  Eigen::VectorXd tau_d(7);
  tau_d = jacobian.transpose()*Kp*error- jacobian.transpose()*Kd*jacobian*dq;
  
  // ROS_INFO("tau_d [%f, %f,%f, %f,%f, %f,%f]",tau_d[0],tau_d[1],tau_d[2],tau_d[3],tau_d[4],tau_d[5],tau_d[6]);
  
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

}


Eigen::Matrix<double, 7, 1> MySubscriber::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}
//===============================================================================yxj0106

// void MySubscriber::update(const ros::Time& /* time */,
//                                             const ros::Duration& period) {
//   elapsed_time_ += period;

//   double radius = 0.3;
//   double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
//   double delta_x = radius * std::sin(angle);
//   double delta_z = radius * (std::cos(angle) - 1);
//   std::array<double, 16> new_pose = initial_pose_;
//   new_pose[12] -= 0.005* elapsed_time_.toSec()*elapsed_time_.toSec();
//   // new_pose[14] -= 0.01* elapsed_time_.toSec();
//   cartesian_pose_handle_->setCommand(new_pose);
//   ROS_INFO("[%f]", new_pose[12]);
// }


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MySubscriber,
                       controller_interface::ControllerBase)
