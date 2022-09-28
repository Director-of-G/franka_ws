// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
#include <mutex>

#include <franka/gripper.h>
#include <franka_example_controllers/teleop_gripper_paramConfig.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int8MultiArray.h"
#include <dynamic_reconfigure/server.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>


#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <Eigen/Dense>

//================================for gripper
// #include <functional>
// #include <memory>
// #include <mutex>

// #include <franka/gripper.h>
// #include <franka_gripper/GraspAction.h>
// #include <franka_gripper/HomingAction.h>
// #include <franka_gripper/MoveAction.h>
// #include <franka_gripper/StopAction.h>

// #include <actionlib/client/simple_action_client.h>
// #include "std_msgs/Int8MultiArray.h"

// using franka_gripper::GraspAction;
// using franka_gripper::HomingAction;
// using franka_gripper::MoveAction;
// using franka_gripper::StopAction;
// using GraspClient = actionlib::SimpleActionClient<GraspAction>;
// using HomingClient = actionlib::SimpleActionClient<HomingAction>;
// using MoveClient = actionlib::SimpleActionClient<MoveAction>;
// using StopClient = actionlib::SimpleActionClient<StopAction>;

using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;

namespace franka_example_controllers {

class MySubscriber
    : public controller_interface::MultiInterfaceController<
                                                            //franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaModelInterface,
                                                            hardware_interface::EffortJointInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  //static void callback1(const geometry_msgs::Point::ConstPtr& msg);//声明成静态成员函数是因为之后要MySubscriber::callback1调用，
  //而不是用 对象.callback1 的方式调用

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  void ControlCommandCallback(const geometry_msgs::Point::ConstPtr& msg);

  ros::NodeHandle _nh;
  ros::Publisher _end_pose_pub; // 用于end pose发布
  ros::Subscriber _command_sub; // 用于接收 control command


  Eigen::Vector3d position_desired;
  const double delta_tau_max_{0.5};
  // double _delta_x=0;
  // double _delta_y=0;
  // double _delta_z=0;roid Sans Mono
  // const double ACCEL_LIMIT = 10 * 1e-6;

  // GraspClient grasp_client_("/grasp", true);
  // MoveClient move_client_("/move", true);
  // StopClient stop_client_("/stop", true){};
  // bool grasping_(false);
  // double max_width_{0.07};  // Default value. It will be reset when gripper is homed [m]
  // double grasp_force_;                 // [N]
  // double grasp_epsilon_inner_{0.001};  // [m]
  // double grasp_epsilon_outer_scaling_{0.9};
  // double move_speed_;  // [m/s]
  // std::mutex subscriber_mutex_;//mutex互斥量
  // ros::Subscriber _gripper_sub; // 用于接收 gripper command
  // void GripperCallback(const std_msgs::Int8MultiArray& msg);


//   franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;  
//   std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
//   std::array<double, 16> initial_pose_{};
  Eigen::Vector3d _new_pose;
  Eigen::Vector3d position_d_;//初始位置
  Eigen::Quaterniond orientation_d_;//初始方向

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  Eigen::MatrixXd Kp = 80*Eigen::MatrixXd::Identity(6,6);
  Eigen::MatrixXd Kd = 20*Eigen::MatrixXd::Identity(6,6);

  const double _orientation_mid_width = 0.06;
  Eigen::Matrix3d rotM_left;
  Eigen::Matrix3d rotM_right;

  class MyGripper {
   public:
    MyGripper()//构造函数
        : follower_homing_client_("franka_gripper/homing", true),
          grasp_client_("franka_gripper/grasp", true),
          move_client_("franka_gripper/move", true),
          stop_client_("franka_gripper/stop", true){};

    bool init(const std::shared_ptr<ros::NodeHandle>& pnh) {
      grasping_ = false;
      gripper_homed_ = false;//gripper_homed_是个flag。doing a homing in order to estimate the maximum grasping width with the current fingers
      if (!pnh->getParam("gripper_homed", gripper_homed_)) {//读取是否需要进行home操作。false的话就需要进行一次，true的话就不用进行
        ROS_INFO_STREAM(
            "my_gripper_node: Could not read parameter gripper_homed. "
            "Defaulting to "
            << std::boolalpha << gripper_homed_);
      }

      // Init for dynamic reconfigure新建一个节点用来重新给参数
      // dynamic_reconfigure_teleop_gripper_param_node_ =
      //     ros::NodeHandle("dyn_reconf_teleop_gripper_param_node");
      // dynamic_server_teleop_gripper_param_ = std::make_unique<
      //     dynamic_reconfigure::Server<franka_example_controllers::teleop_gripper_paramConfig>>(
      //     dynamic_reconfigure_teleop_gripper_param_node_);
      // dynamic_server_teleop_gripper_param_->setCallback(
      //     boost::bind(&MyGripper::teleopGripperParamCallback, this, _1, _2));//dynamic_server_teleop_gripper_param_回调函数就是teleopGripperParamCallback

      bool homing_success(false);
      if (!gripper_homed_) {
        ROS_INFO("my_gripper_node: Homing Gripper.");
        homing_success = homingGripper();//进行homing操作
      }

      if (gripper_homed_ || homing_success) {//如果已经进行了homing操作，就让_sub_订阅
        // Start action servers and subscriber for gripper
        ros::Duration timeout(2.0);
        if (grasp_client_.waitForServer(timeout) && move_client_.waitForServer(timeout) && 
            stop_client_.waitForServer(timeout)) {
          _sub_ = pnh->subscribe("/haptic/button_state", 1,
                                      &MyGripper::subscriberCallback, this);//leader订阅/my_gripper这么个topic
        } else {
          ROS_ERROR(
              "my_gripper_node: Action Server could not be started. Shutting "
              "down node.");
          return false;
        }
      } else {
        return false;
      }
      return true;
    };

  private:
    double max_width_{0.07};  // Default value. It will be reset when gripper is homed [m]
    bool grasping_;
    bool gripper_homed_;

    double grasp_force_;                 // [N]
    double grasp_epsilon_inner_{0.001};  // [m]
    double grasp_epsilon_outer_scaling_{0.9};
    double move_speed_;  // [m/s]

    double start_pos_grasping_{0.5};  // Threshold position of leader gripper where to start grasping.
    double start_pos_opening_{0.6};   // Threshold position of leader gripper where to open.

    HomingClient follower_homing_client_;
    GraspClient grasp_client_;
    MoveClient move_client_;
    StopClient stop_client_;

    std::mutex subscriber_mutex_;//mutex互斥量
    ros::Subscriber _sub_;

    // std::mutex dynamic_reconfigure_mutex_;
    // ros::NodeHandle dynamic_reconfigure_teleop_gripper_param_node_;
    // std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::teleop_gripper_paramConfig>>
    //     dynamic_server_teleop_gripper_param_;

    // void teleopGripperParamCallback(//配置参数的回调函数
    //     const franka_example_controllers::teleop_gripper_paramConfig& config,
    //     uint32_t /*level*/) {
    //   if (dynamic_reconfigure_mutex_.try_lock()) {//锁住之后输出参数
    //     grasp_force_ = config.grasp_force;
    //     move_speed_ = config.move_speed;
    //     ROS_INFO_STREAM("Dynamic Reconfigure: Gripper params set: grasp_force = "
    //                     << grasp_force_ << " N ; move_speed = " << move_speed_ << " m/s");
    //   }
    //   dynamic_reconfigure_mutex_.unlock();//然后释放
    // };

    bool homingGripper() {//gripper进行homing操作
      if (follower_homing_client_.waitForServer(ros::Duration(2.0))) {
        follower_homing_client_.sendGoal(franka_gripper::HomingGoal());

        if (follower_homing_client_.waitForResult(ros::Duration(10.0))) {
          return true;
        }
      }
      ROS_ERROR("my_gripper_node: HomingAction has timed out.");
      return false;
    }

    void subscriberCallback(const std_msgs::Int8MultiArray& msg) {//一旦/my_gripper有消息，调用这个callback
      // ROS_INFO("subscriber callback");
      std::lock_guard<std::mutex> _(subscriber_mutex_);//被调用的时候，先锁住subscriber_mutex_
      if (!gripper_homed_) {
        // If gripper had to be homed, reset max_width_.
        // max_width_ = 2 * msg.position[0];
        gripper_homed_ = true;
      }
      // double gripper_width = 2 * msg.position[0];
      // ROS_INFO_STREAM("gripper_width"<<gripper_width<<" start_pos_grasping_ * max_width_ "<<start_pos_grasping_ * max_width_ <<" grasping_"<<grasping_);
      if (msg.data[0]==1 && !grasping_) {//if gripper_width<0.042，并且不是正在抓取的情况下，那么就开始抓
        // Grasp object
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.force = 10;
        grasp_goal.speed = 0.3;

        grasp_client_.sendGoal(grasp_goal);
        ROS_INFO("I've sent the grasp_goal!!!!!");
        if (grasp_client_.waitForResult(ros::Duration(5.0))) {
          grasping_ = true;
        } else {
          ROS_INFO("my_gripper_node: GraspAction was not successful.");
          stop_client_.sendGoal(franka_gripper::StopGoal());
        }
      } else if (msg.data[0]==1 && grasping_) {//if gripper_width>0.042，并且是正在抓取的情况下，那么就把爪子打开
        // Open gripper
        franka_gripper::MoveGoal move_goal;
        move_goal.speed = 0.3;
        move_goal.width = 0.07;
        move_client_.sendGoal(move_goal);
        ROS_INFO("I've sent the move_goal!!!!!");
        if (move_client_.waitForResult(ros::Duration(5.0))) {
          grasping_ = false;
        } else {
          ROS_ERROR("my_gripper_node: MoveAction was not successful.");
          stop_client_.sendGoal(franka_gripper::StopGoal());
        }
      }
    }
  } _my_gripper;

};

}  // namespace franka_example_controllers