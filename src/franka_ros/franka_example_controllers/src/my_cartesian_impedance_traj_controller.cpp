// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_cartesian_impedance_traj_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool MyCartesianImpedanceTrajController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &MyCartesianImpedanceTrajController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("MyCartesianImpedanceTrajController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MyCartesianImpedanceTrajController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MyCartesianImpedanceTrajController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyCartesianImpedanceTrajController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MyCartesianImpedanceTrajController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyCartesianImpedanceTrajController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "MyCartesianImpedanceTrajController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "MyCartesianImpedanceTrajController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&MyCartesianImpedanceTrajController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // yxj add 0525
  // M_d << 100* Eigen::MatrixXd::Identity(6, 6);
  // K_d << 50* Eigen::MatrixXd::Identity(6, 6);
  // D_d << 20* Eigen::MatrixXd::Identity(6, 6);
  
  // M_d(0,0) = 4.03012;
  // M_d(1,1) = 2.09689;
  // M_d(2,2) = 1.74778;
  // M_d(3,3) = 0.067025;
  // M_d(4,4) = 0.11314;
  // M_d(5,5) = 0.0275345;
  // // M_d.block<3,3>(0,0) = 100*Eigen::MatrixXd::Identity(3, 3);
  // // M_d.block<3,3>(3,3) = 30*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(0,0) = 2000*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(3,3) = 100*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(0,0) = 28.2843*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(3,3) = 6.32456*Eigen::MatrixXd::Identity(3, 3);
  

  M_d << 1000 * Eigen::MatrixXd::Identity(6, 6);
  K_d.block<3,3>(0,0) = 100*Eigen::MatrixXd::Identity(3, 3);
  K_d.block<3,3>(3,3) = 30*Eigen::MatrixXd::Identity(3, 3);
  D_d.block<3,3>(0,0) = 20*Eigen::MatrixXd::Identity(3, 3);
  D_d.block<3,3>(3,3) = 10.95*Eigen::MatrixXd::Identity(3, 3);

  // M_d << 1000 * Eigen::MatrixXd::Identity(6, 6);
  // K_d.block<3,3>(0,0) = 100000*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(3,3) = 300000*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(0,0) = 200000*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(3,3) = 100095*Eigen::MatrixXd::Identity(3, 3);

  // M_d(0,0) = 1;
  // M_d(1,1) = 10;
  // M_d(2,2) = 2;
  // M_d(3,3) = 0.17;
  // M_d(4,4) = 0.279;
  // M_d(5,5) = 0.039;

  // M_d(0,0) = 4;
  // M_d(1,1) = 2;
  // M_d(2,2) = 2;
  // M_d(3,3) = 0.17;
  // M_d(4,4) = 0.279;
  // M_d(5,5) = 0.039;

  // M_d << 1 * Eigen::MatrixXd::Identity(6, 6);
  // M_d.block<3,3>(0,0) = 1*Eigen::MatrixXd::Identity(3, 3);
  // M_d.block<3,3>(3,3) = 0.3*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(0,0) = 100*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(3,3) = 40*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(0,0) = 20*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(3,3) = 12.65*Eigen::MatrixXd::Identity(3, 3);
  //K_d.block<3,3>(0,0) = 30*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(3,3) = 1.2*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(0,0) = 10.95*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(3,3) = 2.19*Eigen::MatrixXd::Identity(3, 3);

  // // 现在正在用的
  // K_d.block<3,3>(0,0) = 100*Eigen::MatrixXd::Identity(3, 3);
  // K_d.block<3,3>(3,3) = 100*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(0,0) = 14*Eigen::MatrixXd::Identity(3, 3);
  // D_d.block<3,3>(3,3) = 14*Eigen::MatrixXd::Identity(3, 3);
  nullspace_stiffness_ = 0.05;
  // F_ext_filter<<0,0,0,0,0,0;
  
  return true;
}

void MyCartesianImpedanceTrajController::starting(const ros::Time& time) {
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
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  // ROS_INFO("The position_d_ is......");
  // std::cout<<position_d_<<std::endl;

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  // yxj 0529
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;
  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  error_last_ = error;

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  std::array<double,7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;

  //yxj 0625
  elapsed_time_ = ros::Duration(0.0);
  //hyb 0718
  F_ext_filtered_ << 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0;
  orientation_d_last_ = orientation_d_;
  omega_d_last_<< 0,0,0;

  F_steady<<0,0,0,0,0,0;
  // hyb 0720
  init_time = time;
}

void MyCartesianImpedanceTrajController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  count++;
  
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // yxj state variables
  std::array<double, 49> mass = model_handle_->getMass();
  std::array<double,7> gravity_array = model_handle_->getGravity();

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

  // yxj convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_read(robot_state.tau_ext_hat_filtered.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Matrix<double, 7, 1> tau_ext_calc;
  tau_ext_calc = tau_measured - gravity - tau_ext_initial_;

  if (count<=30000){
    F_steady+=F_ext;
  }
  else if (count==30001)
  {
    F_steady/=(count-1);
  }
  else{
    elapsed_time_ +=ros::Duration(0.001);
    F_ext-=F_steady;
    // yxj analytic jacobian
    Eigen::Matrix<double,3,3> T;
    T << orientation.coeffs()[3],-orientation.coeffs()[2],orientation.coeffs()[1],\
    orientation.coeffs()[2],orientation.coeffs()[3],-orientation.coeffs()[0]\
    -orientation.coeffs()[1],orientation.coeffs()[0],orientation.coeffs()[3];
    Eigen::Matrix<double,6,7> jacobian_analytic(jacobian);
    jacobian_analytic.block<3,7>(3,0) = 0.5 * T.transpose() * jacobian_analytic.block<3,7>(3,0);

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error, x_d_ddot;
    error.head(3) << position - position_d_;

    // 0718
    //   x_d_ddot.head(3) << -radius * M_PI * M_PI / 100.0 * std::sin(M_PI / 10.0 * elapsed_time_.toSec()),
    // radius * M_PI * M_PI / 100.0 * std::cos(M_PI / 10.0 * elapsed_time_.toSec()),0;//position acceleration
    // Eigen::Vector3d omega_d;
    // Eigen::AngleAxisd angle_axis(orientation_d_.inverse() * orientation_d_last_);
    // omega_d = angle_axis.angle() * angle_axis.axis()/0.001;//=logarithm of quaternion/dt
    
    // x_d_ddot.tail(3)<<(omega_d-omega_d_last_)/0.001;
    //0625
    double t=elapsed_time_.toSec();
        x_d_ddot << -radius * M_PI * M_PI / 100.0 * std::sin(M_PI / 10.0 *t),
        -radius * M_PI * M_PI / 100.0 * std::cos(M_PI / 10.0 * t),0,0,0,0;
  //   x_d_ddot<<0,0,0,0,0,0;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    //yxj 
    Eigen::MatrixXd jacobian_pinv;
    pseudoInverse(jacobian, jacobian_pinv);
    Eigen::MatrixXd jacobian_analytic_pinv;
    pseudoInverse(jacobian_analytic, jacobian_analytic_pinv);
    Eigen::MatrixXd jacobian_analytic_transpose_pinv;
    pseudoInverse(jacobian_analytic.transpose(), jacobian_analytic_transpose_pinv);
    
    cartesian_stiffness_.block<3,3>(0,0) = 10*Eigen::MatrixXd::Identity(3, 3);
    cartesian_stiffness_.block<3,3>(3,3) = 1*Eigen::MatrixXd::Identity(3, 3);
    cartesian_damping_.block<3,3>(0,0) = 6.32*Eigen::MatrixXd::Identity(3, 3);
    cartesian_damping_.block<3,3>(3,3) = 2*Eigen::MatrixXd::Identity(3, 3);
    nullspace_stiffness_ = 0.02;

    //yxj calculate jacobian_dot
    //double p=period.toSec();
    Eigen::MatrixXd jacobian_dot;
    jacobian_dot = (jacobian - jacobian_last_)/0.001;// here must use =!!!period.toSec()
    Eigen::Matrix<double, 6, 1> error_dot;
    error_dot = (error-error_last_)/0.001;
  // //0714
  //   if (F_ext_list.size()<1000){
  //     F_ext_list.push_back(F_ext);
  //     F_ext_filter=(F_ext_filter*(F_ext_list.size()-1)+F_ext)/F_ext_list.size();

  //   }
  //   else
  //   {
  //     F_ext_filter=(F_ext_filter*1000-F_ext_list[0]+F_ext)/1000;
  //     F_ext_list.erase(F_ext_list.begin());
  //     F_ext_list.push_back(F_ext);
  //   }
    
  //  M_d = jacobian_transpose_pinv * M * jacobian_pinv;
    
    
    tau_task << M * jacobian_pinv * M_d.inverse() * 
                    (M_d * x_d_ddot -K_d * error -D_d * error_dot - M_d * jacobian_dot *dq)
                    +(jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext_filtered_;
  //0713
    // M_d = (jacobian.transpose()*((jacobian*jacobian.transpose()).inverse())).transpose()* M *( jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()));
    // tau_task << M * jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()) * M_d.inverse() * 
    //                 (M_d * x_d_ddot -K_d * error -D_d * error_dot - M_d * jacobian_dot *dq)
    //                 +0*(jacobian.transpose() - M * jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()) * M_d.inverse()) * F_ext;
    
    //tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      // jacobian.transpose() *((jacobian.transpose()*((jacobian*jacobian.transpose()).inverse())).transpose() ))*
                      //    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                      //     (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // Cartesian PD control with damping ratio = 1
  //   tau_task << jacobian.transpose() *
  //                   (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // Desired torque

    tau_d << tau_task + coriolis + tau_nullspace ;

    // tau_d<<0;
    // if (yxj_counter%10000==0){
    //   std::cout<<"11111111111111"<<std::endl;
    
    //      std::cout<< "tau_d" << std::endl << tau_d <<std::endl;
    //     std::cout<< "  M_d " << std::endl <<   M_d  <<std::endl;
    // }

    //cartesian gravity compensation PD control
    // tau_d = jacobian.transpose() *10*Eigen::MatrixXd::Identity(6, 6) * (-error) - jacobian.transpose() * 6.32* Eigen::MatrixXd::Identity(6, 6) * jacobian * dq;
    // tau_d = tau_d+tau_nullspace;
    
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    // if (yxj_counter%10000==0){
    //   std::cout<<"2222222222222"<<std::endl;
    
    //      std::cout<< "tau_d" << std::endl << tau_d <<std::endl;
    // }

    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }

    if (yxj_counter%1000==0){
      yxj_counter=0;
      std::cout<<"============"<<std::endl;
      // std::cout<<"M_d"<<std::endl<<M_d<<std::endl;
      // std::cout<<"K_d"<<std::endl<<K_d<<std::endl;
      // std::cout<<"D_d"<<std::endl<<D_d<<std::endl;
      std::cout<< "error" <<std::endl<< error<<std::endl;
      std::cout<<"11111111111111"<<std::endl<< M * jacobian_pinv * M_d.inverse() * 
                    (M_d * x_d_ddot -K_d * error -D_d * error_dot - M_d * jacobian_dot *dq)<<std::endl;
      std::cout<<"2222222222"<<std::endl<<(jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext_filtered_<<std::endl;
      std::cout<< "F_ext_filter"<<std::endl<<F_ext_filtered_ << std::endl;
      // std::cout<< "-K_d * error - D_d * error_dot"<<std::endl<<-K_d * error - D_d * error_dot<<std::endl;
      // std::cout<< "- M_d * jacobian_dot *dq"<< std::endl<<- M_d * jacobian_dot *dq<<std::endl;

      //std::cout<< " M * jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()) * M_d.inverse()"<< std::endl<< M * jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()) * M_d.inverse()<<std::endl;
      //std::cout<< "(jacobian) "<< std::endl<<(jacobian) <<std::endl;
      //std::cout<< "(jacobian.transpose()) "<< std::endl<<(jacobian.transpose() ) <<std::endl;
      //std::cout<< "jacobian.transpose()*((jacobian*jacobian.transpose()).inverse())"<< std::endl<<jacobian.transpose()*((jacobian*jacobian.transpose()).inverse())<<std::endl;
      // std::cout<< "(jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext_filtered_ "<< std::endl<<((jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext_filtered_) <<std::endl;
      // std::cout<< "(jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) "<< std::endl<<((jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) ) <<std::endl;
      //std::cout<< "........................111111111111 "<< std::endl<<jacobian.transpose() - M * jacobian.transpose()*((jacobian*jacobian.transpose()).inverse()) * M_d.inverse() <<std::endl;
      //std::cout<< " jacobian_pinv "<< std::endl<<jacobian_pinv <<std::endl;
      std::cout<< "jacobian_transpose_pinv * M * jacobian_pinv; "<< std::endl<<jacobian_transpose_pinv * M * jacobian_pinv <<std::endl;
      //td::cout<< " (jacobian* M .inverse()* jacobian.transpose()).inverse() "<< std::endl<<( (jacobian* M .inverse()* jacobian.transpose()).inverse()) <<std::endl;
      
      
      std::cout<<"tau_d"<<std::endl<<tau_d<<std::endl;
      std::cout<< "time" << std::endl << elapsed_time_.toSec() <<std::endl;
      
      // std::cout<< "position_d_" << std::endl << position_d_ <<std::endl;
      // std::cout<< "position" << std::endl << position <<std::endl;
      std::cout<<"F_steady"<<std::endl<<F_steady<<std::endl;
      
      
      //std::cout<< "orientation_d_.coeffs()"<<std::endl <<orientation_d_.coeffs() << std::endl;
      //std::cout<< "orientation"<<std::endl <<orientation.coeffs() << std::endl;
    }

    //yxj 0525
    jacobian_last_ = jacobian;
    error_last_ = error;
    orientation_d_last_ = orientation_d_;
    // omega_d_last_ = omega_d;
    yxj_counter++;
    log_F_ext.push_back(F_ext);
    log_error.push_back(error);
    log_t.push_back(time.toSec() - init_time.toSec());
    log_tau.push_back(tau_d);
    log_F_ext_filtered.push_back(F_ext_filtered_);

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);

    // hyb 0718
    F_ext_filtered_ = filter_params_ * F_ext + (1 - filter_params_) * F_ext_filtered_;
  //   position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  //   orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

      position_d_[0] =    position_d_target_[0]+radius * std::sin(M_PI / 10.0 * (elapsed_time_.toSec()-2));
      position_d_[1] =    position_d_target_[1] - radius + radius *  std::cos(M_PI / 10.0 * (elapsed_time_.toSec()-2));
      // if (position_d_[2]>0.3970) {
      //     position_d_[2] -= 0.00001;
      // }
      // if (elapsed_time_.toSec()<10) {
      //     position_d_target_[0] += 0.00002;
      // }
      //  // double amplitude = 0.5;
      // double amplitude = 0.5;

      // orientation_d_ = Eigen::Quaterniond(std::sin(0.5 * amplitude * std::sin(M_PI / 10.0 * elapsed_time_.toSec())),
      // std::cos(0.5 * amplitude *std::sin(M_PI / 10.0 * elapsed_time_.toSec())),0,0);//define order is w,x,y,z
      //orientation_d_ = Eigen::Quaterniond(0.923731,-0.383041,0.000140861,0.00036416);
  }
}

void MyCartesianImpedanceTrajController::stopping(const ros::Time &time){
  ROS_INFO("Stopping Controller and Saving data......");
  std::string path("/home/roboticslab/wgk/wgk_ws/src/data/0722/");
  std::string filename1("log_error.bin");
  std::string filename2("log_F_ext.bin");
  std::string filename3("log_t.bin");
  std::string filename4("log_tau.bin");
  std::string filename5("log_F_ext_filtered.bin");
  saveData6(log_error,path+filename1);
  saveData6(log_F_ext,path+filename2);
  saveData_time(log_t,path+filename3);
  saveData7(log_tau,path+filename4);
  saveData6(log_F_ext_filtered,path+filename5);
  ROS_INFO("The Data is saved......");
} 



Eigen::Matrix<double, 7, 1> MyCartesianImpedanceTrajController::saturateTorqueRate(
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

void MyCartesianImpedanceTrajController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void MyCartesianImpedanceTrajController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

// //yxj 0603
void MyCartesianImpedanceTrajController::saveData6(std::vector<Eigen::Matrix<double,6,1>> &Data, std::string filePath){
  //format: 4 bytes vector number + 4 bytes totalsize + data
	std::ofstream ofile(filePath.c_str(), std::ios::binary);
	if(ofile.is_open()==false){
		std::cout<<"Open file fail!"<<std::endl;
		exit(1);
	}
	int length = Data.size();
	ofile.write((char*)&length, sizeof(int)); 
	
	int totalSize = Data.size()*sizeof(Data[0]);
	ofile.write((char*)&totalSize, sizeof(int));
	
	ofile.write((char*)&Data[0], totalSize);
	
	ofile.close();
} 

// //yxj 0603
void MyCartesianImpedanceTrajController::saveData7(std::vector<Eigen::Matrix<double,7,1>> &Data, std::string filePath){
  //format: 4 bytes vector number + 4 bytes totalsize + data
	std::ofstream ofile(filePath.c_str(), std::ios::binary);
	if(ofile.is_open()==false){
		std::cout<<"Open file fail!"<<std::endl;
		exit(1);
	}
	int length = Data.size();
	ofile.write((char*)&length, sizeof(int)); 
	
	int totalSize = Data.size()*sizeof(Data[0]);
	ofile.write((char*)&totalSize, sizeof(int));
	
	ofile.write((char*)&Data[0], totalSize);
	
	ofile.close();
} 

void MyCartesianImpedanceTrajController::saveData_time(std::vector<double> &Data, std::string filePath){
  //format: 4 bytes vector number + 4 bytes totalsize + data
	std::ofstream ofile(filePath.c_str(), std::ios::binary);
	if(ofile.is_open()==false){
		std::cout<<"Open file fail!"<<std::endl;
		exit(1);
	}
	int length = Data.size();
	ofile.write((char*)&length, sizeof(int)); 
	
	int totalSize = Data.size()*sizeof(Data[0]);
	ofile.write((char*)&totalSize, sizeof(int));
	
	ofile.write((char*)&Data[0], totalSize);
	
	ofile.close();
} 

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianImpedanceTrajController,
                       controller_interface::ControllerBase)
