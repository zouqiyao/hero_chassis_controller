#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh) {
  try {
    left_front_wheel_joint_ = hw->getHandle("left_front_wheel_joint");
    right_front_wheel_joint_ = hw->getHandle("right_front_wheel_joint");
    left_back_wheel_joint_ = hw->getHandle("left_back_wheel_joint");
    right_back_wheel_joint_ = hw->getHandle("right_back_wheel_joint");
    // 学长/姐 我这里就直接 .. getHandle 不想用 parameter 了...

  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("HeroChassisController: failed to get joint handle: " << e.what());
    return false;
  }

  ros::NodeHandle pid_nh(controller_nh, "wheel_pid");
  pid_fl_.init(pid_nh);
  pid_fr_.init(pid_nh);
  pid_bl_.init(pid_nh);
  pid_br_.init(pid_nh); //这里给每一个控制器同一个 跟 Master 对话的句柄

  target_vel = 5.0;  //设置一个目标 速度
  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  double err_fl = target_vel - left_front_wheel_joint_.getVelocity();
  double err_fr = target_vel - right_front_wheel_joint_.getVelocity();
  double err_bl = target_vel - left_back_wheel_joint_.getVelocity();
  double err_br = target_vel - right_back_wheel_joint_.getVelocity(); //计算erro

  double effort_fl = pid_fl_.computeCommand(err_fl, period);
  double effort_fr = pid_fr_.computeCommand(err_fr, period);
  double effort_bl = pid_bl_.computeCommand(err_bl, period);
  double effort_br = pid_br_.computeCommand(err_br, period);

  left_front_wheel_joint_.setCommand(effort_fl);
  right_front_wheel_joint_.setCommand(effort_fr);
  left_back_wheel_joint_.setCommand(effort_bl);
  right_back_wheel_joint_.setCommand(effort_br);
}



void HeroChassisController::starting(const ros::Time& /*time*/) {
  pid_fl_.reset();
  pid_fr_.reset();
  pid_bl_.reset();
  pid_br_.reset();
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  left_front_wheel_joint_.setCommand(0.0);
  right_front_wheel_joint_.setCommand(0.0);
  left_back_wheel_joint_.setCommand(0.0);
  right_back_wheel_joint_.setCommand(0.0);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}