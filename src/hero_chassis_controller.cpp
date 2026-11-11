#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>


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
  pid_br_.init(pid_nh); //这里给每一个控制器同一个 跟 Master 对话的句柄 拿P I D

  // 用 root_nh /cmd_vel
  cmd_vel_sub = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::callBack, this);

  return true;
}


void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

  double vx, vy, az; //临时存 .. 我的 target_v 但是读写 target_vx_
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_); //解锁读取
    vx = target_vx_;
    vy = target_vy_;
    az = target_az_;
  }

  double kWheelRadius = 0.07625;
  double k = 0.4;
  
  const double target_fl = (vx - vy - k * az) / kWheelRadius;
  const double target_fr = (vx + vy + k * az) / kWheelRadius;
  const double target_bl = (vx + vy - k * az) / kWheelRadius;
  const double target_br = (vx - vy + k * az) / kWheelRadius;

  setpidcommand(target_fl,target_fr,target_bl,target_br,period);

}

void HeroChassisController::setpidcommand(const double &target_fl,
                                          const double &target_fr,
                                          const double &target_bl,
                                          const double &target_br,const ros::Duration &period){
    
  double err_fl = target_fl - left_front_wheel_joint_.getVelocity();
  double err_fr = target_fr - right_front_wheel_joint_.getVelocity();
  double err_bl = target_bl - left_back_wheel_joint_.getVelocity();
  double err_br = target_br - right_back_wheel_joint_.getVelocity(); //计算erro

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

  std::lock_guard<std::mutex> lock(cmd_mutex_);

  // 重新 start 干掉上一条
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_az_ = 0.0;

  pid_fl_.reset();
  pid_fr_.reset();
  pid_bl_.reset();
  pid_br_.reset();
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  pid_fl_.reset();
  pid_fr_.reset();
  pid_bl_.reset();
  pid_br_.reset();

  left_front_wheel_joint_.setCommand(0.0);
  right_front_wheel_joint_.setCommand(0.0);
  left_back_wheel_joint_.setCommand(0.0);
  right_back_wheel_joint_.setCommand(0.0);
}

void HeroChassisController::callBack(const geometry_msgs::Twist::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_); //解锁.. 写入
  target_vx_ = msg->linear.x;
  target_vy_ = msg->linear.y;
  target_az_ = msg->angular.z;

}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}