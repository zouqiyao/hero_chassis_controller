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
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom",10);
  base_sx_ = 0.0;
  base_sy_ = 0.0;
  base_sz_ = 0.0;
  base_taz_ = 0.0;

  odom_sx_ = 0.0;
  odom_sy_ = 0.0;
  odom_sz_ = 0.0;

  sum_odom_sx_ = 0.0;
  sum_odom_sy_ = 0.0;
  sum_odom_sz_ = 0.0;
  sum_odom_theta_ = 0.0;

  pi = 3.14159265358;
  return true;
}


void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

  double vx, vy, az; //临时存 .. 但是读写 target_vx_
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_); //解锁
    vx = target_vx_;
    vy = target_vy_;
    az = target_az_;
  }
  
  kWheelRadius = 0.07625;
  k = 0.4;

  const double target_fl = (vx - vy - k * az) / kWheelRadius;
  const double target_fr = (vx + vy + k * az) / kWheelRadius;
  const double target_bl = (vx + vy - k * az) / kWheelRadius;
  const double target_br = (vx - vy + k * az) / kWheelRadius;

  setpidcommand(target_fl,target_fr,target_bl,target_br,period);
  

  double v_fl = left_front_wheel_joint_.getVelocity();  //得到当前速度
  double v_fr = right_front_wheel_joint_.getVelocity();
  double v_bl = left_back_wheel_joint_.getVelocity();
  double v_br = right_back_wheel_joint_.getVelocity();

  const double base_vx_ = ( (v_fl + v_fr + v_bl + v_br)*kWheelRadius ) / 4.0;
  const double base_vy_ = ( (v_fr + v_bl - v_fl - v_br)*kWheelRadius ) / 4.0;
  const double base_az_ = ( (v_fr + v_br - v_fl - v_bl)*kWheelRadius ) / (4.0 * k);  //正解将底盘的速度求出

  // 计算当前坐标 里面的位移向量 
  base_sx_ = base_vx_ * period.toSec();
  base_sy_ = base_vy_ * period.toSec();
  base_sz_ = 0.0;
  base_taz_ = base_az_ * period.toSec();

  odom_sx_ = base_sx_*cos(sum_odom_theta_) - base_sy_*sin(sum_odom_theta_);
  odom_sy_ = base_sx_*sin(sum_odom_theta_) + base_sy_*cos(sum_odom_theta_);

  sum_odom_sx_ += odom_sx_;
  sum_odom_sy_ += odom_sy_;
  sum_odom_sz_ = 0.0;
  sum_odom_theta_ += base_taz_;  //这里就完成了 现实坐标下的位置积累

  if(sum_odom_theta_ >= 2*pi) sum_odom_theta_ -= 2*pi;
  if(sum_odom_theta_ <= -2*pi) sum_odom_theta_ +=2*pi;

  packOdometry(time,"odom","base_link",
               base_vx_,base_vy_,base_az_,sum_odom_theta_);

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
void HeroChassisController::packOdometry(const ros::Time & time,std::string father ,std::string child ,
                  double vx , double vy ,double az ,double theta_)
{
    nav_msgs::Odometry odom_msg;
  
    odom_msg.header.stamp = time; 
    odom_msg.header.frame_id = father; 
    odom_msg.child_frame_id = child;

    odom_msg.pose.pose.position.x = sum_odom_sx_;
    odom_msg.pose.pose.position.y = sum_odom_sy_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = az;

    odom_pub.publish(odom_msg);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}