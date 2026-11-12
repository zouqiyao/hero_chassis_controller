#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include<mutex>
#include<geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h> 
#include<cmath>

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  void callBack(const geometry_msgs::Twist::ConstPtr& msg);
  void setpidcommand(const double &target_fl,const double &target_fr,const double &target_bl,
                     const double &target_br,const ros::Duration &period); //pid命令发送
  void packOdometry(const ros::Time & time,std::string father ,std::string child ,
                                            double vx , double vy ,double az ,double theta_);

 private:
  hardware_interface::JointHandle left_front_wheel_joint_;
  hardware_interface::JointHandle right_front_wheel_joint_;
  hardware_interface::JointHandle left_back_wheel_joint_;
  hardware_interface::JointHandle right_back_wheel_joint_;

  control_toolbox::Pid pid_fl_;
  control_toolbox::Pid pid_fr_;
  control_toolbox::Pid pid_bl_;
  control_toolbox::Pid pid_br_; //实例化四个 pid控制类

    double target_vx_;
    double target_vy_;
    double target_az_;  //共享指令, 收到第一条 cmd_vel
    std::mutex cmd_mutex_;

    ros::Publisher odom_pub;
    ros::Subscriber cmd_vel_sub;

    double kWheelRadius ;
    double k ;   //轮子

    double base_sx_;
    double base_sy_;
    double base_sz_; //base下的位移向量
    double base_taz_; //旋转角度

    double odom_sx_;
    double odom_sy_;
    double odom_sz_;

    double sum_odom_sx_;
    double sum_odom_sy_;
    double sum_odom_sz_;
    double sum_odom_theta_;

    double pi;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
