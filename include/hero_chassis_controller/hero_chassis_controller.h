#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

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

 private:
  hardware_interface::JointHandle left_front_wheel_joint_;
  hardware_interface::JointHandle right_front_wheel_joint_;
  hardware_interface::JointHandle left_back_wheel_joint_;
  hardware_interface::JointHandle right_back_wheel_joint_;

  control_toolbox::Pid pid_fl_;
  control_toolbox::Pid pid_fr_;
  control_toolbox::Pid pid_bl_;
  control_toolbox::Pid pid_br_; //实例化四个 pid控制类

  double target_vel;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
