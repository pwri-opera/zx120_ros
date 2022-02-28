#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/QuaternionStamped.h>

class Upper_Arm : public hardware_interface::RobotHW
{
public:
  Upper_Arm();

  ros::NodeHandle nh;

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time, ros::Duration);

  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd_[5];
  double pos_[5];
  double vel_[5];
  double eff_[5];

  void jsCallback(const sensor_msgs::JointState::ConstPtr &msg_sub);
  void compassCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg_sub);
  void faketimeCallback(const sensor_msgs::JointState::ConstPtr &msg_sub);

  std_msgs::Float64 swing_setpoint,boom_setpoint, arm_setpoint, bucket_setpoint;
  ros::Publisher swing_setpoint_pub, boom_setpoint_pub, arm_setpoint_pub, bucket_setpoint_pub,swing_state_pub, boom_state_pub, arm_state_pub, bucket_state_pub;
  ros::Subscriber js_sub,compass_sub,fake_sub;
};

