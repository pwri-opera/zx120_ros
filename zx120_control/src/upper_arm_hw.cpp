#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <upper_arm_hw.h>
#include <iostream> // for debug
#include <math.h>

Upper_Arm::Upper_Arm()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_1("swing_joint", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface.registerHandle(state_handle_1);

  hardware_interface::JointStateHandle state_handle_2("boom_joint", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface.registerHandle(state_handle_2);

  hardware_interface::JointStateHandle state_handle_3("arm_joint", &pos_[2], &vel_[2], &eff_[2]);
  jnt_state_interface.registerHandle(state_handle_3);

  hardware_interface::JointStateHandle state_handle_4("bucket_joint", &pos_[3], &vel_[3], &eff_[3]);
  jnt_state_interface.registerHandle(state_handle_4);

  hardware_interface::JointStateHandle state_handle_5("bucket_end_joint", &pos_[4], &vel_[4], &eff_[4]);
  jnt_state_interface.registerHandle(state_handle_5);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("swing_joint"), &cmd_[0]);
  jnt_pos_interface.registerHandle(pos_handle_1);

  hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("boom_joint"), &cmd_[1]);
  jnt_pos_interface.registerHandle(pos_handle_2);

  hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("arm_joint"), &cmd_[2]);
  jnt_pos_interface.registerHandle(pos_handle_3);

  hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("bucket_joint"), &cmd_[3]);
  jnt_pos_interface.registerHandle(pos_handle_4);

  hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("bucket_end_joint"), &cmd_[4]);
  jnt_pos_interface.registerHandle(pos_handle_5);

  registerInterface(&jnt_pos_interface);

  swing_setpoint_pub = nh.advertise<std_msgs::Float64>("swing/setpoint", 100);
  boom_setpoint_pub = nh.advertise<std_msgs::Float64>("boom/setpoint", 100);
  arm_setpoint_pub = nh.advertise<std_msgs::Float64>("arm/setpoint", 100);
  bucket_setpoint_pub = nh.advertise<std_msgs::Float64>("bucket/setpoint", 100);

  swing_state_pub = nh.advertise<std_msgs::Float64>("swing/state", 100);
  boom_state_pub = nh.advertise<std_msgs::Float64>("boom/state", 100);
  arm_state_pub = nh.advertise<std_msgs::Float64>("arm/state", 100);
  bucket_state_pub = nh.advertise<std_msgs::Float64>("bucket/state", 100);

  js_sub = nh.subscribe("sub_joint_states", 10, &Upper_Arm::jsCallback, this);
  fake_sub = nh.subscribe("fake_joint_state", 10, &Upper_Arm::faketimeCallback, this);
}

void Upper_Arm::faketimeCallback(const sensor_msgs::JointState::ConstPtr &msg_sub){
  pos_[4] = msg_sub->position[0];
}

void Upper_Arm::jsCallback(const sensor_msgs::JointState::ConstPtr &msg_sub)
{
  int msg_size = msg_sub->position.size();
  for (int i = 0; i < msg_size; i++)
  {
    if(msg_sub->name[i] == "swing_joint")
      pos_[0] = msg_sub->position[i];
    else if (msg_sub->name[i] == "boom_joint")
      pos_[1] = msg_sub->position[i];
    else if (msg_sub->name[i] == "arm_joint")
      pos_[2] = msg_sub->position[i];
    else if (msg_sub->name[i] == "bucket_joint")
      pos_[3] = msg_sub->position[i];
  }
}

void Upper_Arm::read(ros::Time time, ros::Duration period)
{
}

void Upper_Arm::write(ros::Time time, ros::Duration period)
{
  swing_setpoint.data = cmd_[0];
  boom_setpoint.data = cmd_[1];
  arm_setpoint.data = cmd_[2];
  bucket_setpoint.data = cmd_[3];

  swing_setpoint_pub.publish(swing_setpoint);
  boom_setpoint_pub.publish(boom_setpoint);
  arm_setpoint_pub.publish(arm_setpoint);
  bucket_setpoint_pub.publish(bucket_setpoint);

  std_msgs::Float64 tmp;
  tmp.data = pos_[0];
  swing_state_pub.publish(tmp);
  tmp.data = pos_[1];
  boom_state_pub.publish(tmp);
  tmp.data = pos_[2];
  arm_state_pub.publish(tmp);
  tmp.data = pos_[3];
  bucket_state_pub.publish(tmp);
}
