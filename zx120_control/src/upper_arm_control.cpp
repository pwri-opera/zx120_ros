#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <upper_arm_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "upper_arm_control");
  ros::NodeHandle nh;

  Upper_Arm uarm;
  controller_manager::ControllerManager cm(&uarm, uarm.nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    ros::Time now = uarm.getTime();
    ros::Duration dt = uarm.getPeriod();

    uarm.read(now, dt);
    cm.update(now, dt);

    uarm.write(now, dt);
    dt.sleep();
  }
  spinner.stop();

  return 0;
}

