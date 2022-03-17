#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
//#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>
//#include <can_msgs/Frame.h>

//#include <g2_publisher/g2_publisher.hpp>

sensor_msgs::JointState imubased_js;
double base_link_pitch, base_link_roll;
const int SWING=0;
const int BOOM=1;
const int ARM=2;
const int BUCKET=3;
const int NUM_AXIS=4;

void GetRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
{
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void normalize_PI(double& theta){
    while(abs(theta)>M_PI){
        if(theta > M_PI){
            theta -= M_PI*2.0;
        }
        else if(theta <= -M_PI){
            theta += M_PI*2.0;
        }
    }
}

double get_bucket_angle(double imu_angle){
    double link1(325), link2(362), link3(460), link4(505);
    double th_bk0(0.070058), th_bk3(1.865827);
    double vertial_link5_between_link1_link4 = sqrt(link1 * link1 + link4 * link4 - 2 * link1 * link4 * cos(imu_angle));    
    double th_bk2_1 = acos((link1 * link1 + vertial_link5_between_link1_link4 * vertial_link5_between_link1_link4 - link4 * link4) / (2 * link1 * vertial_link5_between_link1_link4));
    double th_bk2_2 = acos((link2 * link2 + vertial_link5_between_link1_link4 * vertial_link5_between_link1_link4 - link3 * link3) / (2 * link2 * vertial_link5_between_link1_link4));
    
    return th_bk0 + th_bk2_1 + th_bk2_2 + th_bk3 - M_PI;
}

void swing_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double yaw;

    GetRPY(msg->orientation, base_link_roll, base_link_pitch, yaw);
    base_link_roll *= -1.0; //符号反転
}

void boom_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double roll, pitch, yaw;

    GetRPY(msg->orientation, roll, pitch, yaw);
    roll *= -1.0; //符号反転

    imubased_js.position[BOOM] = pitch - base_link_pitch;
    imubased_js.velocity[BOOM] = msg->angular_velocity.y;
}

void arm_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double roll, pitch, yaw;

    GetRPY(msg->orientation, roll, pitch, yaw);
    roll *= -1.0; //符号反転

    imubased_js.position[ARM] = pitch - base_link_pitch - imubased_js.position[BOOM];
    imubased_js.velocity[ARM] = msg->angular_velocity.y;
}

void bucket_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double roll, pitch, yaw;

    GetRPY(msg->orientation, roll, pitch, yaw);
    roll *= -1.0; //符号反転

    imubased_js.position[BUCKET] = pitch - base_link_pitch - imubased_js.position[BOOM] - imubased_js.position[ARM];
    imubased_js.position[BUCKET] = get_bucket_angle(imubased_js.position[BUCKET]);
    imubased_js.velocity[BUCKET] = msg->angular_velocity.y;
}

void ac58_js_callback(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i=0;i < msg->name.size();i++){
        if((msg->name[i])=="swing_joint"){
            imubased_js.position[SWING] = msg->position[i];
            imubased_js.velocity[SWING] = msg->velocity[i];
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imubased_js_publisher");
    ros::NodeHandle nh;

    imubased_js.name.resize(NUM_AXIS);
    imubased_js.position.resize(NUM_AXIS);
    imubased_js.velocity.resize(NUM_AXIS);
    imubased_js.effort.resize(NUM_AXIS);

    ros::Publisher imubased_js_pub = nh.advertise<sensor_msgs::JointState>("imubased/joint_states", 10);
    ros::Subscriber swing_imu_sub = nh.subscribe("swing/g2_imu",5,&swing_g2_callback);
    ros::Subscriber boom_imu_sub = nh.subscribe("boom/g2_imu",5,&boom_g2_callback);
    ros::Subscriber arm_imu_sub = nh.subscribe("arm/g2_imu",5,&arm_g2_callback);
    ros::Subscriber bucket_imu_sub = nh.subscribe("bucket/g2_imu",5,&bucket_g2_callback);
    ros::Subscriber ac58_js_sub = nh.subscribe("ac58_joint_publisher/joint_states",5,&ac58_js_callback);

    ros::Rate loop(50);

    while(ros::ok())
    {
        imubased_js.header.stamp = ros::Time::now();
        imubased_js.name = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint"};

        imubased_js_pub.publish(imubased_js);

        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}