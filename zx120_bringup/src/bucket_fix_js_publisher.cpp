#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf2/convert.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


const int SWING=0;
const int BOOM=1;
const int ARM=2;
const int BUCKET=3;
const int NUM_AXIS=4;

sensor_msgs::Imu swing_imu_;
sensor_msgs::Imu bucket_imu_;

bool is_swing_imu_;
bool is_bucket_imu_;
bool is_ac58_js_;

sensor_msgs::JointState fix_js_;

double boom_angle_ = 0.0;
double arm_angle_ = 0.0;

// Debug
std_msgs::Float64 angle_msg_;
sensor_msgs::Imu quat_swing_ref_;

double normalize_PI(double theta)
{
    while (abs(theta)>M_PI)
    {
        if (theta > M_PI)
        {
            theta -= M_PI*2.0;
        }
        else if (theta <= -M_PI)
        {
            theta += M_PI*2.0;
        }
    }
    return theta;
}

void swing_g2_callback (const sensor_msgs::Imu::ConstPtr& msg)
{
    is_swing_imu_ = true;
    swing_imu_ = *msg;
}

void bucket_g2_callback (const sensor_msgs::Imu::ConstPtr& msg)
{
    is_bucket_imu_ = true;
    bucket_imu_ = *msg;
}


void Get_bucket_angle ()
{

    double angle = 0.0;
    double roll, pitch, yaw;
    double s_roll, s_pitch, s_yaw;
    tf2::Quaternion quat_swing, quat_swing_yaw, quat_bucket, quat_bucket_base_swing;
    const double l1(505), l2(460), l3(325), l4(362);
    // const double th_os_arm(0.070058), th_os_buck(1.865827), th_os_imu_buck(0.25);
    // const double th_os_arm(0.0), th_os_buck(1.865827), th_os_imu_buck(0.00);
    const double th_os_arm(0.00), th_os_buck(1.865827), th_os_imu_buck(0.0);
    
    if (is_bucket_imu_ != true || is_swing_imu_ != true || is_ac58_js_ != true) 
    {
        return;
    }

    tf2::convert (bucket_imu_.orientation, quat_bucket);
    tf2::convert (swing_imu_.orientation, quat_swing);

    tf2::Quaternion quat_yaw180;
    // quat_yaw180.setRPY (0, 0, M_PI);
    // quat_swing = quat_yaw180 * quat_swing;

    tf2::Matrix3x3(quat_swing).getRPY(s_roll, s_pitch, s_yaw);
    quat_swing.setRPY(-s_roll, s_pitch, 0.0);       // imu の yaw の値は信用しない

    quat_swing_yaw.setRPY(0.0, 0.0, fix_js_.position[SWING]);
    quat_swing = quat_swing * quat_swing_yaw.inverse();     /* swing -> baseの角度分のオフセットを取り込む */

    quat_bucket_base_swing = quat_swing.inverse() * quat_bucket;
    tf2::Matrix3x3(quat_bucket_base_swing).getRPY(roll, pitch, yaw);

    // roll 軸反転に関する対処
    if ( roll < -M_PI/2.0 ||  roll >  M_PI/2.0 )
    {
        pitch = M_PI - pitch; 
    }
    pitch = normalize_PI(pitch);

    angle = pitch - fix_js_.position[BOOM] - fix_js_.position[ARM];
    angle = normalize_PI(angle);

    double th_a = angle - th_os_imu_buck - th_os_arm;
    double lx = sqrt(l3*l3 + l1*l1 - 2*l1*l3*cos(th_a));
    double alpha = acos((l3-l1*cos(th_a))/lx);
    double beta = acos((l3*l3 + l1*l1 - l2*l2 + l4*l4 - 2*l1*l3*cos(th_a))/(2*l4*lx));
    double th_buck =  - M_PI + alpha + beta + th_os_buck;

    th_buck = normalize_PI(th_buck);

    fix_js_.position[BUCKET] = th_buck;
    fix_js_.velocity[BUCKET] = bucket_imu_.angular_velocity.y;

    // Debug
    // angle_msg_.data = pitch;
    // ROS_INFO ("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
    // bucket_imu_.header.stamp.toSec(),
    // roll,
    // pitch,
    // yaw,
    // quat_bucket_base_swing.x(),
    // quat_bucket_base_swing.y(),
    // quat_bucket_base_swing.z(),
    // quat_bucket_base_swing.w(),
    // angle,
    // th_buck
    // );

    std::cout << s_roll  << ","
              << s_pitch << ","
              << s_yaw   << ","
              << boom_angle_ << ","
              << arm_angle_ << ","
              << std::endl;

    geometry_msgs::Quaternion quat_swing_ref_msg;
    quat_swing_ref_msg = tf2::toMsg (quat_swing);
    quat_swing_ref_ = swing_imu_;
    quat_swing_ref_.orientation = quat_swing_ref_msg;
}



void AC58_js_callback (const sensor_msgs::JointState::ConstPtr& msg)
{
    is_ac58_js_ = true;
    fix_js_.header.stamp = msg->header.stamp;

    for (int i=0; i<msg->name.size(); i++) 
    {
        if (msg->name[i] == "swing_joint")
        {
            fix_js_.position[SWING] = msg->position[i];
            fix_js_.velocity[SWING] = msg->velocity[i];
        }
        else if (msg->name[i] == "boom_joint")
        {
            fix_js_.position[BOOM] = msg->position[i];
            fix_js_.velocity[BOOM] = msg->velocity[i];
            boom_angle_ = msg->position[i];
        }
        else if (msg->name[i] == "arm_joint")
        {
            fix_js_.position[ARM] = msg->position[i];
            fix_js_.velocity[ARM] = msg->velocity[i];
            arm_angle_ = msg->position[i];
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"bucket_fix_js_publisher");
    ros::NodeHandle nh;

    fix_js_.name.resize(NUM_AXIS);
    fix_js_.position.resize(NUM_AXIS);
    fix_js_.velocity.resize(NUM_AXIS);
    fix_js_.effort.resize(NUM_AXIS);

    is_swing_imu_   = false;
    is_bucket_imu_  = false;
    is_ac58_js_     = false;

    ros::Publisher  fix_js_pub = nh.advertise<sensor_msgs::JointState> ("ac58_fix_bucket_joint_publisher/joint_states", 10);
    ros::Subscriber swing_imu_sub = nh.subscribe ("swing/g2_imu_localframe", 5, &swing_g2_callback);
    ros::Subscriber bucket_imu_sub = nh.subscribe ("bucket/g2_imu", 5, &bucket_g2_callback);

    ros::Subscriber ac58_js_sub = nh.subscribe ("ac58_joint_publisher/joint_states", 5, &AC58_js_callback);

    // Debug
    ros::Publisher  p_angle_pub = nh.advertise<std_msgs::Float64> ("g_angle", 10);
    ros::Publisher  swing_ref_pub = nh.advertise<sensor_msgs::Imu> ("swing/g2_imu/ref", 10);
    
    // ROS_INFO(" 0, 0, 0, 0, time, roll, pitch, yaw, quat.x, quat.y, quat.z, quat.w, angle, th_buck");
    std::cout << "swing_roll, swing_pitch, swing_yaw, boom_ang, arm_ang" << std::endl; 

    ros::Rate loop(50);

    while(ros::ok())
    {
        fix_js_.header.stamp = ros::Time::now();
        Get_bucket_angle();
        fix_js_.name = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint"};
        fix_js_pub.publish (fix_js_);

        p_angle_pub.publish (angle_msg_);
        swing_ref_pub.publish (quat_swing_ref_); 
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}

