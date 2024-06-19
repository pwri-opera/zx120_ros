#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
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


sensor_msgs::JointState fix_js_;

// geometry_msgs::Quaternion q_imu[NUM_AXIS];
// double base_link_pitch, base_link_roll;
// double body_link_pitch;

// void GetRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
// {
//     tf::Quaternion quat(q.x, q.y, q.z, q.w);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
// }

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
    is_bucket_imu_ = false;
    bucket_imu_ = *msg;
}


void Get_bucket_angle ()
{

    double angle = 0.0;
    double roll, pitch, yaw;
    tf2::Quaternion quat_swing, quat_bucket, quat_bucket_base_swing;
    const double l1(505), l2(460), l3(325), l4(362);
    // const double th_os_arm(0.070058), th_os_buck(1.865827), th_os_imu_buck(0.25);
    const double th_os_arm(0.0), th_os_buck(1.865827), th_os_imu_buck(0.00);
    
    if (is_bucket_imu_ != true) 
    {
        return;
    }
    if (is_swing_imu_ != true)
    {
        return;
    }

    tf2::convert (bucket_imu_.orientation, quat_bucket);
    tf2::convert (swing_imu_.orientation, quat_swing);

    quat_bucket_base_swing = quat_swing.inverse() * quat_bucket;
    tf2::Matrix3x3(quat_bucket_base_swing).getRPY(roll, pitch, yaw);

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
}



void AC58_js_callback (const sensor_msgs::JointState::ConstPtr& msg)
{
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
        }
        else if (msg->name[i] == "arm_joint")
        {
            fix_js_.position[ARM] = msg->position[i];
            fix_js_.velocity[ARM] = msg->velocity[i]; 
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

    ros::Publisher  fix_js_pub = nh.advertise<sensor_msgs::JointState> ("ac58_fix_bucket_joint_publisher/joint_states", 10);
    ros::Subscriber swing_imu_sub = nh.subscribe ("swing/g2_imu", 5, &swing_g2_callback);
    ros::Subscriber bucket_imu_sub = nh.subscribe ("bucket/g2_imu", 5, &bucket_g2_callback);

    ros::Subscriber ac58_js_sub = nh.subscribe ("ac58_joint_publisher/joint_states", 5, &AC58_js_callback);
    ros::Rate loop(50);

    while(ros::ok())
    {
        fix_js_.header.stamp = ros::Time::now();
        Get_bucket_angle();
        fix_js_.name = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint"};
        fix_js_pub.publish(fix_js_);
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}

