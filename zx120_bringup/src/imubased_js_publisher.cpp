#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <can_msgs/Frame.h>

//#include <g2_publisher/g2_publisher.hpp>

const int SWING=0;
const int BOOM=1;
const int ARM=2;
const int BUCKET=3;
const int NUM_AXIS=4;

sensor_msgs::JointState imubased_js;
geometry_msgs::Quaternion q_imu[NUM_AXIS];
double base_link_pitch, base_link_roll;

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

double get_bucket_angle(double angle){
    const double l1(505), l2(460), l3(325), l4(362);
    const double th_os_arm(0.070058), th_os_buck(1.865827), th_os_imu_buck(0.25);
    
    double th_a = angle - th_os_imu_buck - th_os_arm;

    double lx = sqrt(l3*l3 + l1*l1 - 2*l1*l3*cos(th_a));

    double alpha = acos((l3-l1*cos(th_a))/lx);
    double beta = acos((l3*l3 + l1*l1 - l2*l2 + l4*l4 - 2*l1*l3*cos(th_a))/(2*l4*lx));

    double th_buck =  - M_PI + alpha + beta + th_os_buck;

    normalize_PI(th_buck);
    //ROS_INFO("input_angle=%f, /_BAD=%f, lx=%f, alpha=%f, beta=%f, output_angle=%f", angle, th_a, lx, alpha, beta, th_buck);
    return th_buck;
}

void swing_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double imu_r, imu_p, imu_y;
    tf2::Quaternion odom2base_quat_tf, rot2base_quat_tf;
    q_imu[SWING] = msg->orientation;

    rot2base_quat_tf.setRPY(0,0,imubased_js.position[SWING]);
    GetRPY(q_imu[SWING], imu_r, imu_p, imu_y);
    imu_r *= -1.0;

    odom2base_quat_tf.setRPY(imu_r, imu_p, 0.0);

    odom2base_quat_tf = odom2base_quat_tf * rot2base_quat_tf.inverse();/* swing -> baseの角度分のオフセットを取り込む */
    odom2base_quat_tf.normalize();

    double odom2base_roll, odom2base_pitch, odom2base_yaw;
    geometry_msgs::Quaternion tmp;
    tmp = tf2::toMsg(odom2base_quat_tf);

    double yaw;
    GetRPY(tmp, base_link_roll, base_link_pitch, yaw);
    base_link_roll *= -1.0; //符号反転
}

void boom_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double angle = 0.0;
    tf2::Quaternion quat0, quat1;

    q_imu[BOOM] = msg->orientation;

    tf2::convert(q_imu[BOOM], quat1);
    tf2::convert(q_imu[SWING], quat0);

    angle = -tf2::angleShortestPath(quat0, quat1);

    imubased_js.position[BOOM] = angle;    
    imubased_js.velocity[BOOM] = msg->angular_velocity.y;
}

void arm_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double angle = 0.0;
    tf2::Quaternion quat0, quat1;

     q_imu[ARM] = msg->orientation;

    tf2::convert(q_imu[ARM], quat1);
    tf2::convert(q_imu[BOOM], quat0);

    angle = tf2::angleShortestPath(quat0, quat1);

    imubased_js.position[ARM] = angle;
    imubased_js.velocity[ARM] = msg->angular_velocity.y;
}

void bucket_g2_callback(const sensor_msgs::Imu::ConstPtr& msg){
    double angle = 0.0;
    tf2::Quaternion quat0, quat1;

    q_imu[BUCKET] = msg->orientation;

    tf2::convert(q_imu[BUCKET], quat1);
    tf2::convert(q_imu[ARM], quat0);

    angle = tf2::angleShortestPath(quat0, quat1);
    normalize_PI(angle);

    imubased_js.position[BUCKET] = get_bucket_angle(angle);
    imubased_js.velocity[BUCKET] = msg->angular_velocity.y;
}

void ac58_js_callback(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i=0;i < msg->name.size();i++){
        if((msg->name[i])=="rotator_joint"){
        //if((msg->name[i])=="swint_joint"){
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