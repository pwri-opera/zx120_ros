#!/usr/bin/python
import  rospy
from    sensor_msgs.msg     import Imu
from    geometry_msgs.msg   import Quaternion
from    geometry_msgs.msg   import Vector3


def pub_bucket_acc_angvel_only_pitch (msg):

    global pub_bucket_imu_pitch

    p_acc       = Vector3 ()
    p_angvel    = Vector3 ()
    quat0       = Quaternion ()
    p_imu       = Imu ()

    p_acc.x  = msg.linear_acceleration.x
    p_acc.z  = msg.linear_acceleration.z

    p_angvel.y = msg.angular_velocity.y

    p_imu = msg
    p_imu.orientation         = quat0
    p_imu.linear_acceleration = p_acc
    p_imu.angular_velocity    = p_angvel

    pub_bucket_imu_pitch.publish (p_imu)


def init ():
    # global encorder_swing_cmd_pub, encorder_boom_cmd_pub, encorder_arm_cmd_pub, encorder_bucket_cmd_pub
    # global imu_swing_cmd_pub, imu_boom_cmd_pub, imu_arm_cmd_pub, imu_bucket_cmd_pub

    global pub_bucket_imu_pitch

    rospy.init_node ('bucket_pitch_acc_angvel', anonymous=True)
    pub_bucket_imu_pitch = rospy.Publisher('bucket/g2_imu_only_pitch/raw',  Imu, queue_size=10)
    rospy.Subscriber ("bucket/g2_imu", Imu, pub_bucket_acc_angvel_only_pitch)

    rospy.spin ()


if __name__ == '__main__':
    init ()