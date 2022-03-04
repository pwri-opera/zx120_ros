#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import tf

pose = PoseStamped()

is_sub = bool()

def pose_cb(data):
    global pose
    pose = data
    global is_sub
    is_sub = True



def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


rospy.init_node('pose_to_odom')

vicon_sub = rospy.Subscriber(
    '/zx120/map2base_pose', PoseStamped, pose_cb, queue_size=10)
odom_pub = rospy.Publisher('/zx120/gnss_odom', Odometry, queue_size=10)

rate = rospy.Rate(50.0)
counter = 0
x = 0.
y = 0.

dt = 1./50.

while not rospy.is_shutdown():

    (v_roll, v_pitch, v_yaw) = quaternion_to_euler_angle(pose.pose.orientation.w,
                                                         pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
    v_phi = float((v_roll))
    v_theta = float((v_pitch))
    v_psi = float((v_yaw))

    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z

    yaw = math.radians(v_psi)

    if counter > 0:
        vel_x_world = (x - x_prev) / dt
        vel_y_world = (y - y_prev) / dt

        x_prev = x
        y_prev = y

        twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
        twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'gnss/base_link'
        odom.header.stamp = rospy.Time.now()

        odom.pose.pose.position.x = pose.pose.position.x
        odom.pose.pose.position.y = pose.pose.position.y
        odom.pose.pose.position.z = pose.pose.position.z

        odom.pose.pose.orientation.x = pose.pose.orientation.x
        odom.pose.pose.orientation.y = pose.pose.orientation.y
        odom.pose.pose.orientation.z = pose.pose.orientation.z
        odom.pose.pose.orientation.w = pose.pose.orientation.w

        odom.twist.twist.linear.x = twist_x
        odom.twist.twist.linear.y = twist_y
        odom.twist.twist.linear.z = (z - z_prev) / dt
        z_prev = z

        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = 0.

        if is_sub == True:
            odom_pub.publish(odom)
            # br = tf.TransformBroadcaster()
            # br.sendTransform((x, y, z), [pose.pose.orientation.x, pose.pose.orientation.y,
            #                          pose.pose.orientation.z, pose.pose.orientation.w], rospy.Time.now(), "gnss/base_link", "map")
            is_sub = False

    else:
        x_prev = x
        y_prev = y
        z_prev = z
        counter += 1

    rate.sleep()
