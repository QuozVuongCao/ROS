#!/usr/bin/env python3

import PyKDL
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from localizer_dwm1001.msg import Tag
import tf
import numpy as np
from numpy import sin, cos, deg2rad
from fusionEKF import FusionEKF
import matplotlib.pyplot as plt
from math import cos, sin, sqrt
import math

pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
vel = {'v': 0.0, 'w': 0.0}
allow_initialpose_pub = False
uwb_done = False
imu_done = False
ekf_done = False
first_scan = False
sub_data = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

def publish_odometry(position, rotation):
    odom_ekf = Odometry()
    odom_ekf.header.stamp = rospy.Time.now()
    odom_ekf.header.frame_id = 'odom_ekf'
    odom_ekf.child_frame_id = 'base_footprint'
    odom_ekf.pose.pose.position = Point(*position)
    odom_ekf.pose.pose.orientation = Quaternion(*rotation)
    odom_ekf.twist.twist.linear  = Vector3(vel['v'], 0, 0)
    odom_ekf.twist.twist.angular = Vector3(0, 0, vel['w'])
    odom_pub_ekf.publish(odom_ekf)

def transform_odometry(position, rotation):
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'odom_ekf'
    trans.child_frame_id = 'base_footprint'
    trans.transform.translation = Vector3(*position)
    trans.transform.rotation = Quaternion(*rotation)
    odom_broadcaster.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = Point(*position)
    initialpose.pose.pose.orientation = Quaternion(*rotation)
    initialpose_pub.publish(initialpose)

def subscriber_uwb_callback(uwb_data):
    global sub_data, uwb_done
    sub_data['x'] = uwb_data.x
    sub_data['y'] = uwb_data.y
    uwb_done = True
  
def subscriber_vel_callback(vel_data):
    global vel
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_imu_callback(imu_data):
    global sub_data, imu_done
    sub_data['yaw'] = np.deg2rad(imu_data.z)
    imu_done = True

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub, pose
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pose['x']) > 0.03 or abs(y - pose['y']) > 0.03 or abs(yaw - pose['yaw']) > np.deg2rad(1):
        allow_initialpose_pub = True	

def compute_ekf_localization():
    global previous_time, first_scan, pose, uwb_done, ekf_done, imu_done
    global x_posterior, P_posterior
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # if not first_scan:
    #     if not imu_done:
    #         return
    #     first_scan = True
    #     ekf_all.x = np.array([[uwb_data.x], [uwb_data.y], [imu_data_yaw]])
    #     return

    u = np.array([[vel['v']], [vel['w']]])
    if not uwb_done:
       z = np.array([sub_data['yaw']])
       x_posterior, P_posterior = ekf_imu.predict_update(x_posterior, P_posterior, z, u, dt)
    else:
       z = np.array([[sub_data['x']], [sub_data['y']], [sub_data['yaw']]])
       x_posterior, P_posterior = ekf_all.predict_update(x_posterior, P_posterior, z, u, dt)
       
    #z = np.array([[sub_data['x']], [sub_data['y']]])
    #x_posterior, P_posterior = ekf_uwb.predict_update(x_posterior, P_posterior, z, u, dt)
    uwb_done = False

    pose['x']   = x_posterior[0,0]
    pose['y']   = x_posterior[1,0]
    pose['yaw'] = x_posterior[2,0]
    ekf_done = True
    
def main():
    rospy.init_node('node_ekf_localization')
    global odom_pub_ekf, initialpose_pub, odom_broadcaster, pose
    global allow_initialpose_pub, vel

    odom_pub_ekf = rospy.Publisher('/odom_ekf', Odometry, queue_size=10)
    initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('/vel_pub', Twist, subscriber_vel_callback)
    rospy.Subscriber('/imu', Vector3, subscriber_imu_callback)
    rospy.Subscriber('/dwm1001/tag', Tag, subscriber_uwb_callback)
    #rospy.Subscriber('/imu', Imu, subscriber_imu_callback)
    # rospy.Subscriber('/agv/amcl_pose', PoseWithCovarianceStamped, subcriber_amcl_callback)

    global previous_time
    previous_time = rospy.Time.now()

    rospy.loginfo('Start node ekf_localization')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        compute_ekf_localization()
        if ekf_done:
            position = (pose['x'], pose['y'], 0)
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            transform_odometry(position, rotation)
            # if allow_initialpose_pub:
            #     allow_initialpose_pub = False
                # publisher_initialpose(position, rotation)
            rospy.loginfo(pose)
        rate.sleep()
    
if __name__ =='__main__':
    dim_x = 3
    noise_x = 0.025
    noise_y = 0.025
    noise_yaw = np.deg2rad(20)
    noise_v = 0.03
    noise_w = 0.02
    P = np.diag([.01, .01, .001])**2
    Q = np.diag([noise_v, noise_w])**2
        
    #IMU
    ekf_imu = FusionEKF(dim_x=dim_x, dim_z=1)
    ekf_imu.P = P
    ekf_imu.Q = Q
    ekf_imu.R = np.diag([noise_yaw])**2

    #UWB
    ekf_uwb = FusionEKF(dim_x=dim_x, dim_z=2)
    ekf_uwb.P = P
    ekf_uwb.Q = Q
    ekf_uwb.R = np.diag([noise_x, noise_y])**2
    
    #UWB+IMU
    ekf_all = FusionEKF(dim_x=dim_x, dim_z=3)
    ekf_all.P = P
    ekf_all.Q = Q
    ekf_all.R = np.diag([noise_x, noise_y, noise_yaw])**2

    x_posterior = np.zeros((3, 1))
    P_posterior = P

    main()
