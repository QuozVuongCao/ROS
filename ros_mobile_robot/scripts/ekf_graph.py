#!/usr/bin/env python3

import rospy
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from localizer_dwm1001.msg import Tag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, atan2
from matplotlib.animation import FuncAnimation

x_True = [[0.0, 3.2, 3.2, 0.0, 0.0], 
         [0.0, 0.0, 2.4, 2.4, 0.0]]
#x_True = [[0.34, 3.14, 3.14, 0.34, 0.34], 
#         [0.0, 0.0, 2.0, 2.0, 0.0]]
theta = np.linspace(0, 2*np.pi, 100)
x_circle = 0.0 + 1.3 * np.cos(theta)
y_circle = 1.3 + 1.3 * np.sin(theta)
vel = {'v': 0.0 , 'w': 0.0}
x_dr = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
th_IMU = []
x_DR = [[], [], []]
shape = np.zeros((3, 1))
x_Odom = shape
x_EKF = shape
x_AMCL = shape
x_UWB = shape
clicked_points1 = []
clicked_points2 = []

def subscriber_vel_callback(vel_data):
    global vel
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z
    dead_reckoning()
    
def subscriber_uwb_callback(uwb_data):
    global x_UWB
    pose = np.array([[uwb_data.x], [uwb_data.y], [uwb_data.z]])
    x_UWB = np.hstack((x_UWB, pose))
    x_UWB = x_UWB.tolist()
    
def subscriber_imu_callback(imu_data):
    global th_IMU
    orientation = np.radians(imu_data.z)
    orientation = atan2(sin(orientation), cos(orientation))
    th_IMU = np.hstack((th_IMU, orientation))

def subscriber_amcl_callback(amcl_data):
    global x_AMCL
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_AMCL = np.hstack((x_AMCL, pose))
    
def dead_reckoning():
    global previous_time, x_dr, vel, x_DR
    
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    
    x_dr['x'] += vel['v']*dt*cos(x_dr['yaw'] + 0.5*vel['w']*dt)
    x_dr['y'] += vel['v']*dt*sin(x_dr['yaw'] + 0.5*vel['w']*dt)
    x_dr['yaw'] += vel['w']*dt
    
    x_DR[0].append(x_dr['x'])
    x_DR[1].append(x_dr['y'])
    x_DR[2].append(x_dr['yaw'])
    x_DR[2] = [normalize_angle(angle) for angle in x_DR[2]]

def subscriber_odom_callback(odom_data):
    global x_Odom
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    rot = odom_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_Odom = np.hstack((x_Odom, pose))
    x_Odom = x_Odom.tolist()
    x_Odom[2] = [normalize_angle(angle) for angle in x_Odom[2]]
 
def subscriber_ekf_callback(ekf_data):
    global x_EKF
    x = ekf_data.pose.pose.position.x
    y = ekf_data.pose.pose.position.y
    rot = ekf_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_EKF = np.hstack((x_EKF, pose))
    x_EKF = x_EKF.tolist()
    x_EKF[2] = [normalize_angle(angle) for angle in x_EKF[2]]
    
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi      
 
def on_click(event):
    global fig, ax1, ax2
    if event.inaxes == ax1:
        x = event.xdata
        y = event.ydata
        clicked_points1.append((x, y))
        ax1.scatter(x, y, color='red', s=20, zorder=5)
        ax1.text(x, y, f'({x:.2f}, {y:.2f})', fontsize=10, color='black', ha='right', va='bottom')
    elif event.inaxes == ax2:
        x = event.xdata
        y = event.ydata
        clicked_points2.append((x, y))
        ax2.scatter(x, y, color='red', s=20, zorder=5)
        ax2.text(x, y, f'({x:.2f}, {y:.2f})', fontsize=10, color='black', ha='right', va='bottom')
    plt.draw()
        
def show_plot(i, ax1, ax2):
    global DR, x_UWB, x_AMCL, x_True, x_Odom, x_EKF, x_circle, y_circle
    global vel, previous_time, fig
    rospy.loginfo('Show plot')

    #plt.subplot(1, 2, 1, label='X-Y Graph')
    ax1.title.set_text('X-Y Graph')
    ax1.cla()
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.axis('equal')
    ax1.grid(True)
    linewidth = 1.0
    
    ax1.plot(x_UWB[0], x_UWB[1], '.g', linewidth = 0.1, label = 'xy_UWB')
    ax1.plot(x_True[0], x_True[1], '-b', linewidth = linewidth, label = 'xy_True')
    ax1.plot(x_DR[0], x_DR[1], '-k', linewidth=linewidth, label='xy_DR')
    #ax1.plot(x_Odom[0], x_Odom[1], '-m', linewidth = linewidth, label = 'xy_Odom')
    ax1.plot(x_EKF[0], x_EKF[1], '--r', linewidth = linewidth, label='xy_EKF')
    #ax1.plot(x_circle, y_circle, '-b', linewidth = linewidth, label = 'xy_True')
    #plt.plot(x_AMCL[0, 1:].flatten(), x_AMCL[1, 1:].flatten(), 'oc', linewidth = linewidth, label = 'xy_AMCL')
    
    for point in clicked_points1:
        ax1.scatter(point[0], point[1], color='red', s=20, zorder=5)
        ax1.text(point[0], point[1], f'({point[0]:.2f}, {point[1]:.2f})', fontsize=10, color='black', ha='right', va='bottom')
 
    ax1.legend(loc='upper right')
        
    #plt.subplot(1, 2, 2, label='Theta Graph')
    ax2.title.set_text('Theta Graph')
    ax2.cla()
    ax2.set_xlabel('Sample (n)')
    ax2.set_ylabel('Yaw (degree)')
    ax2.grid(True)
    
    ax2.plot(np.rad2deg(th_IMU), '-g', linewidth = linewidth, label='yaw_IMU')
    ax2.plot(np.rad2deg(x_DR[2]), '-k', linewidth = linewidth, label='th_DR')
    #ax2.plot(np.rad2deg(x_Odom[2]), '-m', linewidth = linewidth, label='th_Odom')
    ax2.plot(np.rad2deg(x_EKF[2]), '--r', linewidth = linewidth, label='th_EKF')
    #plt.plot(x_True[2], '-b', linewidth = linewidth, label='th_True')
    #plt.plot(x_AMCL[2, 1:].flatten(), '-c', linewidth = linewidth, label = 'yaw_AMCL')
    
    for point in clicked_points2:
        ax2.scatter(point[0], point[1], color='red', s=20, zorder=5)
        ax2.text(point[0], point[1], f'({point[0]:.2f}, {point[1]:.2f})', fontsize=10, color='black', ha='right', va='bottom')
    
    ax2.legend(loc='upper right')

def main():
    global fig, previous_time, vel, center, radius, theta, ax1, ax2
    global DR, x_UWB, x_AMCL, x_True, x_EKF, x_Odom
    rospy.init_node('node_ekf_graph')
    rospy.Subscriber('/dwm1001/tag', Tag, subscriber_uwb_callback)
    rospy.Subscriber('/odom', Odometry, subscriber_odom_callback)
    rospy.Subscriber('/odom_ekf', Odometry, subscriber_ekf_callback)
    #rospy.Subscriber('/imu', Imu, subscriber_imu_callback)
    rospy.Subscriber('/imu', Vector3, subscriber_imu_callback)
    rospy.Subscriber('/vel_pub', Twist, subscriber_vel_callback)
    #rospy.Subscriber('/agv/amcl_pose', PoseWithCovarianceStamped, subscriber_amcl_callback)
    #rospy.Subscriber('/agv/move_done', Bool, subscriber_move_callback)

    rospy.loginfo('Start node ekf_graph')
    previous_time = rospy.Time.now()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
    ani = FuncAnimation(fig, show_plot, fargs=(ax1, ax2), interval=10)
    plt.connect('button_press_event', on_click)
    plt.show()

if __name__ =='__main__':
    main()

