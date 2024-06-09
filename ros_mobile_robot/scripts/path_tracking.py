#!/usr/bin/env python3

import rospy
import math
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
import numpy as np
import matplotlib.pyplot as plt
import PyKDL

#x, y, th = [], [], []
x=0.0
y=0.0
theta=0.0
vel = {'v': 0.0 , 'w': 0.0}
vel_done = False
x1d_plot, y1d_plot =[], []
x1_plot, y1_plot = [], []
b = 0.5  # assume a value for b
Kx, Ky = 0.3, 0.3  # Controller gains


class PublishThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.rate = 50# Hz
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time_start = rospy.Time.now()
        self.condition = threading.Condition()
        self.done = False
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.velocity_publisher.get_num_connections() == 0:
            if i == 0 or i == 20:
                print("Waiting for subscriber to connect to {}".format(self.velocity_publisher.name))
                rospy.sleep(0.5)
                i += 1
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
            
    def shutdown(self):
        rospy.loginfo("Stop navigation sim")
        # Stop message
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)
        
    def publish_velocity(self, linear, angular):
        # Giới hạn vận tốc tuyến tính
        if linear > 1:
            linear = 1
        elif linear < 0:
            linear = 0
    
        # Giới hạn vận tốc góc
        if angular > 1:
            angular =1
        elif angular < 0:
            angular = 0
    
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self.velocity_publisher.publish(velocity_msg)
    
    def stop(self):
        rospy.loginfo("Stop navigation simulation")
        self.publish_velocity(0.0, 0.0)
        rospy.sleep(1)

def update_plot():
    plt.clf()  # Clear the current figure
    
    # Plot actual trajectory
    plt.plot(x1_plot, y1_plot, color='red', label='Actual Trajectory')
    
    # Plot desired trajectory
    plt.plot(x1d_plot, y1d_plot, color='blue', label='Desired Trajectory')
    
    plt.title('Plotting Data from Multiple Topics')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.pause(0.01)  # Add a small pause to allow the plot to update
    
#def subscriber_vel_callback(vel_data):
#    global vel_done, vel
#    vel_done = True
#    vel['v'] = vel_data.linear.x
#    vel['w'] = vel_data.angular.z
    
def subscriber_odom_callback(odom_data):
    global x, y, theta
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    rot = odom_data.pose.pose.orientation
    theta = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
#    x.append(x_odom)
#    y.append(y_odom)
#    th.append(yaw_odom)
 
#def subscriber_ekf_callback(ekf_data):
#    global x, y, theta
#    x = ekf_data.pose.pose.position.x
#    y = ekf_data.pose.pose.position.y
#    rot = ekf_data.pose.pose.orientation
#    theta = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
  

def main():
    global x, y,theta, x1_plot, y1_plot, x1d_plot, y1d_plot, R, w, b, Kx, Ky
    rospy.init_node('node_path_tracking')
    #rospy.Subscriber('/odom_ekf', Odometry, subscriber_odom_callback)
    rospy.Subscriber('/odom', Odometry, subscriber_odom_callback)
#    rospy.Subscriber('/vel_pub', Twist, subscriber_vel_callback)
    rate = rospy.Rate(50)
    publish_thread = PublishThread()
    rospy.loginfo('Start node path_tracking')
    plt.figure()
    try:
        while not rospy.is_shutdown():
                     
                current_time =rospy.Time.now()
                dt = (current_time - publish_thread.time_start).to_sec()
                publish_thread.time_start = current_time
          
                R=1.3
                w = 0.199555#0.05  # rad/s = 2*pi*f
                xd = R * math.cos(w * current_time.to_sec())
                yd = R * math.sin(w * current_time.to_sec())
                xPd = -R * w * math.sin(w * current_time.to_sec())  # Corrected sign
                yPd = R * w * math.cos(w * current_time.to_sec())
                xDPd = -R * w**2 * math.cos(w * current_time.to_sec())  # Corrected sign
                yDPd = -R * w**2 * math.sin(w * current_time.to_sec())  # Corrected sign
    
                #xd -> x1d
                th = math.atan2(yPd, xPd)
                x1d = xd + b * math.cos(th)
                x1d_plot.append(x1d)
                y1d = yd + b * math.sin(th)
                y1d_plot.append(y1d)
                wd = (yDPd * xPd - xDPd * yPd) / (xPd**2 + yPd**2)
                x1Pd = xPd - b * wd * math.sin(th)
                y1Pd = yPd + b * wd * math.cos(th)  # Corrected sign
    
                #odometry
#                x   += vel['v']*dt*math.cos(theta + 0.5*vel['w']*dt)
#                y   += vel['v']*dt*math.sin(theta + 0.5*vel['w']*dt)
#                theta += vel['w']*dt
#                theta = math.atan2(math.sin(theta), math.cos(theta))
                x1 = x + b * math.cos(theta) 
                x1_plot.append(x1)   
                y1 = y + b * math.sin(theta) 
                y1_plot.append(y1)
    
                #controller
                x_error = x1d - x1
                y_error = y1d - y1
                u1 = x1Pd + Kx * x_error #x1P= x1_dot
                u2 = y1Pd + Ky * y_error
    
                #linear, angullar
                v = u1 * math.cos(theta) + u2 * math.sin(theta)
                w = 1 / b * (u2 * math.cos(theta) - u1 * math.sin(theta))
                
                publish_thread.publish_velocity(v, w)
                update_plot()  # Call update_plot to refresh the plot
                print("x_error:",x_error)
                print("y_error:",y_error)
                print("-------------------------")
                print("u1:",u1)
                print("u2",u2)
                print("-------------------------")
                print("x1Pd", x1Pd)
                print("y1Pd", y1Pd)
                print("-------------------------")
                print("linear:", v)                
                print("angular:", w)
                print("-------------------------")
                
                
                
                rate.sleep()
              
                
    finally:
        publish_thread.stop()

if __name__ == '__main__':
    main()

