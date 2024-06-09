#!/usr/bin/env python3

from flask import Flask, render_template, request
import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
from tf.transformations import quaternion_from_euler
import threading
import subprocess
import json
import psutil
import signal
import os

app = Flask(__name__)

class PublishThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_speed = 0.3
        self.angular_speed = 1.0
        self.linear_step = 0.1
        self.angular_step = 0.1
        self.condition = threading.Condition()
        self.done = False

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            with self.condition:
                if self.done:
                    break
            rate.sleep()

    def publish_velocity(self, linear, angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self.velocity_publisher.publish(velocity_msg)

class NavGUI:
    def __init__(self):
        self.fixed_points = [
            (0.0, 0.0, 0.0),
            (2.5, -1.8, 90.0),
            (2.5, 1.2, 180.0),
            (-4.0, 1.2, -90.0)]
            
        self.initROS()

    def initROS(self):
        rospy.init_node('gui_node', anonymous=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def send_pose(self, x, y, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, math.radians(yaw))
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Sent goal: x={}, y={}, yaw={}".format(x, y, yaw))

rospy.init_node('gui_node', anonymous=True)
publish_thread = PublishThread()
publish_thread.start()
nav_gui = NavGUI()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/forward')
def forward():
    publish_thread.publish_velocity(0.3, 0.0)
    return 'Moving forward'

@app.route('/backward')
def backward():
    publish_thread.publish_velocity(-0.3, 0.0)
    return 'Moving backward'

@app.route('/left')
def left():
    publish_thread.publish_velocity(0.0, 1.0)
    return 'Turning left'

@app.route('/right')
def right():
    publish_thread.publish_velocity(0.0, -1.0)
    return 'Turning right'

@app.route('/stop')
def stop():
    publish_thread.publish_velocity(0.0, 0.0)
    return 'Stopping'

@app.route('/fixed_point/<int:point_id>')
def fixed_point(point_id):
    if point_id >= 0 and point_id < len(nav_gui.fixed_points):
        x, y, yaw = nav_gui.fixed_points[point_id]
        nav_gui.send_pose(x, y, yaw)
        return f'Moving to fixed point {point_id}'
    else:
        return 'Invalid fixed point ID'

@app.route('/send_point', methods=['POST'])
def send_point():
    if request.method == 'POST':
        data = request.json
        x = data['x']
        y = data['y']
        yaw = data['yaw']
        nav_gui.send_pose(x, y, yaw)
        return f'Sending point: x={x}, y={y}, yaw={yaw}'
    else:
        return 'Method not allowed'

@app.route('/start_path_tracking')
def start_path_tracking():
    # Start path tracking
    subprocess.Popen(["rosrun", "ros_mobile_robot", "odometry.py"])
    subprocess.Popen(["rosrun", "ros_mobile_robot", "path_tracking.py"])
    return 'Started Path Tracking'

@app.route('/navigation')
def navigation():
    # start navigation
    subprocess.Popen(["roslaunch", "ros_mobile_robot", "navigation.launch"])
    
    return 'Started Navigation'
       
@app.route('/move_circle')
def move_circle():
    # circle tracking
    subprocess.Popen(["rosrun", "ros_mobile_robot", "plot_circle.py"])
    subprocess.Popen(["rosrun", "ros_mobile_robot", "move_circle.py"])
    return 'Started move circle'
@app.route('/emergency')
def emergency():
    # Dừng tiến trình navigation.py
    for proc in psutil.process_iter():
        try:
            cmdline = " ".join(proc.cmdline())
            if "navigation" in cmdline:
                proc.send_signal(signal.SIGINT)  # Gửi tín hiệu Ctrl+C
        except psutil.NoSuchProcess:
            pass

    # Dừng tiến trình path_tracking.py
    for proc in psutil.process_iter():
        try:
            cmdline = " ".join(proc.cmdline())
            if "path_tracking" in cmdline:
                proc.send_signal(signal.SIGINT)  # Gửi tín hiệu Ctrl+C
        except psutil.NoSuchProcess:
            pass

     #Dừng tiến trình move_circle.py
 #   for proc in psutil.process_iter():
 #       try:
 #           cmdline = " ".join(proc.cmdline())
 #           if "move_circle" in cmdline:
 #               proc.send_signal(signal.SIGINT)  # Gửi tín hiệu Ctrl+C
 #       except psutil.NoSuchProcess:
 #           pass
    #Dừng tiến trình plot_circle.py
 #   for proc in psutil.process_iter():
 #       try:
 #           cmdline = " ".join(proc.cmdline())
 #           if "plot" in cmdline:
 #               proc.send_signal(signal.SIGINT)  # Gửi tín hiệu Ctrl+C
 #       except psutil.NoSuchProcess:
 #           pass


    

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
#    app.run(host='192.168.1.67', port=8080, debug=True)



