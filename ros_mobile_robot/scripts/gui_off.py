#!/usr/bin/env python3

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QGridLayout, QGroupBox
from rospy import init_node, Publisher
from geometry_msgs.msg import PoseStamped, Twist
import math
from tf.transformations import quaternion_from_euler
import subprocess
import os
from std_msgs.msg import String
import tkinter as tk
import threading
from PyQt5.QtCore import Qt
import yaml
from ruamel.yaml import YAML

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
        self.start()

    def publish_velocity(self, linear, angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.linear.y = 0.0
        velocity_msg.linear.z = 0.0
        velocity_msg.angular.x = 0.0
        velocity_msg.angular.y = 0.0
        velocity_msg.angular.z = angular
        self.velocity_publisher.publish(velocity_msg)

class NavGUI(QWidget):

    def __init__(self):
        super(NavGUI, self).__init__()
        self.initUI()
        self.initROS()
        self.publish_thread = PublishThread()
        self.counter = 0

    def initUI(self):
        self.setWindowTitle('MY GUI')
        layout = QGridLayout()
        
        group_1 = QGroupBox('Teleop')
        layout_1 = QVBoxLayout()
        self.w_button = QPushButton('FORWARD')
        self.s_button = QPushButton('BACKWARD')
        self.a_button = QPushButton('TURN LEFT')
        self.d_button = QPushButton('TURN RIGHT')
        self.q_button = QPushButton('STOP')
        self.w_button.clicked.connect(lambda: self.send_teleop_command('w'))
        self.s_button.clicked.connect(lambda: self.send_teleop_command('s'))
        self.a_button.clicked.connect(lambda: self.send_teleop_command('a'))
        self.d_button.clicked.connect(lambda: self.send_teleop_command('d'))
        self.q_button.clicked.connect(lambda: self.send_teleop_command('q'))
        layout_1.addWidget(self.w_button)
        layout_1.addWidget(self.s_button)
        layout_1.addWidget(self.a_button)
        layout_1.addWidget(self.d_button)
        layout_1.addWidget(self.q_button)
        group_1.setLayout(layout_1)
        
        group_2 = QGroupBox('Goal')
        layout_2 = QVBoxLayout()
        self.coord_label = QLabel('Enter coordinates (x,y,yaw(degree)):')
        self.coord_input = QLineEdit()
        self.send_button = QPushButton('Send Point')
        self.send_button.clicked.connect(self.send_goal)
        layout_2.addWidget(self.coord_label)
        layout_2.addWidget(self.coord_input)
        layout_2.addWidget(self.send_button)
        group_2.setLayout(layout_2)
        
        group_3 = QGroupBox('Fixed Point')
        layout_3 = QVBoxLayout()
        # Danh sách các điểm cố định và nhãn tương ứng
        self.fixed_points = [
            (0.0, 0.0, 0.0),
            (2.5, -0.75, 90.0),
            (-3.5, -1.0, 0.0),
            (-4.7, 2.1, 0.0)]

        # Tạo nút nhấn và nhãn tương ứng cho mỗi điểm cố định
        for i, point in enumerate(self.fixed_points):
            button_label = f'Point {i+1}: {point}'
            button = QPushButton(button_label)
            button.clicked.connect(lambda _, idx=i: self.send_fixed_point(idx))
            layout_3.addWidget(button)
        group_3.setLayout(layout_3)
        
        group_4 = QGroupBox('Nav/Fit')
        layout_4 = QVBoxLayout()
        self.nav_fit_button = QPushButton('Mode')
        self.nav_fit_button.clicked.connect(self.nav_fit_toggle)
        layout_4.addWidget(self.nav_fit_button)
        group_4.setLayout(layout_4)
        
        group_5 = QGroupBox('Velocity Settings') # Thêm nhóm cho cài đặt vận tốc
        layout_5 = QVBoxLayout()
        self.max_vel_x_label = QLabel('Max Linear Velocity (m/s):')
        self.max_vel_x_input = QLineEdit()
        self.min_vel_x_label = QLabel('Min Linear Velocity (m/s):')
        self.min_vel_x_input = QLineEdit()
        self.set_vel_button = QPushButton('Set')
        self.set_vel_button.clicked.connect(self.set_velocity)
        layout_5.addWidget(self.max_vel_x_label)
        layout_5.addWidget(self.max_vel_x_input)
        layout_5.addWidget(self.min_vel_x_label)
        layout_5.addWidget(self.min_vel_x_input)
        layout_5.addWidget(self.set_vel_button)
        group_5.setLayout(layout_5)
        
        layout.addWidget(group_1, 0, 0)
        layout.addWidget(group_2, 0, 1)
        layout.addWidget(group_3, 1, 0)
        layout.addWidget(group_4, 1, 1)
        layout.addWidget(group_5, 2, 0, 1, 2) # Thêm nhóm cài đặt vận tốc vào cột 0, dòng 2, rộng 1, cao 2
        
        self.setLayout(layout)
        
    def initROS(self):
        rospy.init_node('node_gui', anonymous=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    def send_fixed_point(self, idx):
        x, y, yaw = self.fixed_points[idx]
        self.send_pose(x, y, yaw)
        
    def send_goal(self):
        data_string = self.coord_input.text()
        try:
            x, y, yaw = map(float, data_string.split(','))
        except ValueError:
            rospy.logerr("Invalid input format. Please enter all values separated by commas.")
            return
        
        self.send_pose(x, y, yaw)
    
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
        rospy.loginfo("Sent goal: x={}, y={},yaw={}".format(x, y, yaw))
        
    def start_teleop(self):
        teleop_script = os.path.join(os.path.dirname(__file__), 'teleop_1.py')
        subprocess.Popen(['python3', teleop_script])
        
    def start_nav(self):
        nav_script = os.path.join(os.path.dirname(__file__), '..', 'launch', 'navigation.launch')
        subprocess.Popen(['roslaunch', nav_script])
        
    def nav_fit_toggle(self):
        self.counter += 1
        if self.counter % 2 == 1:
            self.start_nav()
            self.nav_fit_button.setText('Nav')
            self.nav_fit_button.setStyleSheet("QPushButton { background-color: yellow }")
        else:
            self.start_teleop()
            self.nav_fit_button.setText('Fit')
            self.nav_fit_button.setStyleSheet("QPushButton { background-color: lightgreen }")
            
    def update_yaml_file(self, max_vel_x, min_vel_x):
        # Đường dẫn tới file YAML
        yaml_file_path = os.path.join(os.path.dirname(__file__), '..', 'config','param', 'teb_local_planner_params.yaml')
	# Tạo một đối tượng YAML từ thư viện ruamel.yaml
        yaml = YAML()
        # Đọc dữ liệu từ file YAML
        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.load(file)

        # Cập nhật giá trị max_vel_x và min_vel_x
        yaml_data['TrajectoryPlannerROS']['max_vel_x'] = max_vel_x
        yaml_data['TrajectoryPlannerROS']['min_vel_x'] = min_vel_x

        # Ghi lại vào file YAML
        with open(yaml_file_path, 'w') as file:
            yaml.dump(yaml_data, file)

    def set_velocity(self):
        try:
            max_vel_x = float(self.max_vel_x_input.text())
            min_vel_x = float(self.min_vel_x_input.text())
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', 'Please enter valid velocity values.')
            return
        
        # Cập nhật giá trị trong file YAML
        self.update_yaml_file(max_vel_x, min_vel_x)
        
        # Sau đó khởi chạy lại file move_base_test.launch
        move_base_script = os.path.join(os.path.dirname(__file__), '..', 'launch', 'move_base_test.launch')
        subprocess.Popen(['roslaunch', move_base_script])
        
    def send_teleop_command(self, command):
        if command == 'w':
            linear_vel = self.publish_thread.linear_speed
            angular_vel = 0.0
            self.publish_thread.publish_velocity(linear_vel, angular_vel)
            print(f'FORWARD\r')
           
        elif command == 's':
            linear_vel = -self.publish_thread.linear_speed
            angular_vel = 0.0
            self.publish_thread.publish_velocity(linear_vel, angular_vel)
            print(f'BACKWARD\r')
            
        elif command == 'a':
            linear_vel = 0.0
            angular_vel = self.publish_thread.angular_speed
            self.publish_thread.publish_velocity(linear_vel, angular_vel)
            print(f'LEFT_forward\r')
             
        elif command == 'd':
            linear_vel = 0.0
            angular_vel = -self.publish_thread.angular_speed
            self.publish_thread.publish_velocity(linear_vel, angular_vel)
            print(f'RIGHT_forward\r')
             
        elif command == 'q':
            linear_vel = 0.0
            angular_vel = 0.0
            self.publish_thread.publish_velocity(linear_vel, angular_vel)
            print(f'STOP\r')

if __name__ == '__main__':
    app = QApplication([])
    gui = NavGUI()
    gui.show()
    app.exec_()

