#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


x1 = []
x2 = []
y1 = []
y2 = []

# Hàm callback cho topic 1
def callback_gazebo(data):
    global x1,y1
    x1.append(data.pose.pose.position.x)
    y1.append(data.pose.pose.position.y)

def callback_goal(data):
    global x2,y2
    x2.append(data.pose.position.x)
    y2.append(data.pose.position.y)


# Thiết lập nút ROS và đăng ký các subscriber
rospy.init_node('plotter_node')
rospy.Subscriber('/odom_ekf', Odometry, callback_gazebo)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
plt.figure()

# Hàm cập nhật đồ thị
def update_plot():
    global x1,y1,x2,y2
    # Xóa đồ thị cũ

    # Vẽ dữ liệu từ topic 1
    plt.plot(x1, y1, color='red')
    
    plt.plot(x2, y2, color='blue')

    # Đặt tiêu đề và nhãn trục
    plt.title('Plotting Data from Multiple Topics')
    plt.xlabel('X')
    plt.ylabel('Y')

    # Hiển thị chú thích
    plt.legend()

    # Cập nhật đồ thị
    plt.pause(1)

# Vòng lặp chính
while not rospy.is_shutdown():
    update_plot()
    rospy.sleep(1)
