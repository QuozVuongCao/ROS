#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import quaternion_from_euler

def send_goal(x_center, y_center, radius, angular_velocity):
    rospy.init_node('circle_movement', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(3) #2: odom 

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Tính góc theta dựa trên thời gian hiện tại và tốc độ góc
        theta = angular_velocity * current_time.to_sec()
        yaw = math.radians(theta)

        # Tạo một PoseStamped message và đặt giá trị goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = current_time
        goal_msg.header.frame_id = "map"  # Frame ID cho hệ thống toạ độ

        # Tính toạ độ x và y của mục tiêu dựa trên góc theta và bán kính
        goal_msg.pose.position.x = x_center + radius * math.cos(yaw)
        goal_msg.pose.position.y = y_center + radius * math.sin(yaw)

        # Chuyển đổi góc theta thành quaternion
        quaternion = quaternion_from_euler(0, 0, yaw)

        # Đặt giá trị quaternion trong tin nhắn goal
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]

        # Gửi thông điệp goal tới topic
        goal_pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_goal(0.0, 0.0, 1.4, 5)  #0 0 2 8  Gửi mục tiêu di chuyển theo quỹ đạo hình tròn với tâm (0, 0), bán kính 2 và vận tốc góc 8 độ/giây #odom: 1.6 -> w=7, rate=2

    except rospy.ROSInterruptException:
        pass
