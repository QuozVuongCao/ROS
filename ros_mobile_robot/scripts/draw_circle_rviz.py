#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import cos, sin, pi

def draw_circle():
    rospy.init_node('circle_marker_node', anonymous=True)
    marker_pub = rospy.Publisher('circle_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # Tốc độ gửi tin nhắn (10 Hz)

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"  # Chỉ định frame_id cho Marker
        marker.ns = "circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Độ dày của đường tròn
        marker.color.r = 1.0  # Màu đỏ
        marker.color.a = 1.0  # Độ trong suốt

        # Tạo các điểm trên đường tròn
        radius = 1.3  # Bán kính của đường tròn
        num_points = 200  # Số điểm trên đường tròn

        for i in range(num_points):
            angle = i * (2 * pi / num_points)
            point = Point()
            point.x = radius * cos(angle)
            point.y = radius * sin(angle)
            point.z = 0.0
            marker.points.append(point)

        marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass
