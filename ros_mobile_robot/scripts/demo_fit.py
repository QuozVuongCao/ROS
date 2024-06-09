#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import math
from tf.transformations import quaternion_from_euler

def calculate_yaw(x0,y0,x1,y1):
    global yaw1
    delta_x = x1 - x0
    delta_y = y1 - y0
    yaw1 = math.atan2(delta_y,delta_x)


def send_goal_1(x,y,yaw):
    global goal_publisher
    # Khởi tạo node ROS
    rospy.init_node('goal_sender_node', anonymous=True)
    # Tạo một publisher để gửi goal
    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    # Chờ một lát để đảm bảo publisher đã sẵn sàng
    rospy.sleep(1)

    # Tạo một message goal
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    # Gửi goal
    goal_publisher.publish(goal)

    rospy.loginfo("Goal sent!")

def goal_status_callback(msg):
    global current_status
    if len(msg.status_list) > 0:
        latest_status = msg.status_list[-1]
        current_status = latest_status.status
        if not current_status == 3: 
           rospy.loginfo("Robot is still navigating to the goal.")
        else:  
           rospy.loginfo("Robot has reached the goal!")
           send_goal_2(-3,1,0)
           rospy.signal_shutdown('Node is stopping')
def check_goal_status():
    
    # Tạo một subscriber để nhận thông điệp trạng thái từ topic "move_base/result"
    rospy.Subscriber('move_base/status', GoalStatusArray, goal_status_callback)
    rospy.spin()
    # Vòng lặp ROS để giữ chương trình chạy
def send_goal_2(x,y,yaw):
    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    # Chờ một lát để đảm bảo publisher đã sẵn sàng
    rospy.sleep(1)

    # Tạo một message goal
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    # Gửi goal
    goal_publisher.publish(goal)

    rospy.loginfo("Goal sent!")

def main():
    calculate_yaw(0,0,-3,1)
    send_goal_1(0,0,yaw1)
    check_goal_status()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
