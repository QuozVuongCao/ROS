#! /usr/bin/env python3

from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist
import threading
import rospy
import sys, select, termios, tty
import math

LINEAR_STEP =0.1
ANGULAR_STEP =0.1
current_time=0
pre_time=0

class PublishThread(threading.Thread):
    def __init__(self):
        super().__init__()
        
        self.rate=50 #Hz
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time_start=rospy.Time.now()
        #self.mode_run=rospy.get_param('~mode_run','circle') #circle|square|num8

        self.x=0.0
        self.th=0.0
        self.condition = threading.Condition()
        self.done = False
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.velocity_publisher.get_num_connections() == 0:
            if i == 0 or i == 20:  # Fixed the condition here
                print("Waiting for subscriber to connect to {}".format(self.velocity_publisher.name))
                rospy.sleep(0.5)
                i += 1
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

   # def spin(self):#ok
   #     rospy.loginfo("[ROS][mtq]Start Navigation Test")
   #     self.update()
    
    def shutdown(self):
        rospy.loginfo("Stop navigation sim")
        #Stop message
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

   
    #def update(self, linear_vel,angular_vel):
     #   pass
        
      #  self.condition.acquire()
       # self.x=linear_vel
        #self.th=angular_vel

        #self.condition.release()

    #def run(self):
     #   pass
        #twist=Twist()
      #  while not self.done:
       #     self.condition.acquire()
        #    velocity_msg.linear.x=self.x
         #   velocity_msg.linear.y=0.0
          #  velocity_msg.linear.z=0.0
           # velocity_msg.angular.x=0.0
            #velocity_msg.angular.y=0.0
            #velocity_msg.anfular.z=self.th
            #self.condition.release()
            #self.velocity_publisher.publish(twist)

    def run_circle(self):
        pass
        rospy.loginfo("[ROS][mtq] Start Navigation Test:One circle")
        rate=rospy.Rate(200)   #200Hz=5ms
        rospy.on_shutdown(self.shutdown)

        v = 0.24 #0.5
        w = 0.24 #0.5
        time_start=rospy.Time.now()
        timeOneCircle=(2*math.pi/w)
        rospy.loginfo('[ROS][mtq] timeOneCircle='+str(timeOneCircle))

        velocity_msg=Twist()
        velocity_msg.linear.x=v 
        velocity_msg.angular.z=w
        i=0#20*5ms=100ms

        while not rospy.is_shutdown():
            time_duration=(rospy.Time.now()-time_start).to_sec()

            #code here
            i+=1
            if i>=20: #~100ms=10Hz
               i=0
               self.velocity_publisher.publish(velocity_msg)

            if time_duration>timeOneCircle:
               rospy.signal_shutdown('[ROS][mtq] Done node_nav_Circle')
            
            rospy.loginfo('[ROS][mtq] time duration = ' + str(time_duration))
            #rate.sleep() 
    
    def run_square(self):
        pass
        rospy.loginfo("[ROS][mtq] Start Navigation Test: One Circle")
        rate = rospy.Rate(200)     #200Hz = 5ms
        rospy.on_shutdown(self.shutdown)
        #
        v = 0.0
        w = 0.0
        timeOneSquare = 31  #s

        time_start = rospy.Time.now()
        rospy.loginfo('[ROS][mtq] timeOneSquare = ' + str(timeOneSquare))
        #
        velocity_msg= Twist()
        velocity_msg.linear.x   = v     # v = 0.5 m/s
        velocity_msg.angular.z  = w    # w = 0.785398163397448 rad/s
 
        # cho truyen toc do cham thoi .... ~ 100ms 
        i  = 0 # 20 * 5ms = 100ms
        #
        while not rospy.is_shutdown():
            time_duration = (rospy.Time.now() - time_start).to_sec()
            # code here
            i += 1
            if i >= 20: #~ 100ms = 10Hz
                i = 0
                self.velocity_publisher.publish(velocity_msg)
            # calc
            if time_duration < 6:
                pass
                v = 0.2 #0.5
                w = 0.0
            if time_duration >= 6 and time_duration < 7.88:
                pass
                v = 0.0
                w = -0.88
            if time_duration >= 7.89 and time_duration < 13.89:
                pass
                v = 0.2
                w = 0.0
            if time_duration >= 13.875 and time_duration < 15.75:
                pass
                v = 0.0
                w = -0.88
            if time_duration >= 15.75 and time_duration < 21.75:
                pass
                v = 0.2
                w = 0.0
            if time_duration >= 21.75 and time_duration < 23.625:
                pass
                v = 0.0
                w = -0.88
            if time_duration >= 23.625 and time_duration < 29.625:
                pass
                v = 0.2
                w = 0.0
           
            if time_duration >=29.25:
                v = w = 0
            #set v, w
            velocity_msg.linear.x   = v     # v = m/s
            velocity_msg.angular.z  = w     # w = rad/s
            #
            if time_duration > timeOneSquare:
                rospy.signal_shutdown('[ROS][mtq] Done node_nav_Circle') #PASSED
            #
            rospy.loginfo('[ROS][mtq] time duration = ' + str(time_duration))
            rate.sleep()   

    def run_num8(self):
        pass
        rospy.loginfo("[ROS][mtq] Start Navigation Test: One Circle")
        rate = rospy.Rate(200)     #200Hz = 5ms
        rospy.on_shutdown(self.shutdown)
        #
        v = v_ = 0.24  #0.5
        w = w_ = 0.24 #0.5
        timeOneNum8 = 53.5 #28  #s
        time_start = rospy.Time.now()
        rospy.loginfo('[ROS][mtq] timeOneNum8 = ' + str(timeOneNum8))
        #
        velocity_msg = Twist()
        velocity_msg.linear.x   = v     # v = 0.5 m/s
        velocity_msg.angular.z  = w    # w = 0.5 rad/s
 
        # cho truyen toc do cham thoi .... ~ 100ms 
        i  = 0 # 20 * 5ms = 100ms
        #
        while not rospy.is_shutdown():
            time_duration = (rospy.Time.now() - time_start).to_sec()
            # code here
            i += 20
            if i >= 1: #~ 100ms = 10Hz
                i = 0
                self.velocity_publisher.publish(velocity_msg)
            # calc
            # calc
            if time_duration < 10 :#6.283:
                pass
                v = v_
                w = w_
            if time_duration >= 10 and time_duration < 36.2: #18.85
                pass
                v = v_
                w = -w_
            if time_duration >= 36.2 and time_duration < 53.4: #25
                pass
                v = v_
                w = w_
            if time_duration >= 53.4: #25
                v = w = 0.0
            #
            #set v, w
            velocity_msg.linear.x   = v     # v = m/s
            velocity_msg.angular.z  = w     # w = rad/s
            #
            if time_duration > timeOneNum8:
                rospy.signal_shutdown('[ROS][mtq] Done node_nav_Circle') #PASSED
            #
            rospy.loginfo('[ROS][mtq] time duration = ' + str(time_duration))
            rate.sleep()

    def publish_velocity(self, linear,angular):
        #velocity_msg = Float32(value)
        #self.velocity_publisher.publish(velocity_msg)
        velocity_msg=Twist()
        velocity_msg.linear.x=linear
        #velocity_msg.angular.z=angular
        #velocity_msg.linear.x=self.x
        velocity_msg.linear.y=0.0
        velocity_msg.linear.z=0.0
        velocity_msg.angular.x=0.0
        velocity_msg.angular.y=0.0
        #velocity_msg.angular.z=self.th
        velocity_msg.angular.z=angular
        self.velocity_publisher.publish(velocity_msg)
    
        
    def stop(self):
        self.done = True
        self.update(0.0,0.0)
        self.join()


def getKey():
    try:
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    linear_vel=0.0
    angular_vel=0.0
    linear_speed=0.3
    angular_speed=1
    rospy.init_node('node_teleop')
    publish_thread = PublishThread()

    try:
        publish_thread.wait_for_subscribers()
        #publish_thread.update(linear_vel,angular_vel)

        while True:
            #current_time=rospy.Time.now()
            key = getKey()
            if key == 'w': #forward
                linear_vel=linear_speed
                angular_vel=0.0
                publish_thread.publish_velocity(linear_vel,angular_vel)
                print(f'FORWARD\r')
                
            elif key=='s': #backward
                linear_vel=-linear_speed
                angular_vel=0.0
                publish_thread.publish_velocity(linear_vel,angular_vel)
                print(f'BACKWARD\r')
            
            elif key=='d': #turn right
                linear_vel=0.0
                angular_vel=-angular_speed
                publish_thread.publish_velocity(linear_vel,angular_vel)
                print(f'RIGHT_forward\r')
            
            elif key=='a': #turn left
                linear_vel=0.0
                angular_vel=angular_speed
                publish_thread.publish_velocity(linear_vel,angular_vel)
                print(f'LEFT_forward\r')
            
            elif key=='x': #decrease speed
                linear_speed -= LINEAR_STEP
                publish_thread.publish_velocity(linear_vel,linear_vel)
                print(f'DECREASE SPEED\r')

            elif key=='z': #increase speed
                linear_speed += LINEAR_STEP
                publish_thread.publish_velocity(linear_vel,linear_vel)
                print(f'INSCREASE SPEED\r')
            
            
            elif key=='o':  #circle
                publish_thread.run_circle() 
                print(f'CIRCLE\r')

            elif key=='v':  #vuong
                publish_thread.run_square()
                print(f'SQUARE\r')

            elif key=='n': #num8
                publish_thread.run_num8()   
                print(f'NUM8\r')

            elif key=='q': #STOP
                linear_vel=0.0
                angular_vel=0.0
                publish_thread.publish_velocity(linear_vel,angular_vel)
                print(f'STOP\r')

            else:
                if key == '\x03':
                    break
            #publish_thread.update(linear_vel,angular_vel)
            #if ((current_time-pre_time).to_sec()>=0.1):
             #   publish_thread.publish()
              #  pre_time=current_time
        
            
            
    finally:
        publish_thread.stop()
      

if __name__ == '__main__':
    main()

