#! /usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
print ("Yeet")
from sensor_msgs.msg import LaserScan
rospy.init_node('scan_values')
pub = rospy.Publisher ('cmd_vel', Twist, queue_size=16)
move = Twist()
def callback(msg):
    print (len(msg.ranges))
    hit = any([rang < 0.5 for rang in msg.ranges[:30] + msg.ranges[-30:]]) #60-degree cone, had an obstacle been detected within it, the robot will turn
    
    if  not hit:
        move.linear.x = 0.2
        move.angular.z = random.randrange(-10, 10)/20
    else:
        move.linear.x = 0.0
        move.angular.z = 0.5
    pub.publish(move)



sub = rospy.Subscriber('/scan', LaserScan, callback)
#! rospy.spin() 
def talker():
   
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass  

