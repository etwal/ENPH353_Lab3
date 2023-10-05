#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, 
  queue_size=1)
rate = rospy.Rate(10)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.2

while not rospy.is_shutdown():
   pub.publish(move)
   rate.sleep()