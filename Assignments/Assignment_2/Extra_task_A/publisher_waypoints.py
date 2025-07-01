#!/usr/bin/env python

import rospy
import math
import numpy
from math import pi
from geometry_msgs.msg import PoseWithCovariance

# define waypoints

waypoints = [ 
	[0, 0, 0],
	[5, 0, pi/2],
	[5, 5, 5/4*pi],
	[-5, -5, pi/2],
	[-5, 5, 0],
	[0, 0, 0],
	[3, 3, 3/4*pi],
	[-3, 0, 3/2*pi],
	[0, -3, pi/4],
	[3, 0, pi/2],
	[0, 0, 3/2*pi]
]

def pub_waypoints():
	pub = rospy.Publisher('waypoints', PoseWithCovariance, queue_size=100)
	rospy.init_node('publisher_waypoints', anonymous=False)
	rate = rospy.Rate(100) # 100 Hz

	while not rospy.is_shutdown():
		msg = PoseWithCovariance()

		for i,row in enumerate(waypoints):
			msg.covariance[3*i:3*(i+1)] = row
			
			if i == len(waypoints)-1 and 3*len(waypoints)<36:
	    			msg.covariance[34]=1001
	    			msg.covariance[35]=len(waypoints)
		
		#rospy.loginfo(msg) # this allows for displayng the message in the terminal 
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		pub_waypoints()
	except rospy.ROSInterruptException:
		pass
