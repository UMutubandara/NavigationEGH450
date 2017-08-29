import rospy
import tf.transformations
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped


class Navigation():

	def __init__(self):
		#set up publisher 
		self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size =10)
		self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_callback)

		#set up test waypoints and waypoint counter
		self.waypoint = [[0 , 0 , 0], 
						[0 , 0 , 2], 
						[2, 2, 2], 
						[2, -2, 2], 
						[-2, 2, 2], 
						[-2 ,-2 ,2 ],
						[0 ,0 ,0 ]]
		self.waypoint_counter = 0

		# Set up the subscriber
		self.sub_ping = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback)

	def pub_callback(self, event):
		#print 'Timer called at ' + str(event.current_real)
		msg_out= PoseStamped()
		#fill in goal msg
		msg_out.pose.position.x = self.waypoint[self.waypoint_counter][0]
		msg_out.pose.position.y = self.waypoint[self.waypoint_counter][1]
		msg_out.pose.position.z = self.waypoint[self.waypoint_counter][2]
		msg_out.pose.orientation.x = 0
		msg_out.pose.orientation.y = 0
		msg_out.pose.orientation.z = 1
		msg_out.pose.orientation.w = 1


		self.pub.publish(msg_out)


	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ping.unregister()

	def callback(self, msg):

		# Copying for simplicity
		self.uav_pose = msg.pose

		
		self.distanceto = np.sqrt(((self.uav_pose.position.x - self.waypoint[self.waypoint_counter][0])**2)+((self.uav_pose.position.y - 
		self.waypoint[self.waypoint_counter][1])**2)+((self.uav_pose.position.z - self.waypoint[self.waypoint_counter][2])**2))

		rospy.loginfo(self.distanceto)

		self.near_waypoint = 0.2


		if (self.distanceto < self.near_waypoint) and (self.waypoint_counter < 6):
		 	self.waypoint_counter += 1
		 	time.sleep(5)



if __name__ == '__main__':
	rospy.init_node('navnode', anonymous=True)
	nav = Navigation()

	# Loop here until quit
	try:
		rospy.spin()

	except rospy.ROSInterruptException:
		# Shutdown
		rospy.loginfo("Shutting down...")
		nav.shutdown()
		rospy.loginfo("Done!")