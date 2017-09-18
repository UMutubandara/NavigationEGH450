import rospy
import tf.transformations
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import std_msgs.msg

class Navigation():

	def __init__(self):
		#set up publisher 
		self.msg_out= PoseStamped()
		self.msg_out.header.frame_id = "world"
		self.currentwp = Pose()
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
		self.land = 0

		# Set up the subscriber
		self.sub_ping = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback)
		self.sub_land = rospy.Subscriber("/land", Int8, self.ground)

	def pub_callback(self, event):
		#print 'Timer called at ' + str(event.current_real)
		self.msg_out.header.stamp = event.current_real

		#fill in goal msg
		self.msg_out.pose= self.currentwp
			
		self.pub.publish(self.msg_out)
		rospy.loginfo("NextPoint Position: [ %f, %f, %f ]"%(self.msg_out.pose.position.x, self.msg_out.pose.position.y, self.msg_out.pose.position.z))

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ping.unregister()

	def callback(self, msg):

		# Copying for simplicity
		self.uav_pose = msg.pose

		
		self.near_waypoint = 0.10

		self.distanceto = np.sqrt(((self.uav_pose.position.x - self.waypoint[self.waypoint_counter][0])**2)+((self.uav_pose.position.y - 
		self.waypoint[self.waypoint_counter][1])**2)+((self.uav_pose.position.z - self.waypoint[self.waypoint_counter][2])**2))

		if ((self.distanceto < self.near_waypoint) and (self.waypoint_counter < 6)):
		 	self.waypoint_counter += 1
		 	self.currentwp.position.x = self.waypoint[self.waypoint_counter][0]
			self.currentwp.position.y = self.waypoint[self.waypoint_counter][1]
			self.currentwp.position.z = self.waypoint[self.waypoint_counter][2]
			self.currentwp.orientation.x = 0
			self.currentwp.orientation.y = 0
			self.currentwp.orientation.z = 1
			self.currentwp.orientation.w = 1


	
	def ground(self, msg):
		self.eground = msg.data
		#rospy.loginfo(msg.data)
		if (self.eground > 2):
			self.currentwp.position.x = 0#self.uav_pose.position.x
			self.currentwp.position.y = 0#self.uav_pose.position.y
			self.currentwp.position.z = 3

		else:
			self.currentwp.position.x = self.waypoint[self.waypoint_counter][0]
			self.currentwp.position.y = self.waypoint[self.waypoint_counter][1]
			self.currentwp.position.z = self.waypoint[self.waypoint_counter][2]
			self.currentwp.orientation.x = 0
			self.currentwp.orientation.y = 0
			self.currentwp.orientation.z = 1
			self.currentwp.orientation.w = 1


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