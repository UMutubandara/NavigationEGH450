import rospy
import tf.transformations
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from nav_msgs.msg import *

import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Time

class Navigation():

	def __init__(self):
		#set up publisher 
		self.waypoints = list()
		self.have_waypoints = False	
		
		#self.sub_ping = rospy.Subscriber("/emulator/grid_test", OccupancyGrid, self.og_sub)
		self.sub_ping = rospy.Subscriber("/grid", OccupancyGrid, self.og_sub)

		self.msg_out= PoseStamped()
		self.msg_out.header.frame_id = "map"
		self.currentwp = Pose()
		self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size =10)
		
		self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_callback)

		#set up test waypoints and waypoint counter
		self.servopub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
		self.servo_msg_out = OverrideRCIn()
		self.servo_msg_out.channels[0] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[1] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[2] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[3] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[4] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[5] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[6] = self.servo_msg_out.CHAN_NOCHANGE
		self.servo_msg_out.channels[7] = self.servo_msg_out.CHAN_NOCHANGE

		self.servo_msg_out.channels[7] = 2000

		self.servopub.publish(self.servo_msg_out)

		self.waypoint_counter = -1
		self.currentwp.position.x = 0
		self.currentwp.position.y = 0
		self.currentwp.position.z = 1.5
		self.currentwp.orientation.x = 0
		self.currentwp.orientation.y = 0
		self.currentwp.orientation.z = 0
		self.currentwp.orientation.w = 1
		
		
		self.timewphit = rospy.Time(0)
		
		self.longsample= False
		self.land = 0
		
		self.blackfound = False
		
		self.redfound = False
		# Set up the subscriber
		self.yellowfound = False
		
		#self.sub_ping = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback) #jamsim testing
		self.sub_ping = rospy.Subscriber("/vicon/UAVTAQG2/UAVTAQG2", PoseStamped, self.callback) 	#demo testing
		self.uav_pose= Pose()
		



		self.sub_land = rospy.Subscriber("/land", Int8, self.ground)
		
		self.sub_black= rospy.Subscriber("/black_sign_location", Time, self.black_callback)
		self.sub_red= rospy.Subscriber("/red_sign_location", Time, self.red_callback)
		self.sub_yellow= rospy.Subscriber("/yellow_sign_location", Time, self.yellow_callback)
		
		
		
		self.tfbf = tf2_ros.Buffer()
		self.tfln = tf2_ros.TransformListener(self.tfbf)
		
		self.tfbr = tf2_ros.StaticTransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "vicon/UAVTAQG2/UAVTAQG2"
		t.child_frame_id = "camera" #0.05 0 -0.12 -1.57 0 3.14
		t.transform.translation.x = 0.05
		t.transform.translation.y = 0
		t.transform.translation.z = -0.12
		q = tf.transformations.quaternion_from_euler(3.14, 0, -1.57)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		self.tfbr.sendTransform(t)
		

	def pub_callback(self, event):
		#print 'Timer called at ' + str(event.current_real)
		self.msg_out.header.stamp = event.current_real

		#fill in goal msg
		self.msg_out.pose= self.currentwp
			
		self.pub.publish(self.msg_out)
		

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ping.unregister()

	def callback(self, msg):

		# Copying for simplicity
		if(self.have_waypoints):
			self.uav_pose = msg.pose

			#rospy.loginfo("NextPoint Position: [ %f, %f, %f ]"%(self.msg_out.pose.position.x, self.msg_out.pose.position.y, self.msg_out.pose.position.z))

			self.near_waypoint = 0.2

			self.distanceto = np.sqrt(((self.uav_pose.position.x - self.msg_out.pose.position.x)**2)+
								((self.uav_pose.position.y - self.msg_out.pose.position.y)**2)+
								((self.uav_pose.position.z - self.msg_out.pose.position.z)**2))

			

			"""self.distanceto = np.sqrt(((self.uav_pose.position.x - self.waypoints[self.waypoint_counter][0])**2)+((self.uav_pose.position.y - 
			self.waypoints[self.waypoint_counter][1])**2)+((self.uav_pose.position.z - self.waypoints[self.waypoint_counter][2])**2))"""

			

			if ((self.distanceto < self.near_waypoint) and (self.waypoint_counter < self.list )):
				if(self.timewphit == rospy.Time(0)):
					rospy.loginfo("Waypoint reached!")
					self.timewphit = rospy.get_rostime()
				self.hovertime = rospy.Duration(0)
				
				if self.longsample:
					self.hovertime = rospy.Duration(12)
				else:
					self.hovertime = rospy.Duration(1)
					
				if (rospy.get_rostime() - self.timewphit) > self.hovertime:
					rospy.loginfo("Waited long enough, moving to next!")
					self.longsample = False
					
					if self.waypoint_counter >= self.list-1:
						self.currentwp.position.z = 0
						self.currentwp.position.x = self.uav_pose.position.x
						self.currentwp.position.y = self.uav_pose.position.y
						rospy.loginfo("Finished, landing!")

					elif self.waypoint_counter == 11:
						self.servo_msg_out.channels[7] = 1000
						self.servopub.publish(self.servo_msg_out)
						self.waypoint_counter += 1					
						self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
						self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
						self.currentwp.position.z = 1.5
						self.currentwp.orientation.w = 0.707
						self.currentwp.orientation.x = 0
						self.currentwp.orientation.y = 0
						self.currentwp.orientation.z = 0.707
						self.timewphit = rospy.Time(0)
						rospy.loginfo("Wall 1 rotation!")
						
						t = geometry_msgs.msg.TransformStamped()
						t.header.stamp = rospy.Time.now()
						t.header.frame_id = "vicon/UAVTAQG2/UAVTAQG2"
						t.child_frame_id = "camera" #0.05 0 -0.12 -1.57 0 3.14
						t.transform.translation.x = 0.05
						t.transform.translation.y = 0
						t.transform.translation.z = -0.12
						q = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
						t.transform.rotation.x = q[0]
						t.transform.rotation.y = q[1]
						t.transform.rotation.z = q[2]
						t.transform.rotation.w = q[3]
						self.tfbr.sendTransform(t)
						

					elif self.waypoint_counter == 13:
						self.waypoint_counter += 1					
						self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
						self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
						self.currentwp.position.z = 1.5
						self.currentwp.orientation.w = 0
						self.currentwp.orientation.x = 0
						self.currentwp.orientation.y = 0
						self.currentwp.orientation.z = 1
						self.timewphit = rospy.Time(0)
						rospy.loginfo("Wall 2 rotation!")

					elif self.waypoint_counter == 14:
						self.waypoint_counter += 1					
						self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
						self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
						self.currentwp.position.z = 1.5
						self.currentwp.orientation.w = -0.707
						self.currentwp.orientation.x = 0
						self.currentwp.orientation.y = 0
						self.currentwp.orientation.z = 0.707
						self.timewphit = rospy.Time(0)
						rospy.loginfo("Wall 3 rotation!")

					elif self.waypoint_counter == 15:
						self.waypoint_counter += 1					
						self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
						self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
						self.currentwp.position.z = 1.5
						self.currentwp.orientation.w = -1
						self.currentwp.orientation.x = 0
						self.currentwp.orientation.y = 0
						self.currentwp.orientation.z = 0
						self.timewphit = rospy.Time(0)
						rospy.loginfo("Wall 1 rotation!")

					else:
						self.waypoint_counter += 1					
						self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
						self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
						self.currentwp.position.z = 1.5
						self.currentwp.orientation.w = 1
						self.currentwp.orientation.x = 0
						self.currentwp.orientation.y = 0
						self.currentwp.orientation.z = 0
						self.timewphit = rospy.Time(0)
			else:
				self.timewphit = rospy.Time(0)
	
	def black_callback(self,msg):
			
		if not self.blackfound :
			self.blackfound = True
			#self.blacksign = msg.pose
			timestamp = msg.data
			
			try:
				trans = self.tfbf.lookup_transform("map", "BlackSign", timestamp, rospy.Duration(0.1))

				self.currentwp.position.x = trans.transform.translation.x
				self.currentwp.position.y = trans.transform.translation.y
				self.currentwp.position.z = 0.7
				self.waypoint_counter -= 1

				rospy.loginfo("Black target at : [ %f, %f, %f ]"%(self.currentwp.position.x, self.currentwp.position.y, self.currentwp.position.z))
				self.longsample = True
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.loginfo("Error getting transform: %s" % e)
			
		

	def red_callback(self,msg):
		
		if not self.redfound :
			self.redfound = True
			#self.redsign = msg.pose
			timestamp = msg.data
			
			try:
				trans = self.tfbf.lookup_transform("map", "RedSign", timestamp, rospy.Duration(0.1))

				self.currentwp.position.x = trans.transform.translation.x
				self.currentwp.position.y = trans.transform.translation.y
				self.currentwp.position.z = 0.7
				self.waypoint_counter -= 1

				rospy.loginfo("red target at : [ %f, %f, %f ]"%(self.currentwp.position.x, self.currentwp.position.y, self.currentwp.position.z))
				self.longsample = True
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.loginfo("Error getting transform: %s" % e)
			
	def yellow_callback(self,msg):
		
		if not self.yellowfound :
			self.yellowfound = True
			#self.yellowsign = msg.pose
			timestamp = msg.data
			
			try:
				trans = self.tfbf.lookup_transform("map", "YellowSign", timestamp, rospy.Duration(0.1))
				x = trans.transform.translation.x
				y = trans.transform.translation.y
				z = 1
				
				if(x > 1.5):
					x =1.5
				elif(x< -1.5):
					x = -1.5
					
				if(y > 1.5):
					y =1.5
				elif(y < -1.5):
					y = -1.5
					
					
				self.currentwp.position.x = x
				self.currentwp.position.y = y
				self.currentwp.position.z = z
				
				
				self.waypoint_counter -= 1

				rospy.loginfo("yellow target at : [ %f, %f, %f ]"%(self.currentwp.position.x, self.currentwp.position.y, self.currentwp.position.z))
				self.longsample = True
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.loginfo("Error getting transform: %s" % e)
		


	
	def ground(self, msg):
		self.black = msg.data
		#rospy.loginfo(msg.data)
		

		if (self.black > 2):
			self.currentwp.position.x = self.uav_pose.position.x #x + uav position
			self.currentwp.position.y = self.uav_pose.position.y #y+ uav position
			self.currentwp.position.z = 0
			self.waypoint_counter -= 1
			self.longsample = True

			rospy.loginfo("Land Command recieved, moving to land!")

			if(self.timewphit == rospy.Time(0)):
				self.timewphit = rospy.get_rostime()

			rospy.loginfo("UAV Landing!")

			if (rospy.get_rostime() - self.timewphit) > rospy.Duration(12):
				
				self.currentwp.position.x = self.waypoints[0][(self.waypoint_counter-1)][0]
				self.currentwp.position.y = self.waypoints[0][(self.waypoint_counter-1)][1]
				self.currentwp.position.z = 0
				self.currentwp.orientation.x = 0
				self.currentwp.orientation.y = 0
				self.currentwp.orientation.z = 0
				self.currentwp.orientation.w = 1
				self.timewphit = rospy.Time(0)
		else:
			self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
			self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
			self.currentwp.position.z = 1.5
			self.currentwp.orientation.x = 0
			self.currentwp.orientation.y = 0
			self.currentwp.orientation.z = 0
			self.currentwp.orientation.w = 1
			self.timewphit = rospy.Time(0)

			
	def og_sub(self,msg):
		self.testgrid = msg.data
		self.resolution = msg.info.resolution
		self.width = msg.info.width
		self.height = msg.info.height

		#changes from the real points to occupancy grid points

		waypoint_list = [		[-1.5, 1.5],
						 [-1.7, -1.5],
						 [-0.3, -1.5],
						 [-0.3, 1.5],
						 [1.0,  1.5],
						 [1.0,  -1.5],
						 [1.7,  -1.5],
						 [1.7,  1.5],
						 [1.7, 1.5],
						 [-1.7, 1.9],
						 [-1.7,-1.7],
						 [1.7 , -1.7],
						 [1.7, 1.7],
						 [1.7, 1.7]]

		for point in waypoint_list:
			point[0] = int(round((1/self.resolution)*(point[0]+(2.5))))
			point[1] = int(round((1/self.resolution)*(point[1]+(2.5))))
		
		#Grid Resolution, height and width
		
		
		#rospy.loginfo(self.height)
		
		self.threshold = 3

		#new array using height and width
		self.newarray = np.reshape(self.testgrid, (self.width, -1))
		self.plot = np.array(self.newarray)
		
		fig = plt.figure(figsize=(10, 10))
		#rospy.loginfo(self.plot)
		self.subpoints = list()
		for i in range( 1, len(waypoint_list)):
			
			"""if waypoint_list[i][1] - waypoint_list[i-1][1] == 0:
				for x in range(waypoint_list[i-1][0] + 1, waypoint_list[i][0]):
					if self.plot[x, waypoint_list[i][1]] == 100:
						found = True
						break"""




			if waypoint_list[i][0] - waypoint_list[i-1][0] == 0:
				#checks the direction of the vertical travel
				m = 1 if waypoint_list[i][1] > waypoint_list[i-1][1] else -1

				for y in range(waypoint_list[i-1][1] + 1, waypoint_list[i][1]):
					found = False
					for x in range(waypoint_list[i][0] - self.threshold, waypoint_list[i][0] + self.threshold + 1):
						if self.plot[x, y] == 100:
							found = True
							break

					#check which side of the room the ostacle is and goes plots around the longer side
					obs_width = 5
					if found:
						self.subpoints.append([waypoint_list[i][0], y-m*self.threshold])
						if waypoint_list[i][0] < (self.width/2):
							self.subpoints.append([waypoint_list[i][0]+m*(obs_width+2*self.threshold), y-m*(self.threshold)])
							self.subpoints.append([waypoint_list[i][0]+m*(obs_width+2*self.threshold), y + m*(obs_width+ 2*self.threshold)])
						else:
							self.subpoints.append([waypoint_list[i][0]-m*(obs_width-2*self.threshold), y-m*(self.threshold)])
							self.subpoints.append([waypoint_list[i][0]-m*(obs_width-2*self.threshold), y + m*(obs_width + 2*self.threshold)])
						
						self.subpoints.append([waypoint_list[i][0], y+m*(obs_width+2*self.threshold)])
						waypoint_list[i:i-1] = self.subpoints
						break

			

		for point in waypoint_list:
		 	self.plot[point[0], point[1]] = 200

			
		#plots the waypoints


			#call sub waypoint gen function

		
		for element in waypoint_list:
		 	element[0] = (element[0]*self.resolution) -(self.resolution*self.width*0.5)
		 	element[1] = (element[1]*self.resolution) -(self.resolution*self.width*0.5)



		self.waypoints.append(waypoint_list)
		self.list = len(self.waypoints[0])

		self.have_waypoints = True
		#rospy.loginfo(self.waypoints)
		rospy.loginfo("Grid analysed and Waypoints set!")
		#rospy.loginfo(self.list)


# 		ax = fig.add_subplot(111)
# 		ax.set_title('Occupancy Grid')
# 		plt.imshow(self.plot)
# 	
# 		ax.set_aspect('equal')
#
# 		cax = fig.add_axes([0.12, 0.1, 0.1, 0.8])
# 		ax.invert_yaxis()
# 		cax.get_xaxis().set_visible(False)
# 		cax.get_yaxis().set_visible(False)
# 		#cax.patch.set_alpha(0)
# 		cax.set_frame_on(False)
		#plt.show()
		

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