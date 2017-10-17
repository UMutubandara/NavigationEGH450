import rospy
import tf.transformations
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from nav_msgs.msg import *
from std_msgs.msg import *
import std_msgs.msg
import std_msgs.msg

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
		
						
		self.waypoint_counter = -1
		self.currentwp.position.x = -1.3
		self.currentwp.position.y = 1.5
		self.currentwp.position.z = 1.5
		self.currentwp.orientation.x = 0
		self.currentwp.orientation.y = 0
		self.currentwp.orientation.z = 0
		self.currentwp.orientation.w = 1
		
		self.timewphit = rospy.Time(0)
		
		self.land = 0

		# Set up the subscriber

		#self.sub_ping = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback) #jamsim testing
		self.sub_ping = rospy.Subscriber("/vicon/UAVTAQG2/UAVTAQG2", PoseStamped, self.callback) 	#demo testing


		self.sub_land = rospy.Subscriber("/land", Int8, self.ground)
		#self.sub_land = rospy.Subscriber("/black_sign_location", Int8, self.ground)

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

			self.near_waypoint = 0.1

			self.distanceto = np.sqrt(((self.uav_pose.position.x - self.msg_out.pose.position.x)**2)+
								((self.uav_pose.position.y - self.msg_out.pose.position.y)**2)+
								((self.uav_pose.position.z - self.msg_out.pose.position.z)**2))

			

			"""self.distanceto = np.sqrt(((self.uav_pose.position.x - self.waypoints[self.waypoint_counter][0])**2)+((self.uav_pose.position.y - 
			self.waypoints[self.waypoint_counter][1])**2)+((self.uav_pose.position.z - self.waypoints[self.waypoint_counter][2])**2))"""

			

			if ((self.distanceto < self.near_waypoint) and (self.waypoint_counter < self.list )):
				if(self.timewphit == rospy.Time(0)):
					rospy.loginfo("Waypoint reached!")
					self.timewphit = rospy.get_rostime()
					
				if (rospy.get_rostime() - self.timewphit) > rospy.Duration(2):
					rospy.loginfo("Waited long enough, moving to next!")
					
					if self.waypoint_counter >= self.list-1:
						self.currentwp.position.z = 0
						self.currentwp.position.x = self.uav_pose.position.x
						self.currentwp.position.y = self.uav_pose.position.y
						rospy.loginfo("Finished, landing!")

					elif self.waypoint_counter == 11:
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

					elif self.waypoint_counter == 12:
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

					elif self.waypoint_counter == 13:
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

					elif self.waypoint_counter == 14:
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
				


	
	def ground(self, msg):
		self.black = msg.data
		#rospy.loginfo(msg.data)
		

		if (self.black > 2):
			self.currentwp.position.x = 0 #x + uav position
			self.currentwp.position.y = 0 #y+ uav position
			self.currentwp.position.z = 0.5

			rospy.loginfo("Black Target Found, moving to target!")

			if(self.timewphit == rospy.Time(0)):
				self.timewphit = rospy.get_rostime()

			rospy.loginfo("Target Found, commence sampling!")

			if (rospy.get_rostime() - self.timewphit) > rospy.Duration(12):


				
				self.currentwp.position.x = self.waypoints[0][self.waypoint_counter][0]
				self.currentwp.position.y = self.waypoints[0][self.waypoint_counter][1]
				self.currentwp.position.z = 1.5
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

		waypoint_list = [[-1.3, 1.5],
						 [-1.3, -1.5],
						 [-0.3, -1.5],
						 [-0.3, 1.5],
						 [0.7,  1.5],
						 [0.7,  -1.5],
						 [1.7,  -1.5],
						 [1.7,  1.5],
						 [1.7, 1.7],
						 [-1.7, 1.7],
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
		self.newarray = np.reshape(self.testgrid, (-1, self.width))
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
		rospy.loginfo(self.waypoints)
		#rospy.loginfo(self.list)


		# ax = fig.add_subplot(111)
		# ax.set_title('Occupancy Grid')
		# plt.imshow(self.plot)
		
		# ax.set_aspect('equal')

		# cax = fig.add_axes([0.12, 0.1, 0.1, 0.8])
		# cax.get_xaxis().set_visible(False)
		# cax.get_yaxis().set_visible(False)
		# #cax.patch.set_alpha(0)
		# cax.set_frame_on(False)
		# plt.show()
		

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