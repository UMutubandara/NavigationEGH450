import rospy
import tf.transformations
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import matplotlib.pyplot as plt

from nav_msgs.msg import *
from std_msgs.msg import *
import std_msgs.msg




class WaypontGen(object):
	
	def og_sub(self,msg):
		self.testgrid = msg.data
		self.resolution = msg.info.resolution
		self.width = msg.info.width
		self.height = msg.info.height

		#changes from the real points to occupancy grid points

		self.waypoints = [	[-1.3, 1.5],
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
						 [1.7, 1.7]]

		for point in self.waypoints:
			point[0] = int(round((1/self.resolution)*(point[0]+(self.resolution*self.width*0.5))))
			point[1] = int(round((1/self.resolution)*(point[1]+(self.resolution*self.width*0.5))))
		

		#Grid Resolution, height and width
		
		
		#rospy.loginfo(self.height)
		
		self.threshold = 3

		#new array using height and width
		self.newarray = np.reshape(self.testgrid, (-1, self.width))
		self.plot = np.array(self.newarray)
		
		fig = plt.figure(figsize=(10, 10))
		#rospy.loginfo(self.plot)
		self.subpoints = list()
		for i in range( 1, len(self.waypoints)):
			
			"""if self.waypoints[i][1] - self.waypoints[i-1][1] == 0:
				for x in range(self.waypoints[i-1][0] + 1, self.waypoints[i][0]):
					if self.plot[x, self.waypoints[i][1]] == 100:
						found = True
						break"""

			if self.waypoints[i][0] - self.waypoints[i-1][0] == 0:
				#checks the direction of the vertical travel
				m = 1 if self.waypoints[i][1] > self.waypoints[i-1][1] else -1

				for y in range(self.waypoints[i-1][1] + 1, self.waypoints[i][1]):
					found = False
					for x in range(self.waypoints[i][0] - self.threshold, self.waypoints[i][0] + self.threshold + 1):
						if self.plot[x, y] == 100:
							found = True
							break

					#check which side of the room the ostacle is and goes plots around the longer side
					obs_width = 5
					if found:
						self.subpoints.append([self.waypoints[i][0], y-m*self.threshold])
						if self.waypoints[i][0] < (self.width/2):
							self.subpoints.append([self.waypoints[i][0]+m*(obs_width+2*self.threshold), y-m*(self.threshold)])
							self.subpoints.append([self.waypoints[i][0]+m*(obs_width+2*self.threshold), y + m*(obs_width+ 2*self.threshold)])
						else:
							self.subpoints.append([self.waypoints[i][0]-m*(obs_width-2*self.threshold), y-m*(self.threshold)])
							self.subpoints.append([self.waypoints[i][0]-m*(obs_width-2*self.threshold), y + m*(obs_width + 2*self.threshold)])
						
						self.subpoints.append([self.waypoints[i][0], y+m*(obs_width+2*self.threshold)])
						self.waypoints[i-1:i-1] = self.subpoints
						break

		#plots the waypoints
		for point in self.waypoints:
		 	self.plot[point[0], point[1]] = 150 
		
		

			#call sub waypoint gen function
		self.realpoints = self.waypoints	
		
		for element in self.realpoints:
		 	element[0] = (point[0]*self.resolution)-(self.resolution*self.width*0.5)
		 	element[1] = (point[1]*self.resolution)-(self.resolution*self.width*0.5)


		ax = fig.add_subplot(111)
		ax.set_title('Occupancy Grid')
		plt.imshow(self.plot)
		
		ax.set_aspect('equal')

		cax = fig.add_axes([0.12, 0.1, 0.1, 0.8])
		cax.get_xaxis().set_visible(False)
		cax.get_yaxis().set_visible(False)
		#cax.patch.set_alpha(0)
		cax.set_frame_on(False)
		plt.show()

	def __init__(self):
		#set up
		self.realpoints = list()
		

		self.sub_ping = rospy.Subscriber("/emulator/grid_test", OccupancyGrid, self.og_sub)
		#rospy.loginfo(self.waypoints)

	

		

if __name__ == '__main__':
	rospy.init_node('waypoints', anonymous=True)
	p = WaypontGen()

	try:
		rospy.spin()	
	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down...")
		points.shutdown()
		rospy.loginfo("Done!")