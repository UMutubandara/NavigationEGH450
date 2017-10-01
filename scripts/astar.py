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
	


	def __init__(self):
		#set up 
		
		
		self.real_points = [[-1.3, 1.5],
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

		self.sub_ping = rospy.Subscriber("/emulator/grid_test", OccupancyGrid, self.og_sub)

		

	def og_sub(self,msg):
		self.testgrid = msg.data
		self.resolution = msg.info.resolution
		self.width = msg.info.width
		self.height = msg.info.height

		for point in self.waypoints:
			point[0] = int(round((1/self.resolution)*(point[0]+(self.resolution*self.width*0.5))))
			point[1] = int(round((1/self.resolution)*(point[1]+(self.resolution*self.width*0.5))))
		

		#Grid Resolution, height and width
		rospy.loginfo(self.resolution)
		#rospy.loginfo(self.width)
		#rospy.loginfo(self.height)
		


		#new array using height and width
		self.newarray = np.reshape(self.testgrid, (-1, self.width))
		self.plot = np.array(self.newarray)
		rospy.loginfo(self.plot[12,40])
		fig = plt.figure(figsize=(10, 10))
		#rospy.loginfo(self.plot)


		for i in range( 1, len(self.waypoints)):
			found = False
			if self.waypoints[i][1] - self.waypoints[i-1][1] == 0:
				for x in range(self.waypoints[i-1][0] + 1, self.waypoints[i][0]):
					if self.plot[x, self.waypoints[i][1]] == 100:
						found = True
						break
			if self.waypoints[i][0] - self.waypoints[i-1][0] == 0:
				for y in range(self.waypoints[i-1][1] + 1, self.waypoints[i][1]):
					if self.plot[self.waypoints[i][0], y] == 100:
						found = True
						break
			
			if found
			
			#call sub waypoint gen function


		ax = fig.add_subplot(111)
		ax.set_title('Occupancy Grid')
		plt.imshow(self.plot)
		ax.set_aspect('equal')

		cax = fig.add_axes([0.12, 0.1, 0.78, 0.8])
		cax.get_xaxis().set_visible(False)
		cax.get_yaxis().set_visible(False)
		#cax.patch.set_alpha(0)
		cax.set_frame_on(False)
		plt.show()

		

if __name__ == '__main__':
	rospy.init_node('waypoints', anonymous=True)
	points = WaypontGen()

	try:
		rospy.spin()	
	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down...")
		points.shutdown()
		rospy.loginfo("Done!")