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
		self.grid1 = OccupancyGrid()
		self.sub_ping = rospy.Subscriber("/emulator/grid_test", OccupancyGrid, self.og_sub)

	def og_sub(self, msg):
		self.testgrid = msg.data
		self.resolution = msg.info.resolution
		self.width = msg.info.width
		self.height = msg.info.height

		#Grid Resolution, height and width
		rospy.loginfo(self.resolution)
		rospy.loginfo(self.width)
		rospy.loginfo(self.height)

		#new array using height and width
		self.newarray = np.reshape(self.testgrid, (-1, 50))
		self.plot = np.array(self.newarray)
		fig = plt.figure(figsize=(6, 6))

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
	rospy.init_node('navnode', anonymous=True)
	points = WaypontGen()

	try:
		rospy.spin()	
	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down...")
		points.shutdown()
		rospy.loginfo("Done!")