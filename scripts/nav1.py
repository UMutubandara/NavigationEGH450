import roslib; roslib.load_manifest('tut')
import rospy
import tf.transformations
import numpy as np
from geometry_msgs.msg import PoseStamped


  	# while not rospy.is_shutdown():
  	# 	rospy.loginfo("Point Position: [ %f, %f, %f ]"%(new_x, new_y, new_z))
   #  	rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
   #  	pub.Publish("Point Position: [ %f, %f, %f ]"%(new_x, new_y, new_z))
   #  	pub.Publish("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, guat.w))
   #  	rate.sleep()

class Navigation():

	def __init__(self):
		#set up publisher 
		self.pub = rospy.Publisher('/newgoal', PoseStamped, queue_size =10)
  		self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_callback)
  		#set up test waypoints and waypoint counter


  			c_wp = Pose();

  			self.waypoint = [c_wp,[0 , 1 , 1.5],[0, 1, 0]]
		self.waypoint_counter = 0

		# Set up the subscriber
		self.sub_ping = rospy.Subscriber("/emulator/pose", PoseStamped, self.callback)

	def pub_callback(self, event):
    	#print 'Timer called at ' + str(event.current_real)
    	msg_out = PoseStamped()

    	#fill in goal msg
    	msg_out.pose.position.x = self.waypoint[self.waypoint_counter][0]
    	msg_out.pose.position.y = self.waypoint[self.waypoint_counter][1]
    	msg_out.pose.position.z = self.waypoint[self.waypoint_counter][2]
    	msg_out.pose.orientation.x = self.uav_pose.orientation.x
    	msg_out.pose.orientation.y = self.uav_pose.orientation.y
    	msg_out.pose.orientation.z = self.uav_pose.orientation.z
    	msg_out.pose.orientation.w = self.uav_pose.orientation.w
    	
    	
    	self.pub.publish(msg_out)


	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ping.unregister()

	def callback(self, msg):

	    # Copying for simplicity
	    self.uav_pose = msg.pose
	    
	    # rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
	    # rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

	    # Also print Roll, Pitch, Yaw
		#self.euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	    # rospy.loginfo("Euler Angles: %s"%str(euler)) 


	    

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
    