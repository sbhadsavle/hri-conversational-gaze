#!/usr/bin/env python  
#Priyanka Khante (pk7426)
import rospy
import rosbag
from object_detector.msg import GazeTopic

def gaze_callback(data):
	#print("Received:   " + str(data.mutual) + "\r")
	if data.mutual:
		print("Robot")    
	print("Received:   " + str(data.coordinates) + "\r")
	if data.coordinates.data > 0:
		print("Got it")
    
def listener():
	# In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gaze_detector', anonymous=True)

    rospy.Subscriber("/object_detector_bridge", GazeTopic, gaze_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
