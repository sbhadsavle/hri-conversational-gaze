#!/usr/bin/env python  
#Priyanka Khante (pk7426)
#Deduces if the human is looking at the "robot, object, other" and publishes to a topic
import rospy
from object_detector.msg import GazeTopic
from std_msgs.msg import String
from random import randint
import time

gazePub = rospy.Publisher('gaze', String, queue_size=50)

def gaze_callback(data):
	print("Received:   " + str(data.mutual) + "\r")
	gaze_str = ""
	gaze_other = False

	# For mutual gaze
	if data.mutual.data:
		print ("@robot")
		gaze_str = "@robot" 
	   
	# For object and gaze aversion
	#TODO: Gaze aversion
	#print("Received:   " + str(data.coordinates) + "\r")
	if not data.mutual.data:
		if data.coordinates.data > 0:
			print("@object")
			gaze_str = "@object"
	
	gazePub.publish(gaze_str)

    
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
    starttime = time.time()

    
    
