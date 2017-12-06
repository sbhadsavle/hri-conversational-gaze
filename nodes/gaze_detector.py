#!/usr/bin/env python  
#Priyanka Khante (pk7426)
#Deduces if the human is looking at the "robot, object, other" and publishes to a topic
# PARAMETERS TO REMEMBER: Manhattan distance Threshold: 160, Robot's head at 0.3 tilt. Red markings on the table and the floor are ours.
import rospy
from object_detector.msg import GazeTopic
from std_msgs.msg import String
from random import randint
import time

gazePub = rospy.Publisher('gaze', String, queue_size=50)
predict_object = [4.5, 250, 315.5, 539]
predict_other = [613.5, 299, 804.5, 512]

def gaze_callback(data):
	print("Received:   " + str(data.mutual) + "\r")
	gaze_str = ""
	gaze_other = False

	# For mutual gaze
	if data.mutual.data:
		print ("@robot")
		gaze_str = "@robot" 
	   
	#print("Received:   " + str(data.coordinates) + "\r")
	if not data.mutual.data:
		if data.coordinates.data > 0:
			#print("@object")
			# For object and gaze aversion
			if (data.coordinates.data[0]>predict_object[0] and data.coordinates.data[0]<predict_object[2]) and (data.coordinates.data[1]>predict_object[1] and data.coordinates.data[1]<predict_object[3]): 
				gaze_str = "@object"
			if (data.coordinates.data[0]>predict_other[0] and data.coordinates.data[0]<predict_other[2]) and (data.coordinates.data[1]>predict_other[1] and data.coordinates.data[1]<predict_other[3]): 
				gaze_str = "@other"
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

# DELETE LATER - Randomly generate OTHER for a certain time duration
		#if randint(0,1) == 1:
			#print("Here")
			#gaze_other = True
					
	#print("Publishing: ", gaze_str)
	#if gaze_other == False:
		#print("Here21")
		#gazePub.publish(gaze_str)
	#else:
		#t_end = time.time() + 3.0
		#while True:
			#print("Other")
			#gaze_str = "Other"
			#gazePub.publish(gaze_str)
			#if time.time() > t_end:
				#break
    
    
