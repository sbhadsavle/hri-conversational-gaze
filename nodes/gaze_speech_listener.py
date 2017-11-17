#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def gaze_callback(data):
    print("Received:   " + str(data.data) + "\r")

def speech_callback(data):
    print("Received:   " + str(data.data) + "\r")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gaze_speech_listener', anonymous=True)

    rospy.Subscriber("gaze", String, gaze_callback)
    rospy.Subscriber("speech", String, speech_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()