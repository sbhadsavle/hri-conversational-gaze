#!/usr/bin/env python
import rospy
from std_msgs.msg import String

recent_key = 'none'

def gaze_callback(data):
    print("Received:   " + str(data.data) + "\r")

def speech_callback(data):
    #print("Received:   " + str(data.data) + "\r")
    global recent_key
    if data.data == '1':
        recent_key = 'object'
        #speechPub.publish("object")
    elif data.data == '2':
        recent_key = 'other'
        #speechPub.publish("other")
    elif data.data == '3':
        recent_key = 'uhum'
        #speechPub.publish("uhum")
    elif data.data == '4':
        recent_key = 'none'
        #speechPub.publish("none")
    else:
        recent_key = 'none'
        #speechPub.publish("none")

   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gaze_speech_listener', anonymous=True)

    rospy.Subscriber("gaze", String, gaze_callback)
    rospy.Subscriber("key", String, speech_callback)

    #gazePub = rospy.Publisher('gaze', String, queue_size=50)
    speechPub = rospy.Publisher('speech', String, queue_size=50)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        #gazePub.publish(rotationStr)
        speechPub.publish(recent_key)
        print("Publishing:   " + str(recent_key) + "\r")
        rate.sleep()

if __name__ == '__main__':
    listener()
