#!/usr/bin/env python
import rospy
from std_msgs.msg import String

recent_speech_key = 'none'
recent_gaze_key = '@robot'

def key_callback(data):
    #print("Received:   " + str(data.data) + "\r")
    global recent_speech_key
    global recent_gaze_key
    if data.data == '1':
        recent_speech_key = 'object'
    elif data.data == '2':
        recent_speech_key = 'other'
    elif data.data == '3':
        recent_speech_key = 'uhum'
    elif data.data == '4':
        recent_speech_key = 'none'
    elif data.data == '5':
        recent_gaze_key = '@robot'
    elif data.data == '6':
        recent_gaze_key = '@object'
    elif data.data == '7':
        recent_gaze_key = '@other'
    else:
        None
   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gaze_speech_listener', anonymous=True)

    rospy.Subscriber("key", String, key_callback)

    gazePub = rospy.Publisher('gaze', String, queue_size=50)
    speechPub = rospy.Publisher('speech', String, queue_size=50)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        gazePub.publish(recent_gaze_key)
        speechPub.publish(recent_speech_key)
        print("Publishing:   " + str(recent_speech_key) + ", " + str(recent_gaze_key) +  "\r")
        rate.sleep()

if __name__ == '__main__':
    listener()
