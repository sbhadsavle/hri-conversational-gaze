#!/usr/bin/env python  
# Sarang Bhadsavle (ssb2243)  
import roslib
roslib.load_manifest('gaze_turtle')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
import turtlesim.srv

recent_key = 'none'

def key_callback(data):
    # print("---received key:   " + str(data.data) + "\r")
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


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(7, 5.5, math.pi, 'turtle2')
    spawner(6.5, 7, math.pi, 'object')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    gazePub = rospy.Publisher('gaze', String, queue_size=50)
    speechPub = rospy.Publisher('speech', String, queue_size=50)
    rospy.Subscriber("key", String, key_callback)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        #print("\r")
        try:
            (trans,rot) = listener.lookupTransform('/turtle1', '/world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #print("exception")
            continue

        
        rotationStr = None
        if rot[2] > -0.25 and rot[2] < 0.25:
            rotationStr = "@robot"
        elif rot[2] < -0.25 and rot[2] > -0.6:
            rotationStr = "@object"
        else:
            rotationStr = "@none"

        # print("Rotation: " + str(rot[2]) + "\r")
        

        gazePub.publish(rotationStr)
        speechPub.publish(recent_key)

        rate.sleep()
