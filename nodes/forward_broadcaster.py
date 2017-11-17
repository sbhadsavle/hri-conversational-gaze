#!/usr/bin/env python  
# Demonstrates social navigation, adapted from Turtlesim tf tutorial
# Sarang Bhadsavle (ssb2243)  
import roslib
roslib.load_manifest('gaze_turtle')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('forward_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((4.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "subgoal",
                         "turtle1")
        rate.sleep()