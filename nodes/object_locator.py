#!/usr/bin/env python  
#Priyanka Khante (pk7426)
# Reads in from the /belief/clusters topic from hlpr_segmentation and extracts the intial 3D object coordinates
import rospy
from geometry_msgs.msg import Point, PointStamped
from tf import TransformListener
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from hlpr_perception_msgs.msg import SegClusters 

nan = float('nan')

def get_avg_position(pc, tf, frame='/base_link'):
    print("Getting average position")
    position = np.zeros(3)
    n = 0

    for point in pc2.read_points(pc):
        position += [point[0], point[1], point[2]]
        n += 1

    position /= n

    ps = PointStamped()
    ps.header.frame_id = pc.header.frame_id
    ps.point.x = position[0]
    ps.point.y = position[1]
    ps.point.z = position[2]
    ps_tf = tf.transformPoint(frame, ps)

    return [ps_tf.point.x, ps_tf.point.y, ps_tf.point.z]

def vision_callback(data):
    print("In vision callback")
    positions = []
    tf = TransformListener(True, rospy.Duration(50))
	
    for cluster in data.clusters:
		position = get_avg_position(cluster, tf)
		positions.append[position]
	
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('initial_object_locator', anonymous=True)

    rospy.Subscriber("/beliefs/clusters", SegClusters, vision_callback)
    print("Here")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
	listener()
		
