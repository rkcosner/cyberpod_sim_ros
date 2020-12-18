#!/usr/bin/env python

import rospy 
import numpy as np

from visualization_msgs.msg import Marker


topic = "obstacle_marker"
publisher = rospy.Publisher(topic, Marker, queue_size=10)


rospy.init_node('obstacle')

cube = 1 # rviz marker identifier 

while not rospy.is_shutdown(): 
    marker = Marker() 
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacle"
    marker.id = 0
    marker.action = 0 
    marker.type = cube
    marker.pose.position.x = 2.05
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5 
    marker.pose.orientation.x = 0 
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0 
    marker.pose.orientation.w = 0
    marker.scale.x = 0.1
    marker.scale.y = 2.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.3
    marker.lifetime = rospy.Duration()
    publisher.publish(marker)
    rospy.sleep(1)


