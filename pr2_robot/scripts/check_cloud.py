#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

def pcl_callback(pcl_msg):

#CHECKING POINT CLOUD	
	pcl_objects_pub.publish(pcl_msg)

if __name__ == '__main__':


    # TODO: ROS node initialization #Change the name
	rospy.init_node('final_object_recog', anonymous=True)
    # TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
#CHECKING POINT CLOUD	
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)

	
    # Initialize color_list
	get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
	 rospy.spin()

