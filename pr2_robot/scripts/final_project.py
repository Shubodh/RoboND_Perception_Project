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


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data: 
	#This function takes in a ROS message of type PointCloud2 and converts it to PCL 	PointXYZRGB format.
	cloud_ros = ros_to_pcl(pcl_msg)
	

    # TODO: Voxel Grid Downsampling
	vox = cloud_ros.make_voxel_grid_filter()
	LEAF_SIZE = 0.01
	
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	cloud_filtered = vox.filter()	
	cloud_filtered_bleh =  pcl_to_ros(cloud_filtered)   	
	pcl_basic.publish(cloud_filtered_bleh)
#    #TODO: Applying Outlier Removal Filter to remove noise
# 
	outlier_filter = cloud_filtered.make_statistical_outlier_filter()
#	#Number of neighboring points to analyze for any given point	
	outlier_filter.set_mean_k(20)

#	#Threshold Scale Factor	
	x = 0.001
#	
#	# Any point with a mean distance larger than global (mean distance+x*std_dev) will 		be considered outlier
	outlier_filter.set_std_dev_mul_thresh(x)

#	# Finally call the filter function for magic
	cloud_filtered = outlier_filter.filter()


#	

#   # TODO: PassThrough Filter
	passthrough_z = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	passthrough_z.set_filter_field_name(filter_axis)
	axis_min = 0.6
	axis_max = 1.1
	passthrough_z.set_filter_limits(axis_min, axis_max)	
	cloud_filtered = passthrough_z.filter()

	passthrough_x = cloud_filtered.make_passthrough_filter()
	filter_axis = 'x'
	passthrough_x.set_filter_field_name(filter_axis)
	axis_min = 0.4
	axis_max = 1
	passthrough_x.set_filter_limits(axis_min, axis_max)	
	cloud_filtered = passthrough_x.filter()


	cloud_filtered_ros =  pcl_to_ros(cloud_filtered)	
#	pcl_basic_in.publish(cloud_filtered_ros)

#    # TODO: RANSAC Plane Segmentation 
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)
	inliers, coefficients = seg.segment()


#	
#    # TODO: Extract inliers and outliers
	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	extracted_outliers = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert XYZRGB to XYZ
	tree = white_cloud.make_kdtree()
	ec = white_cloud.make_EuclideanClusterExtraction()
	#Cluster Tolerance will determine which cluster will get which color, i.e. if it is too high, nearby objects might get same colour. So experiment and optimize.	
	ec.set_ClusterTolerance(0.02)
	ec.set_MinClusterSize(50)
	ec.set_MaxClusterSize(10000)	
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
	#Assign a color corresponding to each segmented object in scene
	cluster_color = get_color_list(len(cluster_indices))

	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
	    for i, indice in enumerate(indices):
		color_cluster_point_list.append([white_cloud[indice][0],
		                                white_cloud[indice][1],
		                                white_cloud[indice][2],
		                                 rgb_to_float(cluster_color[j])])

	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
#		
#	


#    # TODO: Convert PCL data to ROS messages


	cloud_objects =  pcl_to_ros(extracted_outliers)
	ros_cloud_table = pcl_to_ros(extracted_inliers)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)

#    # TODO: Publish ROS messages
	pcl_objects_pub.publish(cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)

##CHECKING POINT CLOUD	
#	pcl_objects_pub.publish(pcl_msg)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)

	detected_objects_labels = []
	detected_objects = []	

	for index, pts_list in enumerate(cluster_indices):
		# Grab the points for the cluster from the extracted outliers (cloud_objects)
		pcl_cluster = extracted_outliers.extract(pts_list)
		# TODO: convert the cluster from pcl to ROS using helper function
		ros_cluster = pcl_to_ros(pcl_cluster)

		# Extract histogram features
		# TODO: complete this step just as is covered in capture_features.py
		chists = compute_color_histograms(ros_cluster, using_hsv=True) 
		normals = get_normals(ros_cluster) 
		nhists = compute_normal_histograms(normals) 
		feature = np.concatenate((chists, nhists)) 
		#labeled_features.append([feature, model_name]) 
		

		# Make the prediction, retrieve the label for the result
		# and add it to detected_objects_labels list
		prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)

		# Publish a label into RViz
		label_pos = list(white_cloud[pts_list[0]])
		label_pos[2] += .4
		object_markers_pub.publish(make_label(label,label_pos, index))

		# Add the detected object to the list of detected objects.
		do = DetectedObject()
		do.label = label
		do.cloud = ros_cluster
		detected_objects.append(do)

	rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
	detected_objects_pub.publish(detected_objects)
 	

#    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
#    # Could add some logic to determine whether or not your object detections are robust
#    # before calling pr2_mover()
	try:
		pr2_mover(detected_objects)
	except rospy.ROSInterruptException:
        	pass

#This function compares detected labels with actual ones, takes input as object_list_param and detected_objects in pr2_mover function and returns the accuracy number out of total i.e. len(object_list_param)
def get_accuracy(object_l_p, detected_objs):

	accuracy_num = 0
	
	actual_labels = [i_object['name'] for i_object in object_l_p]

	for detected_obj in detected_objs:
		detected_obj_label = detected_obj.label
		
		if detected_obj_label in actual_labels:
			accuracy_num += 1
#Now we have to delete the detected object label from the whole actual list because the wrongly detected label might match with already existing label and give us a wrong prediction
			actual_labels.remove(detected_obj_label)
		else:
			accuracy_num += 0

	return accuracy_num	


#Since the order of detected objects list and actual object list need not necessarily be same, we should define another function which gives the right order of detected objects as per actual list we get from parameter server: (Same input arguments as before function obviously)
def ordered_list(object_l_p, detected_objs):
	ordered_objects = []
	
	for i in range(len(object_l_p)):
		
		actual_label = object_l_p[i]['name']

		for detected_obj in detected_objs:
			if detected_obj.label == actual_label:
				ordered_objects.append(detected_obj)
				#Same logic for removing here as previous function.
				detected_objs.remove(detected_obj)
				#Once it is detected, we're done.
				break
	return ordered_objects


#Ordered drop groups as per previous function
def ordered_drop_groups(ordered_l, object_l_p):
	drop_groups = []
	for object_i in ordered_l:
		for object_j in object_l_p:
			if object_j['name'] == object_i.label:
				drop_groups.append(object_j['group'])
				break
	return drop_groups	





#Function which gives coordinates of centroid of each cluster: Takes ordered_list function as input
def centroid_coordinates(ordered_l):
	#labels = []
	centroids = [] # to be list of tuples (x, y, z)
	for object_i in ordered_l:
		#labels.append(object_i.label)
		points_arr = ros_to_pcl(object_i.cloud).to_array()
		centroids.append(np.mean(points_arr, axis=0)[:3])
	return centroids

## function to load parameters and request PickPlace service
def pr2_mover(detected_objects_list):

#    # TODO: Initialize variables

#    # TODO: Get/Read parameters

	# get parameters (-object list, test scene num, and so on) from ros parameter server which are loaded from project launch file
	object_list_param = rospy.get_param('/object_list')
	#Dropbox later?


	# This variable is not a simple integer but a ROS message of type std_msgs/Int32. 
	#1: Test scene number
	test_scene_num = Int32()
	#CHANGE THIS 1,2,3	
	test_scene_value = 2 
	test_scene_num.data = int(test_scene_value)



#    # TODO: Parse parameters into individual variables
#    # TODO: Rotate PR2 in place to capture side tables for the collision map: LATER	
#    # TODO: Loop through the pick list
#        # TODO: Get the PointCloud for a given object and obtain it's centroid
	accuracy = get_accuracy(object_list_param, detected_objects_list)
	rospy.loginfo('Accuracy {} in total {} objects.'.format(accuracy,len(object_list_param)))

	ordered_list_final = ordered_list(object_list_param, detected_objects_list)
	#rospy.loginfo('ordered list is {}'.format(ordered_list_final))
	drop_groups_final = ordered_drop_groups(ordered_list_final, object_list_param)
	centroids = centroid_coordinates(ordered_list_final)

	dict_list = []
	for i in range(len(ordered_list_final)):
		#Let's rock n roll
		#2: Object name
		object_name = String()
		object_name.data = str(ordered_list_final[i].label)

		# 3: Arm Name
		object_group = drop_groups_final[i]
		arm_name = String()

		if object_group == 'green':
			arm_name.data = 'right'
		else:
			arm_name.data = 'left'

		#4: Pick Pose (Centroid)
		pick_pose = Pose()

		current_centroid = centroids[i]
		#WARNING: ROS messages expect native Python data types but having computed centroids as above your list centroids will be of type numpy.float64. To recast to native Python float type you can use np.asscalar(), 
		float_centroid = [np.asscalar(centroid_t) for centroid_t in current_centroid]
		
		pick_pose.position.x = float_centroid[0]
		pick_pose.position.y = float_centroid[1]
		pick_pose.position.z = float_centroid[2]

		#5: Later
		place_pose = Pose()
		
		place_pose.position.x = 0
		place_pose.position.y = 0
		place_pose.position.z = 0


	    # Populate various ROS messages
		yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
	    	dict_list.append(yaml_dict)
		

##Based on the group associated with each object (that you extracted from the pick list .yaml file), decide which arm of the robot should be used. 


#        # TODO: Create 'place_pose' for the object, Assign the arm to be used for pick_place, Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

#        # Wait for 'pick_place_routine' service to come up
		rospy.wait_for_service('pick_place_routine')

		try:
        		pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

           # TODO: Insert your message variables to be sent as a service request
          		resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

			print("Response: {}".format(resp.success))

		except rospy.ServiceException, e:
			print("Service call failed: {}".format(e))

#    # TODO: Output your request parameters into output yaml file
	yaml_filename = "output_{}.yaml".format(test_scene_num.data) 
	send_to_yaml(yaml_filename, dict_list)


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

	pcl_basic = rospy.Publisher("/basic_stuff", PointCloud2, queue_size=1)
#	pcl_basic_in = rospy.Publisher("/basic_in", PointCloud2, queue_size=1)
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/cluster", PointCloud2, queue_size=1)
	object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk: READING IN YOUR TRAINED MODEL
	model = pickle.load(open('model_final_2.sav', 'rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']
    # Initialize color_list
	get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
	 rospy.spin()



