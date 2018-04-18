## Project: Perception Pick & Place 
## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points 
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.   
Please note that all the submission files - final scripts and output YAML files have been added to `RoboND-Perception-Project/pr2_robot/scripts` for your convenience. However, during implementation, the scripts were executed separately as a part of `sensor_stick`(for training) and   `pr2_robot` subdirectories. If you want to execute separately, make `sensor_stick` and `RoboND-Perception-Project` main directories of catkin src folder.
### Exercise 1, 2 and 3 pipeline implemented The following is executed in final_project.py.
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented. 
1.	Firstly, to process and apply techniques to extract information from the point cloud, it is converted from ROS to PCL format.
2.	Now, we apply Voxel Grid Downsampling Filter. Essentially, it is a technique to divide 3D point cloud into 3D grid of volume elements so as to make it less computationally demanding. A leaf size of 0.007 was chosen after experimentation as a trade-off between my PC’s computational capacity and point cloud resolution.
3.	There is noise associated with any data gathered, therefore to remove these outlier data, we apply Outlier Removal Filter. I experimented with the two parameters – x (outlier threshold) and k (number of neighbouring pixels to analyse), and by visually observing the output, I could finalize the values at 0.5 and 10 respectively.
4.	Passthrough filter, as the name suggests, is used to select the area of interest. By manually experimenting and checking the cropped output, I found out that my area of interest lies between 0.6 and 1.1 in z direction & 0.4 and 1 in x direction. 
5.	RANSAC Plane Segmentation Algorithm is a great way to segment out planar surfaces from your area of interest. The default value was left as it is as it seemed to be doing the job. For further processing of the point cloud, we use clustering algorithms as follows.


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented. 
Alright, we have segmented out our objects from the plane by applying various filtering and segmentation techniques. But how do we cluster each object separately?
This is where clustering techniques like c/k-means, Euclidean (DBSCAN) clustering kick in. For this project, I have used Euclidean clustering, however for further improvement, I will be using k-means later.
DBSCAN is a density based clustering algorithms and basically works on the basis of proximity of one point from other point. It is necessary to ensure Minimum Cluster Size is not too high to ensure that small objects are not missed, and also it should not be too less as undesirable outlier points might be classified as objects. Similar reasoning can be done to Maximum Cluster Size parameter for avoiding very low parameter value. Very high value should be avoided as two different objects might be classified as one. Therefore, the values were logically chosen and tuned based on observations as 50 and 5000 respectively.
 
#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented. The following is executed in captures_features.py.
So we have clustered each object, but how do we achieve object recognition, i.e. how to classify it as a particular object?
We use a Machine Learning Algorithm - Support Vector Machine Algorithm. A particular object one by one is spawned at various location and orientation, and this generated data acts as training input for the algorithm.
There are 3 fundamental ways in which the SVM classifier can be improved.
1.	Increase the number of times the object is getting spawned: I have varied the number between 5 to 25, have increased it as the number of objects in the scene increased. If we are able to say from the confusion matrix that the object is not wrongly getting classified, I stopped increasing this parameter. (in captures_features.py)
2.	Turning on HSV instead of RGB: HSV is less sensitive to lighting, therefore it is more accurate in all cases. Turning that on has significantly improved the results. (in train_svm.py)
3.	Tuning the parameters of SVM algorithm (in train_svm.py): There was no necessity to do that at this stage, but to further improve the model, these parameters can certainly be tuned later.

As it is evident from the below Normalized Confusion Matrix for Scene 3, it is able to receive good cross validation score, such that it is distinguishable enough from other objects. Similarly, its parameters were tuned till I reached this stage for Scene 1 and 2 as well.


 
![Normalized_Confusion](https://github.com/Shubodh/RoboND_Perception_Project/Normalized_Confusion.png) 
 
### Pick and Place Setup 
#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format. 
 Alright, so we’re now ready with our trained model. Now this model is used in `pcl_callback` function to infer class labels for finding the most likely object name. However, it is necessary to define more functions as explained below:
Since the order of detected objects list and actual object list need not necessarily be same, I have defined another function `ordered_list(olbjects, detected_objects)` which gives the right order of detected objects as per actual list we get from parameter server. 
Another function which gives coordinates of centroid of each cluster also has been defined `centroid_coordinates(ordered_list)` to send the desired output to yaml files.
However, I have not worked on the placing part yet, but have successfully completed everything for the passing of this project. I will work on place aspect and make PR2 happy soon.
For correctly setting up pick and place, we have to edit the launch file and final_project.py file such that only the appropriate test world is getting launched and appropriate .sav file is being read.
All the objects were successfully recognized in all the test scenes as shown below:

World 1:
Please find the output_1.yaml file and the corresponding screenshot:

![World1_Recognition](https://github.com/Shubodh/RoboND_Perception_Project/World1_Recognition.png) 

World 2:
Please find the output_2.yaml file and the corresponding screenshot and terminal output information:
[INFO] [1524034738.269987, 1129.040000]: Detected 5 objects: ['biscuits', 'book', 'soap', 'soap2', 'glue']

![World2_Recognition](https://github.com/Shubodh/RoboND_Perception_Project/World2_Recognition.png) 

World 3:
Please find the output_3.yaml file and the corresponding screenshot and terminal output information:

[INFO] [1524035461.593624, 1348.157000]: Detected 8 objects: ['snacks', 'biscuits', 'book', 'soap', 'eraser', 'sticky_notes', 'soap2', 'glue']

![World3_Recognition](https://github.com/Shubodh/RoboND_Perception_Project/World3_Recognition.png) 
 
Making PR2 happy: Placing the objects + Extra Challenges – Coming soon!   
 
 

