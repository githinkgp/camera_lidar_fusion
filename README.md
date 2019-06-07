data fusion/association for astra+dnn_detect and rplidar_a3

need to align the astra and the lidar

--
ROS message published:
Fused person state measurement
Lidar cloud points of all
Lidar cloud points of person


--
Further dev note
The detected human may be blocked by obstacles in the Lidar's view; need to disregard them


#########################################################################################

cmd to run astra, dnn_detect and lidar:
roscore
roslaunch astra_launch astra.launch
roslaunch dnn_detect dnn_detect.launch camera:=/camera/rgb image:=/image_raw

roslaunch rplidar_ros test_rplidar_a3.launch

rostopic echo /dnn_objects
