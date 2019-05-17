#Sensor Fusion

This repo contains a simple ROS node and corresponding launch file to produce stereo pointclouds, and transform those pointclouds into the frame of an SL sensor. 

## Launch file: sensor_fusion.launch 
This launch file crops input images by four (to help with processing speed), feeds the cropped images into the standard ROS node stereo_image_proc 
to produce stereo pointclouds, while also running the node pointcloud_fusion, which publishes the static extrinsic tf transform between the SL sensor frame and the stereo frame and transforms the stereo pointcloud into the SL sensor's frame.

The launch file has been configured to work with pointgrey cameras and a seikowave SL sensor. As such, the launch file as currently configured, have the following input output structure:

### Topic Inputs
* /camera/left/image_raw (sensor_msgs/Image)
	Left input image 
* /camera/right/image_raw (sensor_msgs/Image)
	Right input image 
* /camera/left/camera_info (sensor_msgs/CameraInfo)
	Left camera info
* /camera/right/camera_info (sensor_msgs/CameraInfo)
	Right camera info
* /seikowave_node/cloud (sensor_msgs/PointCloud2)
	SL pointcloud input

### Topic Outputs
* /camera/points2 (sensor_msgs/PointCloud2)
	Stereo camera pointcloud output at frame rate
* /fusion/stereo/cloud (sensor_msgs/PointCloud2)
	Stereo camera pointcloud output synced with SL sensor at lowest framerate
* /fusion/SL/cloud (sensor_msgs/PointCloud2)
	SL sensor pointcloud output synced with stero pointcloud at lowest framerate
* /tf
	Publishes the transform between the stereo pointcloud frame (default: head_stereo/left_optical_frame) and seikowave pointcloud frame (default: seikowave/optical_frame)

### Pointcloud Fusion


