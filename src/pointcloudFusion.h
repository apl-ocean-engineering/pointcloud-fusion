#include <ros/ros.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "sensor_msgs/PointCloud2.h"
#include "tf_conversions/tf_eigen.h"

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    SyncPolicy;

class PointcloudFusion {
  // Inital pointcloud pointers
  PointCloudT::Ptr stereoCloud;
  PointCloudT::Ptr SLCloud;
  PointCloudT::Ptr cloudTransform;

  // Sensor frame names
  std::string stereoFrameID;
  std::string SLFrameID;

  // Transformation between frames
  Eigen::Matrix4f G;

  // ROS node handeler
  ros::NodeHandle nh_;

  // ROS publishers and transform broadcaster
  std::string steroTopic = "fusion/stereo/cloud";
  ros::Publisher stereoPub = nh_.advertise<PointCloudT>(steroTopic, 1);
  std::string SLTopic = "fusion/SL/cloud";
  ros::Publisher SLPub = nh_.advertise<PointCloudT>(SLTopic, 1);
  tf::TransformBroadcaster TB;

public:
  // Constructor/destructor
  PointcloudFusion(Eigen::Matrix4f _G, std::string _stereoFrameID,
                   std::string _SLFrameID);
  ~PointcloudFusion();
  // Pointcliud callback
  void pointcloudSyncCallback(const sensor_msgs::PointCloud2ConstPtr &stereo_PC,
                              const sensor_msgs::PointCloud2ConstPtr &sl_PC);
  // Main tf publisher
  void tf_pub();
};
