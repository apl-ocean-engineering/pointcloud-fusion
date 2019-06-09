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

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/fusionConfig.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudPointXYZ;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    SyncPolicy;

class PointcloudFusion {
  // Inital pointcloud pointers
  PointCloudT::Ptr stereoCloud, stereoCloudSeperate;
  PointCloudPointXYZ::Ptr SLCloud;
  PointCloudT::Ptr cloudTransform, cloudTransformSeperate;

  // Sensor frame names
  std::string stereoFrameID;
  std::string SLFrameID;

  // Transformation between frames
  Eigen::Matrix4f G;

  // ROS node handeler
  ros::NodeHandle nh_;

  bool recieveSeikowave;

  // ROS publishers and transform broadcaster
  std::string steroTopic = "fusion/stereo/cloud";
  ros::Publisher stereoPub = nh_.advertise<PointCloudT>(steroTopic, 1);
  std::string SLTopic = "fusion/SL/cloud";
  ros::Publisher SLPub = nh_.advertise<PointCloudT>(SLTopic, 1);
  std::string concatenateTopic = "fusion/concatenated_cloud";
  ros::Publisher concatenatePub = nh_.advertise<PointCloudT>(concatenateTopic, 1);
  tf::TransformBroadcaster TB;

  float _x;
  float _y;
  float _z;
public:
  // Constructor/destructor
  PointcloudFusion(Eigen::Matrix4f _G, std::string _stereoFrameID,
                   std::string _SLFrameID, bool live_time_TF, bool republish_pointclouds);
  ~PointcloudFusion();
  // Pointcliud callback
  void pointcloudSyncCallback(const sensor_msgs::PointCloud2ConstPtr &stereo_PC,
                              const sensor_msgs::PointCloud2ConstPtr &sl_PC);
  void stereoCallback(const sensor_msgs::PointCloud2ConstPtr &stereo_PC);
  // Main tf publisher
  void tf_pub();

  void dynamicReconfigureCallback(sensor_fusion::fusionConfig &config,
                                  uint32_t level) {

    downsamplePC = config.downsamle_seikowave;
    pointKeepNum = config.pc_keep_num;
    concatenateOutput = config.concatenate_output;
    republishPC = config.republish_pointclouds;

    _x = config.x_motion;
    _y = config.y_motion;
    _z = config.z_motion;

    //std::cout << _x << std::endl;
  }


  //Live fusion param helpers
  ros::Time cameraPCTime;
  bool liveTF;
  bool republishPC;

  //Downsample params
  bool downsamplePC;
  int pointKeepNum;

  //Concatenate params
  bool concatenateOutput;

  std::string SLWorldFrame;
};
