/*
Main fusion ROS node
*/
#include "pointcloudFusion.h"
#include <pcl/common/centroid.h>

PointcloudFusion::PointcloudFusion(Eigen::Matrix4f _G,
                                   std::string _stereoFrameID,
                                   std::string _SLFrameID, bool live_time_TF, bool republish_pointclouds)
    : stereoCloud(new PointCloudT), SLCloud(new PointCloudPointXYZ),
      cloudTransform(new PointCloudT), G(_G), stereoFrameID(_stereoFrameID),
      SLFrameID(_SLFrameID), liveTF(live_time_TF), republishPC(republish_pointclouds) {
        cameraPCTime = ros::Time::now();
      }

PointcloudFusion::~PointcloudFusion() {}

void PointcloudFusion::pointcloudSyncCallback(
    const sensor_msgs::PointCloud2ConstPtr &stereo_PC,
    const sensor_msgs::PointCloud2ConstPtr &sl_PC) {

  cameraPCTime = stereo_PC->header.stamp;
  if (republishPC){
    // Transform to PCL cloud type
    pcl::fromROSMsg(*stereo_PC, *stereoCloud);
    pcl::fromROSMsg(*sl_PC, *SLCloud);

    // pcl::CentroidPoint<PointXYZ> centroid;
    // pcl::computeCentroid(*SLCloud, centroid);
    // PointXYZ point;
    // centroid.get(point);
    //
    // ROS_INFO("SL pointcloud centroid: %f, %f, %f", point.x, point.y, point.z);


    // Transform stereo cloud to seikowave frame
    pcl::transformPointCloud(*stereoCloud, *cloudTransform, G);

    // Publish stereo cloud
    cloudTransform->header.frame_id = SLFrameID;
    pcl_conversions::toPCL(ros::Time::now(), cloudTransform->header.stamp);
    stereoPub.publish(*cloudTransform);

    // Publish SL cloud
    SLCloud->header.frame_id = SLFrameID;
    pcl_conversions::toPCL(ros::Time::now(), SLCloud->header.stamp);
    SLPub.publish(*SLCloud);
  }
}

void PointcloudFusion::tf_pub() {
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    // Convert from Eigen matrix to tf
    Eigen::Quaterniond quaternion(G.block<3, 3>(0, 0).cast<double>());
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(G(0, 3), G(1, 3), G(2, 3)));
    tf::Quaternion q;
    tf::quaternionEigenToTF(quaternion, q);
    transform.setRotation(q);

    // Publish tf transform
    if (liveTF){
      TB.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(),
                                          stereoFrameID, SLFrameID));
    }
    else{
      TB.sendTransform(tf::StampedTransform(transform.inverse(), cameraPCTime,
                                          stereoFrameID, SLFrameID));
    }

    // Get ROS info from callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void loadExtrinsics(ros::NodeHandle nh_, Eigen::Matrix4f &G) {
  std::vector<double> _R;
  std::vector<double> _T;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;

  // Load rotation matrix
  int idx = 0;
  if (nh_.getParam("/SL_extrinsics/R/data", _R)) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R(i, j) = _R.at(idx);
        idx++;
      }
    }
  } else {
    ROS_FATAL("Failed to load SL rotation matrix. Suppressing extrinsics");
    R = Eigen::Matrix3d::Identity();
  }

  // Load translation vector
  if (nh_.getParam("/SL_extrinsics/t/data", _T)) {
    T = Eigen::Vector3d(_T.data());
  } else {
    ROS_FATAL("Failed to load SL translation matrix. Suppressing extrinsics");
    T = Eigen::Vector3d(0, 0, 0);
  }

  // Set Se3 transformation
  G.block<3, 3>(0, 0) = R.cast<float>();
  G.block<3, 1>(0, 3) = T.cast<float>();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_fusion");
  ros::NodeHandle nh_("pointcloud_fusion");

  // Load extrinsics from rosparam
  std::string extrinsics_ns;
  if (nh_.getParam("extrinsics_ns", extrinsics_ns)) {
    ROS_INFO("Setting the extrinsics namepsace as %s ", extrinsics_ns.c_str());
  } else {
    ROS_WARN("Setting the extrinsics ns as /SL_extrinsics");
    extrinsics_ns = "/SL_extrinsics";
  }
  Eigen::Matrix4f G;
  loadExtrinsics(nh_, G);

  // Set default topic names
  std::string stereo_topic = ros::names::resolve("stereo/points2");
  std::string SL_topic = ros::names::resolve("SL/points2");
  std::string stereo_frame, SL_frame;

  // Get stereo frame (e.g., camera_left)
  if (nh_.getParam("stereo_frame", stereo_frame)) {
    ROS_INFO("Setting frame %s as stereo output frame", stereo_frame.c_str());
  } else {
    ROS_WARN("No stereo frame specified, setting stereo frame output frame to "
             "SL/optical_frame");
    stereo_frame = "stereo/left_optical_frame";
  }

  // Get stereo frame (e.g., seikowave/optical_frame)
  if (nh_.getParam("SL_frame", SL_frame)) {
    ROS_INFO("Setting frame %s as SL output frame", SL_frame.c_str());
  } else {
    ROS_WARN("No SL frame specified, setting SL_frame output frame to "
             "SL/optical_frame");
    SL_frame = "SL/optical_frame";
  }

  bool republish_pointclouds;
  // Decide if to re-publish pointcloud data or not
  if (nh_.getParam("republish_pointclouds", republish_pointclouds)) {
    ROS_INFO("Will republish input pointcloud topic");
  } else {
    ROS_INFO("Will not re-publish pointclouds");
    republish_pointclouds = true;
  }

  //Determine if we want to publish in live time or a camera pointcliud time
  // (publishing at live time seemed to fail with rosbag -> live data fusions)

  bool live_tf;
  if (nh_.getParam("live_tf", live_tf)) {
    if (live_tf){
      ROS_INFO("Will publish tf at clock time");
    }
    else{
      ROS_INFO("Will publish tf at camera pointcloud time");
    }
  } else {
    ROS_INFO("Will publish tf at camera pointcloud time");
    live_tf = true;
  }

  // Create pointcloud fusion object
  PointcloudFusion pointcloudFusion(G, stereo_frame, SL_frame, live_tf, republish_pointclouds);
  // Approx sync incoming pointcloud topics
  message_filters::Subscriber<sensor_msgs::PointCloud2> stereo_sub(
      nh_, stereo_topic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> SL_sub(nh_, SL_topic,
                                                               1);
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), stereo_sub,
                                                 SL_sub);
  sync.registerCallback(boost::bind(&PointcloudFusion::pointcloudSyncCallback,
                                    &pointcloudFusion, _1, _2));


  // Main ROS loop. Publish tf
  pointcloudFusion.tf_pub();

  return 0;
}
