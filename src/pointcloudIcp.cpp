/*
   Main fusion ROS node
 */
#include "pointcloudIcp.h"
#include <pcl/common/centroid.h>


PointcloudIcp::PointcloudIcp(Eigen::Matrix4f _G)
        : stereoCloud(new PointCloudT), SLCloud(new PointCloudPointXYZ),
        cloudTransform(new PointCloudT), G(_G) {

          ros::spin();
      }

PointcloudIcp::~PointcloudIcp() {
}

void PointcloudIcp::pointcloudSyncCallback(
        const sensor_msgs::PointCloud2ConstPtr &stereo_PC,
        const sensor_msgs::PointCloud2ConstPtr &sl_PC) {

        // Transform to PCL cloud type
        pcl::fromROSMsg(*stereo_PC, *stereoCloud);
        pcl::fromROSMsg(*sl_PC, *SLCloud);

        Eigen::Matrix4f _G = Eigen::Matrix4f::Identity();

        pcl::transformPointCloud(*stereoCloud, *cloudTransform, G);

        PointCloudPointXYZ::Ptr XYZStereoPC(new PointCloudPointXYZ);
        pcl::copyPointCloud(*cloudTransform, *XYZStereoPC);


        pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        icp.setInputSource(XYZStereoPC);
        icp.setInputTarget(SLCloud);
        pcl::PointCloud<PointXYZ> Final;
        icp.align(Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;



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
        G(3,3) = 1.0;
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "pointcloud_icp");
        ros::NodeHandle nh_("pointcloud_icp");

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
        // Create pointcloud fusion object
        PointcloudIcp pointcloudIcp(G);

        std::string stereo_topic = "/camera/points2";
        std::string SL_topic = "/seikowave_node/cloud";

        // Approx sync incoming pointcloud topics
        message_filters::Subscriber<sensor_msgs::PointCloud2> stereo_sub(
                nh_, stereo_topic, 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> SL_sub(nh_, SL_topic,
                                                                     1);
        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), stereo_sub,
                                                       SL_sub);
        sync.registerCallback(boost::bind(&PointcloudIcp::pointcloudSyncCallback,
                                          &pointcloudIcp, _1, _2));


        return 0;
}
