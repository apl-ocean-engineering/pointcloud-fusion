/*
   Main fusion ROS node
 */
#include "pointcloudFusion.h"
#include <pcl/common/centroid.h>


PointcloudFusion::PointcloudFusion(Eigen::Matrix4f _G,
                                   std::string _stereoFrameID,
                                   std::string _SLFrameID, bool live_time_TF, bool republish_pointclouds)
        : stereoCloud(new PointCloudT), stereoCloudSeperate(new PointCloudT), SLCloud(new PointCloudPointXYZ),
        cloudTransform(new PointCloudT), cloudTransformSeperate(new PointCloudT), G(_G), stereoFrameID(_stereoFrameID),
        SLFrameID(_SLFrameID), liveTF(live_time_TF), republishPC(republish_pointclouds), _x(0),
        _y(0), _z(0), recieveSeikowave(false) {
        cameraPCTime = ros::Time::now();
        downsamplePC = false;
        pointKeepNum = 1;
        concatenateOutput = true;
      }

PointcloudFusion::~PointcloudFusion() {
}

void PointcloudFusion::pointcloudSyncCallback(
        const sensor_msgs::PointCloud2ConstPtr &stereo_PC,
        const sensor_msgs::PointCloud2ConstPtr &sl_PC) {


        recieveSeikowave = true;

        cameraPCTime = stereo_PC->header.stamp;
        if (republishPC) {
                // Transform to PCL cloud type
                pcl::fromROSMsg(*stereo_PC, *stereoCloud);
                pcl::fromROSMsg(*sl_PC, *SLCloud);

                //Downsample PC
                if (downsamplePC) {
                        PointCloudPointXYZ::Ptr cloudFiltered(new PointCloudPointXYZ);

                        for (int i=0; i<SLCloud->size(); i++) {
                                if (i%pointKeepNum != 0) {
                                        continue;
                                }
                                PointXYZ point = SLCloud->at(i);
                                cloudFiltered->points.push_back(point);
                        }

                        SLCloud = cloudFiltered;
                }
                // Transform stereo cloud to seikowave frame
                Eigen::Matrix4f _G = Eigen::Matrix4f::Identity();
                //
                _G(0, 3) = _x;
                _G(1, 3) = _y;
                _G(2, 3) = _z;
                //_G = _G*G;
                //std::cout << G << std::endl;

                pcl::transformPointCloud(*stereoCloud, *cloudTransform, G);

                // Publish stereo cloud
                cloudTransform->header.frame_id = SLFrameID;
                pcl_conversions::toPCL(ros::Time::now(), cloudTransform->header.stamp);
                stereoPub.publish(*cloudTransform);

                // Publish SL cloud
                SLCloud->header.frame_id = SLFrameID;
                pcl_conversions::toPCL(ros::Time::now(), SLCloud->header.stamp);
                SLPub.publish(*SLCloud);

                //Publish concatenated output
                if (concatenateOutput){
                  PointCloudPointXYZ XYZStereoPC;
                  pcl::copyPointCloud(*cloudTransform, XYZStereoPC);
                  PointCloudPointXYZ concatenatedPc;
                  concatenatedPc = XYZStereoPC;
                  concatenatedPc += *SLCloud;

                  concatenatedPc.header.frame_id = SLWorldFrame;
                  if (liveTF) {
                    pcl_conversions::toPCL(ros::Time::now(), concatenatedPc.header.stamp);
                  }
                  else{
                    pcl_conversions::toPCL(sl_PC->header.stamp, concatenatedPc.header.stamp);
                  }
                  concatenatePub.publish(concatenatedPc);
                  //recieveSeikowave = false;
                }
        }
}

void PointcloudFusion::stereoCallback(const sensor_msgs::PointCloud2ConstPtr &stereo_PC){
    pcl::fromROSMsg(*stereo_PC, *stereoCloudSeperate);
    pcl::transformPointCloud(*stereoCloudSeperate, *cloudTransformSeperate, G);
    PointCloudPointXYZ XYZStereoPC;
    pcl::copyPointCloud(*cloudTransformSeperate, XYZStereoPC);
    PointCloudPointXYZ concatenatedPc;
    concatenatedPc = XYZStereoPC;
    concatenatedPc.header.frame_id = SLWorldFrame;
    if (liveTF) {
      pcl_conversions::toPCL(ros::Time::now(), concatenatedPc.header.stamp);
    }
    else{
      pcl_conversions::toPCL(stereo_PC->header.stamp, concatenatedPc.header.stamp);
    }
    if (!recieveSeikowave){
      concatenatePub.publish(concatenatedPc);
    }
    else{
      recieveSeikowave = false;
    }
}

void PointcloudFusion::tf_pub() {
        ros::Rate loop_rate(30);
        while (ros::ok()) {
                // Convert from Eigen matrix to tf
                // Eigen::Quaterniond quaternion(G.block<3, 3>(0, 0).cast<double>());
                // tf::Transform transform;
                // transform.setOrigin(tf::Vector3(G(0, 3), G(1, 3), G(2, 3)));
                // tf::Quaternion q;
                // tf::quaternionEigenToTF(quaternion, q);
                // transform.setRotation(q);
                //
                // // Publish tf transform
                // if (liveTF) {
                //         TB.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(),
                //                                               stereoFrameID, SLWorldFrame));
                // }
                // else{
                //         TB.sendTransform(tf::StampedTransform(transform.inverse(), cameraPCTime,
                //                                               stereoFrameID, SLWorldFrame));
                // }
                //
                // // Get ROS info from callbacks


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
        G(3,3) = 1.0;
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
        std::string stereo_frame, SL_frame, SL_world_frame;

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

        if (nh_.getParam("SL_world_frame", SL_world_frame)) {
                ROS_INFO("Setting frame %s as SL output world frame", SL_world_frame.c_str());
        } else {
                ROS_WARN("No SL frame specified, setting SL_frame output frame to "
                         "SL_link");
                SL_world_frame = "SL_world_frame";
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
                if (live_tf) {
                        ROS_INFO("Will publish tf at clock time");
                }
                else{
                        ROS_INFO("Will publish tf at camera pointcloud time");
                }
        } else {
                ROS_INFO("Will publish tf at camera pointcloud time");
                live_tf = true;
        }

        //Determine if we want to downsample the Seikowave (assuming the stereo will not ever be downsampled)
        bool downsampleSeikowave;
        int pcKeepNum;
        if (nh_.getParam("downsamle_seikowave", downsampleSeikowave)) {
                if (downsampleSeikowave) {
                        ROS_INFO("Will downsample SL pointcloud");
                }
                else{
                        ROS_INFO("Will not downsample pointcloud");
                }
        } else {
                ROS_INFO("Will not downsample pointcloud");
                downsampleSeikowave = true;
        }
        if (nh_.getParam("pc_keep_num", pcKeepNum) && downsampleSeikowave) {
            ROS_INFO("Pc keep num: %i", pcKeepNum);
        } else {
                ROS_INFO("Not downsampling, pcKeepNum set to 1");
                pcKeepNum = 1; //This means keep every point
        }

        bool concatenateOutput = false;
        if (nh_.getParam("concatenate_output", concatenateOutput)) {
                if (concatenateOutput) {
                        ROS_INFO("Will concatenate pointclouds");
                }
                else{
                        ROS_INFO("Will not concatenate pointclouds");
                }
        } else {
                ROS_INFO("Will not concatenate pointclouds");
                concatenateOutput = true;
        }

        // Create pointcloud fusion object
        PointcloudFusion pointcloudFusion(G, stereo_frame, SL_frame, live_tf, republish_pointclouds);

        //Set filtering options
        pointcloudFusion.downsamplePC = downsampleSeikowave;
        pointcloudFusion.pointKeepNum = pcKeepNum;
        pointcloudFusion.concatenateOutput = concatenateOutput;

        pointcloudFusion.SLWorldFrame = SL_world_frame;


        // Approx sync incoming pointcloud topics
        message_filters::Subscriber<sensor_msgs::PointCloud2> stereo_sub(
                nh_, stereo_topic, 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> SL_sub(nh_, SL_topic,
                                                                     1);
        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), stereo_sub,
                                                       SL_sub);
        sync.registerCallback(boost::bind(&PointcloudFusion::pointcloudSyncCallback,
                                          &pointcloudFusion, _1, _2));

        ros::Subscriber stereoStandalone =
            nh_.subscribe<sensor_msgs::PointCloud2>(
                stereo_topic, 1, &PointcloudFusion::stereoCallback,
                &pointcloudFusion);

        dynamic_reconfigure::Server<sensor_fusion::fusionConfig> server;
        dynamic_reconfigure::Server<sensor_fusion::fusionConfig>::CallbackType f;

        f = boost::bind(&PointcloudFusion::dynamicReconfigureCallback, &pointcloudFusion, _1, _2);
        server.setCallback(f);

        // Main ROS loop. Publish tf
        pointcloudFusion.tf_pub();

        return 0;
}
