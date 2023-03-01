/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "System.h"

// leo: for pubbing
#include "Converter.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
// leo: end

using namespace std;

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM): mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;

    // leo: add for pub
    void SetPub(ros::Publisher* pub) {
        orb_pub = pub;
    }
    cv::Mat m1, m2;
    bool do_rectify, pub_tf, pub_pose;
    ros::Publisher* orb_pub;
    // leo: end
};

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << argv[0] << ", " << argv[1] << ", " << argv[2] << endl;
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings username" << endl;        
        return 1;
    }

    ros::init(argc, argv, "slam_" + string(argv[2]));
    ros::start();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[0], argv[1], ORB_SLAM3::System::MONOCULAR, false);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_" + string(argv[2]), 1, &ImageGrabber::GrabImage, &igb);

    // leo: add for pub
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::Quaternion>("slam_res_" + string(argv[2]), 100);
    igb.SetPub(&pose_pub);
    igb.pub_pose = true;
    // leo: end

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    float* s = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec()).data();
    for (int i = 0; i < 4; i++) {
        cout << s[i] << ", ";
    }
    cout << endl;

    geometry_msgs::Quaternion q_msg;
    q_msg.x = s[0];
    q_msg.y = s[1];
    q_msg.z = s[2];
    q_msg.w = s[3];
    orb_pub->publish(q_msg);

    // if (pub_tf || pub_pose) {
    //     if (!(T_.empty())) {

    //         cv::Size s = T_.size();
    //         if ((s.height >= 3) && (s.width >= 3)) {
    //             R_ = T_.rowRange(0,3).colRange(0,3).t();
    //             t_ = -R_*T_.rowRange(0,3).col(3);
    //             vector<float> q = ORB_SLAM3::Converter::toQuaternion(R_);
    //             float scale_factor=1.0;
    //             tf::Transform transform;
    //             transform.setOrigin(tf::Vector3(t_.at<float>(0, 0)*scale_factor, t_.at<float>(0, 1)*scale_factor, t_.at<float>(0, 2)*scale_factor));
    //             tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
    //             transform.setRotation(tf_quaternion);
    //             /*
    //             if (pub_tf)
    //             {
    //             static tf::TransformBroadcaster br_;
    //             br_.sendTransform(tf::StampedTransform(transform, ros::Time(tIm), "world", "ORB_SLAM3_MONO_INERTIAL"));
    //             }
    //             */

    //             if (pub_pose) {
    //                 geometry_msgs::PoseStamped pose;
    //                 //pose.header.stamp = img0Buf.front()->header.stamp;
    //                 pose.header.frame_id ="ORB_SLAM3_MONO_INERTIAL";
    //                 tf::poseTFToMsg(transform, pose.pose);
    //                 orb_pub->publish(pose);
    //             }

    //         }
    //     }
    // }
}