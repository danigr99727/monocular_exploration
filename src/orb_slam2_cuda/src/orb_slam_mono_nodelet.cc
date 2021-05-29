/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>

#include<ros/ros.h>

#include <opencv2/core/core.hpp>
#include <System.h>

#include "orb_slam_wrapper.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class ORBSLAMNodelet : public nodelet::Nodelet {
public:
    ORBSLAMNodelet()=default;

    virtual void onInit() {
        ros::NodeHandle& nh = getPrivateNodeHandle();

        bUseViewer = true;
        bEnablePublishROSTopic = true;
        getParamOrFail(nh, "camera_setting_path", &settings_file);
        getParamOrFail(nh, "vocabulary_path", &voc_file);

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        SLAM = std::unique_ptr<ORB_SLAM2::System>(new ORB_SLAM2::System(voc_file, settings_file,ORB_SLAM2::System::MONOCULAR, bUseViewer));
        SLAMDATA = std::unique_ptr<ORB_SLAM2::SlamData>(new ORB_SLAM2::SlamData(SLAM.get(), &nh, bEnablePublishROSTopic));
        igb = std::unique_ptr<ORB_SLAM2::ImageGrabber>(new ORB_SLAM2::ImageGrabber(SLAM.get(), SLAMDATA.get(), &nh));
    }

    virtual ~ORBSLAMNodelet() {
        // Stop all threads
        SLAM->Shutdown();
        SLAM->SaveKeyFrameTrajectoryTUM(KEYFRAME_TRAJECTORY_TUM_SAVE_FILE_DIR);
        return;
    }
    bool bUseViewer, bEnablePublishROSTopic;
    std::string voc_file, settings_file;
    std::unique_ptr<ORB_SLAM2::System> SLAM;
    std::unique_ptr<ORB_SLAM2::SlamData> SLAMDATA;
    std::unique_ptr<ORB_SLAM2::ImageGrabber> igb;
};

PLUGINLIB_EXPORT_CLASS(ORBSLAMNodelet, nodelet::Nodelet)




