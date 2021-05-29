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
#include<ros/ros.h>
#include <System.h>

#include "orb_slam_wrapper.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");

    ros::start();

    bool bUseViewer, bEnablePublishROSTopic;
    std::string voc_file, settings_file;
    bUseViewer = true;
    bEnablePublishROSTopic = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ros::NodeHandle nh("~");
    getParamOrFail(nh, "camera_setting_path", &settings_file);
    getParamOrFail(nh, "vocabulary_path", &voc_file);
    ORB_SLAM2::System SLAM(voc_file, settings_file,ORB_SLAM2::System::MONOCULAR, bUseViewer);
    ORB_SLAM2::SlamData SLAMDATA(&SLAM, &nh, bEnablePublishROSTopic);
    ORB_SLAM2::ImageGrabber igb(&SLAM, &SLAMDATA, &nh);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM(KEYFRAME_TRAJECTORY_TUM_SAVE_FILE_DIR);

    ros::shutdown();

    return 0;
}



