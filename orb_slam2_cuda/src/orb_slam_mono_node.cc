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
#include<fstream>
#include<chrono>

#include<tf/transform_broadcaster.h>

#include<ros/ros.h>
#include <Converter.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <System.h>

#include "orb_slam_wrapper.h"

//using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");

    ros::start();

    bool bUseViewer, bEnablePublishROSTopic;

    bUseViewer = false;
    bEnablePublishROSTopic = true;
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, bUseViewer);

    ros::NodeHandle nodeHandler;

    ORB_SLAM2::SlamData SLAMDATA(&SLAM, &nodeHandler, bEnablePublishROSTopic);

    ORB_SLAM2::ImageGrabber igb(&SLAM, &SLAMDATA, &nodeHandler);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM(KEYFRAME_TRAJECTORY_TUM_SAVE_FILE_DIR);

    ros::shutdown();

    return 0;
}



