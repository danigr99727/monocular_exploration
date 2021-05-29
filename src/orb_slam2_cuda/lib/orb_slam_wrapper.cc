#include "orb_slam_wrapper.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>

namespace ORB_SLAM2
{

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::Header header_msg;
    header_msg.stamp = msg->header.stamp;
    img_received_pub_.publish(header_msg);
    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_BEGIN);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_CV_PROCESS);

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_SLAM_PROCESS);

    //mpSLAMDATA->CalculateAndPrintOutProcessingFrequency();

    if (Tcw.empty()) {
        return;
    }

    if (mpSLAMDATA->EnablePublishROSTopics())
    {

        mpSLAMDATA->CalculateNewTransform(Tcw, ros::Time(msg->header.stamp),  msg->header.seq);

        //mpSLAMDATA->PublishTFForROS();

        mpSLAMDATA->PublishTFMessage();

        //mpSLAMDATA->PublishPoseForROS();

        //mpSLAMDATA->PublishPointCloudForROS();

        mpSLAMDATA->PublishOdometry();

        //mpSLAMDATA->PublishCurrentFrameForROS();
    }
}

SlamData::SlamData(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nodeHandler, bool bPublishROSTopic)
{
    mpSLAM = pSLAM;
    mpFrameDrawer = mpSLAM->GetpFrameDrawer();
    bEnablePublishROSTopic = bPublishROSTopic;
    // Perform tf transform and publish
    last_transform.setOrigin(tf::Vector3(0,0,0));
    last_transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    last_transform.setRotation(q);

    tf_pub = (*nodeHandler).advertise<geometry_msgs::TransformStamped>("/transform", 12);
    pose_pub = (*nodeHandler).advertise<geometry_msgs::PoseStamped>("/posestamped", 12);
    odom_pub = (*nodeHandler).advertise<nav_msgs::Odometry>("odometry", 12);

    all_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("point_cloud_all",1);
    ref_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("point_cloud_ref",1);

    sent_pub_ = (*nodeHandler).advertise<std_msgs::Header>("/orb_slam2_cuda/sent", 500);

    mInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;  // camera coordinate represented in ground coordinate system
    mInitCam2Ground_t.setZero();     
    mTrans_cam2ground.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;  //< block_rows, block_cols >(pos_row, pos_col)

    image_transport::ImageTransport it_((*nodeHandler));
    current_frame_pub = it_.advertise("/current_frame", 1);
}

void SlamData::SaveTimePoint(TimePointIndex index)
{
    switch (index)
    {
	case TIME_BEGIN:
    	tp1 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_CV_PROCESS:
    	tp2 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_SLAM_PROCESS:
    	tp3 = std::chrono::steady_clock::now();
        break;
    default: 
        break;
    }
}

void SlamData::CalculateAndPrintOutProcessingFrequency(void)
{
    static long spinCnt = 0;
    static double t_temp = 0;

    double time_read= std::chrono::duration_cast<std::chrono::duration<double> >(tp2 - tp1).count();
    double time_track= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp2).count();
    double time_total= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp1).count();
    
    cout << "Image reading time = " << setw(10) << time_read  << "s" << endl;
    cout << "Tracking time =      " << setw(10) << time_track << "s, frequency = " << 1/time_track << "Hz" << endl; 
    cout << "All cost time =      " << setw(10) << time_total << "s, frequency = " << 1/time_total << "Hz" << endl; 
    t_temp = (time_total + t_temp*spinCnt)/(1+spinCnt);
    cout << "Avg. time =          " << setw(10) << t_temp     << "s, frequency = " << 1/t_temp     << "Hz" << endl;
    cout << "\n\n" << endl;

    spinCnt++;
}

void SlamData::CalculateNewTransform(const cv::Mat& Tcw, const ros::Time& time, uint32_t seq)
{
    last_transform = new_transform;
    last_time = new_time;
    new_time = time;
    seq_ = seq;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);


    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));

    tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);

    new_transform.setRotation(tf_quaternion);
}

/*void SlamData::PublishTFForROS()
{
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(new_transform, new_time, "camera_world", "s20cam_wide"));
}*/

void SlamData::PublishTFMessage()
{
    geometry_msgs::TransformStamped tfStamped;
    tfStamped.header.stamp = new_time;
    //std::cout<<"pose sent: " << cv_ptr->header.stamp.toSec()<<std::endl;
    //tfStamped.header.seq = cv_ptr->header.seq;
    tfStamped.header.frame_id =  "camera_world";
    tfStamped.child_frame_id = "camera";
    tfStamped.transform.rotation.w = new_transform.getRotation().w();
    tfStamped.transform.rotation.x = new_transform.getRotation().x();
    tfStamped.transform.rotation.y = new_transform.getRotation().y();
    tfStamped.transform.rotation.z = new_transform.getRotation().z();
    tfStamped.transform.translation.x = new_transform.getOrigin().x();
    tfStamped.transform.translation.y = new_transform.getOrigin().y();
    tfStamped.transform.translation.z = new_transform.getOrigin().z();
    std::cout<<"sending transform..."<<std::endl;
    tf_pub.publish(tfStamped);

    std_msgs::Header header_msg;
    header_msg.stamp = new_time; //ros::Time::now();
    sent_pub_.publish(header_msg);
}

void SlamData::PublishPoseForROS()
{
    static int frame_num = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = new_time;
    pose.header.frame_id ="camera_world";
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);
}

void SlamData::PublishOdometry()
{
    nav_msgs::Odometry odom;

    tf::poseTFToMsg(new_transform, odom.pose.pose);
    odom.header.frame_id="camera_world";
    odom.header.stamp = new_time;
    odom.child_frame_id = "camera";
    auto timeDiff = (new_time - last_time).toSec();
    odom.twist.twist.linear.x = (new_transform.getOrigin()[0] - last_transform.getOrigin()[0])/timeDiff;
    odom.twist.twist.linear.y = (new_transform.getOrigin()[1] - last_transform.getOrigin()[1])/timeDiff;
    odom.twist.twist.linear.z = (new_transform.getOrigin()[2] - last_transform.getOrigin()[2])/timeDiff;

    odom_pub.publish(odom);
}

void SlamData::PublishPointCloudForROS(void)
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    GetCurrentROSPointCloud(allMapPoints, referenceMapPoints);
    all_point_cloud_pub.publish(allMapPoints);
    ref_point_cloud_pub.publish(referenceMapPoints);
}

void SlamData::GetCurrentROSPointCloud(sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGBA> );  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGBA> );     
    
    const std::vector<MapPoint*> &vpMPs = mpSLAM->GetmpMapAllMapPoints();
    const std::vector<MapPoint*> &vpRefMPs = mpSLAM->GetmpMapReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
    {
        return;
    }
	
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGBA p1;
        Eigen::Vector4f p1_temp, p1_temp_t;
        p1_temp(0) = pos.at<float>(0);
        p1_temp(1) = pos.at<float>(1);
        p1_temp(2) = pos.at<float>(2);
        p1_temp(3) = 1; 
        p1_temp_t = mTrans_cam2ground * p1_temp;	
        p1.x = p1_temp_t(0);
        p1.y = p1_temp_t(1);
        p1.z = p1_temp_t(2);
        p1.b = 255;
        p1.g = 255;
        p1.r = 255;
        p1.a = 255;
        cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "camera_world";
    all_point_cloud.header.stamp = ros::Time::now();   
  
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGBA p2;
        Eigen::Vector4f p2_temp, p2_temp_t;
        p2_temp(0) = pos.at<float>(0);
        p2_temp(1) = pos.at<float>(1);
        p2_temp(2) = pos.at<float>(2);
        p2_temp(3) = 1;
        p2_temp_t = mTrans_cam2ground * p2_temp;	
        p2.x = p2_temp_t(0);
        p2.y = p2_temp_t(1);
        p2.z = p2_temp_t(2);
        p2.b = 0;
        p2.g = 0;
        p2.r = 255;
        p2.a = 255;
        cloud_ref->points.push_back( p2 );
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "camera_world";
    ref_point_cloud.header.stamp = ros::Time::now(); 
}

void SlamData::PublishCurrentFrameForROS(void)
{
    cv_bridge::CvImage cvi;
    cv::Mat img;
    cvi.header.frame_id = "s20cam_wide";
    cvi.encoding = "bgr8";
    cvi.header.stamp = ros::Time::now();

    if (mpFrameDrawer)
    {
        img = mpFrameDrawer->DrawFrame();
        // cv::imshow("Current Frame",img);
        // cv::waitKey(1e3/FPS/2);
        cvi.image = img;
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        current_frame_pub.publish(im);
    }
}

bool SlamData::EnablePublishROSTopics(void)
{
    return bEnablePublishROSTopic;
}

} //namespace ORB_SLAM