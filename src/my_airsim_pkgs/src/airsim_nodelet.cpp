#include "ros/ros.h"
#include <airsim_ros_wrapper.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class AirsimNodelet : public nodelet::Nodelet {
public:
    virtual void onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ros::NodeHandle& nh = getNodeHandle();

        std::string host_ip = "localhost";
        private_nh.getParam("host_ip", host_ip);
        airsim_ros_wrapper = std::unique_ptr<AirsimROSWrapper>(new AirsimROSWrapper(nh, private_nh, host_ip));

        if (airsim_ros_wrapper->is_used_img_timer_cb_queue_)
        {
            airsim_ros_wrapper->img_async_spinner_.start();
        }

        if (airsim_ros_wrapper->is_used_lidar_timer_cb_queue_)
        {
            airsim_ros_wrapper->lidar_async_spinner_.start();
        }

    }

    std::unique_ptr<AirsimROSWrapper> airsim_ros_wrapper;
};

PLUGINLIB_EXPORT_CLASS(AirsimNodelet, nodelet::Nodelet)