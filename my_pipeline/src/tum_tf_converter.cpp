#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>

class TfConverter {
public:
    TfConverter()
    : node_(),
      listener_(node_),
      tf_pub_(node_.advertise<geometry_msgs::TransformStamped>("/transform_rgb_frame", 100)),
      cam_info_sub_(node_.subscribe("/camera/rgb/camera_info", 5000, &TfConverter::callback, this))
    {}

private:
    void callback(const sensor_msgs::CameraInfo::ConstPtr& info){
        //listener_.waitForTransform("/openni_rgb_frame", "/world", info->header.stamp, ros::Duration(5.0));
        try {
            listener_.lookupTransform("/world", "/openni_rgb_frame", ros::Time(0), transform_);
            geometry_msgs::TransformStamped transform_msg;
            tf::transformStampedTFToMsg(transform_, transform_msg);
            tf_pub_.publish(transform_msg);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
    }
    ros::NodeHandle node_;
    tf::TransformListener listener_;
    ros::Publisher tf_pub_;
    tf::StampedTransform transform_;
    ros::Subscriber cam_info_sub_;

};


int main(int argc, char** argv){
    ros::init(argc, argv, "tum_tf_converter");

    TfConverter tfConverter;

    ros::spin();
    return 0;
};