#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <map>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>

struct Measurements{
    Measurements()=default;
    double timestamps[9];
    int count;
};

class PerformanceMeter{
public:
    explicit PerformanceMeter(ros::NodeHandle& nh)
    {
        gscam_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "/s20cam_wide/camera_info", 500));
        stamp_subs_[0] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/orb_slam2_cuda/received", 500));
        stamp_subs_[1] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh,"/orb_slam2_cuda/sent", 500));
        stamp_subs_[2] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/flame/received", 500));
        stamp_subs_[3] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/flame/sent", 500));
        stamp_subs_[4] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/sdf_map/received", 500));
        stamp_subs_[5] =  std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/sdf_map/sent", 500));
        stamp_subs_[6] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/tdnet/received", 500));
        stamp_subs_[7] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/tdnet/sent", 500));

        //ROS_INFO("seq_number, airsim_sent, slam_received, slam_sent, flame_received, flame_sent, sdf_map_received, sdf_map_sent, tdnet_received, tdnet_sent");

        gscam_sub_->registerCallback([this](const sensor_msgs::CameraInfoConstPtr& camInfoPtr){
            //std::cout<<"airsim_received"<<camInfoPtr->header.stamp.toSec();
            measurements_map_.insert({camInfoPtr->header.stamp.toSec(), Measurements()});
            stamp_callback(camInfoPtr->header.stamp.toSec(), 0);
        });

        for(int i=0; i<8; i++){
            stamp_subs_[i]->registerCallback([i, this](const std_msgs::HeaderConstPtr& headerPtr){
                //std::cout<<i+1<<" received: "<<headerPtr->stamp<<std::endl;
                stamp_callback(headerPtr->stamp.toSec(), i+1);
            });
        }

        latency_pub_[0]=nh.advertise<std_msgs::Float64>("/slam_latency", 10);
        latency_pub_[1]=nh.advertise<std_msgs::Float64>("/flame_latency", 10);
        latency_pub_[2]=nh.advertise<std_msgs::Float64>("/sdf_map_latency", 10);
        latency_pub_[3]=nh.advertise<std_msgs::Float64>("/tdnet_latency", 10);
        latency_pub_[4]=nh.advertise<std_msgs::Float64>("/total_latency", 10);

        fps_pub_[0]=nh.advertise<std_msgs::UInt8>("/gscam_fps", 10);
        fps_pub_[1]=nh.advertise<std_msgs::UInt8>("/slam_fps", 10);
        fps_pub_[2]=nh.advertise<std_msgs::UInt8>("/flame_fps", 10);
        fps_pub_[3]=nh.advertise<std_msgs::UInt8>("/sdf_map_fps", 10);
        fps_pub_[4]=nh.advertise<std_msgs::Float64>("/tdnet_fps", 10);
    }

    ~PerformanceMeter() = default;

private:
    void stamp_callback(double stamp, int index){
        auto measurements_it  = measurements_map_.find(stamp);
        Measurements& measurement = measurements_it->second;
        double now_secs = ros::Time::now().toSec();
        measurement.timestamps[index] = now_secs;
        if((index&1)==0){
            fps_utils_[index>>1].push_back(now_secs);
            auto it = fps_utils_[index>>1].begin();
            while(now_secs - *it > 1.0f){it++;}
            fps_utils_[index>>1] = std::vector<double>(it, fps_utils_[index>>1].end());
            std_msgs::UInt8 msg;
            msg.data=static_cast<uint8_t>(fps_utils_[index>>1].size());
            fps_pub_[index>>1].publish(msg);
        }

        measurement.count++;

        if (measurement.timestamps[6] != 0 && measurement.count==9){
            std_msgs::Float64 msgs[5];
            for(int i=0; i<4; i++){
                if(measurement.timestamps[i*2+2]!=0 && measurement.timestamps[i*2+1]!=0) {
                    msgs[i].data =
                            (measurement.timestamps[i * 2 + 2] - measurement.timestamps[i * 2 + 1]) * 1000.0f;
                    latency_pub_[i].publish(msgs[i]);
                }
            }
            msgs[4].data = (measurement.timestamps[6]-measurement.timestamps[0])*1000.0f;
            latency_pub_[4].publish(msgs[4]);
            measurements_map_.erase(measurements_map_.begin(), measurements_it++);
        }
    }
    //ros::NodeHandle& nh_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> gscam_sub_;
    std::unique_ptr<message_filters::Subscriber<std_msgs::Header>> stamp_subs_[8];
    std::map<double, Measurements> measurements_map_;
    ros::Publisher latency_pub_[5];
    ros::Publisher fps_pub_[5];
    std::vector<double> fps_utils_[5];
};

int main(int argc, char** argv){
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh("~");
    PerformanceMeter performanceMeter(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}
