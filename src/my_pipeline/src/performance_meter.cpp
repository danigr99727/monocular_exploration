#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <map>
#include <message_filters/subscriber.h>

struct Measurements{
    Measurements()=default;
    double timestamps[7];
    int count;
};

class PerformanceMeter{
public:
    explicit PerformanceMeter(ros::NodeHandle& nh)
    {
        stamp_subs_[0] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/airsim_node/sent", 500));
        stamp_subs_[1] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/orb_slam2_cuda/received", 500));
        stamp_subs_[2] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh,"/orb_slam2_cuda/sent", 500));
        stamp_subs_[3] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/flame/received", 500));
        stamp_subs_[4] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/flame/sent", 500));
        stamp_subs_[5] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/sdf_map/received", 500));
        stamp_subs_[6] = std::unique_ptr<message_filters::Subscriber<std_msgs::Header>>(new message_filters::Subscriber<std_msgs::Header>(nh, "/sdf_map/sent", 500));

        ROS_INFO("seq_number, airsim_sent, slam_received, slam_sent, flame_received, flame_sent, sdf_map_received, sdf_map_sent");

        stamp_subs_[0]->registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            //std::cout<<"airsim_received"<<headerPtr->stamp<<std::endl;
            measurements_map_.insert({headerPtr->stamp.toSec(), Measurements()});
            stamp_callback(headerPtr->stamp.toSec(), 0);
        });

        for(int i=1; i<7; i++){
            stamp_subs_[i]->registerCallback([i, this](const std_msgs::HeaderConstPtr& headerPtr){
                //std::cout<<i<<"received"<<headerPtr->stamp<<std::endl;
                stamp_callback(headerPtr->stamp.toSec(), i);
            });
        }

        latency_pub_[0]=nh.advertise<std_msgs::Float64>("airsim_slam_latency", 10);
        latency_pub_[1]=nh.advertise<std_msgs::Float64>("slam_latency", 10);
        latency_pub_[2]=nh.advertise<std_msgs::Float64>("slam_flame_latency", 10);
        latency_pub_[3]=nh.advertise<std_msgs::Float64>("flame_latency", 10);
        latency_pub_[4]=nh.advertise<std_msgs::Float64>("flame_sdf_map_latency", 10);
        latency_pub_[5]=nh.advertise<std_msgs::Float64>("sdf_map_latency", 10);

        fps_pub_[0]=nh.advertise<std_msgs::UInt8>("airsim_fps", 10);
        fps_pub_[1]=nh.advertise<std_msgs::UInt8>("slam_fps", 10);
        fps_pub_[2]=nh.advertise<std_msgs::UInt8>("flame_fps", 10);
        fps_pub_[3]=nh.advertise<std_msgs::UInt8>("sdf_map_fps", 10);
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
        /*if(index>0){
            std_msgs::Float64 msg;
            msg.data = (measurement.timestamps[index]-measurement.timestamps[index-1])*1000.0f;
            latency_pub_[index-1].publish(msg);
        }*/
        measurement.count++;

        if (measurement.count==7){
            ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f",
                     stamp,
                     measurement.timestamps[0],
                     measurement.timestamps[1],
                     measurement.timestamps[2],
                     measurement.timestamps[3],
                     measurement.timestamps[4],
                     measurement.timestamps[5],
                     measurement.timestamps[6]);
            std_msgs::Float64 msgs[6];
            for(int i=0; i<6; i++){
                    msgs[i].data = (measurement.timestamps[i+1]-measurement.timestamps[i])*1000.0f;
                    latency_pub_[i].publish(msgs[i]);
            }
            measurements_map_.erase(measurements_map_.begin(), measurements_it++);
        }
    }
    //ros::NodeHandle& nh_;
    std::unique_ptr<message_filters::Subscriber<std_msgs::Header>> stamp_subs_[7];

    std::map<double, Measurements> measurements_map_;

    ros::Publisher latency_pub_[6];
    ros::Publisher fps_pub_[4];

    std::vector<double> fps_utils_[4];

};

int main(int argc, char** argv){
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh("~");

    PerformanceMeter performanceMeter(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
