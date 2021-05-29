#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <map>
#include <message_filters/subscriber.h>

struct Measurements{
    Measurements()=default;
    Measurements(double first_measurement){airsim_sent_secs=first_measurement; count=1;};

    double airsim_sent_secs,
            slam_received_secs,
            slam_sent_secs,
            flame_received_secs,
            flame_sent_secs,
            sdf_map_received_secs,
            sdf_map_sent_secs;

    int count;
};

class PerformanceMeter{
public:
    PerformanceMeter(ros::NodeHandle& nh)
            : airsim_sent_sub_(nh, "/airsim_node/sent", 500),
              slam_received_sub_(nh, "/orb_slam2_cuda/received", 500),
              slam_sent_sub_(nh, "/orb_slam2_cuda/sent", 500),
              flame_received_sub_(nh, "/flame/received", 500),
              flame_sent_sub_(nh, "/flame/sent", 500),
              sdf_map_received_sub_(nh, "/sdf_map/received", 500),
              sdf_map_sent_sub_(nh, "/sdf_map/sent", 500)
    {
        ROS_INFO("seq_number, airsim_sent, slam_received, slam_sent, flame_received, flame_sent, sdf_map_received, sdf_map_sent");

        airsim_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"airsim_received"<<headerPtr->stamp<<std::endl;
            measurements_map_.insert({headerPtr->stamp.toSec(), Measurements(ros::Time::now().toSec())});
        });
        slam_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"slam_received"<<headerPtr->stamp<<std::endl;
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            measurement.slam_received_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
        slam_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"slam_sent"<<headerPtr->stamp<<std::endl;
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            measurement.slam_sent_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
        flame_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"flame_received"<<headerPtr->stamp<<std::endl;
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            measurement.flame_received_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
        flame_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"flame_sent"<<headerPtr->stamp<<std::endl;
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            measurement.flame_sent_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
        sdf_map_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            std::cout<<"sdf_received"<<headerPtr->stamp<<std::endl;
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            measurement.sdf_map_received_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
        sdf_map_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
            Measurements& measurement = measurements_map_[headerPtr->stamp.toSec()];
            std::cout<<"sdf_sent"<<headerPtr->stamp<<"COUNT: "<<measurement.count<<std::endl;
            measurement.sdf_map_sent_secs = ros::Time::now().toSec();
            check_full(headerPtr->stamp.toSec(), measurement);
        });
    }
    ~PerformanceMeter() = default;
private:
    void check_full(double stamp, Measurements& measurement){
        measurement.count++;
        if (measurement.count==7){
            ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f",
                     stamp,
                     measurement.airsim_sent_secs,
                     measurement.slam_received_secs,
                     measurement.slam_sent_secs,
                     measurement.flame_received_secs,
                     measurement.flame_sent_secs,
                     measurement.sdf_map_received_secs,
                     measurement.sdf_map_sent_secs);
            measurements_map_.erase(stamp);
        }
    }
    //ros::NodeHandle& nh_;
    message_filters::Subscriber<std_msgs::Header> airsim_sent_sub_,
            slam_received_sub_,
            slam_sent_sub_,
            flame_received_sub_,
            flame_sent_sub_,
            sdf_map_received_sub_,
            sdf_map_sent_sub_;

    std::map<double, Measurements> measurements_map_;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh("~");

    PerformanceMeter performanceMeter(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}

/*
 *
struct Measurements{
    Measurements()=default;
    Measurements(double first_measurement){airsim_sent_secs=first_measurement; count=1;};

    double airsim_sent_secs,
    slam_received_secs,
    slam_sent_secs,
    flame_received_secs,
    flame_sent_secs,
    sdf_map_received_secs,
    sdf_map_sent_secs;

    int count;
};

class PerformanceMeter{
public:
    PerformanceMeter(ros::NodeHandle& nh)
        : airsim_sent_sub_(nh, "/airsim_node/sent", 500),
          slam_received_sub_(nh, "/orb_slam2_cuda/received", 500),
          slam_sent_sub_(nh, "/orb_slam2_cuda/sent", 500),
          flame_received_sub_(nh, "/flame/received", 500),
          flame_sent_sub_(nh, "/flame/sent", 500),
          sdf_map_received_sub_(nh, "/sdf_map/received", 500),
          sdf_map_sent_sub_(nh, "/sdf_map/sent", 500)
          {
              ROS_INFO("seq_number, airsim_sent, slam_received, slam_sent, flame_received, flame_sent, sdf_map_received, sdf_map_sent");

              airsim_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  measurements_map_.insert({headerPtr->seq, Measurements(headerPtr->stamp.toSec())});
              });
              slam_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.slam_received_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
              slam_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.slam_sent_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
              flame_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.flame_received_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
              flame_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.flame_sent_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
              sdf_map_received_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.sdf_map_received_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
              sdf_map_sent_sub_.registerCallback([this](const std_msgs::HeaderConstPtr& headerPtr){
                  Measurements& measurement = measurements_map_[headerPtr->seq];
                  measurement.sdf_map_sent_secs = headerPtr->stamp.toSec();
                  check_full(headerPtr->seq, measurement);
              });
          }
    ~PerformanceMeter() = default;
private:
    void check_full(const uint32_t seq, Measurements& measurement){
        if (measurement.count++==7){
            ROS_INFO("%i, %f, %f, %f, %f, %f, %f, %f",
                     seq,
                     measurement.airsim_sent_secs,
                     measurement.slam_received_secs,
                     measurement.slam_sent_secs,
                     measurement.flame_received_secs,
                     measurement.flame_sent_secs,
                     measurement.sdf_map_received_secs,
                     measurement.sdf_map_sent_secs);
            measurements_map_.erase(seq);
        }
    }
    //ros::NodeHandle& nh_;
    message_filters::Subscriber<std_msgs::Header> airsim_sent_sub_,
    slam_received_sub_,
    slam_sent_sub_,
    flame_received_sub_,
    flame_sent_sub_,
    sdf_map_received_sub_,
    sdf_map_sent_sub_;

    std::map<uint32_t, Measurements> measurements_map_;

};
 * */