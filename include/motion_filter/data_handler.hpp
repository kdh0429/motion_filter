#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <vector>
#include "VR/matrix_3_4.h"
#include <motion_filter/type_def.hpp>
#include <std_msgs/Bool.h>

namespace motion_filter
{
VR::matrix_3_4 isometry3d2VRmsg(Eigen::Isometry3d T);

class DataHandler
{
public:
DataHandler(ros::NodeHandle &nh): nh_(nh)
{
    for (int i=0;i<NUM_TRACKER;i++){
        trackers_sub_[i] = nh_.subscribe("/TRACKER" + std::to_string(i), 100, &DataHandler::trackersCallback, this);
        raw_poses_[i].linear().setIdentity();
        raw_poses_[i].translation().setZero();            
    }
    hmd_sub_ = nh_.subscribe("/HMD", 100, &DataHandler::hmdCallback, this);

    tracker_status_sub_ = nh_.subscribe("/TRACKERSTATUS", 100, &DataHandler::trackersStatusCallback, this);
    raw_poses_[NUM_TRACKER].linear().setIdentity();
    raw_poses_[NUM_TRACKER].translation().setZero();
}
void trackersCallback(const ros::MessageEvent<VR::matrix_3_4>& event);
void hmdCallback(const VR::matrix_3_4 &msg);
void trackersStatusCallback(const std_msgs::Bool & msg);
Eigen::Isometry3d* getRaw() {return raw_poses_;};
Eigen::Isometry3d* getFiltered() {return filtered_poses_;};
void setFiltered(int tracker_id, Eigen::Isometry3d T){ filtered_poses_[tracker_id] = T;};
bool getTrackerStatus() {return tracker_status_;};

template<typename M>
int addPublisher(std::string name, uint32_t queue_size);

template<typename M>
void publish(M message, int index);

private:
ros::NodeHandle &nh_;
ros::Subscriber trackers_sub_[NUM_TRACKER];
ros::Subscriber tracker_status_sub_;
ros::Subscriber hmd_sub_;

std::vector<ros::Publisher> pubs_;

Eigen::Isometry3d raw_poses_[NUM_TRACKER + 1]; //pelvis, chect, lelbow, lhand, relbow, rhand, head
Eigen::Isometry3d filtered_poses_[NUM_TRACKER + 1]; //pelvis, chect, lelbow, lhand, relbow, rhand, head

bool tracker_status_ = false;

};

template<typename M>
int DataHandler::addPublisher(std::string name, uint32_t queue_size)
{
    int index = pubs_.size();
    pubs_.push_back(nh_.advertise<M>(name, queue_size));
    return index;
}
template<typename M>
void DataHandler::publish(M message, int index)
{
    pubs_.at(index).publish(message);
}

}

#endif // DATA_HANDLER_H
