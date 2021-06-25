#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <vector>
#include "VR/matrix_3_4.h"
#include <motion_filter/type_def.hpp>
#include <std_msgs/Bool.h>

namespace motion_filter
{

class DataHandler
{
public:
    DataHandler(ros::NodeHandle &nh)
    {
        for (int i=0;i<NUM_TRACKER;i++){
            trackers_sub_[i] = nh.subscribe("/TRACKER" + std::to_string(i), 100, &DataHandler::trackersCallback, this);
            raw_poses_[i].linear().setIdentity();
            raw_poses_[i].translation().setZero();            
        }
        hmd_sub_ = nh.subscribe("/HMD", 100, &DataHandler::hmdCallback, this);

        tracker_status_sub_ = nh.subscribe("/TRACKERSTATUS", 100, &DataHandler::trackersStatusCallback, this);
        raw_poses_[NUM_TRACKER].linear().setIdentity();
        raw_poses_[NUM_TRACKER].translation().setZero();
    }
    void trackersCallback(const ros::MessageEvent<VR::matrix_3_4>& event);
    void hmdCallback(const VR::matrix_3_4 &msg);
    void trackersStatusCallback(const std_msgs::Bool & msg);
    Eigen::Isometry3d* getObs() {return raw_poses_;};
    bool getTrackerStatus() {return tracker_status_;};
private:
    ros::Subscriber trackers_sub_[NUM_TRACKER];
    ros::Subscriber tracker_status_sub_;
    ros::Subscriber hmd_sub_;

    Eigen::Isometry3d raw_poses_[NUM_TRACKER + 1]; //pelvis, chect, lelbow, lhand, relbow, rhand, head
    bool tracker_status_ = false;

};
}

#endif // DATA_HANDLER_H
