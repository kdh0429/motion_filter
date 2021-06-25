#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <vector>
#include "VR/matrix_3_4.h"
#include <motion_filter/type_def.hpp>

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
        raw_poses_[NUM_TRACKER].linear().setIdentity();
        raw_poses_[NUM_TRACKER].translation().setZero();  
    }
    void trackersCallback(const ros::MessageEvent<VR::matrix_3_4>& event);
    void hmdCallback(const VR::matrix_3_4 &msg);
    Eigen::Isometry3d* getObs() {return raw_poses_;};
private:
    ros::Subscriber trackers_sub_[NUM_TRACKER];
    ros::Subscriber hmd_sub_;

    Eigen::Isometry3d raw_poses_[NUM_TRACKER + 1]; //pelvis, chect, lelbow, lhand, relbow, rhand, head

};
}

#endif // DATA_HANDLER_H
