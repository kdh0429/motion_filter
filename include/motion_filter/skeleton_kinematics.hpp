/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef SKELETON_KINMEMATICS_H
#define SKELETON_KINMEMATICS_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>

#include "bvh11.hpp"

#include "motion_filter/data_handler.hpp"
#include "motion_filter/type_def.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;

namespace motion_filter
{

struct CustomJoint
{
    int parent_id;
    std::vector<bvh11::Channel::Type> types;
    std::string joint_name;
    Vector3d offset;
};

class SkeletonKinectmatics
{
public:
SkeletonKinectmatics(DataHandler* dh, ros::NodeHandle &nh);
~SkeletonKinectmatics();
void printModelInfo();
void CalibOffsets();

private:
DataHandler* dh_;
void publish();
bvh11::BvhObject bvh_;
Model* model_;

}
}
#endif //SKELETON_KINMEMATICS_H

