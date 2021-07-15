/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef SKELETON_KINMEMATICS_LSQ_H
#define SKELETON_KINMEMATICS_LSQ_H

#include "motion_filter/skeleton_kinematics_base.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;

namespace motion_filter
{

class LSQKinect : public SkeletonKinectmatics
{
public:
LSQKinect(DataHandler &dh);
~LSQKinect();
private:
//Common Method
void initParams() override;
void parseToml(std::string &toml_path) override;
void solveIK() override;

CQuadraticProgram qp_;
double alpha_;

};
}
#endif //SKELETON_KINMEMATICS_LSQ_H

