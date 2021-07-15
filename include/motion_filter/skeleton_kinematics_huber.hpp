/*
Skeleton Kinematic

since  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef SKELETON_KINMEMATICS_HUBER_H
#define SKELETON_KINMEMATICS_HUBER_H

#include "motion_filter/skeleton_kinematics_base.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;

namespace motion_filter
{

class HuberKinect : public SkeletonKinectmatics
{
public:
HuberKinect(DataHandler &dh);
~HuberKinect();
private:
//Common Method
void initParams() override;
void parseToml(std::string &toml_path) override;
void solveIK() override;

CQuadraticProgram qp_;

double alpha_; //low pass
double pnoise_;
double gamma_; //from toml
int filter_type_;//from toml
KalmanFilter* kalman_filter_;
double joint_noise_;

MatrixXd fQ_; //process cov
MatrixXd fR_; //measurement cov

};
}
#endif //SKELETON_KINMEMATICS_HUBER_H