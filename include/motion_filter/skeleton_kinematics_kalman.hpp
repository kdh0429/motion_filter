/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef SKELETON_KINMEMATICS_KALMAN_H
#define SKELETON_KINMEMATICS_KALMAN_H

#include "motion_filter/skeleton_kinematics_base.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;

namespace motion_filter
{
  
class KalmanKinect : public SkeletonKinectmatics
{
public:
KalmanKinect(DataHandler &dh);
~KalmanKinect();
private:
//Common Method
void initParams() override;
void parseToml(std::string &toml_path) override;
void solveIK() override;

CQuadraticProgram qp_;

Gaussian state_;
MatrixXd fQ_; //process cov
MatrixXd fR_; //measurement cov

bool filter_first_= true;
double lambda_ = 1.0;
double rho_ = 0.95;
MatrixXd V_ok_;

double pnoise_;

VectorXd single_measurement_noise_;
VectorXd init_state_std_;


};

} //motion filter
#endif //SKELETON_KINMEMATICS_BASE_H

