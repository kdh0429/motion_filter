/*
Lie Group(SE3) (innovation-saturated) Extended Kalman filter LG (IS) EKF
constant velocity(differential kinematic)

Data:  2021.06.21 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef PRE_PROCESS_H
#define PRE_PROCESS_H

#include "manif/SE3.h"
#include <toml.hpp>
#include <ros/package.h>
#include <motion_filter/type_def.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace motion_filter
{

VR::matrix_3_4 isometry3d2VRmsg(Eigen::Isometry3d T);

class PreProcess
{
public:
PreProcess(ros::NodeHandle &nh, int tracker_id, double dt, bool verbose);
~PreProcess();
Eigen::Isometry3d getTransform();
Vector6d getSpatialVel() {return V_;}; //[v;w];
Vector7d getPosQuat() {return T_.coeffs();}; //[v;w];
void step(Eigen::Isometry3d T_m);
void restart();

private:
void parseToml(std::string &toml_path);
void predict();
void ekfUpdate(Eigen::Isometry3d T_m);
void isekfUpdate(Eigen::Isometry3d T_m);
void dynamicClipUpdate(Vector6d z);
void publish();

ros::Publisher pose_pub_;
ros::Publisher fpos_quat_pub_; //filter
ros::Publisher rpos_quat_pub_; //raw

ros::Publisher vel_pub_;


int key_;
int tracker_id_;
const char* algo[2] = {"LGEKF", "LGISEKF"};

double dt_;

bool verbose_;
bool is_first_ = true;
//state variable
manif::SE3d T_;
manif::SE3d T_raw_;

Vector6d V_;

//model jacobian
manif::SE3d::Jacobian J_T_, J_V_;

Matrix12d P_; //state covariance: concentrated gaussian tangent space gaussian
Matrix6d process_cov_;
Matrix6d N_; //measurement cov_;

//measurement jacobian
Matrix6x12d H_;

//dynamic clipping
Vector6d sigma_;
Vector6d epsilon_;

//isekf hyper-params(design params)
Vector6d sigma_init_;
Vector6d epsilon_init_;
Vector6d lambda1_;
Vector6d lambda2_;
Vector6d gamma1_;
Vector6d gamma2_;

//Logger
CsvLogger* rlogger_;
CsvLogger* flogger_;
std::vector<std::string> keys_ = {"x", "y", "z", "qw", "qx", "qy", "qz"};
std::vector<std::string> id2name_ = {"base", "chest", "larm", "lhand", "rarm", "rhand", "head"}
};
}
#endif // PRE_PROCESS_H