/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef SKELETON_KINMEMATICS_BASE_H
#define SKELETON_KINMEMATICS_BASE_H
#include <manif/SE3.h>
#include <manif/SO3.h>

#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <toml.hpp>

#include <Eigen/Dense>

#include <std_msgs/Float64MultiArray.h>

#include "bvh11.hpp"

#include "motion_filter/data_handler.hpp"
#include "motion_filter/type_def.hpp"
#include "motion_filter/kalman_filter.hpp"

#include "quadraticprogram.h"

#include "logger.h"

using namespace RigidBodyDynamics;
using namespace Eigen;

// Order of model id we want
typedef enum ModelMap
{
    THORAX = 0,
    LSH,
    LHAND,
    RSH,
    RHAND,
    HEAD

} ModelMap;

#define MAX 100.0
#define MAX_ITER 100.0
#define INF std::numeric_limits<double>::infinity()

namespace motion_filter
{

struct CustomJoint
{
    int parent_id;
    std::vector<bvh11::Channel::Type> types;
    std::string joint_name;
    Vector3d offset;
};

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}

class SkeletonKinectmatics
{
public:
SkeletonKinectmatics(DataHandler &dh);
~SkeletonKinectmatics();
// void printModelInfo();
// void CalibOffsets();
void publish();
void step();
double getTimeStep(){return h_;};

protected:
void constructModel();
void constructJoint(bvh11::BvhObject bvh);
void computeJacobians();
void computeJacobians(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot);
void updateKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot);
Eigen::Isometry3d* getCurrentTransform();
Eigen::Isometry3d* getCurrentTransform(const Eigen::VectorXd& q);

std::shared_ptr<Model> model_;

Eigen::Isometry3d T_base_init_;
Eigen::Quaterniond R_inits_[6]; //same order as target indicies

bool is_first_ = true;
std::string solver_type_;
std::map<std::string, std::vector<double>> gains_;

Math::VectorNd current_q_;
Math::VectorNd current_qdot_;

Math::VectorNd prev_q_;
Math::VectorNd prev_qdot_;

std::vector<Math::MatrixNd> jacs_; //thorax_jac_, lshoulder_jac, lhand_jac, rshoulder_jac, rhand_jac, head;
std::vector<int> target_indices_; //model id in tracker order;

std::vector<Eigen::Vector3d> end_effectors_;  //head larm rarm
std::map<std::string, Eigen::Vector3d> marker_offsets_;
int num_body_;
DataHandler &dh_;
int num_task_;
double h_; //timestep

//Logger
bool is_log_;
CsvLogger* q_logger_;
std::chrono::high_resolution_clock::time_point start_time_;

std::string bvh_file_;
private:
//Common Method
virtual void initParams() = 0;
virtual void parseToml(std::string &toml_path) = 0;
virtual void solveIK() = 0;

int skel_pub_ind_;
int rpose_pub_ind_[NUM_TRACKER + 1];
int ftracker_pub_ind_[NUM_TRACKER + 1];



std::vector<CustomJoint> custom_joints_;




};
}
#endif //SKELETON_KINMEMATICS_BASE_H

