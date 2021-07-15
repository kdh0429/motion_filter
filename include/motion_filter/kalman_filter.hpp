/*
Skeleton Kinematic

Data:  2021.07.14 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;


namespace motion_filter
{

struct Gaussian
{
  VectorXd mean;
  MatrixXd cov;
};

class KalmanFilter
{
public:
KalmanFilter(VectorXd state_init, MatrixXd cov_init);
~KalmanFilter();
Gaussian step(MatrixXd F, MatrixXd H, MatrixXd Q, MatrixXd R, VectorXd y);
Gaussian step(MatrixXd F, MatrixXd G, MatrixXd H, MatrixXd Q, MatrixXd R, VectorXd u, VectorXd y); //action noise

private:
Gaussian state_;


};


}
#endif //KALMAN_FILTER_H