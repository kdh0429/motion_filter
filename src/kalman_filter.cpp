#include "motion_filter/kalman_filter.hpp"

namespace motion_filter
{

KalmanFilter::KalmanFilter(VectorXd state_init, MatrixXd cov_init)
{
    state_.mean = state_init;
    state_.cov = cov_init;
}

KalmanFilter::~KalmanFilter()
{
    std::cout<<"kalman filter destructor"<<std::endl;
}

Gaussian KalmanFilter::step(MatrixXd F, MatrixXd H, MatrixXd Q, MatrixXd R, VectorXd y)
{
    //predict
    state_.mean = F * state_.mean;
    state_.cov = F * state_.cov * F.transpose() + Q;

    //update
    MatrixXd Z = H * state_.cov * H.transpose() + R;
    MatrixXd K = state_.cov * H.transpose() * Z.inverse();
    state_.mean = state_.mean + K * (y - H * state_.mean);
    state_.cov = state_.cov - K * Z * K.transpose();

    return state_;

}

Gaussian KalmanFilter::step(MatrixXd F, MatrixXd G, MatrixXd H, MatrixXd Q, MatrixXd R, VectorXd u, VectorXd y)
{
    //predict
    state_.mean = F * state_.mean + G * u;
    state_.cov = F * state_.cov * F.transpose() + G * Q * G.transpose();

    //update
    MatrixXd Z = H * state_.cov * H.transpose() + R;
    MatrixXd K = state_.cov * H.transpose() * Z.inverse();
    state_.mean = state_.mean + K * (y - H * state_.mean);
    state_.cov = state_.cov - K * Z * K.transpose();

    return state_;
}


}