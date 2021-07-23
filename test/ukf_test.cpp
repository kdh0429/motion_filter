#include "motion_filter/motion_filter.hpp"

using namespace motion_filter;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dh_test");
    ros::NodeHandle nh("~");

    double dt = 1.0/100.0;

    DataHandler dh = DataHandler(nh);
    ros::Rate loop_rate(1.0/dt);


    int dim = 2;
    MerweScaledSigmaPoints test(dim, 0.1, 2.0, 1.0);

    MatrixXd M(dim, dim);
    VectorXd m(dim);

    M.setIdentity();
    m.setZero();


    std::cout<<test.getSigmaPoints(m, M)<<std::endl;

    Eigen::VectorXd x(4);

    x << 1.0, 2.0, 2.0, 2.0;

    Eigen::MatrixXd y(4, 2);
    y << 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0;

    Eigen::MatrixXd out = x.asDiagonal() * y;

    std::cout<<x<<std::endl;
    std::cout<<y<<std::endl;
    std::cout<<out<<std::endl;
    std::cout<<out.colwise().sum()<<std::endl;
    
    Eigen::MatrixXd tmp = x * x.transpose();
    std::cout<<tmp<<std::endl;

    
    

    return 0;
}