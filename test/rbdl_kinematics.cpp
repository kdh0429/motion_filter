#include <iostream>
#include "motion_filter/data_handler.hpp"
#include "motion_filter/skeleton_kinematics.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace motion_filter;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbdl_test");
    ros::NodeHandle nh("~");
    DataHandler dh = DataHandler(nh);

    SkeletonKinectmatics skeleton_kinect = SkeletonKinectmatics(dh);


    double dt = 1.0/1000.0;
    ros::Rate loop_rate(1.0/dt);

    while(ros::ok())
    {
        skeleton_kinect.solveIK();
        skeleton_kinect.publish();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}