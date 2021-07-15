#include <iostream>
#include <thread>
#include <atomic>
#include "motion_filter/motion_filter.hpp"

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace motion_filter;
#define M 1000000.0
#define PubHz 1000.0
//Filter thread
void filterThread(SE3Filter* filter, DataHandler& dh, std::atomic< bool >& run)
{   
    int tracker_id = filter->getId();
    double dt = filter->getTimeStep();

    while (run)
    {   
        auto begin = std::chrono::high_resolution_clock::now();
        auto next = begin + std::chrono::microseconds(int(M/PubHz));

        // std::cout<<tracker_id<<" Run"<<std::endl;
        bool tracker_status = dh.getTrackerStatus();

        if (tracker_status)
        {
            Isometry3d T = dh.getRaw()[tracker_id];
            filter->step(T, tracker_status);
            dh.setFiltered(tracker_id, filter->getTransform());
        }

        std::this_thread::sleep_until(next);
    }
}

void pubThread(SkeletonKinectmatics* skeleton_kinect, std::atomic< bool >& run)
{
    while (run)
    {   
        auto begin = std::chrono::high_resolution_clock::now();
        auto next = begin + std::chrono::microseconds(int(M/PubHz));
        skeleton_kinect -> publish();
        std::this_thread::sleep_until(next);
    }
}

int main(int argc, char** argv)
{
    std::string algo_param;
    if (argc==2)
    {   
        algo_param = argv[argc-1];
    }
    else
    {
        std::cout<<"add at least one argument(lsq, huber, kalman)!\n";
        return -1;
    }

    ros::init(argc, argv, "skeleton_kinematics");
    ros::NodeHandle nh("~");
    DataHandler dh = DataHandler(nh);


    //Kinematics Module
    SkeletonKinectmatics* skeleton_kinect; //parent class
    
    //polymorphism
    if (algo_param == "lsq")
        skeleton_kinect = new LSQKinect(dh);
    else if (algo_param == "huber")
        skeleton_kinect = new HuberKinect(dh);
    else if (algo_param == "kalman")
        skeleton_kinect = new KalmanKinect(dh);
    else
    {
        std::cout<<"Not Implemented algo param\n";
    }

    double dt = skeleton_kinect->getTimeStep();
    ros::Rate loop_rate(1.0/dt);


    std::atomic< bool > run{ true };

    //Pub thread
    std::thread pub_thread(pubThread, skeleton_kinect, std::ref(run));

    // Filter Moudle
    std::vector<std::thread> filter_threads;
    SE3Filter *filters[NUM_TRACKER + 1];

    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        filters[i] = new SE3Filter(nh, i, dt, false);
    }

    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        std::thread t(filterThread, filters[i], std::ref(dh), std::ref(run));
        filter_threads.push_back(std::move(t));
    }

    while(ros::ok())
    {
        auto begin = std::chrono::high_resolution_clock::now();
        auto next = begin + std::chrono::microseconds(int(dt*M));
        
        skeleton_kinect -> step();
        ros::spinOnce();

        std::this_thread::sleep_until(next);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;
        std::cout<<"\n===================================================================\n"<<std::endl;

    }

    run = false;
    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        filter_threads.at(i).join();
    }
    pub_thread.join();

    return 0;

}