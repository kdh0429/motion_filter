#include <motion_filter/data_handler.hpp>
#include <motion_filter/pre_process.hpp>
#include <thread>
#include <memory>
#include <ros/ros.h>

using namespace motion_filter;


int main(int argc, char **argv)
{
    bool verbose;
    if (argc==2)
    {   
        std::string str = argv[argc-1];
        verbose = std::stoi(str);
    }
    else
    {
        std::cout<<"add at least one argument whether save log file or not(0 not saving, 1 save)!";
        return -1;
    }

    ros::init(argc, argv, "motion_filter");
    ros::NodeHandle nh("~");

    double dt = 1.0/1000.0;

    DataHandler dh = DataHandler(nh);
    PreProcess *filters[NUM_TRACKER + 1];
    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        filters[i] = new PreProcess(nh, i, dt, verbose);
    }
    

    ros::Rate loop_rate(1.0/dt);
    bool restart = false;

    auto func = [&](Eigen::Isometry3d* T, int i, bool restart, PreProcess *filter)
    {
        if (restart)
            filters[i] -> restart();
        double flag = ((T+i)->linear()).determinant();
        
        if (fabs(flag - 1.0) < 1e-6)
        {
            filters[i] -> step(T[i]);
        }
    };

    while(ros::ok())
    {
        Eigen::Isometry3d* T;

        T = dh.getObs();
        
        std::vector<std::thread> ts;

        for (int i=0;i<NUM_TRACKER + 1;i++)
        {
            std::thread t(func, T, i, restart, filters[i]);
            ts.push_back(std::move(t));
        }

        for (unsigned int i=0; i<ts.size(); ++i)
        {
            if (ts[i].joinable())
                ts.at(i).join();
        }

        ros::spinOnce();
        // loop_rate.sleep();
    }

    return 0;
}