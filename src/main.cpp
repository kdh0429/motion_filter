#include <motion_filter/data_handler.hpp>
#include <motion_filter/pre_process.hpp>
#include <thread>
#include <memory>
#include <ros/ros.h>

using namespace motion_filter;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_filter");
    ros::NodeHandle nh("~");

    double dt = 1.0/1000.0;

    DataHandler dh = DataHandler(nh);
    PreProcess *filters[NUM_TRACKER + 1];
    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        filters[i] = new PreProcess(nh, i, dt, false);
    }
    

    ros::Rate loop_rate(1.0/dt);
    ros::param::set("/mp/viz_flag", true);
    ros::param::set("/mp/restart_flag", false);
    bool restart;

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
        ros::param::get("/mp/restart_flag", restart);
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

        ros::param::set("/mp/restart_flag", false);

        ros::spinOnce();
        // loop_rate.sleep();
    }

    ros::param::set("/mp/viz_flag", false);
    return 0;
}