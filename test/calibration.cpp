#include <logger.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <ctime>
#include "motion_filter/data_handler.hpp"
#include <ros/ros.h>
#include <std_msgs/Int8.h>

#define NUM_KEY 7
#define NUM_DATA 6
// #define NUM_COL 10

using namespace motion_filter;
using VectorKd = Eigen::Matrix<double, NUM_DATA , 1>;

int mode = -1;

void calibrationCallback(const std_msgs::Int8 msg)
{
    mode = msg.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh("~");
    DataHandler dh = DataHandler(nh);

    double dt = 1.0/1000.0;
    ros::Rate loop_rate(1.0/dt);

    std::string str = "20200706";
    std::string package_path = ros::package::getPath("motion_filter");
    std::string dirpath = package_path + "/data/" + str;
    std::string fname = "sdh_calibration.csv";

    std::vector<std::string> keys = {"mode", "lx", "ly", "lz", "rx", "ry", "rz"};

    CsvLogger logger(dirpath, fname, keys);

    ros::Subscriber calib_mode = nh.subscribe("/CALIBMODE", 100, calibrationCallback);

    bool status[3] = {false, false, false}; //A, T, F
    while(ros::ok())
    {
        std::vector<double> data;
        if (mode >=0)
        {
            // std::cout<<mode<<std::endl;
            if(status[mode - 1] == false)
            {
                std::cout<<mode<<std::endl;
                Eigen::Isometry3d* T = dh.getObs();
                
                VectorKd eig_data;
                eig_data.head(3) = T[3].translation() - T[1].translation();
                eig_data.tail(3) = T[5].translation() - T[1].translation();
                data.clear();
                data.resize(NUM_DATA + 1);
                data.at(0) = double(mode);
                Eigen::VectorXd::Map(&data[1], NUM_DATA) = eig_data;
                logger.writeRows(data);
                status[mode - 1] = true;
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}


