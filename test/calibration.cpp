#include <logger.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <ctime>
#include "motion_filter/data_handler.hpp"

#define NUM_KEY 6
#define NUM_DATA 6
// #define NUM_COL 10

using VectorKd = Eigen::Matrix<double, NUM_DATA , 1>;


void calibrationCallback()
{
    
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh("~");
    DataHandler dh = DataHandler(nh);

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H_%M_%S",timeinfo);
    std::string str(buffer);

    std::string package_path = ros::package::getPath("motion_filter");
    std::string dirpath = package_path + "/data/" + str;
    std::string fname = "test.csv";

    std::vector<std::string> keys;
    for (int i=0;i<NUM_KEY;i++)
    {
        keys.push_back("x" + std::to_string(i));
    }
    CsvLogger logger(dirpath, fname, keys);

    ros::Subscribe calib_mode = nh.subscribe("/CALIBMODE", 100, &calibrationCallback, this);

    while(ros::ok())
    {
        skeleton_kinect.solveIK();
        skeleton_kinect.publish();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::vector<double> data;
    for (int i=0; i< NUM_COL;i++)
    {
        VectorKd eig_data = VectorKd::Zero();
        data.clear();
        data.resize(NUM_DATA);
        Eigen::VectorXd::Map(&data[0], NUM_DATA) = eig_data;
        logger.writeRows(data);
    }


    return 0;

}


