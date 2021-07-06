#include <motion_filter/data_handler.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

using namespace motion_filter;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dh_test");
    ros::NodeHandle nh("~");

    double dt = 1.0/100.0;

    DataHandler dh = DataHandler(nh);
    ros::Rate loop_rate(1.0/dt);
    int pub1_ind, pub2_ind;

    pub1_ind = dh.addPublisher<std_msgs::Int32>("/int_test", 100);
    pub2_ind = dh.addPublisher<std_msgs::Float64>("/double_test", 100);
    

    while(ros::ok())
    {
        std_msgs::Int32 msg1;
        std_msgs::Float64 msg2;
        msg1.data = 1;
        msg2.data = 5.0;
        
        dh.publish(msg1, pub1_ind);
        dh.publish(msg2, pub2_ind);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}