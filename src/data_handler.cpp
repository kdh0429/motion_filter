#include "motion_filter/data_handler.hpp"

namespace motion_filter
{
void DataHandler::trackersCallback(const ros::MessageEvent<VR::matrix_3_4>& event)
{
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    int index = topic.back() - '0';
    const VR::matrix_3_4ConstPtr& msg = event.getMessage();

    raw_poses_[index].linear().block(0, 0, 1, 3) = Eigen::RowVectorXd::Map(msg->firstRow.data(), 3);
    raw_poses_[index].linear().block(1, 0, 1, 3) = Eigen::RowVectorXd::Map(msg->secondRow.data(), 3);
    raw_poses_[index].linear().block(2, 0, 1, 3) = Eigen::RowVectorXd::Map(msg->thirdRow.data(), 3);

    raw_poses_[index].translation()(0) = msg->firstRow[3];
    raw_poses_[index].translation()(1) = msg->secondRow[3];
    raw_poses_[index].translation()(2) = msg->thirdRow[3];

    // std::cout<<index <<":\t"<<raw_poses_[index].linear().determinant()<<std::endl;    
}
void DataHandler::hmdCallback(const VR::matrix_3_4 &msg)
{

    raw_poses_[NUM_TRACKER].linear().block(0, 0, 1, 3) = Eigen::RowVectorXd::Map(msg.firstRow.data(), 3);
    raw_poses_[NUM_TRACKER].linear().block(1, 0, 1, 3) = Eigen::RowVectorXd::Map(msg.secondRow.data(), 3);
    raw_poses_[NUM_TRACKER].linear().block(2, 0, 1, 3) = Eigen::RowVectorXd::Map(msg.thirdRow.data(), 3);

    raw_poses_[NUM_TRACKER].translation()(0) = msg.firstRow[3];
    raw_poses_[NUM_TRACKER].translation()(1) = msg.secondRow[3];
    raw_poses_[NUM_TRACKER].translation()(2) = msg.thirdRow[3];

    // std::cout<<"head:"<<raw_poses_[NUM_TRACKER].linear().determinant()<<std::endl;    

}

void DataHandler::trackersStatusCallback(const std_msgs::Bool &msg)
{

    tracker_status_ = msg.data;
    // std::cout<<tracker_status_<<std::endl;
}

}