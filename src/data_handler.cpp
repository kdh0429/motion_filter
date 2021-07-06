#include "motion_filter/data_handler.hpp"

namespace motion_filter
{
VR::matrix_3_4 isometry3d2VRmsg(Eigen::Isometry3d T)
{
  Eigen::Matrix4d T_mat = T.matrix();

  VR::matrix_3_4 msg;

  Eigen::RowVector4d rv1, rv2, rv3;
  rv1 = T_mat.block(0, 0, 1, 4);
  rv2 = T_mat.block(1, 0, 1, 4);
  rv3 = T_mat.block(2, 0, 1, 4);
  

  std::vector<double> r1(&rv1[0], rv1.data() + rv1.cols()*rv1.rows());
  std::vector<double> r2(&rv2[0], rv2.data() + rv2.cols()*rv2.rows());
  std::vector<double> r3(&rv3[0], rv3.data() + rv3.cols()*rv3.rows());
  
  msg.firstRow = r1;
  msg.secondRow = r2;
  msg.thirdRow = r3;

  return msg;
}

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
