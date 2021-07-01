#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <bvh11.hpp>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace RigidBodyDynamics;
using namespace Eigen;

void AddEulerZYXJoint(Model* model, Vector3d offset, int parent_id, std::string name, double scale)
{
    model->AddBody(
        parent_id,
        Math::SpatialTransform(Matrix3d::Identity(), offset* scale),
        Joint(RigidBodyDynamics::JointTypeEulerZYX),
        Body(),
        name       
    );
}
void print(std::list<int> const &list)
{
    for (auto const &i: list) {
        std::cout << i << std::endl;
    }
}
 
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Need [BVH_PATH]" << std::endl;
        return -1;
    }

    const std::string bvh_file_path = argv[1];

    ros::init(argc, argv, "rbdl_test");
    ros::NodeHandle nh("~");
    ros::Publisher skel_pub = nh.advertise<std_msgs::Float64MultiArray>("/skel", 100);

    bvh11::BvhObject bvh(bvh_file_path);
    
    std::cout << "#Channels       : " << bvh.channels().size() << std::endl;
    std::cout << "#Frames         : " << bvh.frames()          << std::endl;
    std::cout << "Frame time      : " << bvh.frame_time()      << std::endl;
    std::cout << "Joint hierarchy : " << std::endl;
    bvh.PrintJointHierarchy();
    

    auto joint_list = bvh.GetJointList();
    std::vector<int> ids;
    std::map<std::string, int> string2id;
    
    int id = 1;
    for (auto joint: joint_list)
    {
        string2id.insert(std::pair<std::string, int>(joint->name(), id));
        id +=1;
    }

    for (auto channel: bvh.channels())
    {
        std::cout<<channel.target_joint -> name()<<"\t"<<channel.type <<std::endl;
    }
    return 0;
    string2id.insert(std::pair<std::string, int>("ROOT", 0));

    std::cout<<"RBDL test"<<std::endl;
    Model* model = new Model();
    double scale = 0.05;
    for (auto joint: joint_list)
    {   
        std::string parent_name;
        if (joint-> parent() == nullptr)
            parent_name = "ROOT";
        else
            parent_name = joint->parent()->name();

        AddEulerZYXJoint(model, joint->offset(), string2id[parent_name], joint->name(), scale);
    }


    std::cout<<Utils::GetModelDOFOverview(*model)<<std::endl;
    std::cout<<Utils::GetModelHierarchy(*model)<<std::endl;

    Math::VectorNd QDot = Math::VectorNd::Zero(model->dof_count);
    
    // while (ros::ok())
    // {
    //     Math::VectorNd Q = Math::VectorNd::Zero(model->dof_count);
    //     std_msgs::Float64MultiArray msg;
    //     msg.data.clear();
    //     std::vector<double> tmp;
    //     std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //     for(int i=1;i<=14;i++)
    //     {
    //         tmp.resize(3);
    //         Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model, Q, i, Vector3d::Zero(), true);
    //         msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
    //         tmp.clear();
    //     }
    //     skel_pub.publish(msg);
    //     std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //     // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[ms]" << std::endl;
    // }
    // std::cout<<CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("LeftHand"), Vector3d::Zero(), true).transpose()<<std::endl;
    // std::cout<<CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("RightHand"), Vector3d::Zero(), true).transpose()<<std::endl;
    // std::cout<<CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("LeftHand"), Vector3d::Zero(), true).transpose()<<std::endl;
    // TODO calibration scale & rendering
    // TODO style IK(FABRIK, JACOBIAN)
    // TODO 

    // UpdateKinematics(*model, Q, QDot, NULL);

    
    
    delete model;
    return 0;
}