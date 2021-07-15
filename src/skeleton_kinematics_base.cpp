/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#include "motion_filter/skeleton_kinematics_base.hpp"

namespace motion_filter
{
//----------------------------------------------- Public Metthod ------------------------------------------------------------------
SkeletonKinectmatics::SkeletonKinectmatics(DataHandler &dh): dh_(dh)
{
    skel_pub_ind_ = dh_.addPublisher<std_msgs::Float64MultiArray>("/skel", 100);
    for (int i=0;i<NUM_TRACKER + 1;i++)
    {
        rpose_pub_ind_[i] = dh_.addPublisher<std_msgs::Float64MultiArray>("/Rposquat" + std::to_string(i), 100);

        if (i < NUM_TRACKER)
            ftracker_pub_ind_[i] = dh_.addPublisher<VR::matrix_3_4>("/FTRACKER" + std::to_string(i), 100);
    }

}
SkeletonKinectmatics::~SkeletonKinectmatics()
{
    // delete model_;
    std::cout<<"Destructor"<<std::endl;
}
void SkeletonKinectmatics::publish()
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    std::vector<double> tmp;

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    updateKinematics(current_q_, current_qdot_);

    for(int id=1;id <= num_body_;id++)
    {
        tmp.resize(3);
        Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, id, Vector3d::Zero(), false);
        msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
        tmp.clear();

        if (id == target_indices_.at(ModelMap::HEAD))
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, id, end_effectors_.at(0), false);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
        else if (id == target_indices_.at(ModelMap::LHAND))
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, id,  end_effectors_.at(1), false);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
        else if (id == target_indices_.at(ModelMap::RHAND))
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, id,  end_effectors_.at(2), false);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
    }

    tmp.resize(4);
    Eigen::VectorXd::Map(&tmp[0], 4) = Quaterniond(CalcBodyWorldOrientation(*model_, current_q_, target_indices_.at(ModelMap::LHAND), false).transpose()).coeffs();
    msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
    tmp.clear();

    tmp.resize(4);
    Eigen::VectorXd::Map(&tmp[0], 4) = Quaterniond(CalcBodyWorldOrientation(*model_, current_q_, target_indices_.at(ModelMap::RHAND), false).transpose()).coeffs();
    msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
    tmp.clear();

    dh_.publish(msg, skel_pub_ind_);

    Eigen::Isometry3d* T;
    T = dh_.getRaw();

    for (int i=1;i<NUM_TRACKER+1;i++)
    {   
        std_msgs::Float64MultiArray rposquat; //a.k.a std::vector<double>
        rposquat.data.clear();
        rposquat.data.resize(7);
        Eigen::Isometry3d tmp;
        tmp.translation() = T[i].translation() - T_base_init_.translation();
        // tmp.linear() = T_base_init_.linear().transpose() *  T[i].linear() * R_inits_[i - 1];
        tmp.linear() = T[i].linear();

        // Eigen::AngleAxisd aa(tmp.linear());
        // tmp.linear() = aa.toRotationMatrix();
        // std::cout<<std::setprecision(2)<<tmp.translation().transpose()<<std::endl;
        // std::cout<<std::setprecision(2)<<T[i].translation().transpose()<<std::endl;
        
        Eigen::VectorXd::Map(&rposquat.data[0], 7) = manif::SE3d(tmp).coeffs();   
        // Eigen::VectorXd::Map(&rposquat.data[0], 7) = manif::SE3d(T[i]).coeffs();   
        dh_.publish(rposquat, rpose_pub_ind_[i]);
    }
    // std::cout<<"============================"<<std::endl;


    //Tracker Publisher

    Isometry3d* T_cur = getCurrentTransform();
    for(int i= 1; i < NUM_TRACKER;i++)
    {
    
       dh_.publish(isometry3d2VRmsg(T_cur[i]), ftracker_pub_ind_[i]);
    }
    if(is_log_)
    {
        
        std::vector<double> data;
        data.resize(model_->dof_count + 1);
        auto end = std::chrono::high_resolution_clock::now();
        data.at(0) = std::chrono::duration<double>(end - start_time_).count();
        Eigen::VectorXd::Map(&data[1], model_->dof_count) = current_q_;
        q_logger_ -> writeRows(data);
    }

}
void SkeletonKinectmatics::step()
{
    solveIK();
}
//----------------------------------------------- Private Metthod ------------------------------------------------------------------
void SkeletonKinectmatics::constructJoint(bvh11::BvhObject bvh)
{
    auto joint_list = bvh.GetJointList();

    std::map<std::string, int> string2id;
    int id = 1;
    //make dictionary joint name -> id 
    for (auto joint: joint_list)
    {
        string2id.insert(std::pair<std::string, int>(joint->name(), id));
        id +=1;
    }
    string2id.insert(std::pair<std::string, int>("ROOT", 0));

    //make custom joints execpt joint type
    int body_count = 0;

    for(auto joint: joint_list)
    {
        CustomJoint tmp;
        std::string parent_name;
        if (joint-> parent() == nullptr)
            parent_name = "ROOT";
        else
            parent_name = joint->parent()->name();

        tmp.parent_id = string2id[parent_name];
        tmp.joint_name = joint->name();
        tmp.offset = joint->offset();

        if (joint->has_end_site())
            end_effectors_.push_back(joint->end_site());
        custom_joints_.push_back(tmp);

        body_count ++;
    }
    num_body_ = body_count;
    //joint rotation type
    int index = 0;
    for (auto channel: bvh.channels())
    {
        std::string target_joint_name = channel.target_joint->name();
        // std::cout<<index<<target_joint_name<<channel.type<<std::endl;
        if (custom_joints_.at(index).joint_name == target_joint_name)
        {
           custom_joints_.at(index).types.push_back(channel.type);
        }
        else
        {
            index ++;
            custom_joints_.at(index).types.push_back(channel.type);

        }

    }
    //check
    // for(auto cj: custom_joints_)
    // {
    //     std::cout<<"joint_name: "<<cj.joint_name<<"\tjoint_types: ";
    //     for (auto type: cj.types)
    //     {
    //         std::cout<<type<<" ";
    //     }
    //     std::cout<<std::endl;
    // }

}
void SkeletonKinectmatics::constructModel()
{
    model_ = std::make_shared<Model>(); //new Model();
    Body rbdl_body = Body();
    for(auto cj: custom_joints_)
    {
        Joint rbdl_joint;
        if (cj.types.size()==3)
        {
            rbdl_joint = Joint(JointTypeEulerZYX);
            // rbdl_joint = Joint(JointTypeSpherical);
            
        }
        else if (cj.types.size()==6)
        {
            rbdl_joint = Joint(JointTypeFloatingBase); //maseless is not work z fixed to zero;
        }
        else if (cj.types.size()==2)
        {
            // rbdl_joint = Joint(
            //     Math::SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            //     Math::SpatialVector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            // );
            rbdl_joint = Joint(JointTypeEulerZYX); //maseless is not work z fixed to zero;
            // rbdl_joint = Joint(JointTypeSpherical);

        }

        else if (cj.types.size()==1)
        {
            rbdl_joint = Joint(
                Math::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
            );    
        }
        // std::cout<< cj.parent_id<<"\t"<<cj.joint_name<<std::endl;
        model_ -> AddBody(
            cj.parent_id,
            Math::SpatialTransform(Matrix3d::Identity(), cj.offset),
            rbdl_joint,
            rbdl_body,
            cj.joint_name
        );
    }
    std::cout<<Utils::GetModelDOFOverview(*model_)<<std::endl;
    std::cout<<Utils::GetModelHierarchy(*model_)<<std::endl;
    std::cout<<"Num Body:"<<num_body_<<std::endl;
}
void SkeletonKinectmatics::updateKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
{
  RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}
void SkeletonKinectmatics::computeJacobians()
{
    updateKinematics(current_q_, current_qdot_);

    for (int i=0; i<target_indices_.size();i++)
    {
        jacs_.at(i).setZero();
        Vector3d offset;
        if (i == ModelMap::HEAD)
            offset = marker_offsets_["head"];
        else if (i == ModelMap::THORAX)
            offset = marker_offsets_["thorax"];
        else if (i == ModelMap::LSH)
            offset = marker_offsets_["lshoulder"];
        else if (i == ModelMap::LHAND)
            offset = marker_offsets_["lhand"];
        else if (i == ModelMap::RSH)
            offset = marker_offsets_["rshoulder"];
        else if (i == ModelMap::RHAND)
            offset = marker_offsets_["rhand"];
        else
            offset.setZero();

        CalcPointJacobian6D(*model_, current_q_, target_indices_.at(i), offset, jacs_.at(i), false);

        // if (target_indices_.at(i) == LHAND)
        // {
        //     std::cout<<"Jac"<<std::endl;
        //     std::cout<<std::setprecision(2)<<jacs_.at(i)<<std::endl;
        //     std::cout<<"================================================="<<std::endl;
        // }

    }


    // std::cout<<current_q_.transpose()<<std::endl;

}
void SkeletonKinectmatics::computeJacobians(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
{
    updateKinematics(q, qdot);

    for (int i=0; i<target_indices_.size();i++)
    {
        jacs_.at(i).setZero();
        Vector3d offset;
        if (i == ModelMap::HEAD)
            offset = marker_offsets_["head"];
        else if (i == ModelMap::THORAX)
            offset = marker_offsets_["thorax"];
        else if (i == ModelMap::LSH)
            offset = marker_offsets_["lshoulder"];
        else if (i == ModelMap::LHAND)
            offset = marker_offsets_["lhand"];
        else if (i == ModelMap::RSH)
            offset = marker_offsets_["rshoulder"];
        else if (i == ModelMap::RHAND)
            offset = marker_offsets_["rhand"];
        else
            offset.setZero();

        CalcPointJacobian6D(*model_, q, target_indices_.at(i), offset, jacs_.at(i), false);
    }
}
Eigen::Isometry3d* SkeletonKinectmatics::getCurrentTransform()
{

    Eigen::Isometry3d* T_cur = new Eigen::Isometry3d[6]; 
    for (int i=0; i<target_indices_.size();i++)
    {
        Vector3d local_offset;
        if (i == ModelMap::HEAD)
            local_offset = marker_offsets_["head"];
        else if (i == ModelMap::THORAX)
            local_offset = marker_offsets_["thorax"];
        else if (i == ModelMap::LSH)
            local_offset = marker_offsets_["lshoulder"];
        else if (i == ModelMap::LHAND)
            local_offset = marker_offsets_["lhand"];
        else if (i == ModelMap::RSH)
            local_offset = marker_offsets_["rshoulder"];
        else if (i == ModelMap::RHAND)
            local_offset = marker_offsets_["rhand"];
        else
            local_offset.setZero();


        T_cur[i].translation() = CalcBodyToBaseCoordinates(*model_, current_q_, target_indices_.at(i), local_offset, false);
        T_cur[i].linear() = CalcBodyWorldOrientation(*model_, current_q_, target_indices_.at(i), false).transpose();
    }
    return T_cur;
}
Eigen::Isometry3d* SkeletonKinectmatics::getCurrentTransform(const Eigen::VectorXd& q)
{
    VectorXd qdot = VectorXd::Zero(model_->dof_count);
    updateKinematics(q, qdot);

    Eigen::Isometry3d* T_cur = new Eigen::Isometry3d[6]; 
    for (int i=0; i<target_indices_.size();i++)
    {
        Vector3d local_offset;
        if (i == ModelMap::HEAD)
            local_offset = marker_offsets_["head"];
        else if (i == ModelMap::THORAX)
            local_offset = marker_offsets_["thorax"];
        else if (i == ModelMap::LSH)
            local_offset = marker_offsets_["lshoulder"];
        else if (i == ModelMap::LHAND)
            local_offset = marker_offsets_["lhand"];
        else if (i == ModelMap::RSH)
            local_offset = marker_offsets_["rshoulder"];
        else if (i == ModelMap::RHAND)
            local_offset = marker_offsets_["rhand"];
        else
            local_offset.setZero();


        T_cur[i].translation() = CalcBodyToBaseCoordinates(*model_, q, target_indices_.at(i), local_offset, false);
        T_cur[i].linear() = CalcBodyWorldOrientation(*model_, q, target_indices_.at(i), false).transpose();
    }
    return T_cur;
}

}