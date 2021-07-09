/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#include "motion_filter/skeleton_kinematics.hpp"

namespace motion_filter
{
//----------------------------------------------- Public Metthod ------------------------------------------------------------------
SkeletonKinectmatics::SkeletonKinectmatics(DataHandler &dh): dh_(dh)
{
    std::string package_path = ros::package::getPath("motion_filter");
    std::string toml_path = package_path + "/config/kinematics.toml";

    parseToml(toml_path);

    skel_pub_ind_ = dh_.addPublisher<std_msgs::Float64MultiArray>("/skel", 100);

    for (int i=0;i<NUM_TRACKER+1;i++)
    {
        rpose_pub_ind_[i] = dh_.addPublisher<std_msgs::Float64MultiArray>("/Rposquat" + std::to_string(i), 100);
    }

    std::string skel_path = package_path + "/config/" + bvh_file_;
    double scale = 1.0;
    bvh11::BvhObject bvh(skel_path, scale);
    // bvh.PrintJointHierarchy();
    constructJoint(bvh);
    constructModel();
    initParams();
}

SkeletonKinectmatics::~SkeletonKinectmatics()
{
    delete model_;
    std::cout<<"Destructor"<<std::endl;
}

void SkeletonKinectmatics::publish()
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    std::vector<double> tmp;

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[ms]" << std::endl;

    dh_.publish(msg, skel_pub_ind_);

    Eigen::Isometry3d* T;
    T = dh_.getObs();

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

}
void SkeletonKinectmatics::solveIK()
{
    if (solver_type_ == "lsq")
    {
        lsqIK();
    }
    else if (solver_type_ == "huber")
    {
        huberIK();
    }

    else
    {
        std::cout<<"not implemented"<<std::endl;
    }
}
//----------------------------------------------- Private Metthod ------------------------------------------------------------------
void SkeletonKinectmatics::initParams()
{
    num_task_ =  target_indices_.size() * 6 - 6; //all pose except head

    current_q_.resize(model_->dof_count, 1);
    current_qdot_.resize(model_->dof_count, 1);

    current_q_.resize(model_->dof_count, 1);
    prev_qdot_.resize(model_->dof_count, 1);

    jacs_.resize(target_indices_.size());

    for (int i=0; i<target_indices_.size();i++)
        jacs_.at(i).resize(6, model_->dof_count);
    
    qp_.InitializeProblemSize(model_ -> dof_count, num_task_); 

    T_base_init_.linear().setIdentity();
    current_q_.setZero();
    current_qdot_.setZero();
}
void SkeletonKinectmatics::parseToml(std::string &toml_path)
{
    auto data = toml::parse(toml_path);
    bvh_file_ = toml::find<std::string>(data, "bvh_file");
    solver_type_ = toml::find<std::string>(data, "solver");
    target_indices_ = toml::find<std::vector<int>>(data, "target_index");

    
    auto& gains = toml::find(data, "gains");
    gains_["hand"] = toml::find<std::vector<double>>(gains, "hand");
    gains_["shoulder"] = toml::find<std::vector<double>>(gains, "shoulder");
    gains_["thorax"] = toml::find<std::vector<double>>(gains, "thorax");

    auto& tracker_offsets = toml::find(data, "tracker_offsets");
    std::vector<double> tmp;

    tmp = toml::find<std::vector<double>>(tracker_offsets, "head");
    marker_offsets_["head"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

    tmp = toml::find<std::vector<double>>(tracker_offsets, "lhand");
    marker_offsets_["lhand"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

    tmp = toml::find<std::vector<double>>(tracker_offsets, "lshoulder");
    marker_offsets_["lshoulder"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

    tmp = toml::find<std::vector<double>>(tracker_offsets, "rhand");
    marker_offsets_["rhand"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

    tmp = toml::find<std::vector<double>>(tracker_offsets, "rshoulder");
    marker_offsets_["rshoulder"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

    tmp = toml::find<std::vector<double>>(tracker_offsets, "thorax");
    marker_offsets_["thorax"] = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
}
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
    model_ = new Model();
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

void SkeletonKinectmatics::lsqIK()
{

    Eigen::Isometry3d* T_des_raw;
    Eigen::Isometry3d T_des[NUM_TRACKER + 1]; //base. Thorax, lsh, lhand, rsh, rhand, Head;
    
    Eigen::Isometry3d* T_cur;

    T_des_raw = dh_.getObs();
    bool tracker_status = dh_.getTrackerStatus();

    computeJacobians(); //updateKinmeatics + jacobians
    T_cur = getCurrentTransform(); //Thorax, lsh, lhand, rsh, rhand head;

    if (is_first_ )
    {
        if (tracker_status) //calculate front orientation and translation
    {
            T_base_init_.translation() = T_des_raw[1].translation() - marker_offsets_["thorax"]; //fix

            Vector3d cx, cy, cz; //basis vector
            cy = (T_des_raw[2].translation() - T_des_raw[4].translation()).normalized(); //T-pose eblow;
            cz = (T_des_raw[6].translation() - T_des_raw[1].translation()).normalized(); //chest_head;
            cx = cy.cross(cz);

            Matrix3d R_front;
            R_front.setZero();
            R_front.block(0, 0, 3, 1) = cx;
            R_front.block(0, 1, 3, 1) = cy;
            R_front.block(0, 2, 3, 1) = cz;
            T_base_init_.linear() = R_front;
            // T_base_init_.linear() = T_des_raw[1].linear(); //chest orientation
            

            // T_base_init_.translation() = T_des_raw[0].translation();
            // T_base_init_.linear() = T_des_raw[6].linear();
            is_first_ = false;
            std::cout<< "Tracker Start"<<std::endl;
            std::cout << T_base_init_.translation() << std::endl;
            std::cout << T_base_init_.linear() << std::endl;
            for(int i=1;i < NUM_TRACKER + 1;i++)
            {
                // absolute orientation diff world to model , world to tracker
                Quaterniond a(T_des_raw[i].linear());
                Quaterniond b(T_cur[i - 1].linear());

                a.normalize();
                b.normalize();

                R_inits_[i - 1] = a.inverse() * b;
                R_inits_[i - 1].normalize();
                // R_inits_[i - 1] = Quaterniond(1,0,0,0);
            }
        }
    }
    else
    {
        Quaterniond quat_des[NUM_TRACKER + 1];

        for(int i=1;i < NUM_TRACKER + 1;i++)
        {
            T_des[i].translation() = T_des_raw[i].translation() - T_base_init_.translation();
            
            Quaterniond tmp(T_des_raw[i].linear());
            tmp.normalize();

            quat_des[i] =  tmp * R_inits_[i - 1];
            quat_des[i].normalize();
        }

        //Calculate Task deltas
        VectorXd task_vector;
        task_vector.setZero(num_task_);

        //Construct Stacked Jacobians
        MatrixXd stack_jac;
        stack_jac.setZero(num_task_, model_->dof_count);

        int col_index = 0;
        // std::cout<<num_task_<<std::endl;

        for(int i=0; i<target_indices_.size(); i++)
        {
            // std::cout<<"des"<<manif::SO3d(quat_des[i + 1]).coeffs().transpose()<<std::endl;
            // std::cout<<"cur"<<manif::SO3d(AngleAxisd(T_cur[i].linear())).coeffs().transpose()<<std::endl;
            Vector6d vee;
            Vector3d rot_del;
            Vector3d trans_del;
            Eigen::Isometry3d desired_tf;
            desired_tf.translation() =  T_des[i + 1].translation();
            desired_tf.linear() =  quat_des[i + 1].toRotationMatrix();

            vee = manif::SE3d(desired_tf).lminus(manif::SE3d(T_cur[i])).coeffs();
            
            trans_del = vee.head(3);
            rot_del = vee.tail(3);
            
            // tmp.setZero();
            // tmp = - getPhi(T_cur[i].linear(), quat_des[i + 1].toRotationMatrix());
            // AngleAxisd aa(Quaterniond(quat_des[i + 1].toRotationMatrix() * T_cur[i].linear().transpose()).normalized());
            // rot_del = aa.angle() * aa.axis();
            // trans_del = T_des[i + 1].translation() -  aa.toRotationMatrix() * T_cur[i].translation();

            double rot_gain = 20.0;
            // Hands
            if(i == ModelMap::LHAND || i == ModelMap::RHAND)
            {
                task_vector.segment(col_index, 3) =  rot_del * gains_["hand"].at(0); //tmp.head(3);
                task_vector.segment(col_index + 3, 3) = trans_del * gains_["hand"].at(1);;

                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;
            }
            // Shoulders
            else if(i == ModelMap::LSH || i == ModelMap::RSH)
            {
                task_vector.segment(col_index, 3) =  rot_del * gains_["shoulder"].at(0);
                task_vector.segment(col_index + 3, 3) = trans_del * gains_["shoulder"].at(1);

                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;
            }
            // // Head
            // else if(target_indices_.at(i) == HEAD)
            // {
            //     task_vector.segment(col_index, 3) =  tmp * rot_gain; //tmp.head(3);
            //     task_vector.segment(col_index + 3, 3) = (T_des[i + 1].translation() - T_cur[i].translation())*50.0;

            //     stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
            //     col_index += 6;  
            // }
            // Chest(Thorax)
            else if(i == ModelMap::THORAX)
            {   
                task_vector.segment(col_index, 3) =  rot_del * gains_["thorax"].at(0);
                task_vector.segment(col_index + 3, 3) = trans_del * gains_["thorax"].at(1);
                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;
            }


        }

        // std::cout<<std::setprecision(1)<<stack_jac<<std::endl;
        // std::cout<<std::setprecision(5)<<task_vector.norm()<<std::endl;
        
        //Objective Problem
        MatrixXd Hess;
        VectorXd grad;
        Hess.setZero(model_->dof_count, model_->dof_count);
        grad.setZero(model_->dof_count);

        Hess = stack_jac.transpose() * stack_jac + 0.001*MatrixXd::Identity(model_->dof_count, model_->dof_count); 
        grad = - stack_jac.transpose() * task_vector;// - 0.01* prev_qdot_ ;

        //Constraint
        MatrixXd A;
        A.setZero(num_task_, model_->dof_count);
        A = stack_jac;

        VectorXd lb, ub, lbA, ubA; 
        lb.setZero(model_->dof_count);
        ub.setZero(model_->dof_count);

        lbA.setZero(num_task_);
        ubA.setZero(num_task_);

        lb.setConstant(-MAX);
        ub.setConstant(MAX);

        for (int i=0;i<model_->dof_count;i++)
        {
            lb(i) = max(20.0*( -3.14 - current_q_(i)), -MAX);
            ub(i) = min(20.0*( 3.14 - current_q_(i)), MAX);
        }

        // lb(1) = 0.0;
        // lb(2) = 0.0;
        // ub(1) = 0.0;
        // ub(2) = 0.0;


        lbA.setConstant(-MAX);
        ubA.setConstant(MAX);

        qp_.EnableEqualityCondition(0.0001);
        qp_.UpdateMinProblem(Hess, grad);
        qp_.UpdateSubjectToAx(A, lbA, ubA);
        qp_.UpdateSubjectToX(lb, ub);


        Eigen::VectorXd qpres, q_dot_solution;
        if(qp_.SolveQPoases(200, qpres))
        {
            q_dot_solution = qpres.segment(0, model_ -> dof_count);
        }
        else
        {
            q_dot_solution.setZero(model_ -> dof_count);
            qp_.InitializeProblemSize(model_ -> dof_count, num_task_);
        }

        prev_q_ = current_q_;
        prev_qdot_ = current_qdot_;

        // current_q_ = current_q_ * 0.5 + (current_q_ + q_dot_solution * 0.001)*0.5;
        current_qdot_ = q_dot_solution;
        current_q_ = current_q_ + q_dot_solution * 0.001;
        
        double error;
        error = (stack_jac*current_qdot_ - task_vector).norm();

        if (error > 1e-4)
        {
            // std::cout<<std::setprecision(4)<<q_dot_solution.transpose()<<std::endl;
            std::cout<<"Total Error: "<<std::setprecision(4)<<error<<std::endl;

            Vector3d tmp;
            AngleAxisd aa(Quaterniond(T_cur[2].linear()).normalized().inverse() * quat_des[3]);
            tmp = aa.angle() * aa.axis(); 
        
            // std::cout<<"\tlhand orient: "<<std::setprecision(4)<<tmp.transpose()<<std::endl;

            AngleAxisd bb(Quaterniond(T_cur[4].linear()).normalized().inverse() * quat_des[5]);
            tmp = bb.angle() * bb.axis(); 
            // std::cout<<"\trhand orient: "<<std::setprecision(4)<<tmp.transpose()<<std::endl;
            std::cout<<"\tlshoulder lin: "<< std::setprecision(4) << (T_des[2].translation() - T_cur[1].translation()).transpose()<<std::endl;
            std::cout<<"\tlhand     lin: "<< std::setprecision(4) << (T_des[3].translation() - T_cur[2].translation()).transpose()<<std::endl;

            std::cout<<"\trshoulder lin: "<< std::setprecision(4) << (T_des[4].translation() - T_cur[3].translation()).transpose()<<std::endl;
            std::cout<<"\trhand     lin: "<< std::setprecision(4) << (T_des[5].translation() - T_cur[4].translation()).transpose()<<std::endl;

            
            std::cout<<"\n===================================================================\n"<<std::endl;

        }
        // std::cout<<q_dot_solution.transpose()<<std::endl;
    }

}

void SkeletonKinectmatics::huberIK()
{
    std::cout<<"working"<<std::endl;
}
}