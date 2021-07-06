/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#include "motion_filter/skeleton_kinematics.hpp"

namespace motion_filter
{
SkeletonKinectmatics::SkeletonKinectmatics(DataHandler &dh): dh_(dh)
{
    std::string package_path = ros::package::getPath("motion_filter");
    std::string skel_path = package_path + "/config/skeleton.bvh";

    std::cout<<"init"<<std::endl;
    double scale = 1.0; //this to be modified

    skel_pub_ind_ = dh_.addPublisher<std_msgs::Float64MultiArray>("/skel", 100);

    for (int i=0;i<NUM_TRACKER+1;i++)
    {
        rpose_pub_ind_[i] = dh_.addPublisher<std_msgs::Float64MultiArray>("/Rposquat" + std::to_string(i), 100);
    }

    bvh11::BvhObject bvh(skel_path, scale);
    bvh.PrintJointHierarchy();
    constructJoint(bvh);
    constructModel();
    initParams();
}

SkeletonKinectmatics::~SkeletonKinectmatics()
{
    delete model_;
    std::cout<<"Destructor"<<std::endl;
}
void SkeletonKinectmatics::initParams()
{
    num_task_ =  target_indices_.size() * 6 - 3; //all pose except chest
    // num_task_ = 24;
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
    }

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
}

void SkeletonKinectmatics::publish()
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    std::vector<double> tmp;

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(int i=1;i<=NUM_BODY;i++)
    {
        tmp.resize(3);
        Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, i, Vector3d::Zero(), true);
        msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
        tmp.clear();

        if (i==HEAD)
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, i, end_effectors_.at(0), true);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
        if (i==LHAND)
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, i,  end_effectors_.at(1), true);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
        if (i==RHAND)
        {
            tmp.resize(3);
            Eigen::VectorXd::Map(&tmp[0], 3) = CalcBodyToBaseCoordinates(*model_, current_q_, i,  end_effectors_.at(2), true);
            msg.data.insert(msg.data.end(), tmp.begin(), tmp.end());
            tmp.clear();
        }
    }
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
        if (target_indices_.at(i) == HEAD)
            offset = end_effectors_.at(0);
        else if (target_indices_.at(i) == LHAND)
            offset = end_effectors_.at(1);
        else if (target_indices_.at(i) == RHAND)
            offset = end_effectors_.at(2);
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

void SkeletonKinectmatics::solveIK()
{

    Eigen::Isometry3d* T_des_raw;
    Eigen::Isometry3d T_des[NUM_TRACKER + 1]; //base. Thorax, lsh, lhand, rsh, rhand, Head;
    
    Eigen::Isometry3d* T_cur;

    T_des_raw = dh_.getObs();
    bool tracker_status = dh_.getTrackerStatus();

    computeJacobians(); //updateKinmeatics + jacobians
    T_cur = getCurrentTransform(); //Thorax, lsh, lhand, rsh, rhand head;

    T_base_init_.translation() = T_des_raw[1].translation(); //floating

    if (is_first_ )
    {
        if (tracker_status) //calculate front orientation and translation
        {
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

                R_inits_[i - 1] = b * a.inverse();
                R_inits_[i - 1].normalize();
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

            quat_des[i] =  R_inits_[i - 1] * tmp;
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
            Vector3d tmp;
            tmp.setZero();
            tmp = - getPhi(T_cur[i].linear(), quat_des[i + 1].toRotationMatrix());
            AngleAxisd aa(Quaterniond(quat_des[i + 1]* T_cur[i].linear()).normalized().inverse());
            tmp = aa.angle() * aa.axis();  

            double rot_gain = 0.2;
            // Hands
            if(target_indices_.at(i) == LHAND || target_indices_.at(i) == RHAND)
            {
                task_vector.segment(col_index, 3) =  tmp * rot_gain; //tmp.head(3);
                task_vector.segment(col_index + 3, 3) = (T_des[i + 1].translation() - T_cur[i].translation())*100.0;

                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;
            }
            // Shoulders
            else if(target_indices_.at(i) == LSH || target_indices_.at(i) == RSH)
            {
                task_vector.segment(col_index, 3) =  tmp * rot_gain; //tmp.head(3);
                // task_vector.segment(col_index + 3, 3) = (T_des[i + 1].translation() - T_cur[i].translation())*100.0;

                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;
            }
            // Head
            else if(target_indices_.at(i) == HEAD)
            {
                task_vector.segment(col_index, 3) =  tmp * rot_gain; //tmp.head(3);
                task_vector.segment(col_index + 3, 3) = (T_des[i + 1].translation() - T_cur[i].translation())*50.0;

                stack_jac.block(col_index, 0, 6, model_->dof_count) = jacs_.at(i).block(0, 0, 6, model_->dof_count);
                col_index += 6;  
            }
            // Chest(Thorax)
            else
            {     
                task_vector.segment(col_index, 3) = tmp.tail(3) * 1.0;
                stack_jac.block(col_index, 0, 3, model_->dof_count) = jacs_.at(i).block(0, 0, 3, model_->dof_count); 
                col_index += 3;
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
        // lb(0) = 0.0;
        // lb(1) = 0.0;
        // lb(2) = 0.0;
        lb(6) = 0.0;
        lb(7) = 0.0;
        lb(8) = 0.0;
        lb(18) = 0.0;
        lb(19) = 0.0;
        lb(20) = 0.0;

        ub.setConstant(MAX);
        // ub(0) = 0.0;
        // ub(1) = 0.0;
        // ub(2) = 0.0;
        ub(6) = 0.0;
        ub(7) = 0.0;
        ub(8) = 0.0;
        ub(18) = 0.0;
        ub(19) = 0.0;
        ub(20) = 0.0;

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
            std::cout<<"Error: "<<std::setprecision(4)<<error<<std::endl;
        }
        // std::cout<<q_dot_solution.transpose()<<std::endl;
    }

}

Eigen::Isometry3d* SkeletonKinectmatics::getCurrentTransform()
{

    Eigen::Isometry3d* T_cur = new Eigen::Isometry3d[6]; 
    int index = 0;
    for(auto id: target_indices_)
    {
        Vector3d local_offset;
        if (id==HEAD)
        {
            local_offset = end_effectors_.at(0);
        }
        else if (id==LHAND)
        {
            local_offset = end_effectors_.at(1);
        }
        else if (id==RHAND)
        {
            local_offset = end_effectors_.at(2);
        }
        else
        {
            local_offset.setZero();
        }

        T_cur[index].translation() = CalcBodyToBaseCoordinates(*model_, current_q_, id, local_offset, false);
        T_cur[index].linear() = CalcBodyWorldOrientation(*model_, current_q_, id, false).transpose();

        index ++;
    }
    return T_cur;
}

}