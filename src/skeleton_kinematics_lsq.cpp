/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#include "motion_filter/skeleton_kinematics_lsq.hpp"

namespace motion_filter
{
LSQKinect::LSQKinect(DataHandler &dh)
: SkeletonKinectmatics(dh)
{
    std::string package_path = ros::package::getPath("motion_filter");
    std::string toml_path = package_path + "/config/kinematics.toml";

    parseToml(toml_path);
    std::string skel_path = package_path + "/config/" + bvh_file_;
    double scale = 1.0;
    bvh11::BvhObject bvh(skel_path, scale);
    // bvh.PrintJointHierarchy();
    constructJoint(bvh);
    constructModel();    
    initParams();

    //logger setting
    if(is_log_)
    {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S_",timeinfo);
        std::string dtimestr(buffer);

        std::string dirpath = package_path + "/data/" + dtimestr;
        std::string datafile = solver_type_ + "_joint_data.csv";

        std::vector<std::string> keys;
        keys.resize(model_ -> dof_count + 1);
        keys.at(0) = "time(s)";
        for (int i=1;i<model_ -> dof_count + 1;i++)
        {
            keys.at(i) = "q" + std::to_string(i);
        }
        q_logger_ = new CsvLogger(dirpath, datafile, keys);

        start_time_ = std::chrono::high_resolution_clock::now();
    }
}

LSQKinect::~LSQKinect()
{}

void LSQKinect::initParams()
{
    std::cout<<"LSQ Init"<<std::endl;
    solver_type_ = "LSQKinect";
    int nq = model_->dof_count;

    num_task_ =  target_indices_.size() * 6 - 6; //all pose except head

    current_q_.resize(nq, 1);
    current_qdot_.resize(nq, 1);

    current_q_.resize(nq, 1);
    prev_qdot_.resize(nq, 1);

    jacs_.resize(target_indices_.size());

    for (int i=0; i<target_indices_.size();i++)
        jacs_.at(i).resize(6, nq);
    
    T_base_init_.linear().setIdentity();
    current_q_.setZero();
    current_qdot_.setZero();

}
void LSQKinect::parseToml(std::string &toml_path)
{
    std::cout<<"LSQ TomL"<<std::endl;

    auto data = toml::parse(toml_path);
    bvh_file_ = toml::find<std::string>(data, "bvh_file");
    target_indices_ = toml::find<std::vector<int>>(data, "target_index");
    is_log_ = toml::find<bool>(data, "logging");

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

    auto& lsq = toml::find(data, "lsq");
    alpha_ = toml::find<double>(lsq, "alpha");
    h_ = toml::find<double>(lsq, "dt");

    auto& gains = toml::find(lsq, "gains");
    gains_["hand"] = toml::find<std::vector<double>>(gains, "hand");
    gains_["shoulder"] = toml::find<std::vector<double>>(gains, "shoulder");
    gains_["thorax"] = toml::find<std::vector<double>>(gains, "thorax");

}
void LSQKinect::solveIK()
{

    auto begin = std::chrono::high_resolution_clock::now();

    Eigen::Isometry3d* T_des_raw;
    Eigen::Isometry3d T_des[NUM_TRACKER + 1]; //base. Thorax, lsh, lhand, rsh, rhand, Head;
    
    Eigen::Isometry3d* T_cur;

    T_des_raw = dh_.getFiltered();
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
            // init QP
            qp_.InitializeProblemSize(model_ -> dof_count, num_task_); 

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

        current_qdot_ = q_dot_solution;
        current_q_ = current_q_ + q_dot_solution * alpha_;
        
        // double error;
        // error = (stack_jac*current_qdot_ - task_vector).norm();

    //     if (error > 1e-4)
    //     {
    //         // std::cout<<std::setprecision(4)<<q_dot_solution.transpose()<<std::endl;
    //         std::cout<<"Total Error: "<<std::setprecision(4)<<error<<std::endl;

    //         Vector3d tmp;
    //         AngleAxisd aa(Quaterniond(T_cur[2].linear()).normalized().inverse() * quat_des[3]);
    //         tmp = aa.angle() * aa.axis(); 
        
    //         // std::cout<<"\tlhand orient: "<<std::setprecision(4)<<tmp.transpose()<<std::endl;

    //         AngleAxisd bb(Quaterniond(T_cur[4].linear()).normalized().inverse() * quat_des[5]);
    //         tmp = bb.angle() * bb.axis(); 
    //         // std::cout<<"\trhand orient: "<<std::setprecision(4)<<tmp.transpose()<<std::endl;
    //         std::cout<<"\tlshoulder lin: "<< std::setprecision(4) << (T_des[2].translation() - T_cur[1].translation()).transpose()<<std::endl;
    //         std::cout<<"\tlhand     lin: "<< std::setprecision(4) << (T_des[3].translation() - T_cur[2].translation()).transpose()<<std::endl;

    //         std::cout<<"\trshoulder lin: "<< std::setprecision(4) << (T_des[4].translation() - T_cur[3].translation()).transpose()<<std::endl;
    //         std::cout<<"\trhand     lin: "<< std::setprecision(4) << (T_des[5].translation() - T_cur[4].translation()).transpose()<<std::endl;

            
    //         std::cout<<"\n===================================================================\n"<<std::endl;

    //     }
    //     // std::cout<<q_dot_solution.transpose()<<std::endl;
    // }
    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}

}

}