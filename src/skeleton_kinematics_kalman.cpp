/*
Skeleton Kinematic

Data:  2021.07.01 
Autor: Donghyun Sung sdh1259@snu.ac.kr

*/
#include "motion_filter/skeleton_kinematics_kalman.hpp"

namespace motion_filter
{

KalmanKinect::KalmanKinect(DataHandler &dh)
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

KalmanKinect::~KalmanKinect()
{}

void KalmanKinect::initParams()
{
    std::cout<<"KalmanKinect Init"<<std::endl;
    solver_type_ = "KalmanKinect";

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

    std::cout<<"noise setting"<<std::endl;
    //process noise setting
    fQ_.resize(nq * 2, nq * 2);
    fQ_.setZero();

    MatrixXd Iq = MatrixXd::Identity(nq, nq);

    fQ_.block(0, 0, nq, nq) = Iq * h_* h_* h_ / 3.0;
    fQ_.block(0, nq, nq, nq) = Iq * h_* h_ / 2.0;
    fQ_.block(nq, 0, nq, nq) = Iq * h_* h_ / 2.0;
    fQ_.block(nq, nq, nq, nq) = Iq * h_;

    // fQ_.block(0, 0, nq, nq) = Iq * h_* h_;
    // fQ_.block(0, nq, nq, nq) = Iq * h_;
    // fQ_.block(nq, 0, nq, nq) = Iq * h_;
    // fQ_.block(nq, nq, nq, nq) = Iq;

    fQ_ = fQ_ * pnoise_ * pnoise_;
    // fQ_.block(nq, nq, nq, nq) = Iq * 0.5;


    fR_.resize(num_task_, num_task_);
    fR_.setZero();

    for (int i=0;i< 5 ;i++)
    {
        fR_.block(i*6, i*6, 6, 6) = single_measurement_noise_.asDiagonal();
    }


}
void KalmanKinect::parseToml(std::string &toml_path)
{
    std::cout<<"KalmanKinect TomL"<<std::endl;

    auto data = toml::parse(toml_path);
    bvh_file_ = toml::find<std::string>(data, "bvh_file");
    target_indices_ = toml::find<std::vector<int>>(data, "target_index");
    is_log_ = toml::find<bool>(data, "logging");

    
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

    auto& ekf = toml::find(data, "ekf");

    h_ = toml::find<double>(ekf, "time_step");
    pnoise_ = toml::find<double>(ekf, "process_noise");

    tmp = toml::find<std::vector<double>>(ekf, "measurement_noise");
    single_measurement_noise_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
    single_measurement_noise_ = single_measurement_noise_.array().square();

    tmp = toml::find<std::vector<double>>(ekf, "init_state_std");
    init_state_std_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
    init_state_std_ = init_state_std_.array().square();

}

void KalmanKinect::solveIK()
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
            //init Kalman
            state_.mean.setZero(model_->dof_count * 2);
            state_.cov.resize(model_->dof_count * 2, model_->dof_count * 2);
            
            for (int i = 0 ;i < model_->dof_count; i++)
            {
                state_.cov(i, i) = init_state_std_(0);
                state_.cov(i + model_->dof_count, i + model_->dof_count) = init_state_std_(1);
                
            }
            // init QP
            qp_.InitializeProblemSize(model_ -> dof_count * 2, num_task_); 
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
        
        // std::cout<<"predict"<<std::endl;
        int nq = model_->dof_count;
        
        MatrixXd F;
        F.resize(nq * 2, nq * 2);
        F.setIdentity();
        F.block(0, nq, nq, nq) = MatrixXd::Identity(nq, nq) * h_;

        state_.mean = F * state_.mean;
        // state_bar.cov = lambda_ * F * state_.cov * F.transpose() + fQ_;
        state_.cov = F * state_.cov * F.transpose() + fQ_;

        //update with respect to prior states
        computeJacobians(state_.mean.head(nq), state_.mean.tail(nq));
        T_cur = getCurrentTransform(state_.mean.head(nq));

        //Calculate Task deltas
        VectorXd task_vector;
        task_vector.setZero(num_task_);
        // std::cout<<num_task_<<std::endl;

        for(int i=0; i < target_indices_.size() - 1; i++) //except head
        {
            Vector6d vee; //wv
            Vector3d rot_del;
            Vector3d trans_del;

            manif::SE3d::Jacobian J_T, J_tau_vw, J_tau_wv;
            manif::SE3Tangentd vee_tangent; //vw

            Eigen::Isometry3d desired_tf;
            desired_tf.translation() =  T_des[i + 1].translation();
            desired_tf.linear() =  quat_des[i + 1].toRotationMatrix();

            //right jacobian
            vee_tangent = manif::SE3d(desired_tf).lminus(manif::SE3d(T_cur[i])).coeffs();

            vee.head(3) = vee_tangent.coeffs().tail(3); //w
            vee.tail(3) = vee_tangent.coeffs().head(3); //v

            //observation matrix
            MatrixXd H;
            H.setZero(6, model_->dof_count * 2);
            H.block(0, 0, 6, model_->dof_count) = jacs_.at(i);

            fR_ = single_measurement_noise_.asDiagonal();

            MatrixXd Z = H * state_.cov * H.transpose() + fR_;
            MatrixXd K = state_.cov * H.transpose() * Z.inverse();

            state_.cov = state_.cov - K * Z * K.transpose();
            state_.mean = state_.mean + K * vee;

            // std::cout<< "dz   :\t" << vee.transpose() << std::endl;
            std::cout<< "dq    :\t" <<std::setprecision(4) <<state_.mean.tail(nq).maxCoeff()<<state_.mean.tail(nq).minCoeff() << std::endl;
            std::cout<< "K     :\t" <<std::setprecision(4) << state_.cov.maxCoeff() << std::endl;

        }
        std::cout<< "============================\n\n";

        //assigan to current_q_
        current_q_ = state_.mean.head(model_->dof_count);
        current_qdot_ = state_.mean.tail(model_->dof_count);

        // Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
        // eigensolver.compute(Z);

        // std::cout<< "Z     :\t" << Z.diagonal().maxCoeff() << std::endl;
    }

        // //Objective Problem
        // MatrixXd Hess;
        // VectorXd grad;

        // Hess.setZero(model_->dof_count * 2, model_->dof_count * 2);
        // grad.setZero(model_->dof_count);

        // MatrixXd p_bar_inv = state_bar.cov.inverse();
        // MatrixXd R_inv = fR_.inverse();
        
        // Hess =  p_bar_inv + H.transpose() * R_inv * H;
        // grad = - p_bar_inv * state_bar.mean - H.transpose() * R_inv * (task_vector + right_jacobian_stack_inverse * stack_jac_prior * state_bar.mean.head(model_->dof_count));

        // //Constraint
        // MatrixXd A;
        // A.setZero(num_task_, model_->dof_count * 2);
        // A.block(0, model_->dof_count, num_task_, model_->dof_count) = stack_jac_prior;

        // VectorXd lb, ub, lbA, ubA; 
        // lb.setZero(model_->dof_count * 2);
        // ub.setZero(model_->dof_count * 2);

        // lbA.setZero(num_task_);
        // ubA.setZero(num_task_);

        // //q (-pi, pi)
        // for (int i=0;i<model_->dof_count;i++)
        // {
        //     lb(i) = -3.14;
        //     ub(i) = 3.14;
        // }

        // //qdot (-pi/2.0 , pi/2.0)
        // for (int i=model_->dof_count;i < model_->dof_count*2; i++)
        // {
        //     lb(i) = -3.14;
        //     ub(i) = 3.14;
        // }

        // lbA.setConstant(-MAX);
        // ubA.setConstant(MAX);

        // qp_.EnableEqualityCondition(0.0001);
        // qp_.UpdateMinProblem(Hess, grad);
        // qp_.UpdateSubjectToX(lb, ub);
        // qp_.UpdateSubjectToAx(A, lbA, ubA);


        // Eigen::VectorXd qpres, state_solution;

        // //update mean
        // if(qp_.SolveQPoases(200, qpres))
        // {
        //     state_solution = qpres.segment(0, model_ -> dof_count * 2);
        //     state_.mean = state_solution;
        // }
        // else
        // {
        //     qp_.InitializeProblemSize(model_ -> dof_count * 2, num_task_);
        //     state_.mean = state_bar.mean;
        // }
        //update cov


  


    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

}

}