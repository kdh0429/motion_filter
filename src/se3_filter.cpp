/*
Lie Group(SE3) (innovation-saturated) Extended Kalman filter LG (IS) EKF
constant velocity(differential kinematic)

Data:  2021.06.21 
Autor: Donghyun Sung sdh1259@snu.ac.kr
*/
#include <motion_filter/se3_filter.hpp>

namespace motion_filter
{

double gaussianPdf(VectorXd x, VectorXd mu, MatrixXd sigma)
{
    double det = sigma.determinant();
    double dim = (double)x.size();
    double normalize_const = 1.0 / pow(2.0 * PI , dim/2.0) / sqrt(det);

    double exponents = -0.5 * (x - mu).transpose() * sigma.inverse() * (x - mu);
    return exp(exponents) * normalize_const;

}
double studentTPdf(VectorXd x, VectorXd mu, MatrixXd sigma, int df)
{
    double dim = (double)x.size();
    double det = sigma.determinant();

    double numerator = tgamma((dim + (double)df)/2.0);
    double exponents = (x - mu).transpose() * sigma.inverse() * (x - mu);

    double denominator = tgamma(((double)df )/ 2.0) * pow(PI * df, dim/2.0) * \
                         sqrt(det) * pow(1.0 + exponents/(double)df, (df + dim)/2.0 );
    
    return numerator / denominator;
}

VectorXd pusedoMeasurementForManifold(Vector6d m_diff, Matrix6d gauss_cov, Matrix6d cauchy_cov, double weight)
{
    double gauss_ll = gaussianPdf(m_diff, Vector6d::Zero(), gauss_cov);
    double cauchy_ll = studentTPdf(m_diff, Vector6d::Zero(), cauchy_cov, 1);

    Vector8d features;
    features.setZero();

    features(0) = (1.0 - weight) * gauss_ll;
    features.segment<6>(1) = (1.0 - weight) * gauss_ll * m_diff;
    features(7) = weight * cauchy_ll;

    double denum = (1.0 - weight) * gauss_ll + weight * cauchy_ll;

    return features/denum; 
}



MerweScaledSigmaPoints::MerweScaledSigmaPoints(int n, double alpha, double beta, double kappa)
: n_(n), alpha_(alpha), beta_(beta), kappa_(kappa)
{
    lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;
    computeWeights();
}

MerweScaledSigmaPoints::~MerweScaledSigmaPoints()
{

}

void MerweScaledSigmaPoints::computeWeights()
{
    int np = 2 * n_ + 1;
    Wn.resize(np);
    Wc.resize(np);

    for(int i=0;i<np;i++)
    {
        if (i==0)
        {
            Wn(i) = lambda_ / ( (double)n_ +  lambda_);
            Wc(i) = lambda_ / ( (double)n_ +  lambda_) + 1.0 - alpha_*alpha_ + beta_;
        }
        else
        {
            double tmp = 0.5 / ((double)n_ + lambda_);
            Wn(i) = tmp;
            Wc(i) = tmp;
        }
    }
    // std::cout<<Wn.transpose()<<std::endl;
    // std::cout<<Wc.transpose()<<std::endl;
    
}

Eigen::MatrixXd MerweScaledSigmaPoints::getSigmaPoints(Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
    Eigen::MatrixXd Lu = (((double)n_ + lambda_) * cov).llt().matrixL().toDenseMatrix();
    Eigen::MatrixXd X_set;

    // std::cout<<Lu<<std::endl;
    X_set.resize(2*n_ + 1, n_);

    X_set.block(0, 0, 1, n_) = mean.transpose();

    X_set.block(1, 0, n_, n_) = Lu.rowwise() + mean.transpose();

    X_set.block(n_ + 1, 0, n_, n_) = (- Lu).transpose().rowwise() + mean.transpose();

    return X_set;

}


SE3Filter::SE3Filter(ros::NodeHandle &nh, int tracker_id, double dt, bool verbose)
{
    tracker_id_ = tracker_id;
    dt_ = dt;
    verbose_ = verbose;

    // if (tracker_id_ < NUM_TRACKER)
    //     pose_pub_ = nh.advertise<VR::matrix_3_4>("/FTRACKER" + std::to_string(tracker_id), 100);
    // else
    //     pose_pub_ = nh.advertise<VR::matrix_3_4>("/FHMD", 100);

    // vel_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Fspvel" + std::to_string(tracker_id), 100);
    fpos_quat_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Fposquat" + std::to_string(tracker_id), 100);
    rpos_quat_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Rposquat" + std::to_string(tracker_id), 100);

    std::string package_path = ros::package::getPath("motion_filter");
    std::string toml_path = package_path + "/config/filter_param.toml";
    parseToml(toml_path);

    Matrix6d I6 = Eigen::MatrixXd::Identity(6, 6);
    Matrix6d O6;
    O6.setZero();

    H_ << I6, O6;
    if (verbose_)
    {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S_",timeinfo);
        std::string dtimestr(buffer);

        std::string dirpath = package_path + "/data/" + dtimestr;
        std::string rdatafile = id2name_.at(tracker_id) + "_raw.csv";
        std::string fdatafile = id2name_.at(tracker_id) + "_filter.csv";

        rlogger_ = new CsvLogger(dirpath, rdatafile, keys_);
        flogger_ = new CsvLogger(dirpath, fdatafile, keys_);
    }

}

SE3Filter::~SE3Filter()
{
    std::cout << "SE3Filter Destructor" << std::endl;
}

Eigen::Isometry3d SE3Filter::getTransform()
{
    Vector7d pos_quat = T_.coeffs();
    Eigen::Isometry3d T_iso;
    T_iso.translation() = pos_quat.head(3);
    Eigen::Quaterniond q(pos_quat(6), pos_quat(3), pos_quat(4), pos_quat(5));
    T_iso.linear() = q.normalized().toRotationMatrix();

    return T_iso;
}

 
void SE3Filter::step(Eigen::Isometry3d T_m, bool tracker_status)
{
    if (is_first_)
    {
        is_first_ = false;
        T_ = manif::SE3d(T_m);
        V_.setZero();
        std::cout<<tracker_id_ <<' '<< algo[key_]<<" Filter Start"<<std::endl;
    }
    else
    {
        if (key_ == 0)
        {
            predict();
            ekfUpdate(T_m);
        }
        else if (key_ == 1)
        {
            predict();
            isekfUpdate(T_m);
        }
        else if (key_ == 2)
        {
            lpf(T_m);
        }
        else if (key_ == 3)
        {
            ukfKalman(T_m);
        }
        else
            std::logic_error("Unknown filter type");
    }
    if (is_publish_)
        publish(tracker_status);
}

void SE3Filter::lpf(Eigen::Isometry3d T_m)
{
    T_raw_ = manif::SE3d(T_m);
    T_ = T_ + (T_raw_ - T_) * alpha_;
}
void SE3Filter::restart()
{
    sigma_ = sigma_init_;
    epsilon_ = epsilon_init_;
    is_first_ = true;
}

void SE3Filter::predict()
{
    // ROS_INFO("Predict");
    T_ = T_.plus(manif::SE3Tangentd(V_ * dt_), J_T_, J_V_);

    Matrix12d F;
    Matrix12d Q;

    F << J_T_, J_V_ * dt_, 
         Matrix6d::Zero(), Matrix6d::Identity();
    Q << J_V_ * process_cov_ * J_V_.transpose() * dt_ * dt_, process_cov_ * J_V_.transpose() * dt_,
         J_V_ * process_cov_ * dt_, process_cov_;

    P_ = F * P_ * F.transpose() + Q;
}
void SE3Filter::ekfUpdate(Eigen::Isometry3d T_m)
{
    // ROS_INFO("Update");
    T_raw_ = manif::SE3d(T_m);


    Vector6d z = (T_raw_ - T_).coeffs();

    Matrix6d Z = H_ * P_ * H_.transpose() + N_;
    Matrix12x6d K = P_ * H_.transpose() * Z.inverse();

    Vector12d dx = K * z;
    // Vector12d dx = K * z_clip;

    T_ = T_ + manif::SE3Tangentd(dx.head(6));
    // Heuristic clip
    // Vector6d dx_clip = (dx.tail(6).array().min(sigma_.array()).cwiseMax(-sigma_.array())).matrix();
    V_ = V_ + dx.tail(6);

    P_ = P_ - K * Z * K.transpose();

}
void SE3Filter::ukfKalman(Eigen::Isometry3d T_m)
{
    //-------------------------------------------predict next state-------------------------------------
    //noisy free update
    manif::SE3d T_mean = T_ + manif::SE3Tangentd(V_ * dt_);
    Vector6d V_mean = V_;

    //mean state propgation
    Eigen::MatrixXd zeta_mat = sigma_points_12d_ -> getSigmaPoints(Vector12d::Zero(), P_);

    Matrix12d P_full;
    Matrix6d P_TT, P_TV, P_VV;
    P_full.setZero();
    P_TT.setZero();
    P_TV.setZero();
    P_VV.setZero();
    
    for(int i=1;i<zeta_mat.rows();i++)
    {
        Vector12d zeta = zeta_mat.row(i).transpose();
        manif::SE3d T_sigma = T_ + manif::SE3Tangentd((V_ + zeta.tail(6))*dt_ + zeta.head(6)); //theotherically proof?
        Vector6d V_diff = zeta.tail(6);
        Vector6d T_diff = (T_sigma - T_mean).coeffs();
        // std::cout<<T_diff * T_diff.transpose() * 2<<std::endl;
        // std::cout<<(sigma_points_12d_ -> Wc)<<std::endl;
        
        P_TT += (sigma_points_12d_ -> Wc)(i) * T_diff * T_diff.transpose();
        P_TV += (sigma_points_12d_ -> Wc)(i) * T_diff * V_diff.transpose();
        P_VV += (sigma_points_12d_ -> Wc)(i) * V_diff * V_diff.transpose();
    }

    Eigen::MatrixXd pnoise_mat = sigma_points_6d_ -> getSigmaPoints(Vector6d::Zero(), process_cov_);

    for(int i=1;i<pnoise_mat.rows();i++)
    {
        Vector6d pnoise = pnoise_mat.row(i).transpose();
        manif::SE3d T_sigma = T_ + manif::SE3Tangentd((V_ + pnoise)*dt_); //theotherically proof?
        Vector6d V_diff = pnoise;
        Vector6d T_diff = (T_sigma - T_mean).coeffs();

        P_TT += sigma_points_6d_ -> Wc(i) * T_diff * T_diff.transpose();
        P_TV += sigma_points_6d_ -> Wc(i) * T_diff * V_diff.transpose();
        P_VV += sigma_points_6d_ -> Wc(i) * V_diff * V_diff.transpose();
    }

    P_full.block(0, 0, 6, 6) = P_TT;
    P_full.block(0, 6, 6, 6) = P_TV;
    P_full.block(6, 0, 6, 6) = P_TV.transpose();
    P_full.block(6, 6, 6, 6) = P_TT;

    P_ = P_full;
    T_ = T_mean;
    V_ = V_mean;
    // std::cout<<"predict"<<std::endl;

    //-----------------------------------------------predict mesurement distribution-----------------------
    Matrix6d P_yy; //measurement
    P_yy.setZero();
    manif::SE3d T_y = T_;

    Eigen::MatrixXd mnoise_mat = sigma_points_12d_ -> getSigmaPoints(Vector12d::Zero(), P_);

    for(int i=1;i<mnoise_mat.rows();i++)
    {
        Vector12d mnoise = mnoise_mat.row(i).transpose();

        manif::SE3d T_sigma = T_y + manif::SE3Tangentd(mnoise.head(6)); //theotherically proof?

        Vector6d T_diff = (T_sigma - T_mean).coeffs();

        P_yy += sigma_points_12d_ -> Wc(i) * T_diff * T_diff.transpose();

    }
    P_yy += N_;
    // std::cout<<"body predict"<<std::endl;

    //----------------------------------------------update with pseudo measurement using feature------------
    T_raw_ = manif::SE3d(T_m);
    
    VectorXd f_mean(8);
    MatrixXd P_ff(8,8);
    MatrixXd P_TV_y(12, 6);
    P_TV_y.setZero();

    f_mean.setZero();
    P_ff.setZero();
    MatrixXd P_TV_f;

    MatrixXd f_mat(mnoise_mat.rows(), 8);
    for(int i=0; i<mnoise_mat.rows(); i++)
    {
        Vector12d mnoise = mnoise_mat.row(i).transpose();
        manif::SE3d T_sigma = T_y + manif::SE3Tangentd(mnoise.head(6)); //theotherically proof?
        Vector6d T_diff = (T_sigma - T_y).coeffs();

        // std::cout<<"feature manifold cal"<<std::endl;
        VectorXd f_tmp = pusedoMeasurementForManifold(T_diff, P_yy, cauchy_cov_, cauchy_weight_);
        // std::cout<<"feature assign"<<std::endl;

        f_mat.row(i) = f_tmp.transpose();

        P_TV_y += (sigma_points_12d_ -> Wc)(i) * mnoise * T_diff.transpose();
    }
    std::cout<<"feature predict"<<std::endl;

    f_mean = ((sigma_points_12d_ -> Wn).asDiagonal() * f_mat).colwise().sum().transpose();
    MatrixXd f_diff = f_mat.rowwise() - f_mean.transpose();
    P_ff = f_diff.transpose() * ((sigma_points_12d_ -> Wc).asDiagonal()) * f_diff;

    // how to compute P_TV_f (12, 8)
    P_TV_f = mnoise_mat.transpose() * ((sigma_points_12d_ -> Wc).asDiagonal()) * f_diff;
    // MatrixXd K = P_ff.colPivHouseholderQr().solve(P_TV_f.transpose()).transpose();
    VectorXd f_meaurement = pusedoMeasurementForManifold((T_raw_ - T_y).coeffs(), P_yy, cauchy_cov_, cauchy_weight_);
    // VectorXd dx = K * (f_meaurement - f_mean);
    
    // MatrixXd K = P_yy.colPivHouseholderQr().solve(P_TV_y.transpose()).transpose();
    MatrixXd K = P_TV_y* P_yy.inverse();
    VectorXd dx = K * (T_raw_ - T_).coeffs();

    T_ = T_ + manif::SE3Tangentd(dx.head(6));
    V_ = V_ + dx.tail(6);

    // P_ = P_ - K * P_ff * K.transpose();
    P_ = P_ - K * P_yy * K.transpose();
    

    std::cout<<"V"<<V_.transpose()<<std::endl;

}

void SE3Filter::isekfUpdate(Eigen::Isometry3d T_m)
{
    // ROS_INFO("Update");
    // std::cout<<tracker_id_<<" Traw:\t"<<T_m.linear().determinant()<<std::endl;

    T_raw_ = manif::SE3d(T_m);
    Vector6d z = (T_raw_ - T_).coeffs();
    Vector6d z_clip = (z.array().min(sigma_.array().sqrt()).max(-sigma_.array().sqrt())).matrix();
    // std::cout<<tracker_id_<<" sigma_:\t"<<sigma_.array().sqrt().matrix().transpose()<<std::endl;
    // std::cout<<tracker_id_<<" z_clip:\t"<<z_clip.transpose()<<std::endl;
    

    Matrix6d Z = H_ * P_ * H_.transpose() + N_;
    Matrix12x6d K = P_ * H_.transpose() * Z.inverse();

    Vector12d dx = K * z_clip;

    T_ = T_ + manif::SE3Tangentd(dx.head(6));
    V_ = V_ + dx.tail(6);

    P_ = P_ - K * Z * K.transpose();
    dynamicClipUpdate(z);
    // std::cout<<"T   :\t"<<T_.coeffs().transpose()<<std::endl;

}
void SE3Filter::dynamicClipUpdate(Vector6d z)
{
    sigma_ = (lambda1_.array() * sigma_.array() + gamma1_.array() * epsilon_.array() * Eigen::exp(-epsilon_.array())).matrix();
    epsilon_ = (lambda2_.array() * epsilon_.array() + gamma2_.array() * z.array().square()).matrix();
}
void SE3Filter::publish(bool tracker_status)
{
    //VR message default
    // pose_pub_.publish(isometry3d2VRmsg(getTransform()));

    //pos quat
    std_msgs::Float64MultiArray fposquat; //a.k.a std::vector<double>
    fposquat.data.clear();
    fposquat.data.resize(7);
    Eigen::VectorXd::Map(&fposquat.data[0], 7) = T_.coeffs();
    fpos_quat_pub_.publish(fposquat);

    std_msgs::Float64MultiArray rposquat; //a.k.a std::vector<double>
    rposquat.data.clear();
    rposquat.data.resize(7);
    Eigen::VectorXd::Map(&rposquat.data[0], 7) = T_raw_.coeffs();

    rpos_quat_pub_.publish(rposquat);

    if (verbose_)
    {
        fposquat.data.push_back((double)tracker_status);
        rposquat.data.push_back((double)tracker_status);
        flogger_ -> writeRows(fposquat.data);
        rlogger_ -> writeRows(rposquat.data);
    }

    // //spatial velocity
    // std_msgs::Float64MultiArray spvel; //a.k.a std::vector<double>
    // spvel.data.clear();
    // spvel.data.resize(6);
    // Eigen::VectorXd::Map(&spvel.data[0], 6) = V_;
    // vel_pub_.publish(spvel);

}
void SE3Filter::parseToml(std::string &toml_path)
{
    auto data = toml::parse(toml_path);
    //default kalman filter params
    key_ = toml::find<int>(data, "key");
    is_publish_ = toml::find<bool>(data, "publish");

    if (key_==0)
    {
        auto& ekf = toml::find(data, "EKF");

        std::vector<double> tmp = toml::find<std::vector<double>>(ekf, "init_state_std");
        Vector12d init_state_std = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        init_state_std = init_state_std.array().square();
        P_ = init_state_std.asDiagonal();

        tmp = toml::find<std::vector<double>>(ekf, "process_noise");
        Vector6d process_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        process_noise = process_noise.array().square();
        process_cov_ = process_noise.asDiagonal();

        tmp = toml::find<std::vector<double>>(ekf, "measurement_noise");
        Vector6d measurement_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        measurement_noise = measurement_noise.array().square();
        N_ = measurement_noise.asDiagonal();

        sigma_ << 1.0, 1.0, 1.0, 3.14, 3.14, 3.14;
        sigma_ = sigma_ * dt_;
    } 
    else if (key_ == 1)
    {
        //innovation saturated kalman filter params
        auto& isekf = toml::find(data, "ISEKF");

        std::vector<double> tmp = toml::find<std::vector<double>>(isekf, "init_state_std");
        Vector12d init_state_std = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        init_state_std = init_state_std.array().square();
        P_ = init_state_std.asDiagonal();

        tmp = toml::find<std::vector<double>>(isekf, "process_noise");
        Vector6d process_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        process_noise = process_noise.array().square();
        process_cov_ = process_noise.asDiagonal();

        tmp = toml::find<std::vector<double>>(isekf, "measurement_noise");
        Vector6d measurement_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        measurement_noise = measurement_noise.array().square();
        N_ = measurement_noise.asDiagonal() * dt_ * dt_;

        tmp = toml::find<std::vector<double>>(isekf, "sigma_init");
        sigma_init_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        sigma_init_ = sigma_init_ * dt_ * dt_;

        tmp = toml::find<std::vector<double>>(isekf, "epsilon_init");
        epsilon_init_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());

        std::vector<double> tmp1 = toml::find<std::vector<double>>(isekf, "lambda1");
        lambda1_ << tmp1.at(0), tmp1.at(0), tmp1.at(0), tmp1.at(1), tmp1.at(1), tmp1.at(1);

        tmp1 = toml::find<std::vector<double>>(isekf, "lambda2");
        lambda2_ << tmp1.at(0), tmp1.at(0), tmp1.at(0), tmp1.at(1), tmp1.at(1), tmp1.at(1);

        tmp1 = toml::find<std::vector<double>>(isekf, "gamma1");
        gamma1_ << tmp1.at(0), tmp1.at(0), tmp1.at(0), tmp1.at(1), tmp1.at(1), tmp1.at(1);

        tmp1 = toml::find<std::vector<double>>(isekf, "gamma2");
        gamma2_ << tmp1.at(0), tmp1.at(0), tmp1.at(0), tmp1.at(1), tmp1.at(1), tmp1.at(1);

        sigma_ = sigma_init_;
        epsilon_ = epsilon_init_;
    }

    else if (key_ == 2)
    {
        //low pass filter first order
        auto& lpf = toml::find(data, "LPF");
        alpha_ = toml::find<double>(lpf, "alpha");
    }

    else if (key_ == 3)
    {
        //robust lie group uncented kalman filter
        auto &rukf = toml::find(data, "RUKF");

        sigma_points_12d_ = new MerweScaledSigmaPoints(manif::SE3d::DoF * 2, 
                                                   toml::find<double>(rukf, "alpha"),
                                                   toml::find<double>(rukf, "beta"),
                                                   toml::find<double>(rukf, "kappa"));
        sigma_points_6d_  = new MerweScaledSigmaPoints(manif::SE3d::DoF, 
                                                    toml::find<double>(rukf, "alpha"),
                                                    toml::find<double>(rukf, "beta"),
                                                    toml::find<double>(rukf, "kappa"));
                                                   
        std::vector<double> tmp = toml::find<std::vector<double>>(rukf, "init_state_std");
        Vector12d init_state_std = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        init_state_std = init_state_std.array().square();
        P_ = init_state_std.asDiagonal();

        tmp = toml::find<std::vector<double>>(rukf, "process_noise");
        Vector6d process_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        process_noise = process_noise.array().square();
        process_cov_ = process_noise.asDiagonal();

        tmp = toml::find<std::vector<double>>(rukf, "measurement_noise");
        Vector6d measurement_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        measurement_noise = measurement_noise.array().square();
        N_ = measurement_noise.asDiagonal();

        tmp = toml::find<std::vector<double>>(rukf, "cauchy_noise");
        Vector6d cauchy_noise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
        cauchy_noise = cauchy_noise.array().square();
        cauchy_cov_ = cauchy_noise.asDiagonal();

        cauchy_weight_ = toml::find<double>(rukf, "cauchy_noise_weight");
    }

}


}
