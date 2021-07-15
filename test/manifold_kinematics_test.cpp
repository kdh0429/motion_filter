#include <manif/manif.h>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <iomanip> 

using namespace Eigen;
using namespace manif;
using namespace RigidBodyDynamics;

#define EE 3

void updateKinematics(Model* model, const VectorXd& q, const VectorXd& qdot)
{
  UpdateKinematicsCustom(*model, &q, &qdot, NULL);
}

MatrixXd computeJacobians(Model* model, const VectorXd& q, const VectorXd& qdot)
{
    updateKinematics(model, q, qdot);
    MatrixXd jac_wv(6, q.size()), jac_vw(6, q.size());
    jac_wv.setZero();
    jac_vw.setZero();
    CalcPointJacobian6D(*model, q, EE, Vector3d::Zero(), jac_wv, false);

    jac_vw.block(0, 0, 3, q.size()) = jac_wv.block(3, 0, 3, q.size());//v
    jac_vw.block(3, 0, 3, q.size()) = jac_wv.block(0, 0, 3, q.size());//w

    return jac_vw;
}

Eigen::Isometry3d getCurrentTransform(Model* model, const Eigen::VectorXd& q)
{
    VectorXd qdot = VectorXd::Zero(q.size());
    updateKinematics(model, q, qdot);

    Eigen::Isometry3d T_cur; 
    T_cur.translation() = CalcBodyToBaseCoordinates(*model, q, EE, Vector3d::Zero(), false);
    T_cur.linear() = CalcBodyWorldOrientation(*model, q, EE, false).transpose();
    return T_cur;
}

int main()
{
    std::srand((unsigned int) time(0));

    Model* model;
    model = new Model();
    Body rbdl_body = Body();
    Joint rbdl_joint = Joint(JointTypeEulerZYX);
    // build model
    for(int i=1;i<=3;i++)
    {
        Vector3d offset;
        offset << 1.0, 0.0, 0.0;
        model -> AddBody(
            i - 1,
            Math::SpatialTransform(Matrix3d::Identity(), offset),
            rbdl_joint,
            rbdl_body,
            "j"+std::to_string(i)
        );

    }
    std::cout<<Utils::GetModelDOFOverview(*model)<<std::endl;
    std::cout<<Utils::GetModelHierarchy(*model)<<std::endl;

    //sanity check for manifold operation
    SE3Tangentd a = SE3Tangentd::Random();
    SE3Tangentd b = SE3Tangentd::Random();


    std::cout<<"a:"<<a.coeffs().transpose()<<"\tb:"<<b.coeffs().transpose()<<std::endl;
    std::cout<<"a+b : "<<(a+b).coeffs().transpose()<<std::endl;
    std::cout<<"b+a : "<<(b+a).coeffs().transpose()<<std::endl;
    std::cout<<"a-b : "<<(a-b).coeffs().transpose()<<std::endl;
    std::cout<<"b-a : "<<(b-a).coeffs().transpose()<<std::endl;
    
    
    
    /**
     * Test Log(Y h(x)^-1 Exp(-J_f dq)) ~ Log(Y h(x)^-1) - J_r^-1 J_f dq
     * 1.forward kinematics approximation 
     * 
     *      h(q + dq) = h(q) + J(dq) = Exp(Jdq) * h(q) (global)
     * 
     * 2.
     * 
     **/
    MatrixXd jac;
    VectorXd random_q, random_q_perturb, q_dot;
    int nq = model->dof_count;

    random_q = VectorXd::Random(nq)*1.0;
    random_q_perturb = VectorXd::Random(nq)*0.001;
    q_dot = VectorXd::Zero(nq);
    
    jac = computeJacobians(model, random_q, q_dot);
    auto T = getCurrentTransform(model, random_q);
    auto T_d = getCurrentTransform(model, random_q + random_q_perturb);
    
    SE3d Tm = SE3d(T);
    SE3d Tm_d = SE3d(T_d);


    
    SE3d Tm_d_aprox_global = Tm.lplus(SE3Tangentd(jac * random_q_perturb)); //Exp(Jdq) * h(q)
    SE3d Tm_d_aprox_local = Tm.rplus(SE3Tangentd(jac * random_q_perturb));  //h(q) * Exp(Jdq)
    
    std::cout<<"Forward Kinematics Approximation"<<std::endl;
    std::cout<<std::setprecision(4)<<random_q.transpose()<<"\t"<<random_q_perturb.transpose()<<std::endl;
    std::cout<<"\tTarget : "<<std::setprecision(4)<<Tm_d.coeffs().transpose()<<std::endl;
    std::cout<<"\tGlobal : "<<std::setprecision(4)<<Tm_d_aprox_global.coeffs().transpose()<<std::endl;
    std::cout<<"\tLocal  : "<<std::setprecision(4)<<Tm_d_aprox_local.coeffs().transpose()<<std::endl;


    SE3d Tm_d_noise_global = SE3d(Tm_d).lplus(SE3Tangentd::Random()*0.01);

    SE3Tangentd true_noise = Tm_d_noise_global.lminus(Tm_d); //Log(Y*h(x)^-1)
    //Log(Y* (Exp(Jdq) * h(x_bar))^-1) = Log(Y* h(x_bar)^-1 * Exp(Jdq)^-1)
    SE3Tangentd apporx1 = (Tm_d_noise_global * Tm.inverse()).lminus(SE3Tangentd(jac * random_q_perturb).exp()); 

    //Log(Y* (Exp(Jdq) * h(x_bar))^-1) = Log(Y* h(x_bar)^-1)  - J_r * J * dq

    manif::SE3d::Jacobian J_T, J_right;
    manif::SE3d::Identity().rplus( Tm_d_noise_global.lminus(Tm), J_T, J_right);

    SE3Tangentd apporx2 = Tm_d_noise_global.lminus(Tm) - J_right.inverse() * SE3Tangentd(jac * random_q_perturb);

    std::cout<<"Inverse Kinematics Approximation(GLOBAL)"<<std::endl;
    // std::cout<<std::setprecision(4)<<random_q.transpose()<<"\t"<<random_q_perturb.transpose()<<std::endl;
    std::cout<<"\tTarget_direct  : "<<std::setprecision(4)<<true_noise.coeffs().transpose()<<std::endl;
    std::cout<<"\tTarget_FK  : "<<std::setprecision(4)<<Tm_d_noise_global.lminus(Tm_d_aprox_global).coeffs().transpose()<<std::endl;



    std::cout<<"\tApprox1 : "<<std::setprecision(4)<<apporx1.coeffs().transpose()<<std::endl;
    std::cout<<"\tApprox2 : "<<std::setprecision(4)<<apporx2.coeffs().transpose()<<std::endl;
    std::cout<<"\tWrongAp : "<<std::setprecision(4)<<(Tm_d_noise_global.lminus(Tm) - SE3Tangentd(jac * random_q_perturb)).coeffs().transpose()<<std::endl;




    Vector3d x = Vector3d::Random();
    MatrixXd M;
    M = x * x.transpose();
    std::cout<<x.transpose() * x<<std::endl;
    std::cout<<M<<std::endl;
    




    return 0;
}