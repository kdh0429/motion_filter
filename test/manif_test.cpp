#include <manif/manif.h>
#include <Eigen/Dense>

using namespace Eigen;

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
    // s[i] = s[i].normalized();
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}

int main()
{
    std::srand((unsigned int) time(0));

    auto state = manif::SE3d::Identity();
    std::cout<< state.coeffs().transpose()<<std::endl;

    // auto s1 = manif::SO3d::Random();
    // auto s2 = manif::SO3d::Random();

    auto s1 = manif::SO3d(0, 0, 0);
    auto s2 = manif::SO3d(1.0, 1.0, 1.5);
    

    Matrix3d R1, R2;
    R1 = Quaterniond(s1.coeffs()).toRotationMatrix();
    R2 = Quaterniond(s2.coeffs()).toRotationMatrix();
    
    AngleAxisd aalocal(R2.transpose() * R1);
    AngleAxisd aaglobal(R1 * R2.transpose());
    
    std::cout<<"getphi            : "<<-getPhi(R2, R1).transpose()<<std::endl;
    std::cout<<"manif(local)      : "<<(manif::SO3d(Quaterniond(R1)) - manif::SO3d(Quaterniond(R2))).coeffs().transpose()<<std::endl;
    std::cout<<"manif(global)     : "<<(manif::SO3d(Quaterniond(R1)).lminus(manif::SO3d(Quaterniond(R2)))).coeffs().transpose()<<std::endl;
    std::cout<<"angleaxis(local)  : "<< aalocal.angle() * aalocal.axis().transpose()<<std::endl;
    std::cout<<"angleaxis(global) : "<< aaglobal.angle() * aaglobal.axis().transpose()<<std::endl;
    std::cout<<"scale             : "<< -getPhi(R2, R1).transpose().array() /  (aaglobal.angle() * aaglobal.axis().transpose()).array()<<std::endl;


}