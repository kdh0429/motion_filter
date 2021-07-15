#include <iostream>
#include <memory>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>

using namespace RigidBodyDynamics;
using namespace Eigen;
class Base
{
public:
Base()
{
    std::cout<<"base init"<<std::endl;


}
~Base(){}

void printA()
{
    std::cout<<a_ -> dof_count<<std::endl;
}

protected:
void construtJoint()
{
    a_ = std::make_shared<Model>();
    a_-> AddBody(0, 
                 Math::SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()),
                 Joint(JointTypeEulerZYX),
                 Body(),
                 "j1"
                 );
}
std::shared_ptr<Model> a_;

};

class Derived: public Base
{
public:
Derived()
{
    std::cout<<"Derived init"<<std::endl;
    construtJoint();
}
~Derived(){}

void printB()
{
    std::cout<<a_ -> dof_count<<std::endl;
}

};

int main() {
    Base base;
    // base.printA();

    Derived derived;
    derived.printB();
    derived.printA();
    

    return 0;
}