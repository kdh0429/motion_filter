/*
Data:  2021.06.21 
Autor: Donghyun Sung sdh1259@snu.ac.kr
*/

#ifndef TYPE_DEF_H
#define TYPE_DEF_H

#include "VR/matrix_3_4.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>
#include <logger.h>
#include <ctime>
#include <iomanip> 
#include <cmath>       /* tgamma */

namespace motion_filter
{

#define NUM_TRACKER 6
#define PI 3.141592653589793

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6x12d = Eigen::Matrix<double, 6, 12>;
using Matrix12x6d = Eigen::Matrix<double, 12, 6>;

using Matrix12d = Eigen::Matrix<double, 12, 12>;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;

}

#endif //TYPE_DEF_H