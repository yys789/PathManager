#ifndef _DEFINES_H_
#define _DEFINES_H_

//#include "include/defines.h"
#include <Eigen/Dense>


struct RobotPos
{
    double x, y, z, a, b, c;
};
struct RobotAxle
{
    double a1,a2,a3,a4,a5,a6;
};



void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos);

void pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m);

void PosRotate(RobotPos cpos, RobotPos &pos, double agl);

#endif
