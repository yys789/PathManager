#ifndef _CIRCLEFIT_H
#define _CIRCLEFIT_H

#include<Eigen/Dense>
#include <vector>
#include <QDebug>
using std::vector;

#include "matrix.h"

class circleFit
{
public:
    circleFit() {radius = 0; rms = 0;}
    virtual ~circleFit() {}
    void build();
    void getRms();
    void interpolation(double aglS, double interval, vector<RobotPos> &path);

    vector<RobotPos> pointLst;
    RobotPos cpos;
    double radius, rms;

private:
};

#endif // _CIRCLEFIT_H
