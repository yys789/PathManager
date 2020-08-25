#ifndef _CURVEFIT_H
#define _CURVEFIT_H

#include<Eigen/Dense>
#include <vector>
using std::vector;

class curveFit
{
public:
    curveFit();
    virtual ~curveFit() {}
    void build(int degree);
    void getPoint(double t, vector<double> &point);

private:
    vector<double> curveFitX;
    vector<double> curveFitY;
    vector<double> curveFitZ;
    vector<vector<double>> pointLst;
    vector<double> lenLst;
    Eigen::MatrixXd Fy;
    Eigen::MatrixXd Fz;
    double Sx, Ex;

};

#endif // _CURVEFIT_H
