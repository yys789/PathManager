/*
 * polymodel3d.h
 *
 *  Created on: Nov 4, 2018
 *      Author: zhian
 */

#ifndef SEPMODEL3D_H_
#define SEPMODEL3D_H_

#include "walgo/modelp3d.h"
#include "walgo/polymodel1d.h"
#include <eigen3/Eigen/Dense>
#include "base/data_types.h"
#include <cmath>

using namespace std;
using namespace Eigen;

namespace walgo {
class SepModel3D : virtual public Modelp3D
{
public:
	SepModel3D();
    virtual ~SepModel3D() {}
    virtual void setParams(const std::vector<double>& params);
        virtual void build();
        virtual void init();
	virtual point3d<double> model(double t) const;
        virtual double getRMS() const;
        virtual double getMinT(double i, double & ti);
        virtual double sepMiss(double t1);
        virtual void setRealSE(int s,int e,bool mode=true);
        virtual void printTvec(std::vector<int>& tvec);
    double length;
    double curT;
    vector<vector<int>> cutVec;
    vector<double> rmslimit;
private:
        bool iterMode; ///
        int initCounts;  /// 原始点数
        int start_t;     /// real start t
        int end_t;     /// real end t
        vector<int> lastTvec;
	PolyModel1D  _modelX;
	PolyModel1D  _modelY;
	PolyModel1D  _modelZ;
};

class TracingModel
{
public:
    TracingModel();
    virtual ~TracingModel() {}
    virtual bool build(RobotPos base, vector<RobotPos> vec);
    virtual point3d<double> at(double t) const;
    virtual double getRMS() const;
    double distance;
private:
    double getDis(RobotPos p,RobotPos & pl)  const;
    int initCounts;  /// 原始点数
    vector<RobotPos> tickedVec;
    Vector4d v_y; /// a*x*x+b*x*z+c*y+d*y=1
    Vector4d v_z; /// e*x*x+f*x*z+g*x+h*z=1
};
}

#endif /* SEPMODEL3D_H_ */
