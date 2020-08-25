/*
 * smoothspline3d.cc
 *
 *  Created on: May 19, 2020
 *      Author: zhian
 */


#include "fitHead/smoothspline3d.h"
#include "fitHead/smoothspline.h"
#include <memory>

using namespace walgo;
using namespace std;
walgo::smoothspline3d::smoothspline3d(const vector<point3dd>& points, double lambda)
: _points(points), _lambda(lambda)
{
	for ( int i = 0; i < points.size(); i++) _t.push_back(i);
}

walgo::smoothspline3d::smoothspline3d(const vector<double>& t, const vector<point3dd>& points, double lambda)
: _t(t), _points(points), _lambda(lambda)
{

}

void walgo::smoothspline3d::fit()
{
	int M = _points.size();
	vector<double> x(M);
	vector<double> y(M);
	vector<double> z(M);
	for ( int i = 0; i <M; i++)
	{
		x[i] = _points[i]._x;
		y[i] = _points[i]._y;
		z[i] = _points[i]._z;
	}
	_xmodel = std::make_shared<smoothspline>(_t, x, _lambda);
	_ymodel = std::make_shared<smoothspline>(_t, y, _lambda);
	_zmodel = std::make_shared<smoothspline>(_t, z, _lambda);
	_xmodel->fit();
	_ymodel->fit();
	_zmodel->fit();
}

point3dd walgo::smoothspline3d::model(double t)
{
	point3dd p;
	p._x = _xmodel->model(t);
	p._y = _ymodel->model(t);
	p._z = _zmodel->model(t);
	return p;
}
