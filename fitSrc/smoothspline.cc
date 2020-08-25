/*
 * smoothspline.cc
 *
 *  Created on: May 19, 2020
 *      Author: zhian
 */

#include "fitHead/smoothspline.h"
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace walgo;

using Eigen::MatrixXd;
using Eigen::VectorXd;

walgo::smoothspline::smoothspline(const std::vector<double>& t, const std::vector<double>& data, double lambda)
	: _t(t), _data(data), _lambda(lambda)
{

}

walgo::smoothspline::smoothspline(const std::vector<double>& data, double lambda)
	: _data(data), _lambda(lambda)
{
	for ( int i = 0; i < data.size(); i++) _t.push_back(i);
}

void walgo::smoothspline::fit()
{
	int M = _data.size();
	vector<double> h(M-1);
	vector<double> ih(M-1);
	for ( int i = 1; i < _t.size(); i++)
	{
		h[i-1] = (_t[i]-_t[i-1]);
		ih[i-1] = 1.0/h[i-1];
	}
	MatrixXd D = MatrixXd::Zero(M-2, M);
	MatrixXd T = MatrixXd::Zero(M-2, M-2);
	double c13 = 1.0/3.0;
	double c16 = 1.0/6.0;
	for ( int i = 0; i < M-3; i++)
	{
		D(i,i) = ih[i];
		D(i,i+1) = -ih[i]-ih[i+1];
		D(i,i+2) = ih[i+1];
		T(i,i) = (h[i]+h[i+1])*c13;
		T(i,i+1) = h[i+1]*c16;
		T(i+1,i) = T(i,i+1);
	}
	T(M-3, M-3) = (h[M-3]+h[M-2])*c13;
	MatrixXd C = D.transpose()*T.inverse()*D;
	MatrixXd I = MatrixXd::Identity(M, M);
	MatrixXd A = I + _lambda*C;
	VectorXd B(M);
	for ( int i = 0; i < _data.size(); i++) B(i) = _data[i];
	VectorXd Y = A.fullPivLu().solve(B);
	vector<double> y(M);
	for (int i = 0; i < M; i++) y[i] = Y(i);
	_interp = std::make_shared<cubicspline>(_t, y);
	_interp->fit();
}
