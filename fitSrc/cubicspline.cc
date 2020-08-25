/*
 * cubicspline.cc
 *
 *  Created on: May 17, 2020
 *      Author: zhian
 */
#include "fitHead/cubicspline.h"
#include <vector>
#include <iostream>
#include <Eigen/Dense>
using namespace std;

cubicspline::cubicspline(const std::vector<double>& t, const std::vector<double>& data, BoundaryCond bc)
	: _bc(bc), _t(t), _data(data)
{

}

cubicspline::cubicspline(const std::vector<double>& data, BoundaryCond bc)
	: _bc(bc), _data(data)
{
	for ( int i = 0; i < data.size(); i++) _t.push_back(i);
}

using Eigen::MatrixXd;
using Eigen::VectorXd;
void cubicspline::fit()
{
	int M = _t.size();
	int N = M-1;
	_h.resize(N);
	for ( int i = 1; i < _t.size(); i++)
		_h[i-1] = (_t[i]-_t[i-1]);
	MatrixXd A = MatrixXd::Zero(M, M);
	for ( int i = 0; i < M; i++)
		A(i, i) = 2;
	if ( _bc == TYPE1)
		A(0,1) = 1;
	else if ( _bc == TYPE2 )
		A(0,1) = 0;
	double s2 = 0;
	for ( int i = 1; i < N; i++)
	{
		double s = 1.0/(_h[i-1]+_h[i]);
		double u = _h[i]*s;
		A(i, i+1) = u;
		A(i, i-1) = 1.0-u;
	}
	if ( _bc == TYPE1)
		A(N, N-1) = 1;
	else if ( _bc == TYPE2 )
		A(N, N-1) = 0;
	VectorXd B(M);
	if ( _bc == TYPE1 )
		B[0] = 6.0*(_data[1]-_data[0])/_h[0];
	else if ( _bc == TYPE2 )
		B[0] = 0;
	for ( int i = 1; i < N; i++)
	{
		B[i] = 6.0*((_data[i+1]-_data[i])/_h[i]-(_data[i]-_data[i-1])/_h[i-1])/(_h[i]+_h[i-1]);
	}
	if ( _bc == TYPE1 )
		B[N] = 6.0*(_data[N]-_data[N-1])/_h[N-1];
	else if ( _bc == TYPE2 )
		B[N] = 0;

	VectorXd dx = A.fullPivLu().solve(B);
	_M.resize(M);
	for ( int i = 0; i< M; i++)
		_M[i] = dx[i];
}

double cubicspline::model(double t)
{
	int i = 0;
	int N = _t.size()-1;
	if ( t < _t[0] || t > _t[N] ) {
		cout << "parameter outside of model range!" << endl;
		return 0.0;
	}
	double c6 = 1.0/6.0;
	for ( i = 1; i <= N; i++ )
	{
		if ( t <= _t[i] ) {
			//cout << "t falls between" << i-1 << "  and " <<  i << endl;
			double d1 = _t[i] - t;
			double d2 = t - _t[i-1];
			double d13 = d1*d1*d1;
			double d23 = d2*d2*d2;
			double h = _h[i-1];
		    double h2 = h*h;
		    double ih = 1.0/h;
			double value = _M[i-1]*d13*c6*ih + _M[i]*d23*c6*ih+(_data[i-1]-_M[i-1]*h2*c6)*d1*ih+(_data[i]-_M[i]*h2*c6)*d2*ih;
			return value;
		}
	}
	cout << "parameter not in range! " << endl;
	return 0.0;
}
