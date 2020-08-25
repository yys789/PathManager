/*
 * pathfit.h
 *
 *  Created on: Feb 8, 2020
 *      Author: zhian
 */

#ifndef PATHFIT_H_
#define PATHFIT_H_
#include <vector>
#include "point.h"

namespace walgo
{

double pathfitobjfunc(unsigned n, const double* x, double* grad, void* funcData);

class pathfit
{
public:
	void fit(const std::vector<point3dd>& points, point3dd p0, point3dd v0);

	point3dd model(double t) const;
	const std::vector<point3dd>& getPoints() const { return _points; }
	point3dd getP0() const { return _p0; }
	point3dd getV0() const { return _v0; }
	point3dd getP1() const { return this->model(1.0d); }
	point3dd getV1() const;
private:
	std::vector<double> _coef;

	std::vector<point3dd> _points;
	point3dd _p0;
	point3dd _v0;
};

}
#endif /* PATHFIT_H_ */
