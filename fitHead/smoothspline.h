/*
 * smoothspline.h
 *
 *  Created on: May 19, 2020
 *      Author: zhian
 */

#ifndef SMOOTHSPLINE_H_
#define SMOOTHSPLINE_H_
#include <vector>
#include <memory>
#include "cubicspline.h"

namespace walgo {
class smoothspline
{
public:
	smoothspline(const std::vector<double>& t, const std::vector<double>& data, double lambda);
	smoothspline(const std::vector<double>& data, double lambda);
	void fit();
	double model(double t)   { return _interp->model(t); }
protected:
	double _lambda;
	std::vector<double> _t;
	std::vector<double> _data;
	std::shared_ptr<cubicspline> _interp;
};

};

#endif /* SMOOTHSPLINE_H_ */
