/*
 * cubicspline3d.h
 *
 *  Created on: May 19, 2020
 *      Author: zhian
 */

#ifndef SMOOTHSPLINE3D_H_
#define SMOOTHSPLINE3D_H_

#include "point.h"
#include <vector>
#include <memory>
#include "smoothspline.h"
namespace walgo {
class smoothspline3d
{
public:
	smoothspline3d(const std::vector<double>& t, const std::vector<point3dd>& points, double lambda);
	smoothspline3d(const std::vector<point3dd>& points, double lambda);
	void fit();
	point3dd model(double t);
protected:
	const std::vector<point3dd>& _points;
	std::vector<double> _t;
	std::shared_ptr<smoothspline> _xmodel;
	std::shared_ptr<smoothspline> _ymodel;
	std::shared_ptr<smoothspline> _zmodel;
	double _lambda;
};

};

#endif /* CUBICSPLINE3D_H_ */
