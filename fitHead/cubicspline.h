/*
 * cubicspline.h
 *
 *  Created on: May 17, 2020
 *      Author: zhian
 */

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <vector>

class cubicspline
{
public:
	enum BoundaryCond {
		TYPE1 = 0,
		TYPE2,
		TYPE3
	};
	cubicspline(const std::vector<double>& t, const std::vector<double>& data, BoundaryCond bc=TYPE2);
	cubicspline(const std::vector<double>& data, BoundaryCond bc=TYPE2);
	void fit();
	double model(double t);
protected:
	BoundaryCond _bc;
	std::vector<double> _t;
	std::vector<double> _data;

	std::vector<double> _M;
	std::vector<double> _h;
};


#endif /* CUBICSPLINE_H_ */
