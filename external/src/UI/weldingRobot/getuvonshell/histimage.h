/*
 * histimage.h
 *
 *  Created on: Jan 25, 2019
 *      Author: Hu XP
 */

#ifndef HISTIMAGE_H_2
#define HISTIMAGE_H_2
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <memory>
#include "utils.h"

enum L_TYPE
{
	THRESH = 0,
};

enum TO2_TYPE
{
    TO2_DEFAULT = -1,
    TO2_30,
    TO2_31,
    TO2_32,
    TO2_D,
    TO2_2_1
};

void Bgr2Gray(const cv::Mat &src,cv::Mat &dst,int flags= TO2_DEFAULT);

bool hist(const cv::Mat& input,  std::vector<double>& output, std::map<std::string, int>& config,int ct= 0,bool doMediumBlur= false);

#endif /* HISTIMAGE_H_ */
