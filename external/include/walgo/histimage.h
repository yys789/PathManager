/*
 * histimage.h
 *
 *  Created on: Jan 25, 2019
 *      Author: Hu XP
 */

#ifndef HISTIMAGE_H_
#define HISTIMAGE_H_
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <memory>
#include "utils.h"

using namespace std;

namespace walgo {
enum L_TYPE
{
	THRESH = 0,

};

enum TO_TYPE
{
    TO_DEFAULT = -1,
    TO_30,
    TO_31,
    TO_32,
    TO_D,
    TO_2_1
};

void bgr2gray(const cv::Mat &src,cv::Mat &dst,int flags= TO_DEFAULT);

void color2vector(const cv::Mat &input, vector<uchar> &vVec,int th= 15, cv::Size outPutSize= cv::Size(300,256));

bool hist(const cv::Mat& input,  std::vector<double>& output, std::map<std::string, int>& config,int ct= 0,bool doMediumBlur= false);
}
#endif /* HISTIMAGE_H_ */
