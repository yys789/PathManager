#ifndef HISTCALC_H
#define HISTCALC_H
#include <opencv2/opencv.hpp>

using namespace std;

namespace walgo
{
void hc(const cv::Mat &inPut);
int getBrightPtsProportion(const cv::Mat &inPut, int thresh= 222, bool doPrint= false);
}
#endif // HISTCALC_H
