#ifndef _CIRCLE_H
#define _CIRCLE_H

#include <opencv2/opencv.hpp>
#include "base/data_types.h"

class Circle
{
public:
   Circle( const cv::Mat &mat ) ;
   void reset( const cv::Mat &mat ) ;
   UV center() const;
   void draw( cv::Mat &mat, const cv::Scalar &calar = cv::Scalar(255, 0, 0) ) ;
   static UV center( const cv::Mat &mat ) ;
   static UV  center( cv::Mat &mat ) ;
private:
   UV _center;
   cv::RotatedRect _box ;
} ;

#endif
