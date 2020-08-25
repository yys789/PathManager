#include "tools/circle.h"
#include <opencv2/opencv.hpp>

Circle::Circle(const cv::Mat &mat)
{
   reset( mat );
}

void Circle::reset( const cv::Mat &mat )
{
    using namespace cv;
    using namespace std;
    Mat gray;
    gray = mat.clone();
   // GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
    medianBlur(gray, gray, 19);

    double minp= 0, maxp=0;

    minMaxIdx(gray, &minp, &maxp);
    int thred = ( minp+maxp )/2;
//    Scalar sg= sum(gray);

//    int thred = sg[0]/(gray.rows*gray.cols);

    Mat bImg;
    threshold( gray, bImg, thred, 255, 0);
    vector<vector<Point> > contours_;
    findContours(bImg, contours_, RETR_LIST, CHAIN_APPROX_NONE);
    /// add 2020-02-28
    vector<vector<Point> > contours;
    for(auto c_cur=contours_.begin();c_cur<contours_.end();c_cur++)
    {
        vector<Point> c = *c_cur;
        int cs = c.size();
        if(cs > 300 || cs < 100)
            continue;
        double avgX=0,avgY=0;
        bool border = false;
        for(auto & p : c)
        {
            if(abs(p.x-mat.cols)<cs/4 || p.x<cs/4 ||
                    abs(p.y-mat.rows)<cs/4 || p.y<cs/4)
            {
                border = true;
                break;
            }
            avgX += p.x;
            avgY += p.y;
        }
        if(border)
        {
            continue;
        }
        avgX /= cs;
        avgY /= cs;
        bool check = true;
        for(auto t_cur=contours_.begin();t_cur<contours_.end();t_cur++)
        {
            if(t_cur==c_cur)
                continue;
            if(t_cur->size() > 200)
                continue;
            vector<Point> t = *t_cur;
            for(auto q : t)
            {
                if((q.x-avgX)*(q.x-avgX)+(q.y-avgY)*(q.y-avgY) < cs*cs)
                {
                    check = false;
                    break;
                }
            }
            if(!check)
                break;
        }
        if(check)
            contours.push_back(c);
    }
    if(contours.size() == 0 || contours.size() > 1)
    {
        cv::imwrite("jz/bImg.png",bImg);
    }
    /// end 2020-02-28
    int nMax = 0 ;
    for(size_t i = 1; i < contours.size(); ++i)
    {
       if ( contours[nMax].size() < contours[i].size() ) {
          nMax = i ;
       }
    }
    if ( contours.size() ) {
       Mat pointsf;
       Mat(contours[nMax]).convertTo(pointsf, CV_32F);
       _box = fitEllipse(pointsf);
       //std::cout << "circle:" << _box.center.x << ":" << _box.center.y << std::endl ;
       _center = {_box.center.x, _box.center.y} ;
    }
    else {
      _center = {-1, -1} ;
   }
}

UV Circle::center() const
{
    return _center;
}

UV Circle::center( cv::Mat &mat )
{
   return Circle(mat).center() ;
}

UV Circle::center( const cv::Mat &mat )
{
   return Circle(mat).center() ;
}

void Circle::draw( cv::Mat &mat, const cv::Scalar &calar )
{
   if ( mat.channels() == 1 ) {
      //std::cout << "灰度图" << std::endl ;
      cv::cvtColor(mat, mat, CV_GRAY2BGR) ;
   }
   cv::ellipse(mat, _box, calar, 1, cv::LINE_AA);
}
