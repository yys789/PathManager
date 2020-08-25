#ifndef SELF_ADAPTION_H
#define SELF_ADAPTION_H
#include <opencv2/opencv.hpp>
#include "histimage.h"
#include "utils.h"
#include "walgo/lineanalysis.h"
#include "getsbuva.h"  /// 基础算法接口
#include <fstream>

using namespace std;

static uchar elementArray33[3][3]= {{0, 1, 0},
                                    {1, 1, 1},
                                    {0, 1, 0},};
static uchar elementArray35[3][5]= {{0, 0, 1, 0, 0},
                                    {1, 1, 1, 1, 1},
                                    {0, 0, 1, 0, 0},};
static uchar elementArray57[5][7]= {{0, 0, 0, 1, 0, 0, 0},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {1, 1, 1, 1, 1, 1, 1},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {0, 0, 0, 1, 0, 0, 0},};

//int GetTh(const cv::Mat& input,int& th,const int d= 3,const int max= 234);
////int getTth(const Mat& input,int& th,int& count);

//cv::Mat GetKerLoG(const double size= 3,const int sigma= 128);

class SelfAdaption
{
public:
    SelfAdaption()
    {
        if (!ReadConfig("selfAdaption.info", _config))  cout <<"warning! No selfAdaption.info !!!" << endl;
        _init();
        _log << "initialized "+alg_num << endl;
    }

    virtual ~SelfAdaption() {}

    virtual void set_weld_on()
    {
        _weldOff= false;
    }

    virtual void set_weld_off()
    {
        _weldOff= true;
    }

    virtual void set_shift(vector<double> &params)
    {
        for(auto &p:params) _params.push_back(p);
    }

    virtual void SetSide(bool left= false)
    {
        _rightSide= (left)? false : true;
    }

    virtual void set_center(int x,int y)
    {
        _icenter.x= x;
        _icenter.y= y;
    }

    virtual int AddImg(const cv::Mat& Input);

    //virtual bool confirm(int s);

    //virtual void clear();

    //virtual bool getuv(cv::Point2d &uvc, cv::Point2d &uvl, cv::Point2d &uvr);

    bool getuv(cv::Point2d& uvc,cv::Point2d& uvl,cv::Point2d& uvr)
    {
        int s= _uv.size();
        if(s > 0){
            uvc= _uv[_uv.size()-1];
            uvl= _uvl[_uvl.size()-1];
            uvr= _uvr[_uvr.size()-1];
            return true;
        }
        else {
            uvc= cv::Point2d(-10,-10);
            uvl= cv::Point2d(-10,-10);
            uvr= cv::Point2d(-10,-10);
            return false;
        }
    }

    //uchar tc;
    string seamName;
    //int aResize;

protected:

    virtual void _init();
    virtual int _GetRoi(const cv::Mat& image,int th,cv::Rect2i& roi);
    bool _GetMaskRoi(cv::Mat &mask, cv::Rect2i &roim);
    //void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR);

    map<std::string, int> _config;
    string _logpd;
    int _callTimes; //
    //int _endORstart;
    //int _start;
    //int _end;
    fstream _log;
    const std::string alg_num= "alg_2020.03.22!";
    int _imgCols,_imgRows;
    cv::Point _icenter;
    int _xMin,_xMax;
    int _roiWidthShift;
    int _roiHeightShift;
    int _lineHeight,_lineWidth;
    int _doMediumBlur = 1;
    int _mediumBlurSize = 5;

    int _imgNum,_debugNum;
    int _save;
    int _d2d;
    int _center_x_p;

    int _houghThresh;
    int _houghMinLineLength;
    int _houghMaxLineGap;

    int _angleRange;
    int _binsize;
    int _bluegv,_greengv;

//    vector<int> _debugVec;
//    vector<Mat> _imgRoivec;
//    vector<Vec4i> _bottomLineVec;
    map<int,double> _gapLen;
    map<int,cv::Point2d> _uv,_uvl,_uvr;
    vector<double> _params;
    cv::Mat _kernel,_img_,_imgp;

    bool _blackHole,_rightSide;
    bool _weldOff;
    int _weldCutR;
    int _ht;
    int _lengthen;
    int _laserLineWidthL,_laserLineWidthR;
    int _agl;
    //vector<double> GV;
    int _hplus;
    int _gap;
    bool GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.9, double k0= 0.0868, double k1= 4.9);
};

#endif // SELF_ADAPTION_H
