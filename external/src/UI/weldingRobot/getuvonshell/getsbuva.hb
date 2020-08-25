#ifndef GETSBUVA_H
#define GETSBUVA_H
#include "utils.h"
#include "histimage.h"
#include <opencv2/opencv.hpp>
//#include <string>
//#include <iostream>

//using namespace cv;
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

int GetTh(const cv::Mat& input,int& th,const int d= 3,const int max= 234);
//int getTth(const Mat& input,int& th,int& count);

cv::Mat GetKerLoG(const double size= 3,const int sigma= 128);

enum LINE_TYPE_2
{
    LT_D, //debug
    LT_ONE_Z, //
    LT_TWO_Z, //v
    //LT_E,//end
    LT_J, //j
    LT_VW, //vw
    LT_S, //s
    NUM_LINE_TYPES
};

class GetSBuvA
{
public:
//    static GetSBuvA* GetLineType(LINE_TYPE t);
//    static vector<GetSBuvA*> _obj;

    GetSBuvA(LINE_TYPE_2 t):_lineType(t)
    {
        if (!ReadConfig("getSBuv"+to_string(t)+".info", _config))  cout <<"warning! No getSBuv"+to_string(t)+".info !!!" << endl;
        _init();
        cout << "initialized "+alg_num+to_string(t) << endl;
        _log << "initialized "+alg_num+to_string(t) << endl;
    }

    virtual ~GetSBuvA() {}

    virtual void set_weld_on()
    {
        _weldOff= false;
    }

    virtual void set_weld_off()
    {
        _weldOff= true;
    }

    virtual void SetSide(bool left= true) {}

    virtual int AddImg(const cv::Mat& Input)= 0;

    virtual bool confirm(int s);

    virtual void clear();

    virtual bool getuv(map<int,cv::Point2d>& uv);

    cv::Point2d getuv()
    {
        int s= _uv.size();
        if(s > 0)
            return _uv[_uv.size()-1];
        else
            return cv::Point2d(0,0);
    }

    uchar tc;
    int _agl;
    string seamName;
    vector<double> GV;
    int hplus;
    //int aResize;

protected:


    virtual void _init();
    virtual int _GetRoi(const cv::Mat& image,int th,cv::Rect2i& roi);
    void _GetMaskRoi(cv::Mat &mask, cv::Rect2i &roim);
    //void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR);

    LINE_TYPE_2 _lineType;
    map<std::string, int> _config;
    string _logpd;
    int _callTimes; //
    //int _endORstart;
    //int _start;
    //int _end;
    fstream _log;
    const std::string alg_num= "alg_2020.02.20!";
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
    map<int,cv::Point2d> _uv;
    cv::Mat _kernel,_img_,_imgp;
    //int _lowerBound; //used for u
    //int _getwlr; //used for u

    bool _blackHole,_rightSide;
    bool _weldOff;
    int _weldCutR;
    int _ht;
    int _lengthen;
    int _laserLineWidthL,_laserLineWidthR;

};

class GetSBuvJ: virtual public GetSBuvA
{
public:
    GetSBuvJ(LINE_TYPE_2 t): GetSBuvA(t)
    {
        cout << " J type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double k0= 0.0168, double k1= 4.9);
};

class GetSBuvS: virtual public GetSBuvA
{
public:
    GetSBuvS(LINE_TYPE_2 t): GetSBuvA(t)
    {
        cout << "S type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:

};

class GetSBuvVW: virtual public GetSBuvA
{
public:
    GetSBuvVW(LINE_TYPE_2 t): GetSBuvA(t)
    {
        cout << "VW type." << endl;
        _planbl= false;
        _planbr= false;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double ratio1= 0.2, double k0= 0.0168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
    bool _planbl;
    bool _planbr;
};

//class GetSBuvZ: virtual public GetSBuvA
//{
//public:
//    GetSBuvZ(LINE_TYPE t): GetSBuvA(t)
//    {
//        cout << "Z type." << endl;
//    }

//    int AddImg(const cv::Mat& Input);

//protected:
////    int getxuv(vector<Vec4i>& selectedLinesL, vector<Vec4i>& selectedLinesR, Mat& img, Point2d& cp,double& c0l,double& c1279r,int SH= 0);
////    void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,int& SH);
//};

//class GetSBuvZT: virtual public GetSBuvA
//{
//public:
//    GetSBuvZT(LINE_TYPE t): GetSBuvA(t)
//    {
//        cout << "Z type." << endl;
//    }

//    int AddImg(const cv::Mat& Input);

//    bool getuv(map<int,cv::Point2d>& uv1,map<int,cv::Point2d>& uv2);

//protected:

//    map<int,cv::Point2d> _uv2;

//};

//class getXuvE: virtual public getXuvA
//{
//public:
//    getXuvE(LINE_TYPE t) : getXuvA(t)
//    {
//        cout << " E type." << endl;
//        getSide();
//    }

//    int addimg(const Mat& Input);

//    void getSide(bool left= true) {_leftSide= left;}

//protected:
//    bool _leftSide;
//};

#endif // GETSBUVA_H
