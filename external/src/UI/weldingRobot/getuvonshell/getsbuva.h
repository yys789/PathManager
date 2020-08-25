#ifndef GETSBUVA_H
#define GETSBUVA_H
#include "utils.h"
#include "histimage.h"
#include <opencv2/opencv.hpp>
//#include <string>
//#include <iostream>

//using namespace cv;
using namespace std;


static uchar elementArray_33[3][3]= {{0, 1, 0},
                                    {1, 1, 1},
                                    {0, 1, 0},};
static uchar elementArray_35[3][5]= {{0, 0, 1, 0, 0},
                                    {1, 1, 1, 1, 1},
                                    {0, 0, 1, 0, 0},};
static uchar elementArray_57[5][7]= {{0, 0, 0, 1, 0, 0, 0},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {1, 1, 1, 1, 1, 1, 1},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {0, 0, 0, 1, 0, 0, 0},};
static int elementArray135[13][5]= {{-1, -1, -1, -1, -1},
                                     {-1, -1, -1, -1, -1},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {4, 4, 4, 4, 4},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0},
                                     {-1, -1, -1, -1, -1},
                                     {-1, -1, -1, -1, -1},};

int GetTh(const cv::Mat& input,int& th,const int d= 3,const int max= 234);
//int getTth(const Mat& input,int& th,int& count);

cv::Mat GetKerLoG(const double size= 3,const int sigma= 128);

enum LINE_TYPE
{
    LT_D, //debug
    LT_ONE_Z, //
    LT_TWO_Z, //v
    //LT_E,//end
    LT_J, //j3
    LT_VW, //vw
    LT_M, //M
    LT_EE, //E
    LT_S, //s
    LT_WW, //w
    LT_JS, //Js
    LT_SE, //10
    LT_I, //I
    LT_WC,
    NUM_LINE_TYPES
};

LINE_TYPE AlgoType(string stype);

class GetSBuvA
{
public:
//    static GetSBuvA* GetLineType(LINE_TYPE t);
//    static vector<GetSBuvA*> _obj;
    GetSBuvA(LINE_TYPE t):_lineType(t)
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

    virtual void set_shift(vector<double> &params)
    {
        for(auto &p:params) _params.push_back(p);
    }

    virtual void SetSide(bool left= true) {}

    virtual int AddImg(const cv::Mat& Input)= 0;

    virtual bool confirm(int s);

    virtual void clear();

    virtual bool getuv(map<int,cv::Point2d>& uv);

    virtual bool getuvd(map<int,cv::Point2d>& uv){;}

    cv::Point2d getuv()
    {
        int s= _uv.size();
        if(s > 0)
            return _uv[_uv.size()-1];
        else
            return cv::Point2d(0,0);
    }

    virtual bool GetWhatYouWant(vector<double> &wyw){;}

    uchar tc;
    string seamName;
    //int aResize;

protected:

    virtual void _init();
    virtual int _GetRoi(const cv::Mat& image,int th,cv::Rect2i& roi);
    void _GetMaskRoi(cv::Mat &mask, cv::Rect2i &roim);
    bool _GetMaskRoiB(cv::Mat &mask, cv::Rect2i &roim);
    //void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR);

    LINE_TYPE _lineType;
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
};

class GetSBuvJ: virtual public GetSBuvA
{
public:
    GetSBuvJ(LINE_TYPE t): GetSBuvA(t)
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

class GetSBuvJS: virtual public GetSBuvA
{
public:
    GetSBuvJS(LINE_TYPE t): GetSBuvA(t)
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
    GetSBuvS(LINE_TYPE t): GetSBuvA(t)
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
    GetSBuvVW(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "VW type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double ratio1= 0.2, double k0= 0.10168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
    bool _planbl,_planbr;
};

class GetSBuvM: virtual public GetSBuvA
{
public:
    GetSBuvM(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "M type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double ratio1= 0.2, int minLimit= 0, int maxLimit= 100, double k0= 0.0168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
};

class GetSBuvW: virtual public GetSBuvA
{
public:
    GetSBuvW(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "W type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

    bool GetWhatYouWant(vector<double> &wyw)
    {
        if(_imgNum < 1) return false;
        wyw.push_back(_uvl[_imgNum-1].x);
        wyw.push_back(_uvl[_imgNum-1].y);
        wyw.push_back(_uvr[_imgNum-1].x);
        wyw.push_back(_uvr[_imgNum-1].y);
        return true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double ratio1= 0.2,double discount= 1.0, double k0= 0.0951248, double k1= 1.9);
//    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
//                      double ratio0= 0.8, double ratio1= 0.2, int minLimit= 0, int maxLimit= 100, double k0= 0.0168, double k1= 4.9);
    bool get_laser_width_thresh(const cv::Mat& input,int& bottom,int& top,int dpt,
                              cv::Point2d vn,double vx,double cx,double k,double c,int& th,double& shift,vector<cv::Point>& pts1r);
    map<int,cv::Point2d> _uvl,_uvr;
    cv::Rect _mroil,_mroir;
    bool _rightok,_leftok;
};

class GetSBuvE: virtual public GetSBuvA
{
public:
    GetSBuvE(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "E type." << endl;
        _kernel= (cv::Mat(3,5,CV_8UC1,elementArray_35)).t();
        _shortLineLen= 99;
        GetConfigEntry(_config,"SHORT_LINE_LENTH",_shortLineLen);
        _laserLineWidthL= 11;
        GetConfigEntry(_config,"LASER_LINE_WIDTH",_laserLineWidthL);

    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

    bool getuvd(map<int,cv::Point2d>& uv)
    {
        double xMim= _icenter.x-_roiWidthShift;
        double xMax= _icenter.x+_roiWidthShift;
        double yMim= _icenter.y-_roiHeightShift;
        double yMax= _icenter.y+_roiHeightShift;
        cout << "xMim, xMax, yMim, yMax= " << xMim << ", " << xMax << ", " << yMim << ", " << yMax << endl;

        if(_uvd.size() == _imgNum)
        {
            for(int i= 0;i < _imgNum;i++)
            {
                if(_uvd[i].x > xMim && _uvd[i].x < xMax && _uvd[i].y > yMim && _uvd[i].y < yMax)
                {
                    uv[i]= _uvd[i];
                    _log << i << " " << _uvd[i].x << " " << _uvd[i].y << endl;
                }
            }
            return true;
        }
        else return false;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1,
                      double ratio0= 0.8, double ratio1= 0.2, double k0= 0.0168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
    int _shortLineLen;
    map<int,cv::Point2d> _uvd;
};

class GetSBuvSE: virtual public GetSBuvA
{
public:
    GetSBuvSE(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "SE type." << endl;
        _kernel= (cv::Mat(3,5,CV_8UC1,elementArray_35)).t();
        _shortLineLen= 99;
        GetConfigEntry(_config,"SHORT_LINE_LENTH",_shortLineLen);
        _laserLineWidthL= 11;
        GetConfigEntry(_config,"LASER_LINE_WIDTH",_laserLineWidthL);

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
    int _shortLineLen;
};

class GetSBuvI: virtual public GetSBuvA
{
public:
    GetSBuvI(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "I type." << endl;
        _jt= new GetSBuvJ(LT_J);
        _jt->SetSide(false);
        _jt->set_weld_off();
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1, double disc= 1.0,
                      double ratio0= 0.8, double ratio1= 0.2, double k0= 0.09168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
    bool _planbl,_planbr;
    GetSBuvA* _jt;
};

class GetSBuvWC: virtual public GetSBuvA
{
public:
    GetSBuvWC(LINE_TYPE t): GetSBuvA(t)
    {
        cout << "WC type." << endl;
    }

    int AddImg(const cv::Mat& Input);

    void SetSide(bool left= true)
    {
        _rightSide= (left)? false : true;
    }

protected:
    bool GetHoughLine(const cv::Mat &bimg, cv::Point2d &slcut0, cv::Point2d &slcut1, int SIGN= -1, double disc= 1.0,
                      double ratio0= 0.8, double ratio1= 0.2, double k0= 0.08168, double k1= 4.9);
    cv::Rect _mroil,_mroir;
    bool _planbl,_planbr;
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
