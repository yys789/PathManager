#ifndef GETXUVA_H
#define GETXUVA_H
#include "utils.h"
#include "histimage.h"
#include "base/data_types.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace walgo;

namespace walgo
{
static uchar elementArray55[5][5]= {{0, 0, 1, 0, 0},
                                    {0, 1, 1, 1, 0},
                                    {1, 1, 1, 1, 1},
                                    {0, 1, 1, 1, 0},
                                    {0, 0, 1, 0, 0},};
static uchar elementArray35[3][5]= {{0, 0, 1, 0, 0},
                                    {1, 1, 1, 1, 1},
                                    {0, 0, 1, 0, 0},};
static uchar elementArray79[7][9]= {{1, 1, 1, 1, 1, 1, 1, 1, 1},
                                    {0, 1, 1, 1, 1, 1, 1, 1, 0},
                                    {0, 0, 1, 1, 1, 1, 1, 0, 0},
                                    {0, 0, 0, 1, 1, 1, 0, 0, 0},
                                    {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                    {0, 0, 0, 0, 0, 0, 0, 0, 0},
                                    {0, 0, 0, 0, 0, 0, 0, 0, 0},};

static uchar elementArray33[3][3]= {{0, 1, 0},
                                    {1, 1, 1},
                                    {0, 1, 0},};
static uchar elementArray57[5][7]= {{0, 0, 0, 1, 0, 0, 0},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {1, 1, 1, 1, 1, 1, 1},
                                    {0, 1, 1, 1, 1, 1, 0},
                                    {0, 0, 0, 1, 0, 0, 0},};

int getTh(const Mat& input,int& th,int& count);
int getTth(const Mat& input,int& th,int& count);

static const int Width= 8;
static Mat gg= getGaussianKernel(2*Width+1,Width/3.0);

enum LINE_TYPE
{
    LT_D, //debug
    LT_V, //v
    //LT_VT,
    LT_W, //w
    //LT_WT,
    //LT_WTT,
    LT_U,  //u
    LT_L,
    LT_N, //n
    LT_WC,
    LT_E,//end
    NUM_LINE_TYPES
};

class getXuvA
{
public:
    static getXuvA* getLineType(LINE_TYPE t);
    static vector<getXuvA*> _obj;

    getXuvA(LINE_TYPE t);

    virtual ~getXuvA() {}

    virtual int addimg(const Mat& image)= 0;

    virtual bool confirm(int s);

    virtual void clear();

    //virtual int getxuv(vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,Mat& image,Point2d& cp);
    //virtual int getxuv(vector<Vec4i>& selectedLinesM,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,Point2d& uv)= 0;

    cv::Point2d getuv()
    {
        int s= _uv.size();
        if(s > 0)
            return _uv[_uv.size()-1];
        else
            return cv::Point2d(0,0);
    }
  
    bool getuv(map<int,Point2d>& luv);
//    {
//        //luv= _uv;
//        callTimes++;
//        if(_uv.size() == _imgNum)
//        {
//            for(int i= 0;i < _imgNum;i++)
//            {
//                if(_uv[i].x > 100)
//                {
//                    luv[i]= _uv[i];
//                    _log << i << " " << _uv[i].x << " " << _uv[i].y << endl;
//                }
//            }
//            return true;
//        }
//        else return false;
//    }

    virtual bool getluv(map<int,Point2d>& luv)
    {
        if(_uv.size() == _imgNum)
        {
            for(int i= 0;i < _imgNum;i++)
            {
                if(_uv[i].x > 100)
                {
                    luv[i]= _uv[i];
                    luv[i].x+= -100;
                }
            }
            return true;
        }
        else return false;
    }
    virtual bool getruv(map<int,Point2d>& luv)
    {
        if(_uv.size() == _imgNum)
        {
            for(int i= 0;i < _imgNum;i++)
            {
                if(_uv[i].x > 100)
                {
                    luv[i]= _uv[i];
                    luv[i].x+= 100;
                }
            }
            return true;
        }
        else return false;
    }
    bool getlr(map<int,Point2d>& luv);

    int eraseWeldedParts(int& start,int s= 138);
    bool getPos(const vector<RobotPos>& pos,int getwlr= 0)
    {
        _getwlr= getwlr;
        if(_getwlr > 0)
        {
            for(auto p:pos) _pos_vec.push_back(p);
        }
        return true;
    }

    string logpd;
    Mat kernel,kernel35,kernel55;
    int center_x_p;
    int callTimes; //
    int adjuster; //adjust size
    int ctime; //timekeeping
    uchar tc;
    int agl;
    string seamName;
    int raserLineWidth,raserWidthR;
    vector<double> GV;
    int hplus;
    int aResize;
    int xMin,xMax;

    map<int,Point2d> _wlr;
    map<int,Point2d> _dlr;

protected:
    virtual void init();
    virtual int getRoi(const Mat& image,int th,Rect2i& roi);
    //void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR);

    LINE_TYPE _lineType;
    int _endORstart;
    int _start;
    int _end;
    std::map<std::string, int> _config;
    fstream _log;
    const std::string getuv_num= "getuv1_2019.07.11!";
    int _imgCols,_imgRows;
    int _imgNum,_debugNum;
    int _save;
    int _d2d;
    vector<RobotPos> _pos_vec;

//    vector<int> _debugVec;
//    vector<Mat> _imgRoivec;
//    vector<Vec4i> _bottomLineVec;
    map<int,double> _gapLen;
    map<int,Point2d> _uv;
    Mat _img_,_imgp;
    int _lowerBound; //used for u
    int _getwlr; //used for u

    bool blackHole;
    int lengthen;
    /*
    "0b*******0 没洞
    0b*******1 有洞
    0b*****00* 45度
    0b*****01* 20度
    0b*****11* 70度
    0b***00*** DH宽度正常
    0b***01*** DH 超长 > 10mm
    0b***11*** DH 超短 < 4mm
    */
};

class getXuvW: virtual public getXuvA
{
public:
    getXuvW(LINE_TYPE t,int endORstart= 0,int start= 100,int end= 300) : getXuvA(t)
    {
        cout << " W type." << endl;
        _endORstart= endORstart;
        _start= start;
        _end= end;
    }
    int addimg(const Mat& input);
    bool confirm(int s);
    //void clear();
    bool getluv(map<int,Point2d>& luv);
    bool getruv(map<int,Point2d>& luv);

protected:
    int getxuv(vector<Vec4i>& selectedLinesL, vector<Vec4i>& selectedLinesR, Mat& img, Point2d& cp,Point2d& center,Point2d& lwp,Point2d& rwp,double& c0l,double& c1279r);
    int getxuv2(double& kl,double& kr,Mat& img,Point2d& cp,Point2d& center,Point2d& lwp,Point2d& rwp,double& c0l,double& c1279r);
    void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR);
    bool getLaserwidth_thresh(const Mat& input, int& bottom, int& top, int dpt, Point2d vn, double vx, double cx, double k, double c, int& th, double &shift, vector<Point> &pts1r);

    map<int,Point2d> _luv;
    map<int,Point2d> _ruv;
    map<int,Point2d> _cuv;
    map<int,Point2d> _klr;
//    map<int,Point2d> _wlr;
//    map<int,Point2d> _dlr;
};

class getXuvU: virtual public getXuvA
{
public:
    getXuvU(LINE_TYPE t) : getXuvA(t) {cout << " U type." << endl;}

    int addimg(const Mat& input);

protected:
    int getxuv(vector<Vec4i>& selectedLinesM,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,Point2d& uv);
};

class getXuvWC: virtual public getXuvA
{
public:
    getXuvWC(LINE_TYPE t,int endORstart= 0,int start= 100,int end= 300) : getXuvA(t)
    {
        cout << " WC type." << endl;
        _endORstart= endORstart;
        _start= start;
        _end= end;
    }
    int addimg(const Mat& input);
    //void clear();
    bool getluv(map<int,Point2d>& luv);
    bool getruv(map<int,Point2d>& luv);

protected:
    int getxuv(vector<Vec4i>& selectedLinesL, vector<Vec4i>& selectedLinesR, Mat& img, Point2d& cp,Point2d& center,Point2d& lwp,Point2d& rwp,double& c0l,double& c1279r);
    int getRotatedInf(vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,Mat& img,Point2d& cp,Point2d& center,Point2d& lwp,Point2d& rwp,double& c0l,double& c1279r);
    map<int,Point2d> _luv;
    map<int,Point2d> _ruv;
    map<int,Point2d> _cuv;
    map<int,Point2d> _klr;
};

class getXuvV: virtual public getXuvA
{
public:
    getXuvV(LINE_TYPE t,int endORstart= 0,int start= 100,int end= 300) : getXuvA(t)
      //endORstart= 0无起始检测，1 仅起头，-1 仅末尾,2 两头都检测；start 起头的张数； end 总张数-末尾张数；
    {
        cout << " V type." << endl;
        _endORstart= endORstart;
        _start= start;
        _end= end;
    }

    int addimg(const Mat& Input);

protected:
    int getxuv(vector<Vec4i>& selectedLinesL, vector<Vec4i>& selectedLinesR, Mat& img, Point2d& cp,double& c0l,double& c1279r,int SH= 0);
    void secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,int& SH);
};


//getXuvV(LINE_TYPE t,int endORstart= 0,int start= 100,int end= 300);//endORstart= 0无起始检测，1 仅起头，-1 仅末尾,2 两头都检测；start 起头的张数； end 总张数-末尾张数；

class getXuvL: virtual public getXuvA
{
public:
    getXuvL(LINE_TYPE t) : getXuvA(t)
    {
        cout << " L type." << endl;
    }

    int addimg(const Mat& input);

protected:
    int getxuv(vector<Vec4i>& selectedLinesL, vector<Vec4i>& selectedLinesR, Mat& img, Point2d& cp,double& c0l,double& c1279r);
};

class getXuvN: virtual public getXuvA
{
public:
    getXuvN(LINE_TYPE t) : getXuvA(t)
    {
        cout << " N type." << endl;
    }

    int addimg(const Mat& input);
    bool confirm(int s);

protected:
    bool getLaserwidth_thresh(const Mat& input,int& bottom,int& top,int dpt,Point2d vn,double vx,double cx,double k,double c,int& th,int& maxh);
};

class getXuvE: virtual public getXuvA
{
public:
    getXuvE(LINE_TYPE t) : getXuvA(t)
    {
        cout << " E type." << endl;
    }

    int addimg(const Mat& Input);
};

}
#endif // GETXUVA_H
