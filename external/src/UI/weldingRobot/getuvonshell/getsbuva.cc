#include "getsbuva.h"
//#include "lineanalysis.h"
//#include "d2seamdetector.h"
#include <unistd.h>
#include <algorithm>

using namespace std;

LINE_TYPE AlgoType(string stype)
{
    if(stype == "D")
    {
        return LT_D;
    }
    else if(stype == "J")
    {
        return LT_J;
    }
    else if(stype == "VW")
    {
        return LT_VW;
    }
    else if(stype == "M")
    {
        return LT_M;
    }
    else if(stype == "EE")
    {
        return LT_EE;
    }
    else if(stype == "S")
    {
        return LT_S;
    }
    else if(stype == "W")
    {
        return LT_WW;
    }
    else if(stype == "JS")
    {
        return LT_JS;
    }
    else if(stype == "SE")
    {
        return LT_SE;
    }
};

static double dist(const cv::Vec4i &line4i)
{
    return sqrt(pow(line4i[2]-line4i[0],2)+pow(line4i[3]-line4i[1],2));
}

static cv::Mat getGammaTable(double gamma,int lowThresh)
{
    cv::Mat lookUpTable(1,256,CV_8U);
    uchar *p= lookUpTable.ptr();
    for(int i= 0; i < 256;i++) *(p++)= cv::saturate_cast<uchar>(pow(i/255.0,gamma)*255.0);
    return lookUpTable;
}
//Mat res = img.clone（）;
static void gammaLUT(const cv::Mat &input,cv::Mat &dst,cv::Mat lookUpTable)
{
    //cv::exp(-centralPts*lamda,temp);
}

//static bool cmp(const Vec4i &x,const Vec4i &y)
//{
//    return dist(x) > dist(y);//sqrt(pow(x[2]-x[0],2)+pow(x[3]-x[1],2)) > sqrt(pow(y[2]-y[0],2)+pow(y[3]-y[1],2));
//}

//const int Width= 8;
//Mat gg= getGaussianKernel(2*Width+1,Width/2);
//MatIterator_<double> ggit= gg.begin<double>();
//for(int d= 0;d < 18;d++) cout << *(ggit++) << " ";
//static vector<double> GV;
//{printf("%10.20f ",d);});

cv::Mat GetKerLoG(const double size,const int sigma)
{
    cv::Mat ker= cv::Mat::zeros(size*2+1,size*2+1,CV_64F),KER= cv::Mat::zeros(sigma*6+1,sigma*6+1,CV_64F);
    int x0= sigma*3,y0= sigma*3;
    double ss= pow(sigma*1.0,2);
    double c= 1000*1/3.1415926/ss/ss;
    cout << "c= " << c << endl;
    double *p;
    double xs,t;

    for(int i= 0;i < sigma*6+1;i++)
    {
        p= KER.ptr<double>(i);
        xs= pow(i-x0,2);
        for(int j= 0;j < sigma*6+1;j++)
        {
            t= -0.5*(xs+pow(j-y0,2))/ss;
            *(p++)= c*(1+t)*exp(t);
        }
    }
    cv::imwrite("bigker.png",KER);
    cv::resize(KER,ker,cv::Size(ker.cols,ker.rows),0,0,cv::INTER_AREA);

    return ker;
}



//vector<GetSBuvA*> GetSBuvA::_obj(NUM_LINE_TYPES);

//GetSBuvA* GetSBuvA::GetLineType(LINE_TYPE t)
//{
//    if (_obj[t] == NULL)
//    {
//        if (t == LT_J)//|| t == LT_VT)
//            _obj[t]= new GetSBuvJ(t);

//        else if (t == LT_TWO_Z)
//            _obj[t]= new GetSBuvJ(t);
//    }
//    return _obj[t];
//}
//int getXuvU::callTimes= 0;

int GetTh(const cv::Mat& input,int& th,const int d,const int max)
{
    int count;
    count= input.cols;
    vector<uchar> maxv;
    reduce(input,maxv,0,2);
    vector<uchar> tmaxv= maxv;
    sort(tmaxv.begin(),tmaxv.end());
    for(int i= 0;i < tmaxv.size();i++)
    {
        if(tmaxv[i] > th)
        {
            count= i;
            break;
        }
    }
    if(count > input.cols/2) return -1;
    int nt= int((tmaxv.size()-count*1.0)*d/100);
    th= tmaxv[count+nt]-1;
    th= (th < max)? th:max;
    //cout << "thresh: " << th << " " << count << " " << nt << endl;
    return count;
}

/*
int walgo::getTth(const Mat& input,int& th,int& count)
{
    Mat maxv= Mat::zeros(input.rows,1,CV_8UC1);
    reduce(input,maxv,1,2);
    vector<uchar> tmaxv;
    for(int i= 0;i < input.rows;i++) tmaxv.push_back(maxv.at<uchar>(i,0));
    sort(tmaxv.begin(),tmaxv.end());
    for(int i= 0;i < tmaxv.size();i++)
    {
        if(tmaxv[i] >= th)
        {
            count= i;
            break;
        }
    }
    int nt= round((tmaxv.size()-count)/222.2);
    th= tmaxv[count+nt];
    if(th > 253) th= 253;
    //cout << "thresh: " << th << " " << count << " " << nt << endl;
    return nt;
}
*/


void GetSBuvA::_init()
{
    string path= "uvlog/";
    if(access(path.c_str(),0))
    {
        cout << "uvlog/ is nonexistent\n";
        system("mkdir uvlog");
    }

    std::time_t now= time(0);
    std::tm *dhms= std::localtime(&now);
    _logpd= "uvlog/"+to_string(dhms->tm_mon+1)+"_"+to_string(dhms->tm_mday)+"_"+to_string(dhms->tm_hour)+"_"+\
            to_string(dhms->tm_min)+"_"+to_string(dhms->tm_sec)+"_"+to_string(clock())+"/";
    //cout << pd << endl;
    string bb= "mkdir "+_logpd;
    const char* tpd = bb.c_str();
    system(tpd);
    _log.open(_logpd+"log.txt",ios::out);

    _imgNum= 0;
    _debugNum= 0;
    _save= 1;
    GetConfigEntry(_config, "SAVE_DEBUG_IMAGE", _save);

    _imgCols= 720;
    _imgRows= 540;
    GetConfigEntry(_config, "ROI_WIDTH", _imgCols);
    GetConfigEntry(_config, "ROI_HEIGHT", _imgRows);
    _icenter= cv::Point(_imgCols/2,_imgRows/2);
    GetConfigEntry(_config, "LASER_LINE_HEIGHT_IN_IMAGE", _lineHeight);
    GetConfigEntry(_config, "LASER_LINE_WIDTH_IN_IMAGE", _lineWidth); //_icenter.x+roiWidthShift-leftBound;

    _roiWidthShift= 180;
    _roiHeightShift= 135;
    GetConfigEntry(_config, "ROI_WIDTH_SHIFT", _roiWidthShift);
    GetConfigEntry(_config, "ROI_HEIGHT_SHIFT", _roiHeightShift);

    GetConfigEntry(_config, "SEAM_DETECTOR_MEDIUM_BLUR",_doMediumBlur);
    GetConfigEntry(_config, "SEAM_DETECTOR_MEDIUM_BLUR_SIZE",_mediumBlurSize);

    _d2d= 2;
    GetConfigEntry(_config, "DO_2ND_DETECT", _d2d);

    _houghThresh= 168;
    _houghMinLineLength= 180;
    GetConfigEntry(_config, "HOUGH_THRESH", _houghThresh);
    GetConfigEntry(_config, "HOUGH_MINLINELENGTH", _houghMinLineLength);
    _houghMaxLineGap= _houghMinLineLength/9;
    GetConfigEntry(_config,"HOUGH_MAXLINEGAP",_houghMaxLineGap);

    _angleRange = 0;
    GetConfigEntry(_config,"PATH_ANGLE_RANGE",_angleRange);
    _binsize = 4;
    GetConfigEntry(_config,"ANGLE_BIN_SIZE",_binsize);
    //?_center_x_p= 0;
    //_lowerBound= _imgCols-1;

    _laserLineWidthL= 11;
    _laserLineWidthR= 11;
    GetConfigEntry(_config,"LASER_LINE_WIDTHL",_laserLineWidthL);
    GetConfigEntry(_config,"LASER_LINE_WIDTHR",_laserLineWidthR);

    _agl= 45;
    _blackHole= false;
    _lengthen= 0;
    //_hplus= 0;
    _xMin= 180;
    _xMax= 720-180;
    _bluegv= 185;
    _greengv= 120;
    GetConfigEntry(_config,"BLUE_GRAY_THRESH",_bluegv);
    GetConfigEntry(_config,"GREEN_GRAY_THRESH",_greengv);
    _weldOff= false;
    _weldCutR= 15;
    GetConfigEntry(_config,"WELD_CUT_RATIO",_weldCutR);
    _ht= 79;
    GetConfigEntry(_config,"HIGH_THRESH_THRESH",_ht);
    //GetConfigEntry(_config, "XMIN", xMin);
    //GetConfigEntry(_config, "XMAX", xMax);
}

void GetSBuvA::GetSBuvA::clear()
{
    //upts.clear();
    //vpts.clear();

    if(_callTimes == 0) return;
    _log.close();
    cout << "callTimes: " << _callTimes << endl;
    _imgNum= 0;
    _debugNum= 0;
    _uv.clear();
    //_imgRoivec.clear();
    //_log.close();
    std::time_t now= time(0);
    std::tm *dhms= std::localtime(&now);
    _logpd= "uvlog/"+to_string(dhms->tm_mon)+"_"+to_string(dhms->tm_mday)+"_"+to_string(dhms->tm_hour)+"_"+\
            to_string(dhms->tm_min)+"_"+to_string(dhms->tm_sec)+"/";
    //cout << pd << endl;
    string bb= "mkdir "+_logpd;
    const char* tpd = bb.c_str();
    system(tpd);
    //fstream log;
    _log.open(_logpd+"log.txt",ios::out);
    _gapLen.clear();
    _center_x_p= 0;
//    kernel= Mat(3,3,CV_8UC1, elementArray33);
//    kernel35= Mat(3,5,CV_8UC1,elementArray35);
//    kernel55= Mat(5,5,CV_8UC1,elementArray55);
//    ctime= 0;
}

bool GetSBuvA::GetSBuvA::confirm(int s)
{
    //for(int i= 0;i < _debugVec.size()/2;i++) //_log << _debugVec[2*i] << " " << _debugVec[2*i+1] << endl;
    //for(auto &g:_gapLen) cout << g.first << " " << g.second << endl;
    if(_uv.size() == s && _imgNum == s)
    {
        return true;
    }
    else
    {
        cout << "nonconfirm! " << _uv.size() << " " << s << " " << _imgNum << endl;
        _log << "nonconfirm! " << _uv.size() << " " << s << " " << _imgNum << endl;
        return false;
    }
}

int GetSBuvA::GetSBuvA::_GetRoi(const cv::Mat& image,int th,cv::Rect2i& roi)
{
    //cout << "getRoi: " << endl;
    int s= image.rows;
    cv::Mat maxc= cv::Mat::zeros(s,1,CV_8UC1);
    cv::reduce(image,maxc,1,2);
    cv::threshold(maxc,maxc,th,255,0);
    //medianBlur(maxc,maxc,9);
    //cout  << maxc << endl;
    int eu= 0,ed= 0;
    for(int i= 0;i < s;i++)
    {
        if(maxc.at<uchar>(i,0) > 0)
        {
            eu= i;
            break;
        }
    }
    for(int i= s-1;i > 0 ;i--)
    {
        if(maxc.at<uchar>(i,0) > 0)
        {
            ed= i;
            break;
        }
    }
    //int addition= (ed+20 < image.rows)? 20:0;
    //if(addition == 0) addition= (ed+10 < image.rows)? 10:0;
    //if(addition == 0) addition= (ed+ 15 < image.rows)?  15:0;
    //cout << "eu ed: " << eu << " " << ed << endl;
    if(ed-eu < 20 || image.rows-ed < 1) { return -1;}
    if(image.rows-ed > 26) ed+= 25;
    roi= cv::Rect2i(0,eu,image.cols,ed-eu+1);
    return 0;
}


void GetSBuvA::_GetMaskRoi(cv::Mat &mask, cv::Rect2i &roim)
{
    vector<cv::Point> ptsl;
    findNonZero(mask,ptsl);
    roim.y= (ptsl.begin())->y;
    roim.height= (ptsl.end()-1)->y-roim.y+1;
    int x1= (ptsl.begin())->x,x2= (ptsl.begin())->x;
    for(auto &p:ptsl)
    {
        if(p.x < x1) x1=p.x;
        if(p.x > x2) x2=p.x;
    }
    roim.x= x1;
    roim.width= x2-x1+1;
    //cout << roim << endl;
}

bool GetSBuvA::_GetMaskRoiB(cv::Mat &mask, cv::Rect2i &roim)
{
    vector<cv::Point> ptsl;
    findNonZero(mask,ptsl);
    if(ptsl.size() < 10) return false;
    roim.y= (ptsl.begin())->y;
    roim.height= (ptsl.end()-1)->y-roim.y+1;
    int x1= (ptsl.begin())->x,x2= (ptsl.begin())->x;
    for(auto &p:ptsl)
    {
        if(p.x < x1) x1=p.x;
        if(p.x > x2) x2=p.x;
    }
    roim.x= x1;
    roim.width= x2-x1+1;
    //cout << roim << endl;
    return true;
}

bool GetSBuvA::getuv(map<int,cv::Point2d>& uv)
{
    double xMim= _icenter.x-_roiWidthShift;
    double xMax= _icenter.x+_roiWidthShift;
    double yMim= _icenter.y-_roiHeightShift;
    double yMax= _icenter.y+_roiHeightShift;
    cout << "xMim, xMax, yMim, yMax= " << xMim << ", " << xMax << ", " << yMim << ", " << yMax << endl;

    if(_uv.size() == _imgNum)
    {
        for(int i= 0;i < _imgNum;i++)
        {
            if(_uv[i].x > xMim && _uv[i].x < xMax && _uv[i].y > yMim && _uv[i].y < yMax)
            {
                uv[i]= _uv[i];
                _log << i << " " << _uv[i].x << " " << _uv[i].y << endl;
            }
        }
        return true;
    }
    else return false;
}
