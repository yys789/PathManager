#include "getsbuva.h"
#include "histimage.h"
//#include <unistd.h>

using namespace std;

/*
int GetSBuvZ::AddImg(const cv::Mat& Input)
{
    _log << "pic" << to_string(_imgNum) << ": \n";
    cout << "pic" << to_string(_imgNum) << ": \n";
    _uv[_imgNum]= Point2d(0,0);
    if(input.cols != _imgCols || input.rows != _imgRows)
    {
        _log << "wrong image size!!! " << _imgNum << endl;
        cout << "wrong image size!!! " << _imgNum << endl;
        _imgNum++;
        return -1;
    }
    cv::Mat input;
    if(Input.channels() == 3)
    {
        bgr2gray(Input,input,2);
    }
    else if(Input.channels() == 1) input= Input;

    if(_save > 0) imwrite(logpd+to_string(_imgNum)+"input.jpg",input);

    _icenter;

//    int roiWidthShift= 128;
//    int roiHeightShift= 32;
//    getConfigEntry(_config, "ROI_WIDTH_SHIFT", roiWidthShift);
//    getConfigEntry(_config, "ROI_HEIGHT_SHIFT", roiHeightShift);

    _height;
    int height= 75;
    int width= 360;
    getConfigEntry(_config, "LASER_LINE_HEIGHT_IN_IMAGE", height);
    getConfigEntry(_config, "LASER_LINE_WIDTH_IN_IMAGE", width);
    //int ;
    int upBound= icenter.y-roiHeightShift-height;
    int roiw= icenter.x+roiWidthShift;
    int leftBound= 0;
    if(!_leftSide) leftBound= icenter.x-roiWidthShift;
    cv::Rect2i roi(leftBound,upBound,roiw,_imgRows-upBound);

    cout << roi << endl;

    //int th= 15,count= 30;
    //int nt= -1;
    Mat image;//= input(roi).clone();
    int doMediumBlur = 1;
    int mediumBlurSize = 5;
    getConfigEntry(_config, "SEAM_DETECTOR_MEDIUM_BLUR", doMediumBlur);
    getConfigEntry(_config, "SEAM_DETECTOR_MEDIUM_BLUR_SIZE", mediumBlurSize);

    medianBlur(input(roi),image,mediumBlurSize);//if(_save > 0) imwrite(logpd+to_string(_imgNum)+"input.png",input);
    if(_save > 0) imwrite(logpd+to_string(_imgNum)+"roi.png",image);
    Mat bimg,maskh,maskf;
    int th= cv::threshold(image,bimg,th,255,CV_THRESH_OTSU);
    if(_save > 0) imwrite(logpd+to_string(_imgNum)+"ct.png",bimg);
    _log << "thresh: " << th << endl;

    vector<Vec4i> selectedLinesL,selectedLinesR;

    // mophology transform
    //image(roi).copyTo(img,maskh);
    //cvtColor(img,_img,CV_GRAY2BGR);
//    if(0)
//    {
//        erode(maskh,maskh,kernel);
//        maskf= maskh.clone();
//        kernel= Mat(5,5,CV_8UC1,elementArray55);//Mat::ones(5,7,CV_8UC1);//
//        dilate(maskh,maskh,kernel);
//        //kernel= Mat(3,3,CV_8UC1,elementArray33);
//        //erode(maskh,maskf,kernel);
//        //imwrite(logpd+"ct"+to_string(_imgNum)+".png",img);
//    }

    vector<Vec4i> lines,linesl,linesr;
    int houghThresh = 180;
    int houghMinLineLength = 180;
    int houghMaxLineGap = 55;
    getConfigEntry(_config, "HOUGH_THRESH", houghThresh);
    getConfigEntry(_config, "HOUGH_MINLINELENGTH", houghMinLineLength);
    getConfigEntry(_config, "HOUGH_MAXLINEGAP", houghMaxLineGap);
    double rho= 1;
    double theta= 1;
    HoughLinesP(bimg,lines,1*rho,CV_PI*theta/180,houghThresh,houghMinLineLength,houghMaxLineGap);
    for(auto it:lines)
    {
        double k= (it[3]-it[1]*1.0)/(it[2]-it[0]);
        _log << "k: " << k << endl;
        if(k > -0.5125 && k < 0.5125 && _leftSide && it[0] < 49)
            linesl.push_back(it);
        else if(k > -0.5125 && k < 0.5125 && it[2] >= roi.width-49)
            linesl.push_back(it);
    }
    //_log << "linesl: \n";
//    for(auto line:linesl)
//    {
//        for(int i= 0;i < 4;i++) _log << line[i] << " ";
//        _log << endl;
//    }
//    _log << "linesr: \n";
//    for(auto line:linesr)
//    {
//        for(int i= 0;i < 4;i++) _log << line[i] << " ";
//        _log << endl;
//    }
    int angleRange = 0;
    //getConfigEntry(_config, "PATH_ANGLE_RANGE", angleRange);
    int binsize = _config["ANGLE_BIN_SIZE"];
    int numbins = (int) floor((360.0/(double)binsize)+0.5);
    vector<double> weightsL(numbins);
    vector<vector<Vec4i> > histL(numbins);
    calcAngleHistogram(linesl, histL, weightsL, binsize);
    selectMaxAngles(histL, weightsL, numbins,
            selectedLinesL, angleRange/binsize);

//    vector<double> weightsR(numbins);
//    vector<vector<Vec4i> > histR(numbins);
//    calcAngleHistogram(linesr, histR, weightsR, binsize);
//    selectMaxAngles(histR, weightsR, numbins,
//            selectedLinesR, angleRange/binsize);

//    if(selectedLinesL.size() < 2 || selectedLinesR.size() < 2)
//    {
//        //cout << "bad laser line !!!" << endl;
//        _log << "bad laser " << _imgNum << " line !!!" << " thresh: " << th << endl;
//        imwrite(logpd+to_string(_imgNum)+"t1.png",maskh);

//        if(selectedLinesL.size() < 2 && selectedLinesR.size() < 2)
//        {
//            _log << "all bad lasers line" << endl;
//            _imgNum++;
//            return -3;
//        }
//        //Mat rimg= image(roi);
//        //secondHough(rimg,selectedLinesL,selectedLinesR);
//        if(selectedLinesL.size() < 2 || selectedLinesR.size() < 2)
//        {
//            _log << "one bad laser line" << endl;
//            _imgNum++;
//            return -3;
//        }
//    }

//    else
//    {
    if(selectedLinesL.size() < 1)
    {
        _log << "one bad laser line" << endl;
        _imgNum++;
        return -3;
    }

    Point2d cp,center;
    double c0l,c1279r;
    Mat cimg;
    if(_save  == 2) cvtColor(input(roi),cimg,CV_GRAY2BGR);

    vector<double> lru,rlu,lrv,rlv;
    for(auto it:selectedLinesL)
    {
        _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        lru.push_back(it[2]); lrv.push_back(it[3]);
        if(_save == 2) line(cimg, Point(it[0],it[1]),  Point(it[2],it[3]), Scalar(0,195,0), 1, LINE_AA);
    }
    if(_save == 2) imwrite(logpd+to_string(_imgNum)+"ls.png",cimg);

    sort(lru.begin(),lru.end());
    if(lru.size()%2 == 1) cp.x= lru[(lru.size()-1)/2]+roi.x;
    else cp.x= (lru[(lru.size())/2]+lru[(lru.size()-2)/2])/2+roi.x;
    sort(lrv.begin(),lrv.end());
    if(lrv.size()%2 == 1) cp.y= lrv[(lrv.size()-1)/2]+roi.y;
    else cp.y= (lrv[(lrv.size())/2]+lrv[(lrv.size()-2)/2])/2+roi.y;
    _uv[_imgNum]= cp;//Point2d(cp.x,cp.y+roi.y);
    if(_save > 0)
    {

        Input.copyTo(cimg);
        circle(cimg,cp,15,Scalar(0,255,0),2);
        imwrite(logpd+to_string(_imgNum)+"sl.png",cimg);
    }

    _imgNum++;
    return 0;
}

bool GetSBuvZT::getuv(map<int,Point2d>& uv,map<int,Point2d>& uv2)
{
    double xMim= _icenter.x-_roiWidthShift;
    double xMax= _icenter.x+_roiWidthShift;
    double yMim= _icenter.y-_roiHeightShift;
    double yMax= _icenter.y-_roiHeightShift;
    cout << "xMim, xMax, yMim, yMax= " << xMim << ", " << xMax << ", " << yMim << ", " << yMax << endl;

    if(_uv.size() == _imgNum && _uv2)
    {
        for(int i= 0;i < _imgNum;i++)
        {
            if(_uv[i].x > xMim && _uv[i].x < xMax && _uv[i].y > yMim && _uv[i].y < yMax)
            {
                uv[i]= _uv[i];
                _log << i << " " << _uv[i].x << " " << _uv[i].y << endl;
            }
            if(_uv2[i].x > xMim && _uv2[i].x < xMax && _uv2[i].y > yMim && _uv2[i].y < yMax)
            {
                uv2[i]= _uv2[i];
                _log << i << " " << _uv2[i].x << " " << _uv2[i].y << " uv2" << endl;
            }
        }
        return true;
    }
    else return false;
}
*/
