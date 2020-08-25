/*
 * histimage.cc
 *
 *  Created on: Jan 25, 2019
 *      Author: Hu XP
 */

#include "histimage.h"

using namespace std;
using namespace cv;

bool hist(const cv::Mat& input,  std::vector<double>& output, std::map<std::string, int>& config,int ct,bool doMediumBlur)
{
    int th = 15;
    GetConfigEntry(config, "PEAK1D_THRESH", th);
    ;
    int nc= input.cols, nr= input.rows;
    float* p;
    uchar* jp;
    uchar mi;
    //output.resize(nr, 0);
    Mat max1c= Mat::zeros(1,nr,CV_8UC1);
    reduce(input,max1c,0,2);


    //float a= 15,b= 256;
    int histSize = 256-th;
    float range[] = { float(th),256 };
    const float *histRanges = { range };
    Mat hist;
    calcHist(&max1c,1, 0, Mat(), hist, 1, &histSize, &histRanges, true, false);
    int s= sum(hist)[0];
    //cout << hist.depth() << "s= " << s << endl;
    if( ct == 0)
    {
        int count= 0;
        int nt= round(s/20);
        for(int i= 0;i < hist.rows;i++)
        {
            p= hist.ptr<float>(i);
            //cout << (*p) << endl;

            if(count < nt && count > -1)
            {
                count+= *p;
            }
            else if(count > 0)
            {
                th= i-1+th;
                count= -3;
            }
        }
        count= nr-s;
        cout << th << " " << count << " " << nt << endl;
    }

    Mat tempMean, tempStddv;
    double MEAN, STDDV;// mean and standard deviation of the flame region
    double m = mean(hist)[0];
    cout << "mean=" << m << endl;
    meanStdDev(hist, tempMean, tempStddv);

    MEAN = tempMean.at<double>(0, 0);
    STDDV = tempStddv.at<double>(0, 0);
    if(s == 0) m= 0;
    else m= sum(max1c)[0]/s;
    output.clear();
    output.push_back(s);
    output.push_back(m);
    output.push_back(STDDV);
    for(auto o:output) cout << o << endl;

    return true;

}

void Bgr2Gray(const cv::Mat &src,cv::Mat& dst,int flags)
{
    if(src.channels() == 1)
    {
        dst= src;
        return;
    }
    else if(src.channels() != 3)
        return;

    if(flags == TO2_DEFAULT)
    {
        cv::cvtColor(src,dst,cv::COLOR_BGR2GRAY);
        return;
    }
    cv::Mat Channels[3];
    cv::split(src, Channels);
    switch(flags)
    {
//        case TO_DEFAULT:
//            cv::cvtColor(src,dst,cv::COLOR_BGR2GRAY);
//            break;
        case TO2_D:
            dst= Channels[2]/(Channels[1]/2)+Channels[2]/(Channels[0]/2);
            break;
        case TO2_2_1:
            dst= Channels[2]-Channels[0];
            break;
        default :
            int n= flags%30;
//            cv::Mat aChannels[3];
//            cv::split(src, aChannels);
            dst= Channels[n];
    }
    return;
}

/*
void walgo::color2vector(const cv::Mat &input, vector<uchar> &vVec, int th, cv::Size outPutSize)
{
    vVec.resize(outPutSize.width,0);
    cv::Mat img,to;
    int rs= input.cols/outPutSize.width; cout << "rs= " << rs << endl;
    if(input.channels() == 1)
    {
        cv::resize(input.t(),img,cv::Size(input.rows/rs,input.cols/rs),0,0,cv::INTER_AREA);
    }
    else if(input.channels() == 3)
    {
        bgr2gray(input,to);
        cv::resize(to.t(),img,cv::Size(input.rows/rs,input.cols/rs),0,0,cv::INTER_AREA);
    }
//    cv::namedWindow("hh",CV_WINDOW_AUTOSIZE);
//    cv::imshow("hh",img);

    cv::Mat maxCols;
    cv::reduce(img,maxCols,1,cv::REDUCE_MAX); cout << maxCols.size() << endl;
    //imgShow.setTo(0);
    int max;
    vector<int> maxRowsVec,maxColsVec;
    for(int n= 0;n < img.rows;n++)
    {
        maxRowsVec.clear();
        max= maxCols.at<uchar>(n,0);
        if(max < th)
        {
            maxColsVec.push_back(0);
            continue;
        }
        uchar *p= img.ptr(n);
        for(int m= 0;m < img.cols;m++)
        {
            if(*(p++) == max) maxRowsVec.push_back(m);
        }
        maxColsVec.push_back(maxRowsVec[(maxRowsVec.size()-1)/2]);
    }

    cv::Mat imgShow= cv::Mat::zeros(outPutSize,CV_8UC1);
    int shiftc= (img.rows-outPutSize.width)/2;
    int shiftr= img.cols-outPutSize.height;
    for(int i= 0;i < outPutSize.width;i++)
    {
        int r= maxColsVec[shiftc+i]-shiftr;
        vVec.push_back(r);
        if(r >= 0) imgShow.at<uchar>(r,i)= 255;
    }
//    cv::imwrite("ims.png",imgShow);
//    cv::namedWindow("cc",CV_WINDOW_AUTOSIZE);
//    cv::imshow("cc",imgShow);
//    cv::waitKey();
    return;
}
*/
