#include "getsbuva.h"
#include "histimage.h"
#include "walgo/lineanalysis.h"
//#include "lineanalysis.h"
#include <random>

static default_random_engine rng(time(0));

using namespace std;


static bool ranSac(vector<cv::Point> &_pts,int nIterCnt, double dMaxErrorThreshold, double rmsThreshold, cv::Vec4d &LINE,const int k)
{
    bool result= false;

    int nCnt= _pts.size();
    uniform_int_distribution<int> uniform(0,nCnt-1);

    int circleTimes= 0,rtimes= 1;
    int _innerProportion= 20;
    int nBestRightCnt= int(_innerProportion*_pts.size()/100.0);

    while (nIterCnt)
    {
        //1.随机选择7个点
        set<int> sIndexs;
        sIndexs.clear();
        int _snc= 5;
        while(sIndexs.size() < _snc) sIndexs.insert(uniform(rng));
        //2.得到模型参数

        vector<cv::Point> points;
        for (auto iter = sIndexs.begin(); iter != sIndexs.end(); iter++)
        {
            //cout << *iter << " ";
            points.push_back(_pts[*iter]);
        }
        //cout << endl;

        cv::Vec4d line;
        cv::fitLine(points,line,cv::DIST_L2,0,0.01,0.01);

        //cout << "dr: " << dr << endl;

        int kk= line[1]/line[0]*k;
        if(kk < 0 || kk > 1) continue;
        double dr= line[0]*line[3]-line[1]*line[2],adr2= 0;
        for(auto &p:points)
        {
            //cout << line[0] << " " << line[1] << " " << pow(line[0],2)+pow(line[1],2) << endl;
            adr2+= pow(dr- (line[0]*p.y-line[1]*p.x),2);
        }

        double rms= pow(adr2/points.size(),0.5);
        if(rms > rmsThreshold*rtimes)
        {
            //cout << "RMS = " << rms << endl;
            circleTimes++;
            if(circleTimes > 33)
            {
                if(++rtimes > 9) return result;
                circleTimes= 0;
            }
            continue;
        }

        //3.统计内点个数

        int nRightCnt = 0;
        double distance= 10000;
        vector<cv::Point> vecPoints;
        for(auto &p:_pts)
        {
            distance= abs(dr- (line[0]*p.y-line[1]*p.x));
            if (distance < dMaxErrorThreshold)
            {
                vecPoints.push_back(p);
                nRightCnt++;
            }
        }

        if (nRightCnt >= nBestRightCnt)
        {
            cv::fitLine(vecPoints,line,cv::DIST_HUBER,0,0.01,0.01);
            kk= line[1]/line[0]*k;
            if(kk < 0 || kk > 1) continue;
            dr= line[0]*line[3]-line[1]*line[2];
            adr2= 0;
            for(auto &p:vecPoints)
            {
                adr2+= pow(dr-(line[0]*p.y-line[1]*p.x),2);
            }
            rms= pow(adr2/vecPoints.size(),0.5);
        }
        else continue;
        //4.保存内点个数最多的模型

        if (rms <= rmsThreshold)
        {
            nBestRightCnt= nRightCnt;
            rmsThreshold= rms;
            //cout << "RRMS = " << rms << endl;
            result= true;
            LINE= line;
        }
        rtimes= 1;
        nIterCnt--;
    }
    return result;
}

int GetSBuvW::AddImg(const cv::Mat& Input)
{
    _log << "pic" << to_string(_imgNum) << ": \n";
    cout << "pic" << to_string(_imgNum) << ": \n";
    _uv[_imgNum]= cv::Point2d(-1000,-1000);
    _uvl[_imgNum]= cv::Point2d(-1000,-1000);
    _uvr[_imgNum]= cv::Point2d(-1000,-1000);
    if(Input.cols != _imgCols || Input.rows != _imgRows || Input.channels() != 3)
    {
        _log << "wrong image size!!! " << _imgNum << endl;
        cout << "wrong image size!!! " << _imgNum << endl;
        _imgNum++;
        return -1;
    }

//    if("_isblue" == "_isblue")
//    {
//        vector<cv::Mat> aChannels;
//        cv::split(Input, aChannels);
//        reverse(aChannels.begin(),aChannels.end());
//        cv::merge(aChannels,Input);
//        cv::imwrite(_logpd+to_string(_imgNum)+"r_color.jpg",aChannels.at(0));
//        cv::imwrite(_logpd+to_string(_imgNum)+"g_color.jpg",aChannels.at(1));
//        cv::imwrite(_logpd+to_string(_imgNum)+"b_color.jpg",aChannels.at(2));
//        cv::threshold(aChannels[2],aChannels[2],79,255,8);
//        cv::imwrite(_logpd+to_string(_imgNum)+"b_color_t.jpg",aChannels.at(2));
//        cv::imwrite(_logpd+to_string(_imgNum)+"change_color.jpg",Input);
//        _imgNum++;
//        return -1;
//    }
    cv::Rect2i roil,roir;
    roil.x= 0;
    roil.y= _icenter.y-_roiHeightShift;
    roil.width= _icenter.x+_roiWidthShift;
    roil.height= _imgRows-roil.y-_roiHeightShift;
    cout << roil << endl;

    roir.x= _icenter.x-_roiWidthShift;
    roir.y= _icenter.y-_roiHeightShift;
    roir.width= _imgCols-roir.x;
    roir.height= _imgRows-roir.y-_roiHeightShift;
    cout << roir << endl;

    cv::Mat maskh,maskf,aChannels[3],blue,green,red,redb(_imgRows,_imgCols,CV_8UC1);
    cv::split(Input, aChannels);
    //aChannels[2].copyTo(red);
    //cout << i << " " << src.channels() << " " << aChannels[1].channels() << endl;
    int th;
//    if(!_weldOff)
//    {
//        th= cv::threshold(aChannels[0],blue,_bluegv,255,0);75;
//        th= cv::threshold(aChannels[1],green,_greengv,255,0);60;
//        if(_save == 2)
//        {
//            imwrite(_logpd+to_string(_imgNum)+"input_blue.jpg",blue);
//            imwrite(_logpd+to_string(_imgNum)+"input_green.jpg",green);
//        }
//        //cv::threshold(aChannels[2],red,5,255,0);

//        //cv::dilate(blue,blue,kernel);
//        //cv::dilate(green,green,kernel);
//        aChannels[2].setTo(3,blue);
//        aChannels[2].setTo(3,green);
//    }
    if(!_weldOff)
    {
        cv::Mat fimg,kernel= cv::Mat(13,5,CV_32SC1,elementArray135);
        cv::filter2D(aChannels[2],fimg,CV_16S,kernel);
        fimg/= 20;
        fimg.convertTo(aChannels[2],CV_8UC1);
    }
    cv::medianBlur(aChannels[2],aChannels[2],5);
    //th= cv::threshold(aChannels[2],redb,33,255,8);
    //cout << _imgNum << "s th: " << th << endl;
    if(!_weldOff) {
        //cv::threshold(aChannels[2],redb,59,255,8);
        int subCols= 30,subi= redb.cols/subCols;
        for(int i= 0;i < subi;i++)
        {
            int th= cv::threshold(aChannels[2](cv::Rect(i*subCols,roil.y,subCols,roil.height)),redb(cv::Rect(i*subCols,roil.y,subCols,roil.height)),55,255,8);
            _log << "th= " << th << "\n";
        }
    }
    else {
        cout << "_weldOff\n";
        int subCols= 30,subi= redb.cols/subCols;
        for(int i= 0;i < subi;i++)
        {
            int th= cv::threshold(aChannels[2](cv::Rect(i*subCols,roil.y,subCols,roil.height)),redb(cv::Rect(i*subCols,roil.y,subCols,roil.height)),55,255,8);
            _log << "th= " << th << "\n";
        }
    }
    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",redb);
    aChannels[2].copyTo(red);
//    int w= 1200,h= 1080;
//    cv::Mat res,bs= cv::Mat::zeros(1024,(1280-w)/2,CV_8UC1);
//    cv::resize(redb,res,cv::Size(w,h));
//    vector<cv::Mat> matrices= { bs,res(cv::Rect(0,0,w,1024)),bs,};
//    cv::hconcat(matrices,red);
//    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+".jpg",red);
//    _imgNum++;
//    return -1;


    int SIGN= -1;
    if(_rightSide) SIGN= 1;

    if(_save == 2)
    {
        imwrite(_logpd+to_string(_imgNum)+"input_redl.jpg",redb(roil));
        imwrite(_logpd+to_string(_imgNum)+"input_redr.jpg",redb(roir));
    }

//    _imgNum++;
//    return 0;

    cv::Point2d lcut0,lcut1,rcut0,rcut1;
    cv::Vec4d adline,rdline;
    double dl,dr;
    _leftok= true;
    _rightok= true;
    if(!GetHoughLine(redb(roil),lcut0,lcut1,1,0.5,0.2))
    {
        _leftok= false;
    }

    else
    {
        cv::Rect2i mroi;
        maskh= cv::Mat::zeros(Input(roil).size(),CV_8UC1);
        cv::line(maskh,lcut0,lcut1,255, 36, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        _mroil= mroi;
        cv::Mat lineMaskedL,lineMaskedR;//= maskh+2;
        Input(roil).copyTo(lineMaskedL,maskh);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lm.jpg",lineMaskedL);
        cv::split(lineMaskedL(mroi), aChannels);

        th= cv::threshold(aChannels[2],maskf,15,255,8);
        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        //cv::Vec4d adline;
        cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
        adline[2]+= roil.x+mroi.x;
        adline[3]+= roil.y+mroi.y;
        //double dl,dr;
        dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);

        //_uvl[_imgNum].x= lcut1.x+(lcut1.x-lcut0.x)*2/3;
        //_uvl[_imgNum].y=

        //cv::line(itemp,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

        //    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

        //    _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;
//        cv::Mat itemp,rinput;
//        rinput= (redb(roil)).clone();
//        cv::Rect roils((lcut0.x+lcut1.x)/2,(lcut0.y+lcut1.y)/2,lcut1.x+(lcut1.x-lcut0.x)/2,lcut1.y+(lcut1.y-lcut0.y)/2);
//        if(!(0 <= roils.x && 0 <= roils.width && roils.x + roils.width <= rinput.cols && 0 <= roils.y && 0 <= roils.height && roils.y + roils.height <= rinput.rows))
//        {
//            _imgNum++;
//            cout << "roi1 error!!";
//            return -2;
//        }
//        //cv::line(rinput(roils),cv::Point2d(0,dr/adline[0]),lcut1+lcut1.x+(lcut1.x-lcut0.x)/2,cv::Scalar(0),_laserLineWidthR, cv::LINE_AA);
//        cv::threshold(rinput(roils),itemp,th,255,8);
//        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"il.jpg",rinput(roils));

//        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
//        cv::Mat etemp;
//        cv::erode(itemp,etemp,kernel);
//        vector<cv::Point> pts25;
//        cv::findNonZero(etemp,pts25);
//        int count= 0,pts25size= pts25.size()/2;
//        if(pts25size < 2){
//            _imgNum++;
//            return -10;
//        }
//        auto ptp= (pts25.end()-1);
//        while(count < 11 && pts25size > 0) {
//            for(int i= 1;i < 15;i++)
//            {
//                if((ptp-i)->y-ptp->y < 4) count++;
//            }
//            pts25size--;
//            ptp--;
//        }

//        vector<int> btlx;
//        for(int i= -1;i < 15;i++)
//        {
//            btlx.push_back((ptp-i)->x);
//        }
//        sort(btlx.begin(),btlx.end());
////        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
//        cv::Point2d uvl;
//        uvl.x= btlx[(btlx.size()-1)/2];
//        pts25.clear();
//        cv::findNonZero(etemp(cv::Rect(uvl.x,0,1,itemp.rows)),pts25);
//        if(pts25.size() < 2){
//            _imgNum++;
//            return -10;
//        }
//        vector<int> vy;
//        for(auto &p:pts25) vy.push_back(p.y);
//        uvl.y= vy[(vy.size()-1)/2];
//        _uvl[_imgNum]= uvl;
    }

    if(!GetHoughLine(redb(roir),rcut0,rcut1,-1,0.8,0.5))
    {
        _rightok= false;
    }
    else
    {
        cv::Rect2i mroi;
        maskh= cv::Mat::zeros(Input(roir).size(),CV_8UC1);
        cv::line(maskh,rcut0,rcut1,255, 36, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        _mroir= mroi;
        cv::Mat lineMaskedL,lineMaskedR;
        Input(roir).copyTo(lineMaskedR,maskh);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",lineMaskedR);
        cv::split(lineMaskedR(mroi), aChannels);
        th= cv::threshold(aChannels[2],maskf,15,255,8);

        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        //cv::Vec4d rdline;
        cv::fitLine(pts255, rdline, cv::DIST_HUBER, 0, 0.01,0.01);
        rdline[2]+= roir.x+mroi.x;
        rdline[3]+= roir.y+mroi.y;
        dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
        //dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);
    }
    if(_rightok && !_leftok) {
        //_imgNum++;
        //return -4;
        roil.x= _imgCols/4;
        roil.width= roir.x+rcut0.x-(rcut1.x-rcut0.x)*2/3-70-roil.x;
        if(roil.width < 0) {
            _imgNum++;
            return -4;
        }
        if(!GetHoughLine(redb(roil),lcut0,lcut1,1,0.75,0.25,0.5))
        {
            _imgNum++;
            return -4;
        }
        else
        {
            cv::Rect2i mroi;
            maskh= cv::Mat::zeros(Input(roil).size(),CV_8UC1);
            cv::line(maskh,lcut0,lcut1,255, 36, cv::LINE_AA);
            _GetMaskRoi(maskh,mroi);
            _mroil= mroi;
            cv::Mat lineMaskedL,lineMaskedR;//= maskh+2;
            Input(roil).copyTo(lineMaskedL,maskh);
            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lm0.jpg",lineMaskedL);
            cv::split(lineMaskedL(mroi), aChannels);

            th= cv::threshold(aChannels[2],maskf,15,255,8);
            vector<cv::Point> pts255;
            cv::findNonZero(maskf,pts255);
            //cv::Vec4d adline;
            cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
            adline[2]+= roil.x+mroi.x;
            adline[3]+= roil.y+mroi.y;
            //double dl,dr;
            dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);
        }

    }
    if(!_rightok && _leftok) {
        //_imgNum++;
        //return -4;
        roir.x= lcut1.x+(lcut1.x-lcut0.x)*2/3+70;
        roir.width= _imgCols*3/4-roir.x;
        if(roir.width < 0) {
            _imgNum++;
            return -4;
        }
        imwrite(_logpd+to_string(_imgNum)+"input_redr.jpg",redb(roir));
        if(!GetHoughLine(redb(roir),rcut0,rcut1,-1,0.75,0.25,0.5))
        {
            _imgNum++;
            return -4;
        }
        else
        {
            cv::Rect2i mroi;
            maskh= cv::Mat::zeros(Input(roir).size(),CV_8UC1);
            cv::line(maskh,rcut0,rcut1,255, 36, cv::LINE_AA);
            _GetMaskRoi(maskh,mroi);
            _mroir= mroi;
            cv::Mat lineMaskedL,lineMaskedR;
            Input(roir).copyTo(lineMaskedR,maskh);
            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm0.jpg",lineMaskedR);
            cv::split(lineMaskedR(mroi), aChannels);
            th= cv::threshold(aChannels[2],maskf,15,255,8);

            vector<cv::Point> pts255;
            cv::findNonZero(maskf,pts255);
            //cv::Vec4d rdline;
            cv::fitLine(pts255, rdline, cv::DIST_HUBER, 0, 0.01,0.01);
            rdline[2]+= roir.x+mroi.x;
            rdline[3]+= roir.y+mroi.y;
            dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
        }

    }
    if(!_rightok && !_leftok) {
        _imgNum++;
        return -4;
    }
    cv::Point2d cross;
    cross.x= (dl*rdline[0]-dr*adline[0])/(rdline[1]*adline[0]-adline[1]*rdline[0]);
    cross.y= (dl*rdline[1]-dr*adline[1])/(adline[0]*rdline[1]-rdline[0]*adline[1]);
//    if(_save > 0)
//    {
//        //cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
//        //cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
//        cv::Mat item;
//        Input.copyTo(item);
////        cv::imwrite(_logpd+to_string(_imgNum)+"aChannels[2]R.jpg",aChannels[2]);
////        cv::imwrite(_logpd+to_string(_imgNum)+"maskfr.jpg",maskf);
//        cv::circle(item,cross,15,cv::Scalar(0,255,0),2,cv::LINE_AA);
//        cv::circle(item,_uvl[_imgNum],15,cv::Scalar(255,0,0),2,cv::LINE_AA);
//        cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",item);

//    }
//    th= cv::threshold(lineMasked(mroi),maskf,15,255,CV_THRESH_OTSU);
//    vector<cv::Point> pts255;
//    cv::findNonZero(maskf,pts255);
//    cv::Vec4d lineLeft,lineRight,sdline;
//    fitLine(pts255, sdline, cv::DIST_HUBER, 0, 0.01,0.01);
    //if(_save == 1) Input.copyTo(_img_);

    if(1) {
        cv::Rect2i mroi;
        maskh= cv::Mat::zeros(Input.size(),CV_8UC1);
        lcut1.x+= roil.x;
        lcut1.y+= roil.y;
        cv::line(maskh,cross*.9+lcut1*.1,lcut1,255, 17, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        _mroil= mroi;
        cv::Mat lineMaskedL,lineMaskedR;//= maskh+2;
        red.copyTo(lineMaskedL,maskh);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lm.jpg",lineMaskedL);
        //cv::split(lineMaskedL(mroi), aChannels);

        th= cv::threshold(lineMaskedL(mroi),maskf,15,255,8);


        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray_35);
        cv::Mat etemp;
        cv::erode(maskf,etemp,kernel);
        vector<cv::Point> pts25;
        cv::findNonZero(etemp,pts25);
        int count= 0,pts25size= pts25.size()/2;
        if(pts25size < 2){
            _imgNum++;
            return -10;
        }
        auto ptp= (pts25.end()-1);
        while(count < 11 && pts25size > 0) {
            for(int i= 1;i < 15;i++)
            {
                if((ptp-i)->y-ptp->y < 4) count++;
            }
            pts25size--;
            ptp--;
        }

        vector<int> btlx;
        for(int i= -1;i < 15;i++)
        {
            btlx.push_back((ptp-i)->x);
        }
        sort(btlx.begin(),btlx.end());
//        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
        cv::Point2d uvl;
        _uvl[_imgNum].x= btlx[(btlx.size()-1)/2];
        pts25.clear();
        cv::findNonZero(etemp(cv::Rect(_uvl[_imgNum].x,0,1,etemp.rows)),pts25);
        if(pts25.size() < 2){
            _imgNum++;
            return -10;
        }
        vector<int> vy;
        for(auto &p:pts25) vy.push_back(p.y);
        _uvl[_imgNum].x+= mroi.x;
        _uvl[_imgNum].y= vy[(vy.size()-1)/2]+mroi.y;
        _uvl[_imgNum].y= (dl+adline[1]*(_uvl[_imgNum].x))/adline[0];
    }
    if(1) {
        cv::Rect2i mroi;
        maskh= cv::Mat::zeros(Input.size(),CV_8UC1);
        rcut0.x+= roir.x;
        rcut0.y+= roir.y;
        cv::line(maskh,cross,rcut0,255, 17, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        _mroil= mroi;
        cv::Mat lineMaskedL,lineMaskedR;//= maskh+2;
        red.copyTo(lineMaskedL,maskh);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",lineMaskedL);
        //cv::split(lineMaskedL(mroi), aChannels);

        th= cv::threshold(lineMaskedL(mroi),maskf,15,255,8);


        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray_35);
        cv::Mat etemp;
        cv::erode(maskf,etemp,kernel);
        vector<cv::Point> pts25;
        cv::findNonZero(etemp,pts25);
        int count= 0,pts25size= pts25.size()/2;
        if(pts25size < 2){
            _imgNum++;
            return -10;
        }
        auto ptp= (pts25.end()-1);
        while(count < 11 && pts25size > 0) {
            for(int i= 1;i < 15;i++)
            {
                if((ptp-i)->y-ptp->y < 4) count++;
            }
            pts25size--;
            ptp--;
        }

        vector<int> btlx;
        for(int i= -1;i < 15;i++)
        {
            btlx.push_back((ptp-i)->x);
        }
        sort(btlx.begin(),btlx.end());
//        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
        _uvr[_imgNum].x= btlx[(btlx.size()-1)/2];
        pts25.clear();
        cv::findNonZero(etemp(cv::Rect(_uvr[_imgNum].x,0,1,etemp.rows)),pts25);
        if(pts25.size() < 2){
            _imgNum++;
            return -10;
        }
        vector<int> vy;
        for(auto &p:pts25) vy.push_back(p.y);
        _uvr[_imgNum].x+= mroi.x;
        _uvr[_imgNum].y= vy[(vy.size()-1)/2]+mroi.y;
        _uvr[_imgNum].y= (dr+rdline[1]*(_uvr[_imgNum].x))/rdline[0];

    }
    if(_save > 0)
    {
        //cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
        //cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
        cv::Mat item;
        Input.copyTo(item);
//        cv::imwrite(_logpd+to_string(_imgNum)+"aChannels[2]R.jpg",aChannels[2]);
//        cv::imwrite(_logpd+to_string(_imgNum)+"maskfr.jpg",maskf);
        cv::circle(item,cross,15,cv::Scalar(255,255,0),2,cv::LINE_AA);
        cv::circle(item,_uvl[_imgNum],15,cv::Scalar(255,0,0),2,cv::LINE_AA);
        cv::circle(item,_uvr[_imgNum],15,cv::Scalar(0,255,0),2,cv::LINE_AA);
        cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",item);
    }
    _uv[_imgNum]= _uvl[_imgNum]+_uvr[_imgNum]-cross;
    _imgNum++;
    return 0;
}

bool GetSBuvW::GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1,int SIGN,double ratio0,double ratio1,double discount,double k0,double k1)
{
    //vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;
    vector<cv::Vec4i> lines,slines,selectedLines;
    cv::HoughLinesP(bimg,lines,1,CV_PI/180,_houghThresh*discount,_houghMinLineLength*discount,_houghMaxLineGap*discount);
    _log << "slines: \n";

    double k= 0;
    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if(k > k0 && k < k1) slines.push_back(it);
        //else if(k > 0.08) linesr.push_back(it);
    }

    for(auto &line:slines)
    {
        for(int i= 0;i < 4;i++) _log << line[i] << " ";
        _log << endl;
    }

    int numbins = (int) floor((360.0/(double)_binsize)+0.5);
//    vector<double> weightsL(numbins);
//    vector<vector<Vec4i> > histL(numbins);
//    calcAngleHistogram(linesl, histL, weightsL, binsize);
//    selectMaxAngles(histL, weightsL, numbins,
//            selectedLinesL, angleRange/binsize);

    vector<double> weights(numbins);
    vector<vector<cv::Vec4i> > hist(numbins);
    walgo::calcAngleHistogram(slines, hist, weights, _binsize);
    walgo::selectMaxAngles(hist, weights, numbins,
            selectedLines, _angleRange/_binsize);
    cout << "sl selectedLines.size(): " << selectedLines.size() << endl;
    _log << "sl selectedLines.size(): " << selectedLines.size() << endl;

    if(selectedLines.size() < 1)
    {
        //cout << "bad laser line !!!" << endl;
        _log << "bad laser " << _imgNum << "straight line !!!" << endl;
        return false;
    }

//    Point2d center= Point2d(0,0);
//    Point2d vl,vln,vr,vrn;

//    vector<double> lru,rlu,lrv,rlv;
    vector<double> kv;
    double mk;
    double lineLenth= _houghMinLineLength;
    cv::Vec4i sline;

    cv::Mat cimg;
    cv::cvtColor(bimg,cimg,8);
    _log << to_string(_imgNum)+"straight line: ";
    for(auto &it:selectedLines)
    {
        _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        kv.push_back(k);
        if(_save == 1)
        {
            cv::line(cimg,cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]),cv::Scalar(0,175,0), 1,cv::LINE_AA);
        }
    }
    sort(kv.begin(),kv.end());
    mk= kv[int((kv.size()-1)/2)];

    for(auto &it:selectedLines)
    {
        _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        double temp= cv::norm(cv::Point2d(it[0],it[1])-cv::Point2d(it[2],it[3]));//pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
        cout << "temp: " << temp << endl;
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        //if(lineLenth < temp && k >= mk)
        if(k >= mk)
        {
            lineLenth = temp;
            mk= k;
            sline= it;
        }
    }
    if(_save == 2)
    {
        cv::line(cimg,cv::Point2d(sline[0],sline[1]),cv::Point2d(sline[2],sline[3]),cv::Scalar(0,0,255), 1,cv::LINE_AA);
        imwrite(_logpd+to_string(_imgNum)+"t0.jpg",cimg);
    }
    slcut0.x= sline[0]*ratio0+sline[2]*(1-ratio0);
    slcut0.y= sline[1]*ratio0+sline[3]*(1-ratio0);
    slcut1.x= sline[0]*ratio1+sline[2]*(1-ratio1);
    slcut1.y= sline[1]*ratio1+sline[3]*(1-ratio1);
    return true;
}
