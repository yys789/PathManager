#include "getsbuva.h"
#include "histimage.h"
#include "walgo/lineanalysis.h"
//#include "lineanalysis.h"
#include <random>
//#include <unistd.h>

static default_random_engine rng(time(0));

using namespace std;


bool ranSac(vector<cv::Point> &_pts,int nIterCnt, double dMaxErrorThreshold, double rmsThreshold, cv::Vec4d &LINE,const int k)
{
    bool result= false;

    int nCnt= _pts.size();
    uniform_int_distribution<int> uniform(0,nCnt-1);

    int circleTimes= 0,rtimes= 1;
    int _innerProportion= 30;
    int nBestRightCnt= int(_innerProportion*_pts.size()/100.0);

    while (nIterCnt)
    {
        //1.随机选择7个点
        set<int> sIndexs;
        sIndexs.clear();
        int _snc= 3;
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
        cv::fitLine(points,line,cv::DIST_HUBER,0,0.01,0.01);

        //cout << "dr: " << dr << endl;

        int kk= line[1]/line[0]*k;
        if(kk < 0 || kk > 5) continue;
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

int GetSBuvJ::AddImg(const cv::Mat& Input)
{
    _log << "pic" << to_string(_imgNum) << ": \n";
    cout << "pic" << to_string(_imgNum) << ": \n";
    _uv[_imgNum]= cv::Point2d(-1000,-1000);
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
//        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"change_color.jpg",Input);
//    }

    cv::Mat input,red;
    if(_weldOff){
        if(Input.channels() == 3)
        {
            Bgr2Gray(Input,input,2);
        }
        else if(Input.channels() == 1) input= Input;
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",input);
        //if(_save == 2) Input.copyTo(_img_);
        input.copyTo(red);
    }

    else {
        cv::Mat aChannels[3],blue,green;
        cv::split(Input, aChannels);
        //cout << i << " " << src.channels() << " " << aChannels[1].channels() << endl;
        int th;

        th= cv::threshold(aChannels[0],blue,_bluegv,255,0);75;
        th= cv::threshold(aChannels[1],green,_greengv,255,0);60;
        if(_save == 2)
        {
            imwrite(_logpd+to_string(_imgNum)+"input_blue.jpg",blue);
            imwrite(_logpd+to_string(_imgNum)+"input_green.jpg",green);
        }
        //cv::threshold(aChannels[2],red,5,255,0);

        //cv::dilate(blue,blue,kernel);
        //cv::dilate(green,green,kernel);
        red= aChannels[2]-aChannels[0];
        aChannels[2].setTo(1,blue);
        aChannels[2].setTo(1,green);
        aChannels[2].copyTo(input);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",red);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_redi.jpg",input);
        //if(_save == 2) Input.copyTo(_img_);
    }

    int upBound= _icenter.y-_roiHeightShift;

    cv::Rect2i roisl,roisr;
    int SIGN= -1;
    double wcr= _weldOff?_weldCutR*1.0 : 10.0;
    if(_rightSide)
    {
        SIGN= 1;
        roisl.x= _icenter.x-_roiWidthShift;
        roisl.y= _icenter.y-_roiHeightShift;
        roisl.width= _imgCols-roisl.x;
        roisl.height= _imgRows-roisl.y-int(_roiHeightShift*_weldCutR/wcr);
        roisr.x= 0;
        roisr.y= _icenter.y-_roiHeightShift;
        roisr.width= _icenter.x+_roiWidthShift;
        roisr.height= _imgRows-roisl.y-int(_roiHeightShift*_weldCutR/wcr);
    }
    else
    {
        roisl.x= 0;
        roisl.y= _icenter.y-_roiHeightShift;
        roisl.width= _icenter.x+_roiWidthShift;
        roisl.height= _imgRows-roisl.y-int(_roiHeightShift*_weldCutR/wcr);
        roisr.x= _icenter.x-_roiWidthShift;
        roisr.y= _icenter.y-_roiHeightShift;
        roisr.width= _imgCols-roisr.x;
        roisr.height= _imgRows-roisr.y-int(_roiHeightShift*_weldCutR/wcr);
    }
    cout << roisl;
    if(!(0 <= roisl.x && 0 <= roisl.width && roisl.x + roisl.width <= input.cols && 0 <= roisl.y && 0 <= roisl.height && roisl.y + roisl.height <= input.rows))
    {
        _imgNum++;
        cout << "roi1 error!!";
        return -2;
    }
    cv::Point2d roiShift(roisl.x,roisl.y);//,roiShiftr(roisr.x,roisr,y);

    cv::Mat image,bimg,maskh,maskf;

    if(_doMediumBlur) medianBlur(input(roisr),image,_mediumBlurSize);
    else input(roisr).copyTo(image);
    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"roisr.jpg",image);

    //int th= cv::threshold(image,maskh,th,255,CV_THRESH_OTSU);
    int th= 15,d= 3;
    int count= GetTh(image,th,d);
    cout << "thresh: " << th << " count : " << count << endl;
    _log << "thresh: " << th << " count : " << count << endl;
    cv::threshold(image,maskh,th,255,0);

    //if(_save > 2) imwrite(logpd+to_string(_imgNum)+"ctsl.jpg",maskh);

    vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;

    cv::Mat kernel;
    if(1)
    {
//        maskh= 255-bimg;
//        //maskf= bimg/40;
//        image.copyTo(maskf,maskh);
//        threshold(maskf,bimg,th,255,THRESH_OTSU);
//        if(_save == 2) imwrite(logpd+to_string(_imgNum)+"ct2.png",maskf);
//        bimg+= maskh;
        kernel= cv::Mat(3,5,CV_8UC1,elementArray_35);//Mat::ones(5,7,CV_8UC1);//
        dilate(maskh,bimg,kernel);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"ctsl.jpg",bimg);
    }

    vector<cv::Vec4i> lines,linesl,linesr,slines;

    //if(_save == 2) imwrite("hask"+to_string(_imgNum)+".jpg",maskh);

    cv::HoughLinesP(bimg,lines,1,CV_PI/180,_houghThresh,_houghMinLineLength,_houghMaxLineGap);
    _log << "slines: \n";

    double k= 0;
    for(auto &it:lines)
    {
        k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if( k > 0.07168 && k < 3) slines.push_back(it);
        //else if(k > 0.08) linesr.push_back(it);
    }

    for(auto &line:slines)
    {
        for(int i= 0;i < 4;i++) _log << line[i] << " ";
        _log << endl;
    }

    int numbins = (int) floor((360.0/(double)_binsize)+0.5);


    vector<double> weights(numbins);
    vector<vector<cv::Vec4i> > hist(numbins);
    walgo::calcAngleHistogram(slines, hist, weights, _binsize);
    walgo::selectMaxAngles(hist, weights, numbins,
            selectedLines, _angleRange/_binsize);
    cout << "sl selectedLines.size(): " << selectedLines.size() << endl;
    _log << "sl selectedLines.size(): " << selectedLines.size() << endl;



//    Point2d center= Point2d(0,0);
//    Point2d vl,vln,vr,vrn;

    vector<double> lru,rlu,lrv,rlv;
    vector<double> kv,cv;
    double mk,c,kl,cl,kcr= 0,kcl= 0,cr;
    double lineLenth= _houghMinLineLength;
    cv::Vec4i sline,aline;

    _log << to_string(_imgNum)+"straight line: ";
    for(auto &it:selectedLines)
    {
        _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        k= -SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        kv.push_back(k);
    }
    if(kv.size() > 0) {
        sort(kv.begin(),kv.end());
        mk= kv[int((kv.size()-1)/2)];
    }

    if(selectedLines.size() == 1) sline= selectedLines[0];
    else if(selectedLines.size() > 1) {
        for(auto &it:selectedLines)
        {
            _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
            double temp= pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
            k= -SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
            if(lineLenth <= temp && k >= mk)
            {
                lineLenth = temp;
                sline= it;
            }
        }
    }
    cv::Point2d lcut0,lcut1,rcut0,rcut1,slcut0,slcut1,clcut0,clcut1;
    if(selectedLines.size() > 0)
    {
        cv::Rect2i mroi;
        maskh= 0; cout << sline << endl;
        cv::line(maskh,cv::Point(sline[0]*0.7+sline[2]*0.3,sline[1]*0.7+sline[3]*0.3),cv::Point(sline[0]*0.3+sline[2]*0.7,sline[1]*0.3+sline[3]*0.7),255,29 , cv::LINE_AA);
        if(!_GetMaskRoiB(maskh,mroi)) {
            _imgNum++;
            return -3;
        }
        cv::Mat lineMasked= maskh+1;
        image.copyTo(lineMasked,maskh);

        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rmm.jpg",lineMasked);
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
            return -2;
        th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
        cv::adaptiveThreshold(lineMasked(mroi),maskf,255,0,0,11,-5);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"maskr.jpg",maskf);
        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        cv::Vec4d lineLeft,lineRight,sdline;
        if(pts255.size() > cv::norm(cv::Point(sline[0]*0.7+sline[2]*0.3,sline[1]*0.7+sline[3]*0.3)-cv::Point(sline[0]*0.3+sline[2]*0.7,sline[1]*0.3+sline[3]*0.7))*2)
        {
            fitLine(pts255, sdline, cv::DIST_HUBER, 0, 0.01,0.01);
            double dr= sdline[0]*(sdline[3]+mroi.y+roisr.y)-sdline[1]*(sdline[2]+mroi.x+roisr.x);
            cout << sdline << " " << dr << endl;
            slcut0.x= 0;
            slcut1.x= 720;
            slcut0.y= (dr+sdline[1]*(0))/sdline[0];
            slcut1.y= (dr+sdline[1]*(720))/sdline[0];
            cv::line(input,slcut0,slcut1,cv::Scalar(0), 41,cv::LINE_AA);
            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"inputC.jpg",input);
        }
    }
    //cv::line(_img_,cv::Point2d(sline[0],sline[1])+roiShift,cv::Point2d(sline[2],sline[3])+roiShift,cv::Scalar(255,0,0), 1,cv::LINE_AA);
    //if(_save == 2) imwrite(logpd+to_string(_imgNum)+"t0.jpg",_img_);
    input(roisl).copyTo(image);

    if(0)
    {
        slcut0.x= sline[0]*0.8+sline[2]*0.2;
        slcut0.y= sline[1]*0.8+sline[3]*0.2;
        slcut1.x= sline[0]*0.2+sline[2]*0.8;
        slcut1.y= sline[1]*0.2+sline[3]*0.8;
    }
    else {
        if(!GetHoughLine(red(roisl),slcut0,slcut1,SIGN)) {
            _imgNum++;
            return -3;
        }
    }


//    rcut0.y= (rcut0.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];
//    rcut1.y= (rcut1.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];

    cv::Rect2i mroi;
    maskh= 0;
    cv::line(maskh,slcut0,slcut1,255, 36, cv::LINE_AA);
    if(!_GetMaskRoiB(maskh,mroi)) {
        _imgNum++;
        return -3;
    }
    cv::Mat lineMasked= maskh+1;
    image.copyTo(lineMasked,maskh);

    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lmm.jpg",lineMasked);
    if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
        return -2;
    th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
    cv::adaptiveThreshold(lineMasked(mroi),maskf,255,0,0,11,-5);
    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"maskf.jpg",maskf);
    vector<cv::Point> pts255;
    cv::findNonZero(maskf,pts255);
    cv::Vec4d lineLeft,lineRight,sdline;
    if(pts255.size() > cv::norm(slcut0-slcut1)*2)
        fitLine(pts255, sdline, cv::DIST_HUBER, 0, 0.01,0.01);
    else {
        _imgNum++;
        return -8;
    }
    int nearLineNum= 0;
    for(auto &p:pts255)
    {
        nearLineNum++;
    }
    cout << "nearLineNum: " << nearLineNum << endl;
    //slcut0.x+= roisl.x;
    //slcut1.x+= roisl.x;
    double dl,dr= sdline[0]*(sdline[3]+mroi.y)-sdline[1]*(sdline[2]+mroi.x);
    cout << sdline << " " << dr << endl;
    slcut0.y= (dr+sdline[1]*(slcut0.x))/sdline[0];
    slcut1.y= (dr+sdline[1]*(slcut1.x))/sdline[0];

//    if(_save == 1)
//    {
//        //cv::line(_img_, slcut1, slcut0, cv::Scalar(255,255,255), 1, cv::LINE_AA);
//        cv::line(_img_, slcut0+roiShift, slcut1+roiShift, cv::Scalar(255,255,255), 1, cv::LINE_AA);
//        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"t0.jpg",_img_);
//    }

    if(_rightSide)
    {
        cv::Mat itemp,sinput= input.clone();
        cv::Rect roils(0,0,slcut0.x,slcut1.y);
        cv::line(input(roisl)(roils),cv::Point2d(0,dr/sdline[0]),slcut0,cv::Scalar(0),_laserLineWidthR, cv::LINE_AA);
        if(!(0 <= roils.x && 0 <= roils.width && roils.x + roils.width <= (input(roisl)).cols && 0 <= roils.y && 0 <= roils.height && roils.y + roils.height <= (input(roisl)).rows))
        {
            _imgNum++;
            cout << "roi2 error!!";
            return -2;
        }
        cv::threshold(input(roisl)(roils),itemp,th,255,8);
        //if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",input(roisl)(roils));
        int cx= slcut0.x-(slcut1.x-slcut0.x)*1/3-100;
        if(cx < 0) cx= 0;
        itemp(cv::Rect(0,0,cx,itemp.rows))= 0;
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",itemp);

        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray_35);
        cv::Mat etemp;
        cv::erode(itemp,etemp,kernel);
        vector<cv::Point> pts25;
        cv::findNonZero(etemp,pts25);
        auto ptp= (pts25.end()-1);
        int count= 0,pts25size= pts25.size()/2;
        while(count < 11 && pts25size > 0) {
            for(int i= 1;i < 15;i++)
            {
                if((ptp-i)->y-ptp->y < 4) count++;
            }
            pts25size--;
            ptp--;
        }
        if(pts25size < 2){
            _imgNum++;
            return -10;
        }
        vector<int> btlx;
        for(int i= -1;i < 15;i++)
        {
            btlx.push_back((ptp-i)->x);
        }
        sort(btlx.begin(),btlx.end());
//        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
        cv::Rect roilPts= cv::Rect(0,0,btlx[(btlx.size()-1)/2],itemp.rows);


//        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
//        cv::erode(itemp,itemp,kernel);
//        cv::erode(itemp,itemp,kernel);

//        etemp= 0;
//        etemp(roilPts)= itemp(roilPts);
        itemp(roilPts)= 0;
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"it.jpg",itemp);
        lines.clear();slines.clear();
        cv::HoughLinesP(itemp,lines,1,CV_PI/180,30,33,15);
//        if(lines.size() == 0)
//        {
//            _imgNum++;
//            return -2;
//        }

        kv.clear();
        for(auto &it:lines)
        {
            k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if( k > 0.0968 && k < 2.9) //slines.push_back(it);
            {
                //kv.push_back();
                cv::line(itemp, cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]), cv::Scalar(0), 1, cv::LINE_AA);
                kv.push_back(k);
            }
        }
        if(kv.size() < 1)
        {
            roils.x= roilPts.width;
            roils.width= slcut0.x+15-roils.x;
            cv::threshold(sinput(roisl)(roils),itemp,th,255,8);
            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ur.jpg",sinput(roisl)(roils));

            cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray_35);
            cv::Mat etemp;
            cv::erode(itemp,etemp,kernel);
            vector<cv::Point> pts25;
            cv::findNonZero(etemp,pts25);
            auto ptp= pts25.begin();
            int count= 0,pts25size= pts25.size()/2;
            while(count < 11 && pts25size > 0) {
                count= 0;
                for(int i= 0;i < 15;i++)
                {
                    if((ptp+i)->y-ptp->y < 4) count++;
                }
                pts25size--;
                ptp++;
            }
            if(pts25size < 2){
                _imgNum++;
                return -10;
            }
            vector<int> btlx;
            for(int i= -1;i < 15;i++)
            {
                btlx.push_back((ptp+i)->x);
            }
            sort(btlx.begin(),btlx.end());
            double x= btlx[(btlx.size()-1)/2]+roils.x;
            double y= (dr+sdline[1]*(x))/sdline[0];
            _uv[_imgNum]= cv::Point2d(x,y)+roiShift;
            if(_save > 0)
            {
                cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
                cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
            }
        }
        else
        {
            sort(kv.begin(),kv.end());
            mk= kv[int((kv.size()-1)/2)];
            for(auto &it:lines)
            {
                k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
                //_log << "k: " << k << endl;
                if(abs(k-mk) < 0.1 && k >= mk)
                {
                    mk= k;
                    aline= it;
                }
            }

            maskh= 0;
            cv::line(maskh,cv::Point(aline[0],aline[1]),cv::Point(aline[2],aline[3]),255, 29, cv::LINE_AA);
            if(!_GetMaskRoiB(maskh,mroi)) {
                _imgNum++;
                return -3;
            }
            cv::Mat lineMasked= maskh+2;
            image.copyTo(lineMasked,maskh);

            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lss.jpg",lineMasked);
            if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
            {
                _imgNum++;
                cout << "roi error";
                return -2;
            }
            th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
            vector<cv::Point> pts255;
            cv::findNonZero(maskf,pts255);
            cv::Vec4d adline;

            if(false)//ranSac(pts255, 25, 3.5, 2.5, adline,-1))
            {
                dl= adline[0]*(adline[3]+mroi.y)-adline[1]*(adline[2]+mroi.x);
                cv::line(itemp,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

                if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

                _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;

                if(_save > 0)
                {
                    cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
                    cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
                }
            }

            else
            {
                if(pts255.size() > cv::norm(cv::Point(aline[0],aline[1])-cv::Point(aline[2],aline[3]))*2)
                    cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
                else {
                    _imgNum++;
                    return -8;
                }
                dl= adline[0]*(adline[3]+mroi.y)-adline[1]*(adline[2]+mroi.x);
                cv::line(itemp,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

                if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

                _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;

                if(_save > 0)
                {
                    cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
                    cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
                }
            }
        }
    }
    else
    {
        cout << "leftLine.\n";
        cv::Mat itemp,sinput;
        //input.copyTo(itemp);
        sinput= input(roisl).clone();
        cv::Rect roils(slcut1.x,0,sinput.cols-slcut1.x,slcut0.y);
        cv::line(input(roisl),cv::Point2d(0,dr/sdline[0]),cv::Point2d(sinput.cols,(dr+sdline[1]*sinput.cols)/sdline[0]),cv::Scalar(0),_laserLineWidthR, cv::LINE_AA);
        if(!(0 <= roils.x && 0 <= roils.width && roils.x + roils.width <= sinput.cols && 0 <= roils.y && 0 <= roils.height && roils.y + roils.height <= sinput.rows))
        {
            _imgNum++;
            cout << "roi2 error!!";
            return -2;
        }
        cv::threshold(input(roisl)(roils),itemp,th,255,8);
        int cx= (slcut1.x-slcut0.x)*1.0/3+100;
        if(cx > itemp.cols) cx= itemp.cols;
        itemp(cv::Rect(cx,0,itemp.cols-cx,itemp.rows))= 0;
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",itemp);
        //if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",input(roisl)(roils));

        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray_35);
        cv::Mat etemp;
        cv::erode(itemp,etemp,kernel);
        vector<cv::Point> pts25;
        cv::findNonZero(etemp,pts25);
        auto ptp= (pts25.end()-1);
        int count= 0,pts25size= pts25.size()/2;
        while(count < 11 && pts25size > 0) {
            for(int i= 1;i < 15;i++)
            {
                if((ptp-i)->y-ptp->y < 4) count++;
            }
            pts25size--;
            ptp--;
        }
        if(pts25size < 2){
            _imgNum++;
            return -10;
        }
        vector<int> btlx;
        for(int i= -1;i < 15;i++)
        {
            btlx.push_back((ptp-i)->x);
        }
        sort(btlx.begin(),btlx.end());
//        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);


//        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
//        cv::erode(itemp,itemp,kernel);
//        cv::erode(itemp,itemp,kernel);

//        etemp= 0;
//        etemp(roilPts)= itemp(roilPts);
        itemp(roilPts)= 0;
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"it.jpg",itemp);
        lines.clear();slines.clear();
        cv::HoughLinesP(itemp,lines,1,CV_PI/180,30,30,12);
//        if(lines.size() == 0)
//        {
//            _imgNum++;
//            return -2;
//        }

        kv.clear();
        for(auto &it:lines)
        {
            k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if( k > 0.0968 && k < 2.9) //slines.push_back(it);
            {
                //kv.push_back();
                cv::line(itemp, cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]), cv::Scalar(0), 1, cv::LINE_AA);
                kv.push_back(k);
            }
        }
        if(kv.size() < 1)
        {
            _imgNum++;
            return -10;
        }
        sort(kv.begin(),kv.end());
        mk= kv[int((kv.size()-1)/2)];
        for(auto &it:lines)
        {
            k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if(abs(k-mk) < 0.1 && k >= mk)
            {
                mk= k;
                aline= it;
            }
        }

        maskh= 0;
        cv::line(maskh,cv::Point(aline[0]+roils.x,aline[1]),cv::Point(aline[2]+roils.x,aline[3]),255, 29, cv::LINE_AA);
        if(!_GetMaskRoiB(maskh,mroi)) {
            _imgNum++;
            return -3;
        }
        cv::Mat lineMasked= maskh+2;
        image.copyTo(lineMasked,maskh);

        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lss.jpg",lineMasked);
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
        {
            _imgNum++;
            cout << "roi error";
            return -2;
        }
        th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        cv::Vec4d adline;

        if(1)
        {
            if(pts255.size() > cv::norm(cv::Point(aline[0],aline[1])-cv::Point(aline[2],aline[3]))*2)
                cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
            else {
                _imgNum++;
                return -8;
            }
            dl= adline[0]*(adline[3]+mroi.y)-adline[1]*(adline[2]+mroi.x);
            cv::line(sinput,cv::Point(0,dl/adline[0]),cv::Point((adline[0]*sinput.rows-dl)/adline[1],sinput.rows),255, 1, cv::LINE_AA);

            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);
            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"sinput.jpg",sinput);
            _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;

            if(_save > 0)
            {
                cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
                cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
            }
        }
    }

/*
    else if(_d2d == 2)
    {
        cv::Mat itemp;
        //input.copyTo(itemp);

        cv::Rect roils;
        cv::line(image,cv::Point2d(0,dr/sdline[0]),slcut1,cv::Scalar(0),5, cv::LINE_AA);
        cv::threshold(image,itemp,th,255,8);

        lines.clear();slines.clear();
        cv::HoughLinesP(itemp,lines,1,CV_PI/180,38,40,5);

        if(lines.size() == 0)
        {
            _imgNum++;
            return -2;
        }
        //cv::Scharr
        kv.clear();
        for(auto &it:lines)
        {
            k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if( k > 0.0168 && k < 9) //slines.push_back(it);
            {
                //kv.push_back();
                cv::line(itemp, cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]), cv::Scalar(0), 1, cv::LINE_AA);
                kv.push_back(k);
            }
        }
        sort(kv.begin(),kv.end());
        mk= kv[int((kv.size()-1)/2)];
        for(auto &it:lines)
        {
            k= -SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if(abs(k-mk) < 0.1) aline= it;
        }

        maskh= 0;
        cv::line(maskh,cv::Point(aline[0],aline[1]),cv::Point(aline[2],aline[3]),255, 28, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        cv::Mat lineMasked= maskh+2;
        image.copyTo(lineMasked,maskh);

        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lss.jpg",lineMasked);
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
        {
            _imgNum++;
            cout << "roi error";
            return -2;
        }
        th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        cv::Vec4d adline;
        if(pts255.size() > cv::norm(cv::Point(aline[0],aline[1])-cv::Point(aline[2],aline[3]))*2)
            fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
        else {
            _imgNum++;
            return -8;
        }
        dl= adline[0]*(adline[3]+mroi.y)-adline[1]*(adline[2]+mroi.x);
        cv::line(itemp,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

        _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;

        if(_save > 0)
        {
            cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
            cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
        }
    }
    else
    {
        cv::Mat itemp;
        //cv::Rect roils(0,0,slcut0.x,slcut1.y);
        cv::line(image,cv::Point2d(0,dr/sdline[0]),cv::Point2d(-dr/sdline[1],0),cv::Scalar(0),5, cv::LINE_AA);
        cv::threshold(image,itemp,th,255,8);
        vector<cv::Point> pts255;
        cv::findNonZero(itemp,pts255);
        cv::Vec4d adline;
        if(ranSac(pts255, 25, 3.5, 2.5, adline,1))
        {
            dl= adline[0]*(adline[3])-adline[1]*(adline[2]);
            cv::line(itemp,cv::Point(itemp.cols,(dl+itemp.cols*adline[1])/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

           if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

            _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;

            if(_save > 0)
            {
                cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
                cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
            }
        }
    }
*/
    //Mat inputc= input.clone();
    //line(inputc,rcut0,Point2d(0,dr/sdline[0]), 7, 18, LINE_AA);

    _imgNum++;
    return 0;
}

bool GetSBuvJ::GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1,int SIGN,double ratio0, double k0,double k1)
{
    //vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;
    vector<cv::Vec4i> lines,slines,selectedLines;
    cv::Mat timg(bimg.rows,bimg.cols,CV_8UC1);
    if(true || _weldOff)
    {
        int subCols= 30,subi= bimg.cols/subCols;
        for(int i= 0;i < subi;i++)
        {
            int th= cv::threshold(bimg(cv::Rect(i*subCols,0,subCols,bimg.rows)),timg(cv::Rect(i*subCols,0,subCols,timg.rows)),55,255,8);
            _log << "th= " << th << "\n";
        }
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"t1.jpg",timg);
    }
    else {
        cv::threshold(bimg,timg,_ht,255,0);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"t1.jpg",timg);
    }
    cv::HoughLinesP(timg,lines,1,CV_PI/180,_houghThresh,_houghMinLineLength,_houghMaxLineGap);
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
        if(lineLenth < temp && k >= mk)
        {
            lineLenth = temp;
            sline= it;
        }
    }
    if(_save == 2)
    {
        cv::line(cimg,cv::Point2d(sline[0],sline[1]),cv::Point2d(sline[2],sline[3]),cv::Scalar(0,0,255), 1,cv::LINE_AA);
        imwrite(_logpd+to_string(_imgNum)+"t0.jpg",cimg);
    }
    double ratio1= 1-ratio0;
    slcut0.x= sline[0]*ratio0+sline[2]*(1-ratio0);
    slcut0.y= sline[1]*ratio0+sline[3]*(1-ratio0);
    slcut1.x= sline[0]*ratio1+sline[2]*(1-ratio1);
    slcut1.y= sline[1]*ratio1+sline[3]*(1-ratio1);
    return true;
}

/*
    cv::Rect2i roi;
    roi.y= upBound;
    roi.height= height;

    int rightBound= _imgCols;
    roi.x= sline[2]+roisl.x;
    roi.width= rightBound-roi.x;

    //Mat imagel= input(roi).clone();
    Mat imagec= input(roi);
    Mat img;
    //threshold(imagel,img,15,255,CV_THRESH_OTSU);
    th= 15;
    count= getTh(itemp(roi),th,d);
    cv::threshold(imagec,img,th,255,0);
    cout << "thresh: " << th << " count: " << count << endl;
    kernel= Mat(3,5,CV_8UC1,elementArray35);//Mat::ones(5,7,CV_8UC1);//
    dilate(img,bimg,kernel);
    if(_save > 0) imwrite(logpd+to_string(_imgNum)+"ctcl.jpg",bimg);

    houghMinLineLength= roi.width-50;
    houghThresh= 0.75*houghMinLineLength;
    houghMaxLineGap= 36;
    rho= 1;
    theta= 1;
    lines.clear();
    HoughLinesP(bimg,lines,1*rho,CV_PI*theta/180,houghThresh,houghMinLineLength,houghMaxLineGap);

    _log << "sClines: \n";
    slines.clear();
    for(auto it:lines)
    {
        double k= (it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if(k > 0.035 && k < 12) slines.push_back(it);
        //else if(k > 0.08) linesr.push_back(it);
    }


    for(auto line:slines)
    {
        for(int i= 0;i < 4;i++) _log << line[i] << " ";
        _log << endl;
    }
    selectedLines.clear();
    vector<double> weightsC(numbins);
    vector<vector<Vec4i> > histC(numbins);
    calcAngleHistogram(slines, histC, weightsC, binsize);
    selectMaxAngles(histC, weightsC, numbins,
            selectedLines, angleRange/binsize);
    cout << "cl selectedLines.size(): " << selectedLines.size() << endl;
    _log << "cl selectedLines.size(): " << selectedLines.size() << endl;

//    if(selectedLines.size() < 1)
//    {
//        //cout << "bad laser line !!!" << endl;
//        _log << "bad laser " << _imgNum << "straight line !!!" << " thresh: " << th << endl;
//        _imgNum++;
//        return -3;
//    }

//    Point2d center= Point2d(0,0);
//    Point2d vl,vln,vr,vrn;

    _log << to_string(_imgNum)+"curve line: ";
    for(auto &it:selectedLines)
    {
        _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        k= (it[1]-it[3]*1.0)/(it[0]-it[2]);
        kv.push_back(k);
        if(_save > 0)
        {
            line(_img_, Point(it[0]+roi.x,it[1]+roi.y),  Point(it[2]+roi.x,it[3]+roi.y), Scalar(0,175,0), 1, LINE_AA);
        }
    }
    if(_save > 0) imwrite(logpd+to_string(_imgNum)+"rg.jpg",_img_);

    bool soc= false;
    if(selectedLines.size() > 1)
    {

        sort(kv.begin(),kv.end());
        mk= kv[int((kv.size()-1)/2)];

        kcr= 0;
        for(auto &it:selectedLines)
        {
            _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
            double temp= pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
            k= (it[1]-it[3]*1.0)/(it[0]-it[2]);
            if(kcr < temp && k >= mk)
            {
                kcr = temp;
                sline= it;
            }
        }

        clcut0.x= sline[0]*0.90+sline[2]*0.10;
        clcut0.y= sline[1]*0.90+sline[3]*0.10;
        clcut1.x= sline[0]*0.35+sline[2]*0.65;
        clcut1.y= sline[1]*0.35+sline[3]*0.65;

//        line(_img_,slcut0+Point2d(roi.x,roi.y),slcut1+Point2d(roi.x,roi.y), Scalar(20,0,20), 36, LINE_AA);
//        if(_save > 0) imwrite(logpd+to_string(_imgNum)+"t0.jpg",_img_);
        soc= true;
    }
    else
    {

    }
    //Point2d lcut0,lcut1,rcut0,rcut1,slcut0,slcut1;
    //Point roishift(roisl.x,roisl.y);
    //cout << roishift;



    Point2d cross;
    int poc= 1;//++
    getConfigEntry(_config, "PYTHON_OR_CPP", poc);//++
    if(soc && _imgNum > 43)
    {

//        rcut0.y= (rcut0.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];
//        rcut1.y= (rcut1.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];

        cv::Rect2i mroi;

        maskh= img*0;
        line(maskh,clcut0,clcut1,255, 36, LINE_AA);
        GetMaskRoi(maskh,mroi);
        Mat lineMasked= maskh+2;
        imagec.copyTo(lineMasked,maskh);

        cv::threshold(lineMasked(mroi),maskf,th,255,CV_THRESH_OTSU);
        if(_save == 1) imwrite(logpd+to_string(_imgNum)+"rlmm.jpg",maskf);
        vector<Point> pts255;
        findNonZero(maskf,pts255);
        //Vec4d lineLeft;
        fitLine(pts255, lineLeft, DIST_HUBER, 0, 0.01,0.01);

        dl= lineLeft[0]*(lineLeft[3]+roi.y+mroi.y)-lineLeft[1]*(lineLeft[2]+roi.x+mroi.x); cout << lineLeft << " " << dl << endl;
        cross.x= -(dl*lineRight[0]-dr*lineLeft[0])/(lineLeft[1]*lineRight[0]-lineRight[1]*lineLeft[0]);
        cross.y= (dl*lineRight[1]-dr*lineLeft[1])/(lineLeft[0]*lineRight[1]-lineRight[0]*lineLeft[1]);
        //line(_img_,Point2d(cross.x,(dr+lineRight[1]*(cross.x))/lineRight[0]),lcut1,Scalar(0,255,0), 1 , LINE_AA);
        //circle(_img_,cross,15,Scalar(0,0,255),2,LINE_AA);
    }
    else if(poc == 0)//++
    {
        cout << "curve fit !!!" << endl;
        vector<Point> pts;
        findNonZero(img,pts);
        cout << "pts.size(): " << pts.size() << endl;
        fstream uvlog;
        uvlog.open(logpd+to_string(_imgNum)+"cuvlog.txt",ios::out);
        for(auto &p:pts) uvlog << p.x << " " << p.y << endl;
        uvlog.close();

        char command[99];
        string ts= ("/home/yongxiang/anaconda3/bin/python3 polyfit_ransac.py "+logpd+to_string(_imgNum)).c_str();
        strcpy(command,ts.c_str());
        system(command);
        cout << command << endl;
        ifstream ifsEra(logpd+to_string(_imgNum)+"fitcurve.txt");//"/home/yongxiang/build-blades-getxuv-Desktop_Qt_5_9_3_GCC_64bit-Debug/uvlog/9_24_11_24_53/8fitcurve.txt");//
        if (!ifsEra)
        {
            cout << "can't read fitlog.txt!" << endl;
            _imgNum++;
            return -2;
        }
        string ls;
        double cx,cy,dl,tdl= 222.888;
        Point2d c_r= rcut0-rcut1;
        double lenthc_r= norm(c_r);

        vector<double> adlv;
        while(std::getline(ifsEra, ls))
        {
            istringstream sl(ls);
            sl >> cx >> cy;
            cx+= roi.x;
            cy+= roi.y;
            //cout << cx << " " << cy << endl;
            Point2d cp= Point2d(cx,cy);
            if(_save == 1 && cx-int(cx) == 0) circle(_img_,cp,1,Scalar(0,255,0),1);
            dl= c_r.cross(cp-rcut1)/lenthc_r;
            adlv.push_back(abs(dl));
            if(abs(dl) < tdl)
            {
                tdl= abs(dl);
                cross= cp;
            }
        }
        int dlCount= 0;
        for(auto &a:adlv)
        {
            //cout << a << " ";
            if(a < 1) dlCount++;
        }
        cout << tdl << " " << dlCount << endl;
        if(dlCount > 28)
        {
            _log << "bad fit line" << endl;
            cout << "bad fit line" << endl;
            _imgNum++;
            return -3;
        }
        ifsEra.close();
        circle(_img_,cross,15,Scalar(0,0,255),2,LINE_AA);
        if(0)
        {
            Mat maskl= Mat::zeros(input.rows,input.cols,CV_8UC1);

            int ld= 128;
            lcut0.x= sline[0]+roi.x-lineRight[1]*ld;
            lcut0.y= sline[1]+roi.y+lineRight[0]*ld;
            lcut1.x= 396;
            lcut1.y= (dr+ld)+lineRight[1]*lcut1.x;
            line(maskl,lcut0,lcut1, 255, ld*2-16, LINE_AA);

            vector<Point> pts;
            findNonZero(maskl,pts);
            int x1= (pts.begin())->x,x2= (pts.begin())->x;
            for(auto &p:pts)
            {
                if(p.x < x1) x1=p.x;
                if(p.x > x2) x2=p.x;
            }
            roi.x= x1;
            roi.width= x2-x1;
            roi.y= (pts.begin())->y;
            roi.height= (pts.end()-1)->y-roi.y+1;

            Mat img= Mat::zeros(roi.height,roi.width,CV_8UC1)+7;
            input(roi).copyTo(img,maskl(roi));
            threshold(img,img,15,255,CV_THRESH_OTSU);
            //    th= 15,count= 30;
            //    nt= getTh(img,th,count);
            //    cv::threshold(img,img,th,255,0);
            //    cout << "thresh: " << th << " count and nt: " << count << " " << nt << endl;
            if(_save == 1) imwrite(logpd+to_string(_imgNum)+"lm.jpg",img);
        }
    }
    else if(poc == 1)//++
    {
        cout << "curve fit !!!" << endl;
        vector<Point> pts;
        findNonZero(img,pts);
        cout << "pts.size(): " << pts.size() << endl;
        vector<Point2d> cvpts;

        fitCurve(pts,cvpts);
        string ls;
        double cx,cy,dl,tdl= 222.888;
        Point2d c_r= slcut0-slcut1;
        double lenthc_r= norm(c_r);

        if(cvpts.size() < 66)
        {
            _log << "fitCurve failure" << endl;
            _imgNum++;
            return -13;
        }
        vector<double> adlv;
        for(auto &p:cvpts)
        {
            Point2d cp= Point2d(p.x+roi.x,p.y+roi.y);
            if(_save == 1 && p.x-int(p.x) == 0) circle(_img_,cp,1,Scalar(0,255,0),1);
            dl= c_r.cross(cp-slcut1)/lenthc_r;
            adlv.push_back(abs(dl));
            if(abs(dl) < tdl)
            {
                tdl= abs(dl);
                cross= cp;
            }
        }
        int dlCount= 0;
        for(auto &a:adlv)
        {
            //cout << a << " ";
            if(a < 1) dlCount++;
        }
        cout << tdl << " " << dlCount << endl;
        if(dlCount > 28)
        {
            _log << "bad fit line" << endl;
            cout << "bad fit line" << endl;
            _imgNum++;
            return -3;
        }
    }
    else //++
    {
        _log << "poc ??" << endl;
        _imgNum++;
        return -13;
    }
    line(_img_,Point2d(cross.x,(dr+lineRight[1]*(cross.x))/lineRight[0]),slcut1,Scalar(0,255,0), 1 , LINE_AA);
    if(_save == 1)
    {
        input= Input.clone();
        //line(input,cross,Point(slcut1.x+roi.x,(dl+lineLeft[1]*(slcut1.x+roi.x))/lineLeft[0]),Scalar(0,255,0),1,LINE_AA);
        //line(input,cross,Point(cross.x-200,(dr+lineRight[1]*(cross.x-200))/lineRight[0]),Scalar(0,255,0),1,LINE_AA);
        circle(input,cross,15,Scalar(0,255,0),2,LINE_AA);
        imwrite(logpd+to_string(_imgNum)+"rs.jpg",input);
    }

    //if(_save == 1) imwrite(logpd+to_string(_imgNum)+"sl.jpg",_img_);

    if(f == 1) cross.x= _imgCols-1-cross.x;
    _uv[_imgNum]= cross;

//    if(_save == 1)
//    {
//        _img_= Input.clone();
//        circle(_img_,cross,15,Scalar(0,255,0),2,LINE_AA);
//        imwrite(logpd+to_string(_imgNum)+"lsl.jpg",_img_);
//    }

//    else
//    {
int rt= 0;
//        Point2d cp;
//        double c0l,c1279r;
//        Mat img2;
//        img2= input(roi).clone();//.copyTo(img,maskh);
//        int rt= getxuv(selectedLinesL,selectedLinesR,img2,cp,c0l,c1279r,SH);
//        if(rt == 0)
//        {
//            cp.y+= roi.y;
//            _uv[_imgNum]= cp;//Point2d(cp.x,cp.y+roi.y);
//            if(_save == 2)
//            {
//                Mat cimg;
//                cvtColor(input,cimg,CV_GRAY2BGR);
//                line(cimg,Point2d(0,c0l+roi.y),cp,Scalar(0,255,0),1,LINE_AA);
//                line(cimg,Point2d(1279,c1279r+roi.y),cp,Scalar(0,255,0),1,LINE_AA);
//                circle(cimg,cp,15,Scalar(0,255,0),2);
//                imwrite(logpd+to_string(_imgNum)+"sl.jpg",cimg);

//                fstream uvlog;
//                uvlog.open("cc/"+to_string(_imgNum)+"uv.log",ios::out);
//                threshold(input,cimg,29,255,0);
//                vector<Point> pts255;
//                findNonZero(cimg,pts255);
//                Point2d lpv= Point2d(0,c0l+roi.y)-cp;
//                Point2d rpv= Point2d(1279,c1279r+roi.y)-cp;
//                double lenthl= norm(lpv);
//                double lenthr= norm(rpv);
//                Point2d dp;
//                for (auto &p:pts255)
//                {
//                    dp.x= double(p.x);
//                    dp.y= double(p.y);
//                    if(abs(lpv.cross(dp-cp)/lenthl) < raserLineWidth*2 || abs(rpv.cross(dp-cp)/lenthr) < raserLineWidth*2 || norm(dp-cp) < 100)
//                        uvlog << p.x << " " << p.y << endl;
//                }
//                uvlog.close();
//            }
//        }
        _imgNum++;
        return rt;
    //}
}
*/

/*
void walgo::fitCurve(vector<Point> &pts,vector<Point2d> &cvpts)
{
    if(0)
    {
        deque<double> x;
        deque<double> y;
        for(auto &p:pts)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        PolyModel1D model;
        model.setParams({2});
        model.setData(y,x);
        model.build();
        cout << "RMS = " << model.getRMS() << endl;
        double xmax= *(max_element(x.begin(),x.end()))+80;
        double xmin= *(min_element(x.begin(),x.end()));
        for(double u= xmin;u < xmax;u+= 0.5)
        {
            cvpts.push_back(Point2d(u,model.model(u)));
        }
        return;
    }
    else
    {
        PolyModel1D* model= ransac(pts,36,6,12);
        if(model == NULL) return;
        vector<double> x;
        vector<double> y;
        for(auto &p:pts)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        double xmax= *(max_element(x.begin(),x.end()));
        double xmin= *(min_element(x.begin(),x.end()))-80;
        for(double u= xmin;u < xmax;u+= 0.5)
        {
            cvpts.push_back(Point2d(u,model->model(u)));
        }
    }
}

PolyModel1D* walgo::ransac(vector<Point> &pts,int nIterCnt,double dMaxErrorThreshold,double rmsThreshold)
{
    int nCnt= pts.size();
    //int snc= nCnt/50;
    int snc= 15;
    default_random_engine rng(time(0));
    uniform_int_distribution<int> uniform(0,nCnt);

    PolyModel1D* rightModel= new PolyModel1D();
    int nBestRightCnt = 0;
    int circleTimes= 0;
    int rtimes= 2;
    bool getRight= false;
    while (nIterCnt)
    {
        //1.随机选择两个点
        set<int> sIndexs;
        sIndexs.clear();
        while (1)
        {
            sIndexs.insert(uniform(rng));
            if (sIndexs.size() == snc)
                break;
        }

        //2.得到模型参数
        //set取值要注意
        vector<Point> vecPoints;
        vecPoints.clear();
        for (auto iter = sIndexs.begin(); iter != sIndexs.end(); iter++)
        {
            vecPoints.push_back(pts[*iter]);
        }

        deque<double> x;
        deque<double> y;
        for(auto &p:vecPoints)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        PolyModel1D* model= new PolyModel1D();
        model->setParams({2});
        model->setData(y,x);
        model->build();
        double rms= model->getRMS();

        if(rms > rmsThreshold*rtimes)
        {
            cout << "RMS = " << rms << endl;
            delete model;
            circleTimes++;
            if(circleTimes > 33)
            {
                if(++rtimes > 5) return NULL;;
                circleTimes= 0;
            }
            continue;
        }

        //3.统计内点个数
        int nRightCnt = 0;
        double distance= 10000;
        vecPoints.clear();
        for (int i = 0; i < nCnt; i++)
        {
            distance= abs(pts[i].y-model->model(pts[i].x));
            if (distance < dMaxErrorThreshold)
            {
                vecPoints.push_back(pts[i]);
                nRightCnt++;
            }
        }
        x.clear();y.clear();
        for(auto &p:vecPoints)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        model->setData(y,x);
        model->build();
        rms= model->getRMS();

        //4.保存内点个数最多的模型
        if (nRightCnt > nBestRightCnt && rms < rmsThreshold)
        {
            nBestRightCnt= nRightCnt;
            rmsThreshold= rms;
            rightModel= model;
            cout << "RRMS = " << rms << endl;
            getRight= true;
        }
        nIterCnt--;
    }
    if(getRight)
        return rightModel;
    else
        return NULL;
}

void walgo::getXuvV::secondHough(Mat& rimg,vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,int& SH)
{
    //Mat img= 0;
    vector<int> terminalVec;
    int median= 0;
    for(int i= _imgNum-1;i >= 0;i--)
    {
        if(_uv[i].x > 300/adjuster)
        {
            median= _uv[i].x;
            break;
        }
    }
    Rect2i roi;
    if(selectedLinesL.size() < selectedLinesR.size())
    {
        selectedLinesL.clear();
        if(median == 0)
        {
            for(auto it:selectedLinesR) terminalVec.push_back(it[0]);
            std::nth_element(terminalVec.begin(),terminalVec.begin()+terminalVec.size()/2,terminalVec.end());
            median= terminalVec[terminalVec.size()/2];
            if(median > 386/adjuster) median-= 64/adjuster;
            else return;
        }
        roi= Rect2i(median,0,rimg.cols-median,rimg.rows);
        rimg(roi)= 0;
        SH= -1;
    }
    else
    {
        selectedLinesR.clear();
        if(median == 0)
        {
            for(auto it:selectedLinesL) terminalVec.push_back(it[2]);
            std::nth_element(terminalVec.begin(),terminalVec.begin()+terminalVec.size()/2,terminalVec.end());
            median= terminalVec[terminalVec.size()/2];
            if(median < 894/adjuster) median+= 64/adjuster;
            else return;
        }
        roi= Rect2i(0,0,median,rimg.rows);
        rimg(roi)= 7;
        SH= 1;
    }
    Mat maskh,maskf;
    threshold(rimg,maskh,29,255,THRESH_OTSU);
    imwrite(logpd+to_string(_imgNum)+"tt.png",maskh);
    //kernel= Mat(3,5,CV_8UC1,elementArray35);//Mat::ones(5,7,CV_8UC1);//
    //dilate(maskh,maskh,kernel);
    //imwrite(logpd+to_string(_imgNum)+"td.png",maskh);
    vector<Vec4i> lines,linesl,linesr;


    int houghThresh = 58;
    int houghMinLineLength = 320;
    int houghMaxLineGap = 175;

//    int houghThresh = 64;
//    int houghMinLineLength = 108;
//    int houghMaxLineGap = 55;

    double rho= 2;
    double theta= 2;
    HoughLinesP(maskh,lines,1*rho,CV_PI*theta/180,houghThresh,houghMinLineLength,houghMaxLineGap);

    for(auto it:lines)
    {
        double k= (it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if(k < -0.05 && k > -6) linesl.push_back(it);
        else if(k > 0.05 && k < 6) linesr.push_back(it);
    }
    if((linesl.size() < 1 && SH == -1) || (linesr.size() < 1 && SH == 1))
    {
        linesl.clear();
        linesr.clear();
//        houghThresh = 90;
//        houghMinLineLength = 139;
//        houghMaxLineGap = 39;
        houghThresh = 64;
        houghMinLineLength = 128;
        houghMaxLineGap = 49;

        lines.clear();
        maskh= 0;
        threshold(rimg,maskh,30,255,0);
        HoughLinesP(maskh,lines,1*rho,CV_PI*theta/180,houghThresh,houghMinLineLength,houghMaxLineGap);
        for(auto it:lines)
        {
            double k= (it[3]-it[1]*1.0)/(it[2]-it[0]);
            //_log << "k: " << k << endl;
            if(k < -0.05 && k > -3) linesl.push_back(it);
            else if(k > 0.05 && k < 3) linesr.push_back(it);
        }
    }

    int angleRange = 0;
    //getConfigEntry(_config, "PATH_ANGLE_RANGE", angleRange);
    int binsize = _config["ANGLE_BIN_SIZE"];
    int numbins = (int) floor((360.0/(double)binsize)+0.5);
    vector<double> weightsL(numbins);
    vector<vector<Vec4i> > histL(numbins);
    if(roi.x == 0)
    {
        selectedLinesR.clear();
        calcAngleHistogram(linesr, histL, weightsL, binsize);
        selectMaxAngles(histL, weightsL, numbins,
                        selectedLinesR, angleRange/binsize);
    }
    else
    {
        selectedLinesL.clear();
        calcAngleHistogram(linesl, histL, weightsL, binsize);
        selectMaxAngles(histL, weightsL, numbins,
                        selectedLinesL, angleRange/binsize);
    }

//    vector<double> weightsR(numbins);
//    vector<vector<Vec4i> > histR(numbins);
//    calcAngleHistogram(linesr, histR, weightsR, binsize);
//    selectMaxAngles(histR, weightsR, numbins,
//            selectedLinesR, angleRange/binsize);

    _log << selectedLinesL.size() << " " << selectedLinesR.size() << endl;
}

int getXuvV::getxuv(vector<Vec4i>& selectedLinesL,vector<Vec4i>& selectedLinesR,Mat& img,Point2d& cp,double& c0l,double& c1279r,int SH)
{
    Point2d center= Point2d(0,0);
//    Vec4i lLine{0,0,0,0};
//    Vec4i rLine{0,0,0,0};
//    double cx,cy,vxl,vyl,vxr,vyr,dl,dr;
    Point2d vl,vln,vr,vrn;

    if(0)
    {

    }
    else
    {
        vector<double> lru,rlu,lrv,rlv;
        if(_save == 2) cvtColor(img,_img_, CV_GRAY2BGR);
        vector<double> kv,cv;
        double k,c,kl,cl,kr,cr;
        _log << to_string(_imgNum)+"leftline: ";
        for(auto it:selectedLinesL)
        {
            _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
            lru.push_back(it[2]); lrv.push_back(it[3]);
            k= (it[1]-it[3]*1.0)/(it[0]-it[2]);
            kv.push_back(k);
            c= it[1]-k*it[0];
            cv.push_back(c);
            _log << "k,c= " << k << " " << c << endl;
            if(_save == 2) line(_img_, Point(it[0],it[1]),  Point(it[2],it[3]), Scalar(0,195,0), 1, LINE_AA);
        }
        sort(kv.begin(),kv.end());
        if(kv.size()%2 == 1) kl= kv[(kv.size()-1)/2];
        else kl= (kv[(kv.size())/2]+kv[(kv.size()-2)/2])/2;
        sort(cv.begin(),cv.end());
        if(cv.size()%2 == 1) cl= cv[(cv.size()-1)/2];
        else cl= (cv[(cv.size())/2]+cv[(cv.size()-2)/2])/2;
        _log << "median k,c= " << kl << " " << cl << endl;
        kv.clear();
        cv.clear();

        _log << to_string(_imgNum)+"rightline: ";
        for(auto it:selectedLinesR)
        {
            _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
            rlu.push_back(it[0]);rlv.push_back(it[1]);
            k= (it[1]-it[3]*1.0)/(it[0]-it[2]);
            kv.push_back(k);
            c= it[1]-k*it[0];
            cv.push_back(c);
            _log << "k,c= " << k << " " << c << endl;;
            if(_save == 2) line(_img_, Point(it[0],it[1]),  Point(it[2],it[3]), Scalar(0,195,0), 1, LINE_AA);
        }
        sort(kv.begin(),kv.end());
        if(kv.size()%2 == 1) kr= kv[(kv.size()-1)/2];
        else kr= (kv[(kv.size())/2]+kv[(kv.size()-2)/2])/2;
        sort(cv.begin(),cv.end());
        if(cv.size()%2 == 1) cr= cv[(cv.size()-1)/2];
        else cr= (cv[(cv.size())/2]+cv[(cv.size()-2)/2])/2;
        _log << "median k,c= " << kr << " " << cr << endl;
        kv.clear();
        cv.clear();


        center.x= (cr-cl)/(kl-kr);
        center.y= center.x*kl +cl;
        if(center.x < 380 || center.x > 900)
        {
            cout << "maybe laser line inclined to one side!!!\n";
            return -9;
        }

        vl.x= -1/sqrt(kl*kl+1);
        vl.y= kl*vl.x;
        //rotHalfPi(vl.x,vl.y);
        vr.x= 1/sqrt(kr*kr+1);
        vr.y= kr*vr.x;

        double leftCut1= 108,rightCut1= 108;
        double leftCut2= 361,rightCut2= 361;
        if(SH == -1)
        {
            leftCut1= 32;
            leftCut2= 235;
        }
        else if(SH == 1)
        {
            rightCut1= 32;
            rightCut2= 235;
        }

        leftCut1*= vl.x/adjuster;
        rightCut1*= vr.x/adjuster;
        leftCut2*= vl.x/adjuster;
        rightCut2*= vr.x/adjuster;
        Point2d lc1,lc2,rc1,rc2;
        lc1.x= center.x+leftCut1;
        lc2.x= (center.x+leftCut2 > 22/adjuster)? center.x+leftCut2 : 22/adjuster;
        rc1.x= center.x+rightCut1;
        rc2.x= (center.x+rightCut2 < 1257/adjuster)? center.x+rightCut2 : 1257/adjuster;

        lc1.y= lc1.x*kl+cl;
        lc2.y= lc2.x*kl+cl;
        rc1.y= rc1.x*kr+cr;
        rc2.y= rc2.x*kr+cr;

        double kln= -vl.x/vl.y;
        double krn= -vr.x/vr.y;
        Point2d cln1= Point2d(0,lc1.y-kln*lc1.x);
        Point2d cln2= Point2d(0,lc2.y-kln*lc2.x);
        Point2d crn1= Point2d(0,rc1.y-krn*rc1.x);
        Point2d crn2= Point2d(0,rc2.y-krn*rc2.x);



        //swap_cs(vxr,vyr);
//        cvtColor(img,_img,CV_GRAY2BGR);
//        line(_img,Point2d(0,cl),center,Scalar(255,55,255));
//        line(_img,Point2d(1279,cr+kr*1279),center,Scalar(255,55,255));
//        line(_img,cln1,lc1,Scalar(0,255,0));
//        line(_img,cln2,lc2,Scalar(0,255,0));
//        line(_img,crn1,rc1,Scalar(0,255,0));
//        line(_img,crn2,rc2,Scalar(0,255,0));
//        imwrite(logpd+to_string(_imgNum)+"ll.png",_img);


        vector<Point> left,right;
        Point2d clv1= lc1-cln1;
        Point2d clv2= lc2-cln2;
        Point2d crv1= rc1-crn1;
        Point2d crv2= rc2-crn2;

        Point2d lpv= lc1-lc2;
        Point2d rpv= rc1-rc2;

        double lenthl= norm(lpv);
        double lenthr= norm(rpv);
        int x2= 0,y2= 0;
        double dist;
        Point2d dp;
        int pv= 1;

//        Rect2i roi= Rect2i(0,0,int(center.x),img.rows);
//        threshold(img(roi),img(roi),15,255,THRESH_OTSU);
//        roi= Rect2i(int(center.x),0,img.cols-int(center.x),img.rows);
//        threshold(img(roi),img(roi),15,255,THRESH_OTSU);
        int ac1= 8;
        int ac2= 12;
        ac1/= adjuster;
        ac2/= adjuster;
        Rect2i roi= Rect2i(0,0,int(lc2.x)-ac2,img.rows);
        img(roi)= 0;
        roi= Rect2i(int(lc2.x-ac2),0,int(lc1.x+ac1)-int(lc2.x-ac2),img.rows);
        threshold(img(roi),img(roi),15,255,THRESH_OTSU);

        roi= Rect2i(int(rc1.x-ac1),0,int(rc2.x+ac2)-int(rc1.x-ac1),img.rows);
        threshold(img(roi),img(roi),15,255,THRESH_OTSU);
        roi= Rect2i(int(rc2.x+ac2),0,img.cols-int(rc2.x+ac2),img.rows);
        img(roi)= 0;

        roi= Rect2i(int(lc1.x+ac1),0,int(rc1.x-ac1)-int(lc1.x+ac1),img.rows);
        Mat rimg= img(roi).clone();
        img(roi)= 0;

        //imwrite(logpd+to_string(_imgNum)+"ri.png",img);
        //kernel= Mat(5,5,CV_8UC1,elementArray55);
        //erode(img,img,kernel);
        //imwrite(logpd+to_string(_imgNum)+"er.png",img);

        vector<Point> pts255;
        findNonZero(img,pts255); cout << "pts255.size(): " << pts255.size() << endl;
        for (auto p:pts255)
        {
            dp.x= double(p.x);
            dp.y= double(p.y);
            if(abs(lpv.cross(dp-lc2)/lenthl) < raserLineWidth
                    && clv1.cross(dp-cln1) > 0 && clv2.cross(dp-cln2) < 0) left.push_back(p);
//            {
//                pv= img.at<uchar>(p.y,p.x);
//                if(pv < 100) left.push_back(p);
//                else if(pv < 200)
//                {
//                    left.push_back(p);
//                    left.push_back(p);
//                }
//                else
//                {
//                    left.push_back(p);
//                    left.push_back(p);
//                    left.push_back(p);
//                }
//            }
            else if(abs(rpv.cross(dp-rc2)/lenthr) < raserLineWidth
                    && crv1.cross(dp-crn1) > 0 && crv2.cross(dp-crn2) < 0) right.push_back(p);
//            {
//                pv= img.at<uchar>(p.y,p.x);
//                if(pv < 100) right.push_back(p);
//                else if(pv < 200)
//                {
//                    right.push_back(p);
//                    right.push_back(p);
//                }
//                else
//                {
//                    right.push_back(p);
//                    right.push_back(p);
//                    right.push_back(p);
//                }
//            }
            //else img.at<uchar>(p.y,p.x)= 0;
        }

        cout << "lr size: " << left.size() << " " << right.size() << endl;
//        Mat limg= Mat::zeros(img.rows,img.cols,CV_8UC1);
//        Mat rimg= Mat::zeros(img.rows,img.cols,CV_8UC1);
//        for(auto p:left) limg.at<uchar>(p.y,p.x)= 255;
//        for(auto p:right) rimg.at<uchar>(p.y,p.x)= 255;
//        imwrite(logpd+to_string(_imgNum)+"li.png",limg);
//        imwrite(logpd+to_string(_imgNum)+"ri.png",rimg);

        Vec4d lineLeft,lineRight;
        if(left.size() > 128/adjuster) fitLine(left, lineLeft, DIST_L2, 0, 0.01,0.01);
        else return -4;
        if(right.size() > 128/adjuster) fitLine(right, lineRight, DIST_L2, 0, 0.01,0.01);
        else return -4;
        if(SH == -1) fitLine(left, lineLeft, DIST_HUBER, 0, 0.01,0.01);
        if(SH == 1) fitLine(right, lineRight, DIST_HUBER, 0, 0.01,0.01);
        double vx = lineLeft[0];
        double vy = lineLeft[1];
        double x = lineLeft[2];
        double y = lineLeft[3];
        kl = vy/vx;
        cl = y - kl*x;

        vx = lineRight[0];
        vy = lineRight[1];
        x = lineRight[2];
        y = lineRight[3];
        kr = vy/vx;
        cr = y - kr*x;

        center.x= (cr-cl)/(kl-kr);
        center.y= center.x*kl +cl;
        cp= center;

        c0l= cl;
        c1279r= cr+kr*1279/adjuster;

//        line(_img,Point2d(0,c0l),cp,Scalar(0,255,0),1,LINE_AA);
//        line(_img,Point2d(1279,c1279r),cp,Scalar(0,255,0),1,LINE_AA);

        //_endORstart= -1;
        if(_endORstart)
        {
            lc1.x= center.x+leftCut1*2;
            rc1.x= center.x+rightCut1*2;
            lc1.y= lc1.x*kl+cl;
            rc1.y= rc1.x*kr+cr;
            Point2d c_l= center-lc1;
            double lenthc_l= norm(c_l);
            Point2d r_c= rc1-center;
            double lenthr_c= norm(r_c);
            Point2d r_l= rc1-lc1;
            double lenthr_l= norm(r_l);

            double shl= 0;
            double shr= 0;
            double dl= raserLineWidth;
            int ct= 0;
            //Mat limg= Mat::zeros(rimg.rows,rimg.cols,CV_8UC1);

            int roi_x= roi.x;
            roi= Rect2i(0,0,int(center.x-roi_x),rimg.rows);
            threshold(rimg(roi),rimg(roi),15,255,THRESH_OTSU);
            //imwrite(logpd+to_string(_imgNum)+"lr.png",rimg);
            roi= Rect2i(int(center.x-roi_x),0,rimg.cols-int(center.x-roi_x),rimg.rows);
            threshold(rimg(roi),rimg(roi),15,255,THRESH_OTSU);
            //imwrite(logpd+to_string(_imgNum)+"rr.png",rimg);
            pts255.clear();
            findNonZero(rimg,pts255);//imwrite(logpd+to_string(_imgNum)+"r0.png",rimg);
            double heightOnLeft= 0,heightOnRight= 0;
            for(auto p:pts255)
            {
                dp.x= double(p.x+roi_x);
                dp.y= double(p.y);
                shl= c_l.cross(dp-lc1)/lenthc_l;
                shr= r_c.cross(dp-center)/lenthr_c;
                if(shl > dl && shr > dl && r_l.cross(dp-lc1) < 0)
                {
                    heightOnLeft+= shl;
                    heightOnRight+= shr;
                    rimg.at<uchar>(p.y,p.x)= 168;
                    ct++;
                }
            }
            //cout << heightOnLeft << " heightOnLeft ct " << ct  << " ct heightOnRight " << heightOnRight << endl;
            if(ct > 250) _gapLen[_imgNum]= -0.5*(heightOnLeft+heightOnRight)/ct;
            else _gapLen[_imgNum]= 0;
            cvtColor(rimg,rimg,CV_GRAY2BGR);
            circle(rimg,Point2d(center.x-roi_x,center.y),15,Scalar(0,255,0),2);
            imwrite(logpd+to_string(_imgNum)+"r1.png",rimg);
        }
        return 0;
    }
    return -1;
}
*/