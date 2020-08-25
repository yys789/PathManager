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

int GetSBuvM::AddImg(const cv::Mat& Input)
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

    cv::Mat maskh,maskf,aChannels[3],blue,green,red,redb;
    cv::split(Input, aChannels);
    aChannels[2].copyTo(red);
    //cout << i << " " << src.channels() << " " << aChannels[1].channels() << endl;
    int th;
    if(!_weldOff)
    {
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
        aChannels[2].setTo(3,blue);
        aChannels[2].setTo(3,green);
    }
    cv::medianBlur(aChannels[2],aChannels[2],5);

    //th= cv::threshold(aChannels[2],redb,33,255,8);
    //cout << _imgNum << "s th: " << th << endl;
    cv::threshold(aChannels[2],redb,39,255,0);
    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",redb);

//    int w= 1200,h= 1080;
//    cv::Mat res,bs= cv::Mat::zeros(1024,(1280-w)/2,CV_8UC1);
//    cv::resize(redb,res,cv::Size(w,h));
//    vector<cv::Mat> matrices= { bs,res(cv::Rect(0,0,w,1024)),bs,};
//    cv::hconcat(matrices,red);
//    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+".jpg",red);
//    _imgNum++;
//    return -1;

    cv::Rect2i roil,roir;

    double wcr= _weldOff?_weldCutR*1.0 : 10.0;
    roil.x= 0;
    roil.y= _icenter.y-_roiHeightShift;
    roil.width= _icenter.x+_roiWidthShift/4*3;
    roil.height= _imgRows-roil.y-int(_roiHeightShift*_weldCutR/wcr);
    cout << roil << endl;

    roir.x= _icenter.x-_roiWidthShift;
    roir.y= _icenter.y-_roiHeightShift;
    roir.width= _imgCols-roir.x;
    roir.height= _imgRows-roir.y-int(_roiHeightShift*_weldCutR/wcr);
    cout << roir << endl;

    int SIGN= -1;
    if(_rightSide) SIGN= 1;

    if(_save == 2)
    {
        imwrite(_logpd+to_string(_imgNum)+"input_redl.jpg",redb(roil));
        imwrite(_logpd+to_string(_imgNum)+"input_redr.jpg",redb(roir));
    }

    if(!(0 <= roil.x && 0 <= roil.width && roil.x + roil.width <= redb.cols && 0 <= roil.y && 0 <= roil.height && roil.y + roil.height <= redb.rows))
    {
        _imgNum++;
        cout << "roi1 error!!";
        return -2;
    }
    if(!(0 <= roir.x && 0 <= roir.width && roir.x + roir.width <= redb.cols && 0 <= roir.y && 0 <= roir.height && roir.y + roir.height <= redb.rows))
    {
        _imgNum++;
        cout << "roi2 error!!";
        return -2;
    }
    cv::Point2d lcut0,lcut1,rcut0,rcut1;
    cv::Vec4d adline,rdline;
    double dl,dr;
    if(!GetHoughLine(redb(roil),lcut0,lcut1,-1,0.9,0.6,77,int(roil.width/3)))
    {
        _imgNum++;
        return -10;
        lcut0.x= _mroil.x;
        lcut0.y= _mroil.y+_mroil.height;
        lcut1.x= _mroil.x+_mroil.width;
        lcut1.y= _mroil.y;

        maskh= cv::Mat::zeros(red(roil).size(),CV_8UC1);
        cv::line(maskh,lcut0,lcut1,255, 49, cv::LINE_AA);
        cv::Rect2i mroi;
        _GetMaskRoi(maskh,mroi);
        _mroil= mroi;
        cv::Mat redl= cv::Mat::zeros(red(roil).size(),CV_8UC1);
        red(roil).copyTo(redl,maskh);
        vector<cv::Point> pts;
        cv::findNonZero(redl,pts);
        vector<int> gv;
        for(auto &p:pts)
        {
            if(redl.at<uchar>(p) > 0) gv.push_back(redl.at<uchar>(p));
        }
        sort(gv.begin(),gv.end());
        int mg= gv[(gv.size()-1)/2];
        redl.setTo(mg,255-maskh);
        cv::threshold(redl,redl,35,255,8);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"ransac_redl"+to_string(mg)+".jpg",redl);
        vector<cv::Point> ptsl;
        cv::findNonZero(redl,ptsl);
//        cv::findNonZero(red(mroi),ptsr);
//        cv::Vec4d lineLeft, lineRight;
//        cv::Point2d cross;
//        double dl,dr;
        if(ptsl.size() > mroi.width*5 && ptsl.size() < mroi.width*23)
        {
            if(ranSac(ptsl,25, 3.8, 2.5, adline,-1));
            adline[2]+= roil.x;
            adline[3]+= roil.y;
            dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);
        }
        else
        {
            _imgNum++;
            return -10;
        }

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
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMaskedL.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMaskedL.rows))
        {
            _imgNum++;
            cout << "roi3 error!!";
            return -2;
        }
        cv::split(lineMaskedL(mroi), aChannels);

        th= cv::threshold(aChannels[2],maskf,15,255,CV_THRESH_OTSU);
        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);//cout << "pts255.size()= " << pts255.size() << endl;
        //cv::Vec4d adline;
        if(pts255.size() > cv::norm(lcut1-lcut0)*2)
            cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
        else {
            _imgNum++;
            return -8;
        }
//        adline[2]+= roil.x+mroi.x;
//        adline[3]+= roil.y+mroi.y;
        adline[2]+= mroi.x;
        adline[3]+= mroi.y;
        //double dl,dr;
        dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);//cout << "dl: " << dl << endl;

        int dh= 36;
        int dw= 55;
        int x0= lcut1.x+50;
        int x1= lcut1.x+175;//roil.width;
        int y0= int((dl+dh+adline[1]*x0)/adline[0]);
        int y1= int((dl+dh+adline[1]*x1)/adline[0]);

        maskh= 0;
        cv::line(maskh,cv::Point(x0,y0),cv::Point(x1,y1),255, dw, cv::LINE_AA);
        _GetMaskRoi(maskh,mroi);
        Input(roil).copyTo(lineMaskedR,maskh);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ml.jpg",lineMaskedR);
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMaskedR.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMaskedR.rows))
        {
            _imgNum++;
            cout << "roi4 error!!";
            return -2;
        }
        cv::split(lineMaskedR(mroi), aChannels);
        th= cv::threshold(aChannels[2],maskf,15,255,CV_THRESH_OTSU);
        pts255.clear();
        cv::findNonZero(maskf,pts255);
        cout << "pts255.size()= " << pts255.size() << endl;

        vector<int> shortLiner;
        shortLiner.resize(dh+(dw+1)/2,0);
        int ddl= 0;
        for(auto &p:pts255){
            ddl= int(round(adline[0]*(p.y+mroi.y)-adline[1]*(p.x+mroi.x)-dl));//cout << ddl << endl;
            if(ddl > 0 && ddl < dh+(dw-1)/2) shortLiner[ddl]++;
        }
        auto maxIter= max_element(shortLiner.begin(),shortLiner.end());
        int shift= 0;
        while(maxIter != shortLiner.begin()) {
            maxIter--;
            shift++;
        }
        //for(auto &s:shortLiner) cout << s << endl;
        maxIter= max_element(shortLiner.begin(),shortLiner.end());
        int max= *(maxIter)/2;
        int count0= 0,count1= 0;
        while(maxIter != shortLiner.begin() && *(--maxIter) > max) count0++;
        cout << "count0: " << count0 << " ";
        while(maxIter != shortLiner.end() && *(++maxIter) > max) count1++;
         cout << "count1: " << count1 << " ";
        dl+= roil.y*abs(adline[0]/sqrt(pow(adline[0],2)+pow(adline[1],2)))+shift-count0+count1/2;
        //cv::line(Input,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),cv::Scalar(0,255,0),1);
        //if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ssl.jpg",Input);
        //_imgNum++;
        //return -10;


        //cv::line(itemp,cv::Point(0,dl/adline[0]),cv::Point(-dl/adline[1],0),255, 1, cv::LINE_AA);

        //    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ls0m.jpg",itemp);

        //    _uv[_imgNum]= cv::Point2d((-dr*adline[0]+dl*sdline[0])/(sdline[1]*adline[0]-adline[1]*sdline[0]),(dr*adline[1]-dl*sdline[1])/(sdline[0]*adline[1]-adline[0]*sdline[1]))+roiShift;
    }

    if(!GetHoughLine(redb(roir),rcut0,rcut1,1,0.85,1,int(roir.width/2),roir.width-88))
    {
        _imgNum++;
        return -10;
        rcut0.x= _mroir.x;
        rcut0.y= _mroir.y;
        rcut1.x= _mroir.x+_mroir.width;
        rcut1.y= _mroir.y+_mroir.height;

        maskh= cv::Mat::zeros(red(roir).size(),CV_8UC1);
        cv::line(maskh,rcut0,rcut1,255, 49, cv::LINE_AA);
        cv::Rect2i mroi;
        _GetMaskRoi(maskh,mroi);
        _mroir= mroi;
        cv::Mat redr= cv::Mat::zeros(red(roir).size(),CV_8UC1);
        red(roir).copyTo(redr,maskh);
        vector<cv::Point> pts;
        cv::findNonZero(redr,pts);
        vector<int> gv;
        for(auto &p:pts)
        {
            if(redr.at<uchar>(p) > 0) gv.push_back(redr.at<uchar>(p));
        }
        sort(gv.begin(),gv.end());
        int mg= gv[(gv.size()-1)/2];
        redr.setTo(mg,255-maskh);
        cv::threshold(redr,redr,35,255,8);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"ransac_redr"+to_string(mg)+".jpg",redr);
        vector<cv::Point> ptsr;
        cv::findNonZero(redr,ptsr);
//        cv::findNonZero(red(mroi),ptsr);
//        cv::Vec4d lineLeft, lineRight;
//        cv::Point2d cross;
//        double dl,dr;
        if(ptsr.size() > mroi.width*5 && ptsr.size() < mroi.width*23)
        {
            if(ranSac(ptsr,25, 3.8, 2.5, rdline,1));
            rdline[2]+= roir.x;
            rdline[3]+= roir.y;
            dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
        }
        else
        {
            _imgNum++;
            return -10;
        }
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
        if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMaskedR.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMaskedR.rows))
        {
            _imgNum++;
            cout << "roi5 error!!";
            return -2;
        }
        cv::split(lineMaskedR(mroi), aChannels);
        th= cv::threshold(aChannels[2],maskf,15,255,CV_THRESH_OTSU);

        vector<cv::Point> pts255;
        cv::findNonZero(maskf,pts255);
        if(pts255.size() > cv::norm(rcut1-rcut0)*2)
            cv::fitLine(pts255, rdline, cv::DIST_HUBER, 0, 0.01,0.01);
        else {
            _imgNum++;
            return -8;
        }
        //cv::Vec4d rdline;
        rdline[2]+= roir.x+mroi.x;
        rdline[3]+= roir.y+mroi.y;
        dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
        //dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);
    }
    cv::Point2d cross;
    cross.x= (dl*rdline[0]-dr*adline[0])/(rdline[1]*adline[0]-adline[1]*rdline[0]);
    cross.y= (dl*rdline[1]-dr*adline[1])/(adline[0]*rdline[1]-rdline[0]*adline[1]);
    if(_save > 0)
    {
        //cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
        //cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
        cv::Mat item;
        Input.copyTo(item);
//        cv::imwrite(_logpd+to_string(_imgNum)+"aChannels[2]R.jpg",aChannels[2]);
//        cv::imwrite(_logpd+to_string(_imgNum)+"maskfr.jpg",maskf);
        cv::circle(item,cross,15,cv::Scalar(0,255,0),2,cv::LINE_AA);
        cout << "cross: " << cross << endl;
        cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",item);

    }
//    th= cv::threshold(lineMasked(mroi),maskf,15,255,CV_THRESH_OTSU);
//    vector<cv::Point> pts255;
//    cv::findNonZero(maskf,pts255);
//    cv::Vec4d lineLeft,lineRight,sdline;
//    fitLine(pts255, sdline, cv::DIST_HUBER, 0, 0.01,0.01);
    //if(_save == 1) Input.copyTo(_img_);

    _uv[_imgNum]= cross;
    _imgNum++;
    return 0;
}

bool GetSBuvM::GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1,int SIGN,double ratio0,double ratio1,int minLimit,int maxLimit,double k0,double k1)
{
    //vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;
    vector<cv::Vec4i> lines,slines,selectedLines;
    cv::HoughLinesP(bimg,lines,1,CV_PI/180,int(_houghThresh*ratio1),int(_houghMinLineLength*ratio1),_houghMaxLineGap);
    _log << minLimit << "~" << maxLimit << "slines: \n";

    double k= 0;
    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if(k > k0 && k < k1 && it[0] < minLimit && it[2] > maxLimit) slines.push_back(it);
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
    double lineLenth= _houghMinLineLength*ratio1;
    cv::Vec4i sline;

    cv::Mat cimg;
    cv::cvtColor(bimg,cimg,CV_GRAY2BGR);
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
        imwrite(_logpd+to_string(_imgNum)+"h"+to_string(SIGN)+".jpg",cimg);
    }
    slcut0.x= sline[0]*ratio0+sline[2]*(1-ratio0);
    slcut0.y= sline[1]*ratio0+sline[3]*(1-ratio0);
    slcut1.x= sline[0]*(1-ratio0)+sline[2]*(ratio0);
    slcut1.y= sline[1]*(1-ratio0)+sline[3]*(ratio0);
    return true;
}
