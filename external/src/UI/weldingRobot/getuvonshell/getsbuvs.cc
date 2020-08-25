#include "getsbuva.h"
#include "walgo/lineanalysis.h"
#include "walgo/polymodel1d.h"
#include<unistd.h>
#include<random>
#include<memory.h>
#include<set>

using namespace cv;
using namespace std;


void fitCurve(vector<Point> &pts, vector<Point2d> &cvpts);

walgo::PolyModel1D* ransac(vector<Point> &pts,int nIterCnt,double dMaxErrorThreshold,double rmsThreshold);

int GetSBuvS::AddImg(const Mat& Input)
{
    _log << "pic" << to_string(_imgNum) << ": \n";
    cout << "pic" << to_string(_imgNum) << ": \n";
    _uv[_imgNum]= Point2d(0,0);
    if(Input.cols != _imgCols || Input.rows != _imgRows)
    {
        _log << "wrong image size!!! " << _imgNum << endl;
        cout << "wrong image size!!! " << _imgNum << endl;
        _imgNum++;
        return -1;
    }

    cv::Mat input,tp;
    if(!_rightSide)
    {
        Bgr2Gray(Input,tp,2);
        cv::flip(tp,input,1);
        cout << "FLIP\n";
    }
    else Bgr2Gray(Input,input,2);

    if(_save == 1) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",input);
    if(_save == 1) Input.copyTo(_img_);

    int upBound= _icenter.y-_roiHeightShift;

    cv::Rect2i roisl;
    int SIGN= 1;

    roisl.x= _icenter.x-_roiWidthShift;
    roisl.y= _icenter.y-_roiHeightShift;
    roisl.width= _imgCols-roisl.x;
    roisl.height= _imgRows-roisl.y;

    cout << roisl;
    cv::Point2d roiShift(roisl.x,roisl.y);

    cv::Mat image,bimg,maskh,maskf;

    if(_doMediumBlur) medianBlur(input(roisl),image,_mediumBlurSize);
    else input(roisl).copyTo(image);
    if(_save == 1) cv::imwrite(_logpd+to_string(_imgNum)+"roisl.jpg",image);

    //int th= cv::threshold(image,maskh,th,255,CV_THRESH_OTSU);
    int th= 15,d= 3;
    int count= GetTh(image,th,d);
    cout << "thresh: " << th << " count : " << count << endl;
    _log << "thresh: " << th << " count : " << count << endl;
    cv::threshold(image,maskh,th,255,0);

    //if(_save > 0) imwrite(logpd+to_string(_imgNum)+"ctsl.jpg",maskh);

    vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;

    cv::Mat kernel;
    if(1)
    {
//        maskh= 255-bimg;
//        //maskf= bimg/40;
//        image.copyTo(maskf,maskh);
//        threshold(maskf,bimg,th,255,THRESH_OTSU);
//        imwrite(logpd+to_string(_imgNum)+"ct2.png",maskf);
//        bimg+= maskh;
        kernel= cv::Mat(3,5,CV_8UC1,elementArray_35);//Mat::ones(5,7,CV_8UC1);//
        dilate(maskh,bimg,kernel);
        if(_save == 1) cv::imwrite(_logpd+to_string(_imgNum)+"ctsl.jpg",bimg);
    }

    vector<cv::Vec4i> lines,linesl,linesr,slines;

    //imwrite("hask"+to_string(_imgNum)+".jpg",maskh);

    cv::HoughLinesP(bimg,lines,1,CV_PI/180,_houghThresh,_houghMinLineLength,_houghMaxLineGap);
    _log << "slines: \n";

    double k= 0;
    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if( k > 0.0168 && k < 9) slines.push_back(it);
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
        _log << "bad laser " << _imgNum << "straight line !!!" << " thresh: " << th << endl;
        _imgNum++;
        return -3;
    }

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
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        kv.push_back(k);
        if(_save == 1)
        {
            cv::line(_img_,cv::Point2d(it[0],it[1])+roiShift,cv::Point2d(it[2],it[3])+roiShift,cv::Scalar(0,175,0), 1,cv::LINE_AA);
        }
    }
    sort(kv.begin(),kv.end());
    mk= kv[int((kv.size()-1)/2)];

    for(auto &it:selectedLines)
    {
        _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        double temp= pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        if(lineLenth < temp && k >= mk)
        {
            lineLenth = temp;
            sline= it;
        }
    }
    cv::line(_img_,cv::Point2d(sline[0],sline[1])+roiShift,cv::Point2d(sline[2],sline[3])+roiShift,cv::Scalar(255,0,0), 1,cv::LINE_AA);
    //if(_save == 2) imwrite(logpd+to_string(_imgNum)+"t0.jpg",_img_);
    cv::Point2d lcut0,lcut1,rcut0,rcut1,slcut0,slcut1,clcut0,clcut1;

    slcut0.x= sline[0]*0.9+sline[2]*0.1;
    slcut0.y= sline[1]*0.9+sline[3]*0.1;
    slcut1.x= sline[0]*0.5+sline[2]*0.5;
    slcut1.y= sline[1]*0.5+sline[3]*0.5;

//    rcut0.y= (rcut0.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];
//    rcut1.y= (rcut1.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];

    cv::Rect2i mroi;
    maskh= 0;
    cv::line(maskh,slcut0,slcut1,255, 36, cv::LINE_AA);
    _GetMaskRoi(maskh,mroi);
    cv::Mat lineMasked= maskh+2;
    image.copyTo(lineMasked,maskh);

    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lmm.jpg",lineMasked);

    th= cv::threshold(lineMasked(mroi),maskf,15,255,CV_THRESH_OTSU);
    vector<cv::Point> pts255;
    cv::findNonZero(maskf,pts255);
    cv::Vec4d lineLeft,lineRight,sdline;
    fitLine(pts255, sdline, cv::DIST_HUBER, 0, 0.01,0.01);

    int nearLineNum= 0;
    for(auto &p:pts255)
    {
        nearLineNum++;
    }

    slcut0.x+= roisl.x;
    slcut1.x+= roisl.x;
    double dl,dr= sdline[0]*(sdline[3]+mroi.y+roisl.y)-sdline[1]*(sdline[2]+mroi.x+roisl.x);
    cout << sdline << " " << dr << endl;
    slcut0.y= ((dr-10)+sdline[1]*(slcut0.x))/sdline[0];
    slcut1.y= ((dr-10)+sdline[1]*(slcut1.x))/sdline[0];

    if(_save == 1)
    {
        //cv::line(_img_, slcut1, slcut0, cv::Scalar(255,255,255), 1, cv::LINE_AA);
        cv::line(_img_, slcut0, slcut1, cv::Scalar(255,255,255), 1, cv::LINE_AA);
        //cv::imwrite(_logpd+to_string(_imgNum)+"t0.jpg",_img_);
    }

    cv::line(input,slcut0,cv::Point2d(-(dr-10)/sdline[1],0),0, 39, LINE_AA);
    roisl.x= 0;
    roisl.y= 0;
    roisl.width= slcut0.x-8;
    roisl.height= slcut0.y+75;
    image= input(roisl);
    if(_save == 1) cv::imwrite(_logpd+to_string(_imgNum)+"rl2.jpg",image);

    lines.clear();slines.clear();kv.clear();
    cv::threshold(image,bimg,th,255,0);
    cv::HoughLinesP(bimg,lines,1,CV_PI/180,_houghThresh/2,_houghMinLineLength/2,_houghMaxLineGap);
    _log << "alines: \n";

    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
        if( k > 0.0168 && k < 9) slines.push_back(it);
        kv.push_back(k);
        if(_save == 1)
        {
            cv::line(_img_,cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]),cv::Scalar(0,175,0), 1,cv::LINE_AA);
        }
    }

    for(auto &line:slines)
    {
        for(int i= 0;i < 4;i++) _log << line[i] << " ";
        _log << endl;
    }

//    int numbins = (int) floor((360.0/(double)_binsize)+0.5);
////    vector<double> weightsL(numbins);
////    vector<vector<Vec4i> > histL(numbins);
////    calcAngleHistogram(linesl, histL, weightsL, binsize);
////    selectMaxAngles(histL, weightsL, numbins,
////            selectedLinesL, angleRange/binsize);

//    vector<double> weights(numbins);
//    vector<vector<cv::Vec4i> > hist(numbins);
//    walgo::calcAngleHistogram(slines, hist, weights, _binsize);
//    walgo::selectMaxAngles(hist, weights, numbins,
//            selectedLines, _angleRange/_binsize);
//    cout << "sl selectedLines.size(): " << selectedLines.size() << endl;
//    _log << "sl selectedLines.size(): " << selectedLines.size() << endl;

    if(lines.size() < 1)
    {
        //cout << "bad laser line !!!" << endl;
        _log << "bad laser " << _imgNum << "straight line !!!" << " thresh: " << th << endl;
        _imgNum++;
        return -3;
    }

//    Point2d center= Point2d(0,0);
//    Point2d vl,vln,vr,vrn;


    lineLenth= _houghMinLineLength/2;
    aline;

    _log << to_string(_imgNum)+"another line: ";

    sort(kv.begin(),kv.end());
    mk= kv[int((kv.size()-1)/2)];

    int rightPoint= 180;
    for(auto &it:slines)
    {
        _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        double temp= pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        if(lineLenth < temp && k >= mk)
        {
            lineLenth = temp;
            aline= it;
        }
        if(it[2] > rightPoint && it[0] < 120) rightPoint= it[2];
    }
    cv::line(_img_,cv::Point2d(aline[0],aline[1]),cv::Point2d(aline[2],aline[3]),cv::Scalar(255,0,0), 1,cv::LINE_AA);
    //if(_save == 2) imwrite(logpd+to_string(_imgNum)+"t0.jpg",_img_);

    slcut0.x= aline[0]*0.85+aline[2]*0.15;
    slcut0.y= aline[1]*0.85+aline[3]*0.15;
    slcut1.x= aline[0]*0.1+aline[2]*0.9;
    slcut1.y= aline[1]*0.1+aline[3]*0.9;

    if(_save == 1)
    {
        cv::line(_img_, slcut0, slcut1, cv::Scalar(255,255,255), 1, cv::LINE_AA);
        cv::imwrite(_logpd+to_string(_imgNum)+"t0.jpg",_img_);
    }


    maskh= cv::Mat::zeros(image.size(),CV_8UC1);
    cv::line(maskh,slcut0,slcut1,255, 36, cv::LINE_AA);
    _GetMaskRoi(maskh,mroi);
    lineMasked= maskh+2;
    image.copyTo(lineMasked,maskh);

    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lam.jpg",lineMasked);

    th= cv::threshold(lineMasked(mroi),maskf,15,255,CV_THRESH_OTSU);
    vector<cv::Point> pts250;
    cv::findNonZero(maskf,pts250);
    cv::Vec4d adline;
    fitLine(pts250, adline, cv::DIST_HUBER, 0, 0.01,0.01);

//    int nearLineNum= 0;
//    for(auto &p:pts255)
//    {
//        nearLineNum++;
//    }

    slcut0.x+= roisl.x;
    slcut1.x+= roisl.x;
    dl= adline[0]*(adline[3]+mroi.y+roisl.y)-adline[1]*(adline[2]+mroi.x+roisl.x);
    cout << adline << " " << dr << endl;
    slcut0.y= (dl+adline[1]*(slcut0.x))/adline[0];
    slcut1.y= (dl+adline[1]*(slcut1.x))/adline[0];

    rightPoint-= 80;
    cv::line(image,cv::Point(0,dl/adline[0]),cv::Point(rightPoint,(rightPoint*adline[1]+dl)/adline[0]),0,29,cv::LINE_AA);
    roisl.x= rightPoint;
    roisl.width= image.cols-roisl.x;
    roisl.y= _icenter.y-_roiHeightShift;
    roisl.height-= roisl.y;
    bimg= image(roisl);
    //cv::imwrite(_logpd+to_string(_imgNum)+"lb.jpg",bimg);
    cv::threshold(bimg,bimg,th,255,8);
    if(_save == 1) cv::imwrite(_logpd+to_string(_imgNum)+"lb.jpg",bimg);

    cout << "curve fit !!!" << endl;
    vector<cv::Point> pts;
    cv::findNonZero(bimg,pts);
    cout << "pts.size(): " << pts.size() << endl;
    vector<cv::Point2d> cvpts;

    fitCurve(pts,cvpts);

    double tdl= 222.888;
    slcut0.x= 0;
    slcut0.y= dr/sdline[0];
    slcut1.x= -dr/sdline[1];
    slcut1.y= 0;
    cv::Point2d c_r= slcut0-slcut1;
    double lenthc_r= norm(c_r);

    if(cvpts.size() < 10)
    {
        _log << "fitCurve failure" << endl;
        _imgNum++;
        return -13;
    }
    vector<double> adlv;
    cv::Point2d cross,cp= Point2d(cvpts[0].x+roisl.x,cvpts[0].y+roisl.y);
    dr= c_r.cross(cp-slcut1)/lenthc_r;
    for(auto &p:cvpts)
    {
        cp.x= p.x+roisl.x;
        cp.y= p.y+roisl.y;
        if(_save == 1 && p.x-int(p.x) == 0) circle(_img_,cp,1,Scalar(0,255,0),1);
        dl= c_r.cross(cp-slcut1)/lenthc_r;
        adlv.push_back(abs(dl));
        if(abs(dl) < tdl)
        {
            tdl= abs(dl);
            cross= cp;
        }
        if(dr*dl < 0) break;
        dr= dl;
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


    if(_save > -1)
    {
        Input.copyTo(input);
        //input= Input.clone();
        //line(input,cross,Point(slcut1.x+roi.x,(dl+lineLeft[1]*(slcut1.x+roi.x))/lineLeft[0]),Scalar(0,255,0),1,LINE_AA);
        //line(input,cross,Point(cross.x-200,(dr+lineRight[1]*(cross.x-200))/lineRight[0]),Scalar(0,255,0),1,LINE_AA);
        cv::circle(input,cross,15,Scalar(0,255,0),2,LINE_AA);
        cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",input);
    }

    if(_save == 1) cv::imwrite(_logpd+to_string(_imgNum)+"s+l.jpg",_img_);

    if(!_rightSide) cross.x= _imgCols-1-cross.x;
    _uv[_imgNum]= cross;


    int rt= 0;

    _imgNum++;
    return rt;
}

void fitCurve(vector<Point> &pts,vector<Point2d> &cvpts)
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
        walgo::PolyModel1D model;
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
        walgo::PolyModel1D* model= ransac(pts,99,3.9,3);
        if(model == NULL) return;
        vector<double> x;
        vector<double> y;
        for(auto &p:pts)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        double xmax= *(max_element(x.begin(),x.end()))+20;
        double xmin= *(min_element(x.begin(),x.end()));
        for(double u= xmin;u < xmax;u+= 0.25)
        {
            cvpts.push_back(cv::Point2d(u,model->model(u)));
        }
    }
}

walgo::PolyModel1D* ransac(vector<Point> &pts,int nIterCnt,double dMaxErrorThreshold,double rmsThreshold)
{
    int nCnt= pts.size();
    //int snc= nCnt/50;
    int snc= 9;
    default_random_engine rng(time(0));
    uniform_int_distribution<int> uniform(0,nCnt-1);

    walgo::PolyModel1D* rightModel= new walgo::PolyModel1D();
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
        walgo::PolyModel1D* model= new walgo::PolyModel1D();
        model->setParams({3});
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
