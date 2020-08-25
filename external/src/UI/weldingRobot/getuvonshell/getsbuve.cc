#include "getsbuva.h"
#include "histimage.h"
#include "walgo/lineanalysis.h"
//#include "lineanalysis.h"

using namespace std;

int GetSBuvE::AddImg(const cv::Mat& Input)
{
    _log << "pic" << to_string(_imgNum) << ": \n";
    cout << "pic" << to_string(_imgNum) << ": \n";
    _uv[_imgNum]= cv::Point2d(-1000,-1000);
    _uvd[_imgNum]= cv::Point2d(-1000,-1000);
    if(Input.cols != _imgCols || Input.rows != _imgRows || Input.channels() != 3)
    {
        _log << "wrong image size!!! " << _imgNum << endl;
        cout << "wrong image size!!! " << _imgNum << endl;
        _imgNum++;
        return -1;
    }

    cv::Mat maskh,maskf,aChannels[3],Blue,green,red,redb;
    cv::split(Input, aChannels);
    aChannels[2].copyTo(red);
    //cout << i << " " << src.channels() << " " << aChannels[1].channels() << endl;
    int th;
    if(!_weldOff)
    {
        th= cv::threshold(aChannels[0],Blue,_bluegv,255,8);75;cout << th << endl;
        th= cv::threshold(aChannels[1],green,_greengv,255,8);60;cout << th << endl;
        th= cv::threshold(aChannels[2],red,39,255,8);60;cout << th << endl;
        if(_save == 2)
        {
            imwrite(_logpd+to_string(_imgNum)+"input_blue.jpg",Blue);
            imwrite(_logpd+to_string(_imgNum)+"input_green.jpg",green);
            imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",red);
        }
        _imgNum++;
        return 0;
        //cv::threshold(aChannels[2],red,5,255,0);

        //cv::dilate(blue,blue,kernel);
        //cv::dilate(green,green,kernel);
        aChannels[2].setTo(3,Blue);
        aChannels[2].setTo(3,green);
    }


//    int w= 1200,h= 1080;
//    cv::Mat res,bs= cv::Mat::zeros(1024,(1280-w)/2,CV_8UC1);
//    cv::resize(redb,res,cv::Size(w,h));
//    vector<cv::Mat> matrices= { bs,res(cv::Rect(0,0,w,1024)),bs,};
//    cv::hconcat(matrices,red);
//    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+".jpg",red);

    cv::Rect2i roisl,rois;
    int SIGN= -1;
    double wcr= _weldOff?_weldCutR*1.0 : 10.0;
    if(_rightSide)
    {
        SIGN= 1;
        rois.x= _icenter.x-_roiWidthShift;
        rois.y= _icenter.y-_roiHeightShift;
        rois.width= _imgCols-rois.x;
        rois.height= _imgRows-rois.y-int(_roiHeightShift*_weldCutR/wcr);
    }
    else
    {      
        rois.x= 0;
        rois.y= _icenter.y-_roiHeightShift;
        rois.width= _icenter.x+_roiWidthShift;
        rois.height= _imgRows-rois.y-int(_roiHeightShift*_weldCutR/wcr);
    }

    if(!(0 <= rois.x && 0 <= rois.width && rois.x + rois.width <= red.cols && 0 <= rois.y && 0 <= rois.height && rois.y + rois.height <= red.rows))
        return -2;
    cv::medianBlur(aChannels[2](rois),redb,5);

     //th= cv::threshold(aChannels[2],redb,33,255,8);
     //cout << _imgNum << "s th: " << th << endl;
     //th= cv::threshold(aChannels[0],redb,49,255,0); cout << _imgNum << "'s th: " << th << endl;
     th= cv::threshold(redb,maskh,10,255,0); cout << _imgNum << "'s th: " << th << endl;

     //if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_blue.jpg",blue);
     if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_rois_t.jpg",maskh);
     cv::Mat maskg;
     red(rois).copyTo(maskg);
     cv::Point2d cut0,cut1;
     if(!GetHoughLine(maskh,cut0,cut1,1,0.85,1,-.15,0.15))
     {
         _imgNum++;
         return -10;
         cut0.x= _mroir.x;
         cut0.y= _mroir.y;
         cut1.x= _mroir.x+_mroir.width;
         cut1.y= _mroir.y+_mroir.height;

         maskh= cv::Mat::zeros(red(rois).size(),CV_8UC1);
         cv::line(maskh,cut0,cut1,255, 49, cv::LINE_AA);
         cv::Rect2i mroi;
         if(!_GetMaskRoiB(maskh,mroi)) {
             _imgNum++;
             return -3;
         }
         _mroir= mroi;
         cv::Mat redr= cv::Mat::zeros(red(rois).size(),CV_8UC1);
         red(rois).copyTo(redr,maskh);
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
 //            if(ranSac(ptsr,25, 3.8, 2.5, rdline,1));
 //            rdline[2]+= rois.x;
 //            rdline[3]+= rois.y;
 //            dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
         }
     }
     cv::Vec4d rdline; //拟合的直线
     double rk;
     if(_rightSide) {
         cout << "_rightSide\n";
         cv::Point2d rcut0,rcut1;
         rcut0.x= cut0.x+_shortLineLen;
         rcut1.x= rcut0.x+_houghMinLineLength/2;
         rcut0.y= cut1.y+(cut0.y-cut1.y)*(rcut0.x-cut1.x)/(cut0.x-cut1.x);
         rcut1.y= cut1.y+(cut0.y-cut1.y)*(rcut1.x-cut1.x)/(cut0.x-cut1.x);

         cv::Rect2i mroi;
         maskh= cv::Mat::zeros(red(rois).size(),CV_8UC1);
         cv::line(maskh,rcut0,rcut1,255, 36, cv::LINE_AA);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",maskh);

         if(!_GetMaskRoiB(maskh,mroi)) {
             _imgNum++;
             return -3;
         }
         _mroir= mroi;

         cv::Mat lineMaskedL,lineMaskedR= maskh.clone();
         red(rois).copyTo(lineMaskedR,maskh);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",lineMaskedR);
         //cv::split(lineMaskedR(mroi), aChannels);
         if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMaskedR.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMaskedR.rows))
         {
             _imgNum++;
             cout << "roi2 error!!";
             return -2;
         }
         th= cv::threshold(lineMaskedR(mroi),maskf,15,255,8);

         vector<cv::Point> pts255;
         //cv::Vec4d rdline;
         cv::findNonZero(maskf,pts255);
         if(pts255.size() > cv::norm(rcut1-rcut0)*2)
             cv::fitLine(pts255, rdline, cv::DIST_HUBER, 0, 0.01,0.01);
         else {
             _imgNum++;
             return -8;
         }
         rdline[2]+=mroi.x;
         rdline[3]+=mroi.y;
         //double dr= rdline[0]*(rdline[3]+0)-rdline[1]*(rdline[2]+0);
         //dl= adline[0]*(adline[3]+0)-adline[1]*(adline[2]+0);
         //_uv[_imgNum]= cv::Point2d(cut0.x+rois.x,(dr+rdline[1]*(cut0.x+rois.x))/rdline[0]);

         rk= rdline[1]/rdline[0];



     }
     else {
         cout << "_leftSide\n";
         cv::Point2d lcut0,lcut1;
         lcut1.x= cut1.x-_shortLineLen;
         lcut0.x= lcut1.x-_houghMinLineLength/2;
         lcut0.y= cut1.y+(cut0.y-cut1.y)*(lcut0.x-cut1.x)/(cut0.x-cut1.x);
         lcut1.y= cut1.y+(cut0.y-cut1.y)*(lcut1.x-cut1.x)/(cut0.x-cut1.x);

         cv::Rect2i mroi;
         maskh= cv::Mat::zeros(red(rois).size(),CV_8UC1);
         cv::line(maskh,lcut0,lcut1,255, 36, cv::LINE_AA);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",maskh);

         if(!_GetMaskRoiB(maskh,mroi)) {
             _imgNum++;
             return -3;
         }
         _mroir= mroi;

         cv::Mat lineMaskedR= maskh.clone();
         red(rois).copyTo(lineMaskedR,maskh);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"rm.jpg",lineMaskedR);
         if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMaskedR.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMaskedR.rows))
         {
             _imgNum++;
             cout << "roi1 error!!";
             return -2;
         }
         th= cv::threshold(lineMaskedR(mroi),maskf,15,255,8);

         vector<cv::Point> pts255;
         //cv::Vec4d rdline;
         cv::findNonZero(maskf,pts255);
         if(pts255.size() > cv::norm(lcut1-lcut0)*2)
             cv::fitLine(pts255, rdline, cv::DIST_HUBER, 0, 0.01,0.01);
         else {
             _imgNum++;
             return -8;
         }
         //cv::Vec4d rdline;
         rdline[2]+= mroi.x;
         rdline[3]+= mroi.y;

         rk= rdline[1]/rdline[0];
         cout<<"rdline: "<<rdline<<endl;
     }

     //cut0为角向上取roi,截出斜线

     cv::Rect2i roix;
     if(_rightSide) {

         roix.x= cut0.x-200>0?cut0.x-200:0;
         roix.y=cut0.y-100>0?cut0.y-100:0;
         roix.width=cut0.x-roix.x;
         roix.height=cut0.y-roix.y-25;

     }
     else{
         roix.x= cut1.x;
         roix.y=cut1.y-100>0?cut1.y-100:0;
         roix.width=cut1.x+200>rois.width?rois.width-cut1.x:200;
         roix.height=cut1.y-roix.y-25;
     }
     cout<<"roix"<<roix<<endl;
     cv::Vec4d adline;
     double lk;
     if(1){ //取斜线的点，拟合
         cv::Rect2i mroi;
         cv::Mat maskf;
         cv::Mat maskh= cv::Mat::zeros(red(rois).size(),CV_8UC1);
         cv::Mat maskroi=maskh(roix);
         maskg(roix).copyTo(maskroi);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"roix.jpg",maskh);

         th= cv::threshold(maskh(roix),maskf,5,255,0);

         vector<cv::Point> pts255;

         cv::findNonZero(maskf,pts255);
         cout<<"pts255.size()"<<pts255.size()<<endl;
         if(pts255.size() > 10)
             cv::fitLine(pts255, adline, cv::DIST_HUBER, 0, 0.01,0.01);
         else {
             _imgNum++;
             return -8;
         }
         adline[2]+= roix.x;
         adline[3]+= roix.y;
         //double lk;
         lk= adline[1]/adline[0];
     }

     cv::Point2d cross;
     cross.x=(rdline[3]-adline[3]-rk*rdline[2]+lk*adline[2])/(lk-rk);
     cross.y=lk*(cross.x-adline[2])+adline[3];

     _uv[_imgNum].x=cross.x+rois.x;
     _uv[_imgNum].y=cross.y+rois.y;

     //取短线区域，
     cv::Rect2i roid;
     if(_rightSide) {

         roid.x= cross.x+10;
         roid.y=cross.y-40;
         roid.width=50;
         roid.height=80;

     }
     else{
         roid.x= cross.x-60;
         roid.y=cross.y-40;
         roid.width=50;
         roid.height=80;
     }
     if(!(0 <= roid.x && 0 <= roid.width && roid.x + roid.width <= redb.cols && 0 <= roid.y && 0 <= roid.height && roid.y + roid.height <= redb.rows))
         return -2;

     cout<<"roid"<<roid<<endl;
     double md;
     if(1){ //二值话，求点到长直线距离的中位数
         cv::Mat maskf;
         cv::Mat maskh= cv::Mat::zeros(maskg.size(),CV_8UC1);
         cv::Mat maskroi=maskh(roid);
         maskg(roid).copyTo(maskroi);
         if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"roid.jpg",maskh);

         th= cv::threshold(maskh(roid),maskf,5,255,8);

         vector<cv::Point> pts255;

         cv::findNonZero(maskf,pts255);
         cout<<"pts255.size()"<<pts255.size()<<endl;
         if(pts255.size() > 0)
         {
             double d;
             vector<double>dv;
             for(auto &pts:pts255)
             { double x0=double(pts.x+roid.x);
               double y0=double(pts.y+roid.y);
               double x1=rdline[2];
               double y1=rdline[3];
               d=(y1-y0+rk * (x0-x1))/sqrt(1+pow(rk,2));
               dv.push_back(d);
             }
             sort(dv.begin(),dv.end());
             md= dv[int((dv.size()-1)/2)];
             cout<<"md: "<<md<<endl;
         }
         else {
             _imgNum++;
             return -8;
         }

     }
     //求_uvd
     double rc=rdline[3]-rk*rdline[2];
     rc=rc-md*sqrt(1+pow(rk,2));
     double lc=adline[3]-lk*adline[2];
     cv::Point2d crossd;
     crossd.x=(rc-lc)/(lk-rk);
     crossd.y=(lk*rc-rk*lc)/(lk-rk);

     _uvd[_imgNum].x=crossd.x+rois.x;
     _uvd[_imgNum].y=crossd.y+rois.y;

     if(_save > 0)
      {

          cv::Mat item;
          Input.copyTo(item);

          cv::circle(item,_uv[_imgNum],15,cv::Scalar(255,0,0),2,cv::LINE_AA);
          cv::circle(item,_uvd[_imgNum],15,cv::Scalar(0,255,0),2,cv::LINE_AA);
          cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",item);
      }



    _imgNum++;
    return 0;
}

bool GetSBuvE::GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1,int SIGN,double ratio0,double ratio1,double k0,double k1)
{
    vector<cv::Vec4i> lines,slines,selectedLines;
    //cv::HoughLinesP(bimg,lines,1,CV_PI/180,_houghThresh,_houghMinLineLength,_houghMaxLineGap);
    //cout << _houghMaxLineGap << " " << _gap << endl;
    cv::Mat maskh;//Mat::ones(5,7,CV_8UC1);//
    //cv::erode(bimg,maskh,kernel);

    cv::dilate(bimg,maskh,_kernel);
    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+to_string(SIGN)+"dl.jpg",maskh);
    cv::HoughLinesP(maskh,lines,1,CV_PI/180,int(_houghThresh*ratio1),int(_houghMinLineLength*ratio1),_houghMaxLineGap);

    _log << "slines: \n";
    //const int oo= 00;oo= 98;
    double k= 0;
    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        cout << "k: " << k << endl;
        if(k > k0 && k < k1) slines.push_back(it);
        //else if(k > 0.08) linesr.push_back(it);
    }

    for(auto &line:slines)
    {
        for(int i= 0;i < 4;i++) cout << line[i] << " ";
        cout << endl;
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
        if(lineLenth < temp && 0.05 >= abs(k-mk))
        {
            lineLenth= temp;
            sline= it;
        }
    }
    cout<<"sline: "<<sline<<endl;
    if(_save == 2)
    {
        cv::line(cimg,cv::Point2d(sline[0],sline[1]),cv::Point2d(sline[2],sline[3]),25, 1,cv::LINE_AA);
        imwrite(_logpd+to_string(_imgNum)+"t0.jpg",cimg);
    }
    slcut0= cv::Point2d(sline[0],sline[1]);
    slcut1= cv::Point2d(sline[2],sline[3]);
//    slcut0.x= sline[0]*ratio0+sline[2]*(1-ratio0);
//    slcut0.y= sline[1]*ratio0+sline[3]*(1-ratio0);
//    slcut1.x= sline[0]*(1-ratio0)+sline[2]*(ratio0);
//    slcut1.y= sline[1]*(1-ratio0)+sline[3]*(ratio0);
    return true;
}

