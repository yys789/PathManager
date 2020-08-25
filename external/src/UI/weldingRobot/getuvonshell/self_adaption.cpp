#include "self_adaption.h"
#include <unistd.h>

void SelfAdaption::_init()
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
    _weldOff= true;
    _weldCutR= 15;
    GetConfigEntry(_config,"WELD_CUT_RATIO",_weldCutR);
    _ht= 79;
    GetConfigEntry(_config,"HIGH_THRESH_THRESH",_ht);
    //GetConfigEntry(_config, "XMIN", xMin);
    //GetConfigEntry(_config, "XMAX", xMax);
}

int SelfAdaption::AddImg(const cv::Mat& Input)
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
//        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"change_color.jpg",Input);
//    }

    cv::Mat input,tp;
    if(_weldOff){
        if(Input.channels() == 3)
        {
            Bgr2Gray(Input,input,2);
        }
        else if(Input.channels() == 1) input= Input;
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",input);
        if(_save == 2) Input.copyTo(_img_);
    }

    else {
        cv::Mat aChannels[3],blue,green,red;
        cv::split(Input, aChannels);
        aChannels[2].copyTo(red);
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
        aChannels[2].setTo(3,blue);
        aChannels[2].setTo(3,green);
        input= aChannels[2];
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"input_red.jpg",input);
        if(_save == 2) Input.copyTo(_img_);
    }
//    _imgNum++;
//    return 0;
    int upBound= _icenter.y-_roiHeightShift;

    cv::Rect2i roisl;
    int SIGN= -1;
    double wcr= _weldOff?_weldCutR*1.0 : 10.0;
    if(_rightSide)
    {
        SIGN= 1;
        roisl.x= _icenter.x-_roiWidthShift;
        roisl.y= _icenter.y-_roiHeightShift;
        roisl.width= _imgCols-roisl.x;
        roisl.height= _imgRows-roisl.y-int(_roiHeightShift*_weldCutR/wcr);
    }
    else
    {
        roisl.x= 0;
        roisl.y= _icenter.y-_roiHeightShift;
        roisl.width= _icenter.x+_roiWidthShift;
        roisl.height= _imgRows-roisl.y-int(_roiHeightShift*_weldCutR/wcr);
    }
    cout << roisl;
    if(!(0 <= roisl.x && 0 <= roisl.width && roisl.x + roisl.width <= input.cols && 0 <= roisl.y && 0 <= roisl.height && roisl.y + roisl.height <= input.rows))
    {
        _imgNum++;
        cout << "roi1 error!!";
        return -2;
    }
    cv::Point2d roiShift(roisl.x,roisl.y);

    cv::Mat image,bimg,maskh,maskf;

    if(_doMediumBlur) medianBlur(input(roisl),image,_mediumBlurSize);
    else input(roisl).copyTo(image);
    if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"roisl.jpg",image);

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
        kernel= cv::Mat(3,5,CV_8UC1,elementArray35);//Mat::ones(5,7,CV_8UC1);//
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


    vector<double> weights(numbins);
    vector<vector<cv::Vec4i> > hist(numbins);
    walgo::calcAngleHistogram(slines, hist, weights, _binsize);
    walgo::selectMaxAngles(hist, weights, numbins,
            selectedLines, _angleRange/_binsize);
    cout << "sl selectedLines.size(): " << selectedLines.size() << endl;
    _log << "sl selectedLines.size(): " << selectedLines.size() << endl;

//    if(selectedLines.size() < 1)
//    {
//        //cout << "bad laser line !!!" << endl;
//        _log << "bad laser " << _imgNum << "straight line !!!" << " thresh: " << th << endl;
//        _imgNum++;
//        return -3;
//    }

//    Point2d center= Point2d(0,0);
//    Point2d vl,vln,vr,vrn;

    vector<double> lru,rlu,lrv,rlv;
    vector<double> kv,cv;
    double mk= 1,c,kl,cl,kcr= 0,kcl= 0,cr;
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
    if(kv.size() > 0) mk= kv[int((kv.size()-1)/2)];

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

    if(0)
    {
        slcut0.x= sline[0]*0.8+sline[2]*0.2;
        slcut0.y= sline[1]*0.8+sline[3]*0.2;
        slcut1.x= sline[0]*0.2+sline[2]*0.8;
        slcut1.y= sline[1]*0.2+sline[3]*0.8;
    }
    else {
        if(!GetHoughLine(input(roisl),slcut0,slcut1,SIGN,0.8)) {
            _imgNum++;
            return -3;
        }
    }


//    rcut0.y= (rcut0.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];
//    rcut1.y= (rcut1.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];

    cv::Rect2i mroi;
    maskh= 0;
    cv::line(maskh,slcut0,slcut1,255, _laserLineWidthR*2, cv::LINE_AA);
    if(!_GetMaskRoi(maskh,mroi)){
        _imgNum++;
        return -6;
    }
    cv::Mat lineMasked= maskh+2;
    image.copyTo(lineMasked,maskh);

    if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lmm.jpg",lineMasked);
    if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
    {
        _imgNum++;
        return -2;
    }
    th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
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
    slcut0.x-= (slcut1.x-slcut0.x)/5.0;
    slcut0.y= (dr+sdline[1]*(slcut0.x))/sdline[0];
    slcut1.y= (dr+sdline[1]*(slcut1.x))/sdline[0];
    _uv[_imgNum]= slcut0+roiShift;

//    if(_save == 1)
//    {
//        //cv::line(_img_, slcut1, slcut0, cv::Scalar(255,255,255), 1, cv::LINE_AA);
//        cv::line(_img_, slcut0+roiShift, slcut1+roiShift, cv::Scalar(255,255,255), 1, cv::LINE_AA);
//        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"t0.jpg",_img_);
//    }

    if(_rightSide)
    {
        cv::Mat itemp,rinput;
        rinput= (input(roisl)).clone();
        cv::Rect roils(0,0,slcut0.x,slcut1.y+30);
        if(!(0 <= roils.x && 0 <= roils.width && roils.x + roils.width <= rinput.cols && 0 <= roils.y && 0 <= roils.height && roils.y + roils.height <= rinput.rows))
        {
            _imgNum++;
            cout << "roi1 error!!";
            cout << "roi1 error!!";
            return -2;
        }
        cv::line(rinput(roils),cv::Point2d(0,dr/sdline[0]),slcut0,cv::Scalar(0),_laserLineWidthR, cv::LINE_AA);
        cv::threshold(rinput(roils),itemp,th,255,8);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"il.jpg",rinput(roils));

        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
        cv::Mat etemp;
        cv::erode(itemp,etemp,kernel);
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
        uvl.x= btlx[(btlx.size()-1)/2];
        pts25.clear();
        cv::findNonZero(etemp(cv::Rect(uvl.x,0,1,itemp.rows)),pts25);
        if(pts25.size() < 2){
            _imgNum++;
            return -10;
        }
        vector<int> vy;
        for(auto &p:pts25) vy.push_back(p.y);
        uvl.y= vy[(vy.size()-1)/2];
        _uvl[_imgNum]= uvl+roiShift;

        roils.x= (slcut0.x+slcut1.x)/2;
        roils.y= (slcut0.y+slcut1.y)/2;
        roils.width= 69;
        roils.height= 39;
        if(!(0 <= roils.x && 0 <= roils.width && roils.x + roils.width <= rinput.cols && 0 <= roils.y && 0 <= roils.height && roils.y + roils.height <= rinput.rows))
        {
            _imgNum++;
            cout << "roi1 error!!";
            return -2;
        }
	cv::threshold(rinput(roils),itemp,th,255,8);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",rinput(roils));

        //cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
        //cv::Mat etemp;
        cv::erode(itemp,etemp,kernel);if(_save == 2) //cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",etemp);
        pts25.clear();
        cv::findNonZero(etemp,pts25);
        count= 0;pts25size= pts25.size()/2;
        if(pts25size < 2){
            _imgNum++;
            return -10;
        }
        ptp= (pts25.end()-1);
        while(count < 11 && pts25size > 0) {
            for(int i= 1;i < 15;i++)
            {
                if((ptp-i)->y-ptp->y < 4) count++;
            }
            pts25size--;
            ptp--;
        }

        btlx.clear();
        for(int i= -1;i < 15;i++)
        {
            btlx.push_back((ptp-i)->x);
        }
        sort(btlx.begin(),btlx.end());
//        cv::Rect roilPts= cv::Rect(btlx[(btlx.size()-1)/2],0,itemp.cols-btlx[(btlx.size()-1)/2],itemp.rows);
        cv::Point2d uvr;
        uvr.x= btlx[(btlx.size()-1)/2];
        pts25.clear();
        cv::findNonZero(etemp(cv::Rect(uvr.x,0,1,etemp.rows)),pts25);
        if(pts25.size() < 2){
            _imgNum++;
            return -10;
        }
        vy.clear();
        for(auto &p:pts25) vy.push_back(p.y);
        uvr.x+= roils.x;
        uvr.y= vy[(vy.size()-1)/2]+roils.y;
        _uvr[_imgNum]= uvr+roiShift;

        roisl.width= uvr.x;
        if(GetHoughLine(input(roisl),slcut0,slcut1,SIGN,0.8)) {
        //    rcut0.y= (rcut0.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];
        //    rcut1.y= (rcut1.x-sline[2])*(sline[1]-sline[3])/(sline[0]-sline[2])+sline[3];

            cv::Rect2i mroi;
            maskh= 0;
            cv::line(maskh,slcut0,slcut1,255, _laserLineWidthR*2, cv::LINE_AA);
            if(!_GetMaskRoi(maskh,mroi)){
                _imgNum++;
                return -6;
            }
            cv::Mat lineMasked= maskh+2;
            image.copyTo(lineMasked,maskh);

            if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"lmm.jpg",lineMasked);
            if(!(0 <= mroi.x && 0 <= mroi.width && mroi.x + mroi.width <= lineMasked.cols && 0 <= mroi.y && 0 <= mroi.height && mroi.y + mroi.height <= lineMasked.rows))
            {
                _imgNum++;
                return -2;
            }
            th= cv::threshold(lineMasked(mroi),maskf,15,255,8);
            vector<cv::Point> pts255;
            cv::findNonZero(maskf,pts255);
            cv::Vec4d sdline;
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
            slcut0.x-= (slcut1.x-slcut0.x)/5.0;
            slcut0.y= (dr+sdline[1]*(slcut0.x))/sdline[0];
            slcut1.y= (dr+sdline[1]*(slcut1.x))/sdline[0];
            _uv[_imgNum]= slcut0+roiShift;
        }

        if(_save > 0)
        {
            cv::circle(Input,_uv[_imgNum],15,cv::Scalar(0,255,0),2);
            cv::circle(Input,_uvl[_imgNum],15,cv::Scalar(0,255,0),2);
            cv::circle(Input,_uvr[_imgNum],15,cv::Scalar(0,255,0),2);
            cv::imwrite(_logpd+to_string(_imgNum)+"rs.jpg",Input);
        }
        _imgNum++;
        return 0;

        cv::Rect roilPts= cv::Rect(0,0,btlx[(btlx.size()-1)/2],itemp.rows);


//        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
//        cv::erode(itemp,itemp,kernel);
//        cv::erode(itemp,itemp,kernel);

//        etemp= 0;
//        etemp(roilPts)= itemp(roilPts);
        itemp(roilPts)= 0;
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"it.jpg",itemp);
        lines.clear();slines.clear();
        cv::HoughLinesP(itemp,lines,1,CV_PI/180,13,19,5);
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
        cv::line(maskh,cv::Point(aline[0],aline[1]),cv::Point(aline[2],aline[3]),255, 29, cv::LINE_AA);
        if(!_GetMaskRoi(maskh,mroi)){
            _imgNum++;
            return -6;
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
    else
    {
        cv::Mat itemp;
        //input.copyTo(itemp);

        cv::Rect roils(0,0,slcut0.x,slcut1.y);
        cv::line(input(roisl)(roils),cv::Point2d(0,dr/sdline[0]),slcut0,cv::Scalar(0),_laserLineWidthR, cv::LINE_AA);
        cv::threshold(input(roisl)(roils),itemp,th,255,8);
        if(_save == 2) cv::imwrite(_logpd+to_string(_imgNum)+"ir.jpg",input(roisl)(roils));

        cv::Mat kernel= cv::Mat(3,3,CV_8UC1, elementArray35);
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
        cv::HoughLinesP(itemp,lines,1,CV_PI/180,33,39,5);
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
        cv::line(maskh,cv::Point(aline[0],aline[1]),cv::Point(aline[2],aline[3]),255, 29, cv::LINE_AA);
        if(!_GetMaskRoi(maskh,mroi)){
            _imgNum++;
            return -6;
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

    //Mat inputc= input.clone();
    //line(inputc,rcut0,Point2d(0,dr/sdline[0]), 7, 18, LINE_AA);

    _imgNum++;
    return 0;
}

bool SelfAdaption::GetHoughLine(const cv::Mat &bimg,cv::Point2d &slcut0,cv::Point2d &slcut1,int SIGN,double ratio0, double k0,double k1)
{
    //vector<cv::Vec4i> selectedLinesL,selectedLinesR,selectedLines;
    vector<cv::Vec4i> lines,slines,selectedLines;
//    cv::Mat timg;
//    if(_weldOff)
//    {
//        int th= cv::threshold(bimg,timg,55,255,8);
//        _log << "th= " << th << "\n";
//        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"t1.jpg",timg);
//    }
    cv::Mat timg(bimg.rows,bimg.cols,CV_8UC1);
    if(_weldOff)
    {
        int subCols= 30,subi= bimg.cols/subCols;
        for(int i= 0;i < subi;i++)
        {
            int th= cv::threshold(bimg(cv::Rect(i*subCols,0,subCols,bimg.rows)),timg(cv::Rect(i*subCols,0,subCols,bimg.rows)),55,255,8);
            _log << "th= " << th << "\n";
        }
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"t1.jpg",timg);
    }
    else {
        cv::threshold(bimg,timg,_ht,255,0);
        if(_save == 2) imwrite(_logpd+to_string(_imgNum)+"t1.jpg",timg);
}
    cv::HoughLinesP(timg,lines,1,CV_PI/180,_houghThresh,_houghMinLineLength,_houghMaxLineGap);
    _log << "slines: \n";

    double k= 0;
    for(auto &it:lines)
    {
        k= SIGN*(it[3]-it[1]*1.0)/(it[2]-it[0]);
        //_log << "k: " << k << endl;
//        if(_imgNum == 0 && k > k0 && k < k1 && it[0] > _roiWidthShift-57) slines.push_back(it);
//        else if(k > k0 && k < k1 && it[0] > _uvl[_imgNum].x-(_icenter.x-_roiWidthShift)-77) slines.push_back(it);
        if(k > k0 && k < k1 && it[0] > _roiWidthShift-90) slines.push_back(it);
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

    if(slines.size() < 1)
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
    for(auto &it:slines)
    {
        _log  << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
        kv.push_back(k);
        if(_save == 2)
        {
            cv::line(cimg,cv::Point2d(it[0],it[1]),cv::Point2d(it[2],it[3]),cv::Scalar(0,175,0), 1,cv::LINE_AA);
        }
    }
    sort(kv.begin(),kv.end());
    mk= kv[int((kv.size()-1)/2)];

    for(auto &it:slines)
    {
        _log << it[0] << " " << it[1] << " " << it[2] << " " << it[3] << endl;
        double temp= cv::norm(cv::Point2d(it[0],it[1])-cv::Point2d(it[2],it[3]));//pow((it[0]-it[1]),2)+pow((it[2]-it[3]),2);
        cout << "temp: " << temp << endl;
        k= SIGN*(it[1]-it[3]*1.0)/(it[0]-it[2]);
//        if(lineLenth < temp && k >= mk)
//        {
//            lineLenth = temp;
//            sline= it;
//        }
        if(abs(k-mk) < 0.1 && k >= mk)
        {
            mk = k;
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

int SelfAdaption::_GetRoi(const cv::Mat& image,int th,cv::Rect2i& roi)
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


bool SelfAdaption::_GetMaskRoi(cv::Mat &mask, cv::Rect2i &roim)
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
