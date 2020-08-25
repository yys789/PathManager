#include "seamhandler.h"
#include "common/dbinterface.h"
#include "base/errorcode.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <dirent.h>
#include <QDebug>
#include <thread>
#include <exception>
#include <future>
#include "walgo/utils.h"
#include "walgo/histimage.h"
#include "walgo/histcalc.h"
#include "base.h"
#include "common/dbinterface.h"

UV algoUV;

#define _ERRTAG_ cout<<"["<<std::this_thread::get_id()<<"]"<<"["<<getCurrentTime().c_str()<<"]["<<__FILE__<<"]["<<__LINE__<<"]["<<__FUNCTION__<<"]"<<endl;

extern void Insertion_point(RobotPos p0, RobotPos p1, RobotPos p2, RobotPos p3, double len, vector<RobotPos> & pOut);

extern double gMotorposs;
extern double gMotorangle1s;
extern double gMotorangle2s;

SeamHandler::SeamHandler():
    m_bTaking{false}
  ,m_bWelding{false}
  ,m_bCheckSuccess{false}
  ,m_bRobotException{false}
  ,m_bCanStart{false}
  ,_camera_ptr{Singleton<Camera>::get()}
  ,_mb_ptr{Singleton<mbControl>::get()}
  ,_sqlite_ptr{Singleton<mySqliteApi>::get()}
{
    //绝对安全位置
    _sqlite_ptr->selectSQL1({"SAFEPOS"},{"CADID","'ALL'"});
    if(_sqlite_ptr->m_iRows != 1 )
        throw ERR_DB_DATA_ERROR;
    string str = _sqlite_ptr->m_vResult[0]["POS"];
    if(str.empty())
        throw ERR_DB_DATA_ERROR;
    sPosTodPos(str,AMidPointAlex);
}

SeamHandler::~SeamHandler()
{
    /// save seamInfo to db
    sync();
}

/// 空走、焊接指定焊缝
void SeamHandler::weld(int seamid)
{
    try
    {
    }catch(ReturnStatus error_code)
    {

    }catch(...)
    {

    }
}

/// 事务开始时的准备
void SeamHandler::init()
{

}

/// 事务结束后的清理
void SeamHandler::end()
{

}

bool SeamHandler::beenSensorChecked()
{
    try
    {
        _ERRTAG_;
        string path = "sensorCheck";
        bool isDone = false;
        if ( !boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path) ;
        }
        ifstream ifs(path + "/sensorCheck.log",ios::app);
        string currDate = getDate();
        string line;

        while(getline(ifs,line))
        {
            QString qline = QString::fromStdString(line);
            if(qline.split(",").size() < 4)
                continue;
            QString qy = qline.split(",")[2].split(":")[1];
            QString qz = qline.split(",")[3].split(":")[1];
            if((currDate.compare(line.substr(0,10)) == 0))
            {
                if((abs(qy.toDouble())<2)|| (abs(qz.toDouble())<2))
                {
                    isDone = true;
                }
                break;
            }
        }
        cout<<"[beenSensorChecked][OVER]"<<endl;
        return isDone;

    }
    catch(...)
    {
        qCritical() << getErrString(ERR_CHECK_BOOL_SENSOR_FAILED) <<endl;
        throw ERR_CHECK_BOOL_SENSOR_FAILED;
    }
}

void SeamHandler::send_cad_list()
{
    try
    {
        _LOGINFO_;
        _sqlite_ptr->selectSQL2({"CAD"},{"CADCODE","CADID"},{"enabled","1"});//读取所有工件给PLC
        auto vec = _sqlite_ptr->m_vResult;
        int count = vec.size();

        vector<string> vec_cads;
        vector<int> vec_cadcode;
        for(auto& v: vec){
            auto it = v.begin();
            while(it != v.end()){
                if(it->first == "CADCODE")
                    vec_cadcode.push_back(stringToNum(it->second));
                if(it->first == "CADID")
                    vec_cads.push_back(it->second);
                it++;
            }
        }
        cout<<"[displayALLCAD] cadlist : "<< endl;
        for(int i = 0; i < vec_cads.size(); i++)
            cout<<"[arg"<<i<<"] : "<<vec_cads[i] <<endl;

        vector<uint16_t> tempDate;
        for(int i = 0; i< vec_cads.size(); i++){
            tempDate.push_back(vec_cadcode[i]);
            vector<int> temp;
            stringToAsc(temp,vec_cads[i]);
            for(auto& t:temp)
                tempDate.push_back(t);
        }
        _mb_ptr->send_cad_list(tempDate);//将工件名写入寄存器
        vector<uint16_t>().swap(tempDate);
    }catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }catch(...){
        qCritical()<<getErrString(ERR_FUNC_DISPLAY_CADS_LIST_FAILED)<<endl;
        throw ERR_FUNC_DISPLAY_CADS_LIST_FAILED;
    }
}

double SeamHandler::getFeature(Mat m)
{
}

void SeamHandler::reset()
{
    try
    {
        _ERRTAG_;
        //        wData.release_AllSeamPathMap();
        //        map<int,RelateInfo>().swap(relateMap);
        getMidPointById(m_sCurrMark.seamID,m_sCurrMidPoint,AMidPointAlex);
        _robot->absoluteMove(AMidPointAlex);//絕對安全點
        updateComInfo("'currentMidPoint'","'midpoint_a'");
        if(currentPosSafe(AMidPointAlex))//比较安全点和中间点位置差距
        {
            cout<<"[RESET][OVER]"<<endl;
        }else{
            throw ERR_ROBOT_POS_NOT_SAFE;
        }
    }catch(ReturnStatus error_code)
    {
        throw;
    }catch(...)
    {
        throw ERR_FUNC_RESET_FAILED;
    }
}

void SeamHandler::syncMotorAngle()
{
    try{
        _ERRTAG_;
        //        _mb_ptr->writeOver();
        vector<int> data(100,0);
        //_mb_ptr->getData(data);

        gMotorposs = data[95]/10.0;//地轨
        gMotorangle2s= data[96]/10.0;//旋转
        gMotorangle1s = data[97]/10.0;//翻转
        cout<<"current currmotorpos : "<<gMotorposs<<endl;
        cout<<"current currmotorangle1 : "<<gMotorangle1s<<endl;
        cout<<"current currmotorangle2 : "<<gMotorangle2s<<endl;

        updateComInfo("'CURMOTORPOS'",to_string(gMotorposs));
        updateComInfo("'CURMOTORANGLE2'",to_string(gMotorangle2s));
        updateComInfo("'CURMOTORANGLE1'",to_string(gMotorangle1s));

    }catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }catch(...)
    {
        throw ERR_MOTOR_SYNC_FAILED;
    }
}

void SeamHandler::NewSyncMotorAngle()
{
    try{
        _ERRTAG_;
        auto data = _mb_ptr->obj.msgBody;
        gMotorposs = data[20]/10.0;//地轨
        gMotorangle2s= data[21]/10.0;//旋转
        gMotorangle1s = data[22]/10.0;//翻转
        cout<<"current currmotorpos : "<<gMotorposs<<endl;
        cout<<"current currmotorangle1 : "<<gMotorangle1s<<endl;
        cout<<"current currmotorangle2 : "<<gMotorangle2s<<endl;

        updateComInfo("'CURMOTORPOS'",to_string(gMotorposs));
        updateComInfo("'CURMOTORANGLE2'",to_string(gMotorangle2s));
        updateComInfo("'CURMOTORANGLE1'",to_string(gMotorangle1s));

    }catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }catch(...)
    {
        throw ERR_MOTOR_SYNC_FAILED;
    }
}

bool SeamHandler::currentPosSafe(RobotAxle midPos)
{
    _ERRTAG_;
    bool result = false;
    RobotAxle currPos = _robot->getCurrAxle();
    double maxA = abs(currPos.a1-midPos.a1);
    maxA = maxA<abs(currPos.a2-midPos.a2)?abs(currPos.a2-midPos.a2):maxA;
    maxA = maxA<abs(currPos.a3-midPos.a3)?abs(currPos.a3-midPos.a3):maxA;
    maxA = maxA<abs(currPos.a4-midPos.a4)?abs(currPos.a4-midPos.a4):maxA;
    maxA = maxA<abs(currPos.a5-midPos.a5)?abs(currPos.a5-midPos.a5):maxA;
    maxA = maxA<abs(currPos.a6-midPos.a6)?abs(currPos.a6-midPos.a6):maxA;
    if(maxA < 1)//比较安全点和中间点位置差距
        result = true;
    cout<<"[currentPosSafe][OVER]"<<endl;
    return result;
}

void SeamHandler::sensorCheck()
{
    try
    {
        _ERRTAG_;
        if(!_mb_ptr || !_camera_ptr)
            throw ERR_DEVICE_DISCONNECTED;

        Config _sys_config(SYS);
        getComInfo(mComInfo);
        auto exposure = stoi(mComInfo["cameraAdjust.exposure"]);
        auto gain = stoi(mComInfo["cameraAdjust.gain"]);
        auto sUV = mComInfo["cameraAdjust.uv"];
        RobotPos adjustPos = RobotPos::instance(mComInfo["cameraAdjust.pos"]);
        UV adjustUV = strToUV(sUV);

        _camera_ptr->setExposure(exposure);
        _camera_ptr->setGain(gain);

        _robot->absoluteMove(adjustPos);
        _mb_ptr->openLaser();
        auto mat = _camera_ptr->takePicture();
        _mb_ptr->closeLaser();
        exposure = _sys_config.get<int>("camera.exposure",-1);
        gain = _sys_config.get<float>("camera.gain",-1);
        if(exposure == -1 || gain == -1)
        {
            throw ERR_SYSINFO_DATA_ERROR;
        }
        _camera_ptr->setExposure(exposure);
        _camera_ptr->setGain(gain);
        string time = getCurrentTime();
        cv::imwrite("sensorCheck/"+time+".png",mat);
        UV uv;
        int p = 0;
        int thresh = 16;
        uv = getJUV(mat, p, thresh,time);
        uv.u = uv.u + _sys_config.get<int>("camera.roi_offset_x");
        uv.v = uv.v + _sys_config.get<int>("camera.roi_offset_y");
        cout<<" u , v :"<<uv.u<<" ,"<<uv.v<<endl;
        uv.u -= adjustUV.u;
        uv.v -= adjustUV.v;
        float y, z;
        laser_yz(uv.u, uv.v, y, z);
        cout<<" y , z :"<<y<<" ,"<<z<<endl;
        string path = "sensorCheck";
        if ( !boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path) ;
        }
        ofstream ofs(path + "/sensorCheck.log",ios::app) ;
        ofs << time <<" u:"<<uv.u<<" , v:"<<uv.v<<" , y:"<<y<<" ,z:"<<z<<"\n";
        ofs.close();
        if( abs(y) > 2 || abs(z) > 2 )
        {
            throw ERR_CHECK_EXCESSIVE_DEVIATION;
        }
        //轨迹规划，回中间点
        cout<<"[sensorCheck][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        throw;
    }
    catch(...)
    {
        qCritical()<<getErrString(ERR_CHECK_SENSOR_FAILED)<<endl;
        throw ERR_CHECK_SENSOR_FAILED;
    }
}

void SeamHandler::checkBrightness(int code)
{
    try
    {
        _ERRTAG_;
        m_bCheckSuccess = true;
        m_bRobotException = false;
        if(!_mb_ptr || !_camera_ptr )
            throw ERR_DEVICE_DISCONNECTED;

        getComInfo(mComInfo);
        auto sPos = mComInfo["checkBright.pos1"];
        auto gain = stoi(mComInfo["checkBright.gain"]);
        auto brightnessf = stoi(mComInfo["checkBright.brightnessFloor"]);
        auto brightnessc = stoi(mComInfo["checkBright.brightnessCeil"]);

        //        RobotAxle axle;
        //        sPosTodPos(sPos,axle);
        //        _robot->absoluteMove(axle);

        std::this_thread::sleep_for(100_ms);
        int exposure = 1000;

        Config _sys_config("etc/sys.info");
        auto _sys_exposure = _sys_config.get<int>("camera.exposure",2500);
        auto _sys_gain = _sys_config.get<int>("camera.gain",0);
        bool result = false;
        int step = 100;
        while(1)
        {
            _camera_ptr->setExposure(exposure);
            _camera_ptr->setGain(gain);
            _mb_ptr->openLaser();
            std::this_thread::sleep_for(10_ms);
            Mat mat;
            _camera_ptr->takePicture().copyTo(mat);
            _mb_ptr->closeLaser();
            if(mat.cols < 100)
                throw ERR_CHECK_BRIGHTNESS_FAILED;
            cv::imwrite("testLog/hist.png",mat);
            string fileName("etc/image.info");
            std::map<std::string, int> config;
            string mTypeSeamNO = "22_seam1";
            if (!walgo::myReadConfig(fileName.c_str(),mTypeSeamNO, config)) {
                std::cout <<"WARNING! No config "<<fileName.c_str()<< " file" << std::endl;
            }
            std::vector<double> output{};
            int ct = 0;
            bool doMediumBlur= false;
            //result = hist(mat,output,config,ct,doMediumBlur);
            if(result)
            {
                if((output[1] > brightnessf && output[1] < brightnessc)
                        || (exposure == 300))
                {
                    m_bCheckSuccess = true;

                    _robot->absoluteMove(AMidPointAlex) ;

                    _sqlite_ptr->updateSQL({"COMPARAMS"},{"comValue",to_string(exposure)},{"comName","checkBright.exposure"});
                    _sqlite_ptr->updateSQL({"COMPARAMS"},{"comValue",to_string(gain)},{"comName","checkBright.gain"});

                    double rate = exposure/500.0;
                    _sys_gain = rate>1.001?round(log10(rate)*20):0;
                    _sys_exposure = rate>1.001?2500:exposure*5;
                    _sys_config.put<int>("camera.exposure",_sys_exposure);
                    _sys_config.put<int>("camera.gain",_sys_gain);
                    _sys_config.sync();
                    _camera_ptr->setExposure(_sys_exposure);
                    _camera_ptr->setGain(_sys_gain);
                    break;
                }
                else
                {
                    if(output[1] > brightnessc && exposure-step>=300)
                    {
                        exposure -= step;
                    }
                    if(output[1] < brightnessf)
                    {
                        if(exposure+step/2<=1000)
                        {
                            if(step < 25)
                            {
                                throw ERR_CHECK_BRIGHTNESS_FAILED;
                            }
                            step /=2;
                            exposure += step;
                        }
                        else
                        {
                            throw ERR_CHECK_BRIGHTNESS_FAILED;
                        }
                    }
                }
            }else{
                throw ERR_CHECK_BRIGHTNESS_FAILED;
            }
        }
    }
    catch(ReturnStatus error_code)
    {
        m_bCheckSuccess = false;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            m_bRobotException = true;

        qCritical()<<getErrString(error_code)<<endl;
        throw ;
        //        if(error_code == ERR_BRIGHTNESS_IS_LOW)//亮度太暗
        //        {
        //            myMessageBox mb(NULL,QString("亮暗度检测"),QString("图片亮度过低，请尝试用洁净纸轻擦传感器底部防飞溅玻璃，并重新检测亮度。如擦拭后亮度依然过低，请点击取消并联系厂家维护。"));
        //            mb.setAttribute(Qt::WA_AlwaysStackOnTop);
        //            if(QDialog::Accepted == mb.exec()){
        //                checkBrightness();
        //            }else{
        //                m_bCheckSuccess = false;
        //            }
        //        }else{
        //            if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
        //                m_bRobotException = true;
        //            else
        //                m_bCheckSuccess = false;
        //        }
    }
    catch(...)
    {
        m_bCheckSuccess = false;
        qCritical()<<"check brightness failed ."<<endl;
        throw ERR_CHECK_BRIGHTNESS_FAILED;
    }
}

void SeamHandler::rotateMotor(int seamId)//need correct
{
    try
    {
        _LOGINFO_;
        if(!_mb_ptr )
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        getSeamMotorAngleInfo(seamId,angleInfo);

        cout<<"[rotateMotor]:"<<angleInfo.motors.angle1<<" | "<<angleInfo.motors.angle2 \
           <<" | "<<angleInfo.motors.angle3<<endl;

        _sqlite_ptr->selectSQL2({"CAD"},{"SAFELOC","SAFEANGLE1"},{"CADID",m_strCADID});
        if(_sqlite_ptr->m_vResult.size() == 0) throw;
        double loc = stringToNum(_sqlite_ptr->m_vResult[0]["SAFELOC"]);
        double safeA1 = stringToNum(_sqlite_ptr->m_vResult[0]["SAFEANGLE1"]);
        map<string,string> mComInfo;
        getComInfo(mComInfo);
        cout<<"gMotorposs:"<<gMotorposs<<"gMotorangle1s:"
           <<gMotorangle1s<<"gMotorangle2s:"<<gMotorangle2s<<endl;
        if(abs(gMotorposs-angleInfo.motors.angle1) > 0.1
         ||abs(gMotorangle1s-angleInfo.motors.angle2) > 0.1
         ||abs(gMotorangle2s-angleInfo.motors.angle3) > 0.1)
        {
            if(gMotorposs > loc)
            {
                _mb_ptr->rotateNew(loc,30,gMotorangle1s,10,gMotorangle2s,10,1);
            }
            if(gMotorangle1s < safeA1)
            {
                _mb_ptr->rotateNew(gMotorposs,30,safeA1,10,gMotorangle2s,10,1);
            }
            _mb_ptr->rotateNew(angleInfo.motors.angle1,30,angleInfo.motors.angle2,10,angleInfo.motors.angle3,10,1);
        }
        _sqlite_ptr->updateSQL({"GOODSHIS"},{"ORDERID",to_string(m_sCurrMark.orderID)}\
                               ,{"GOODSID",to_string(m_iCurrGoodSid),"CADID",m_strCADID});
        cout<<"[rotateMotor][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        qCritical() << getErrString(ERR_MOTOR_ROTATE_FAILED) <<endl;
        throw ERR_MOTOR_ROTATE_FAILED;
    }
}

bool SeamHandler::findSeamPath(int seamId)
{
    _ERRTAG_;
    bool result = false;
    auto iter = wData.AllSeamPathMap.find(seamId);
    if(iter != wData.AllSeamPathMap.end())
        result = true;
    cout<<"[findSeamPath][OVER]"<<endl;
    return result;
}

bool SeamHandler::correlation()
{
    try
    {
        _LOGINFO_;
        auto seamId = m_sCurrMark.seamID;
        map<string,string> sInfo;
        getSeamInfo(seamId,sInfo);
        string path = "testLog/"+ m_sCADID + "_" + sInfo["SEAMNAME"] + "_" + m_sCurrTime  ;

        if ( !boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path) ;
        }
        int rank0 = stoi(sInfo["RANK"])+1;
        bool ret = fitSeamPath(path,wData.AllSeamPathMap[seamId],rank0);
        if(ret)
        {
            if(ret)
            {
                ret = correlationBySep(path,wData.AllSeamPathMap[seamId],rank0, relateMap[seamId]);
            }
            ///龙骨数据扩展
            if(stoi(sInfo["TRACING"]) != 1)
                externPath(seamId,stoi(sInfo["TRACING"]), wData.AllSeamPathMap[seamId]);
            std::vector<RobotPos> vec_pos;
            getScanPos(seamId,vec_pos);
            double sumlen = 0;
            ret = checkPath(wData.AllSeamPathMap[seamId],vec_pos,sumlen);
            if(ret)
            {
                map<string,string> externInfos;
                getExternInfo(seamId,externInfos);
                if(externInfos.size() > 0)  /// 需要延展
                {
                    vector<RobotPos> keel0;
                    getFramePos(seamId,keel0);
                    int id1 = atoi(externInfos["RELATE1"].c_str());
                    vector<RobotPos> keel1;
                    getFramePos(id1,keel1);
                    int id2 = atoi(externInfos["RELATE2"].c_str());
                    vector<RobotPos> keel2;
                    getFramePos(id2,keel2);
                    int id3 = atoi(externInfos["RELATE3"].c_str());
                    vector<RobotPos> keel3;
                    getFramePos(id3,keel3);
                    externPath3(wData.AllSeamPathMap[seamId],keel0,keel1,keel2,keel3);
                    wData.AllSeamPathMap[id1].swap(keel1);
                    wData.AllSeamPathMap[id2].swap(keel2);
                    wData.AllSeamPathMap[id3].swap(keel3);
                }
                map<string,string> mComInfo;
                getComInfo(mComInfo);
                double currentTotalLen = stringToNum(mComInfo["currentTotalLen"]);
                currentTotalLen += sumlen;
                updateComInfo("'currentTotalLen'","'"+to_string(currentTotalLen)+"'");
            }
            std::vector<RobotPos>().swap(vec_pos);
        }
        cout<<"[correlation][OVER]"<<endl;
        return ret;

    }catch(ReturnStatus error_code)
    {
        throw ERR_FUNC_CORRELATION_FAILED;
    }
    catch(...)
    {
        throw ERR_FUNC_CORRELATION_FAILED;
    }
}

void SeamHandler::moveToMidPoint(int seamId)
{
    try
    {
        _ERRTAG_;
        //获取当前记录的安全点位置
        getMidPointById(seamId,m_sCurrMidPoint,currSeamMidAxle);
        //絕對安全點
        _robot->absoluteMove(currSeamMidAxle);

        updateComInfo("'currentMidPoint'","'" + m_sCurrMidPoint + "'");
        if(!currentPosSafe(currSeamMidAxle))
            throw ERR_ROBOT_POS_NOT_SAFE;

        cout<<"[moveToMidPoint][OVER]"<<endl;
    }catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }catch(...)
    {
        qCritical()<<getErrString(ERR_ROBOT_POS_NOT_SAFE)<<endl;
        throw ERR_ROBOT_POS_NOT_SAFE;
    }
}

int SeamHandler::dealSeamImages(int seamId)
{
    _ERRTAG_;
    Config _sys_config(SYS);
    getMidPointById(seamId,m_sCurrMidPoint,AMidPointAlex);
    try{
        m_bCanStart = false;
        wData.release_pair_vec();
        //扫描焊缝
        std::future<int> f1 = std::async(std::launch::async,&SeamHandler::scanSeam,this,seamId);
        while(!m_bCanStart)
        {
            std::this_thread::sleep_for(1_ms);
        }
        //拍取焊缝图像
        std::future<int> f2 = std::async(std::launch::async,&SeamHandler::takePictures,this,seamId);
        //处理焊缝图像流程
        std::future<int> f3 = std::async(std::launch::async,&SeamHandler::dealImages,this,seamId);
        int result2 = f2.get();
        cout<<"result2 : "<<result2<<endl;
        if(result2 != 0)
        {
            wData.saveImages();
            throw (ReturnStatus)result2;
        }

        int result3 = f3.get();
        cout<<"result3 : "<<result3<<endl;
        if(result3 != 0)
        {
            wData.saveImages();
            throw (ReturnStatus)result3;
        }

        int result1 = f1.get();
        cout<<"result1 : "<<result1<<endl;
        if(result1 != 0)
        {
            //            wData.saveImages();
            throw (ReturnStatus)result1;
        }
        wData.release_pair_vec();

        cout<<"[dealSeamImages][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        m_bTaking = false;
        wData.release_pair_vec();
        wData.AllSeamPathMap.erase(seamId);
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        if(error_code!= ERR_ROBOT_MOVEING_EXCEPTION)
            _robot->absoluteMove(currSeamMidAxle);//絕對安全點
        throw;
    }
    catch(...){
        m_bTaking = false;
        wData.release_pair_vec();
        wData.AllSeamPathMap.erase(seamId);
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        _robot->absoluteMove(currSeamMidAxle);//絕對安全點
        throw ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

int SeamHandler::takePictures(int seamId)
{
    try
    {
        _LOGINFO_;
        m_bTaking = true;
        m_imgCounts = 0;
        deque<cv::Mat>().swap(mat_que);
        wData.release_pair_vec();
        if(!_mb_ptr || !_camera_ptr)
            throw ERR_DEVICE_DISCONNECTED;

        std::this_thread::sleep_for(10_ms);
        RobotPos prePos = RobotPos::instance();
        while(m_bTaking)
        {
            std::lock_guard<std::mutex> unlck(_mtx);
            auto currPos = _robot->getCurrPos( RobotTCP::UPLOAD );
            if(pow(currPos.x-prePos.x,2) +pow(currPos.y-prePos.y,2) +pow(currPos.z-prePos.z,2) < 1)//机器人位移1mm拍一张照
                continue;
            prePos = currPos;
            Mat mat;
            _camera_ptr->takePicture().copyTo(mat);//拍照
            mat_que.push_back(mat);//deque 容器存放照片
            SensorDataPack td = {mat, currPos,getMicTime(),0};
            wData.pair_vec.emplace_back(td) ;//vector容器存放照片和对应的坐标
            m_imgCounts++;//累计照片数量
        }

        wData.map_pair.insert(make_pair(seamId,wData.pair_vec));
        _mb_ptr->closeLaser();

        updateSeamStatus(seamId,m_iCurrGoodSid,SCANING);
        cout<<"[takePictures][OVER]"<<endl;
        return RESULT_OK;

    }catch(ReturnStatus error_code)
    {
        return error_code;
    }
    catch(...)
    {
        deque<cv::Mat>().swap(mat_que);
        m_imgCounts = 0;
        m_bTaking = false;
        wData.release_pair_vec();
        if(_mb_ptr)
            _mb_ptr->closeLaser();
        return ERR_CAMERA_TAKE_PICTURES_FAILED;
    }
}

int SeamHandler::dealImages(int seamId)
{
    // return 0;
    try
    {
        _LOGINFO_;
        Config _sys_config(SYS);

        if(m_iAction == 3) // 扫描
            return -1;

        map<string,string> sInfo;
        getSeamInfo(seamId,sInfo);
        string seamType = sInfo["SEAMTYPE"];
        string option = sInfo["OPTION"];
        cout<<"seamType =  "<<seamType <<" , option = "<< option <<endl;
        uint tc = 0;
        for(uint i=0;i<option.length();i++)
        {
            tc *= 2;
            if(option.substr(i,1) == "1")
            {
                tc+=1;
            }
        }
        if(seamType == "0")
        {
            return ERR_DB_DATA_ERROR;
        }
        GetSBuvA* _gxA = NULL;
        if(seamType == "M")
        {
            _gxA = new GetSBuvM(LT_M);
        }else if(seamType == "S")
        {
            _gxA = new GetSBuvS(LT_S);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "J")
        {
            _gxA = new GetSBuvJ(LT_J);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "JS")
        {
            _gxA = new GetSBuvJS(LT_JS);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "EE")
        {
            _gxA = new GetSBuvE(LT_EE);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "W")
        {
            _gxA = new GetSBuvW(LT_WW);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "I")
        {
            _gxA = new GetSBuvI(LT_I);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "SE")
        {
            _gxA = new GetSBuvSE(LT_SE);
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }else if(seamType == "V")
        {
            _gxA = new GetSBuvVW(LT_VW);
            _gxA->set_weld_off();
        }
        if(1)//u
        {
            _gxA->clear();
            while(m_bTaking || mat_que.size() !=0)
            {
                bool needadd = false;
                cv::Mat mat;
                {
                    if(mat_que.size())
                    {
                        std::lock_guard<std::mutex> unlck(_mtx);
                        mat_que.front().copyTo(mat);
                        mat_que.pop_front();
                        needadd = true;
                    }
                }
                //m= clock();//hu++
                if(needadd)
                {
                    if(_gxA->AddImg(mat) != 0)
                    {
                        continue;
                    }
                }
                //cout << "addOneImage used: " << clock()-m << "us." <<endl;//hu++
            }

            fstream grayLog;
            grayLog.open("gray.log",ios::app);
            grayLog<< seamId <<" "<<m_sCurrTime<<" _exposure:"<<_sys_config.get<int>("camera.exposure",0)<<":";
            int sum_m = 0;

            sum_m = m_imgCounts>0?sum_m/m_imgCounts:0;
            grayLog<<sum_m;
            grayLog << endl;
            grayLog.close();
            _gxA->confirm(m_imgCounts);
            vector<RobotPos> pos_vec;
            UV2path * up;
            map<string,string> confs;

            confs["rank"] = sInfo["RANK"];       // 参数1
            confs["width"] = sInfo["WIDTH"];    // 参数2
            confs["height"] = sInfo["HEIGHT"];  // 参数3
            confs["AUTOFIT"] = sInfo["AUTOFIT"];
            string seamName = sInfo["SEAMNAME"];
            getSeamWeldInfo(seamId,weldConf);
            confs["OFFSET"] = weldConf[0].offset.toStr();
            confs["S_W"] = to_string(weldConf[0].WI.S_W);             // 参数6
            confs["I_W"] = to_string(weldConf[0].WI.I_W);             // 参数6
            confs["V_W"] = to_string(weldConf[0].WI.V_W);             // 参数6
            up = new UV2path(); /// 海康 720*540 新图像算法
            up->setDbInfos(confs);
            map<int,Point2d> pts;
            _gxA->getuv(pts);
            map<int,Point2d> lts;
            if(seamType.compare("EE") == 0)
            {
                _gxA->getuvd(lts);
                up->setLUV(lts);
            }
            up->setMUV(pts);
            map<int,RobotPos> poss;
            int idx = 0;
            for(auto s : wData.pair_vec)
            {
                poss[idx++] = s.r;
            }

            up->setPos(poss);
            up->setCurTime(m_sCurrTime);
            up->setCurCad(m_sCADID);
            up->setSeamName(seamName);
            up->get3dPath(pos_vec);
            if(confs["AUTOFIT"] == "1")
            {
                _sqlite_ptr->updateSQL({"WELDPARAMS"},{"S_W",up->getInfos("S_W"),\
                                                       "I_W",up->getInfos("I_W"),\
                                                       "V_W",up->getInfos("V_W")},\
                {"SEAMID",to_string(seamId)});      // 返回更新1
                _sqlite_ptr->updateSQL({"SEAMWELDINFO"},{"offset","'"+up->getInfos("offset")+"'"},\
                {"SEAMID",to_string(seamId),"WELDORDER","0"}); // 返回更新4
            }
            delete up;
            _gxA->clear();
            delete _gxA;
            _sqlite_ptr->selectSQL2({"CAD"},{"BASEPOINT"},{"CADID","'"+m_sCADID+"'"});
            RobotPos tool = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
            MOTORDEGRE degree;
            degree.angle1 = stringToNum(sInfo["MOTORPOSS"]);
            degree.angle2 = stringToNum(sInfo["MOTORANGLE1S"]);
            degree.angle3 = stringToNum(sInfo["MOTORANGLE2S"]);
            vector<RobotPos> pos_tool;
            RobotPos zero = RobotPos::instance();
            for(auto p : pos_vec)
            {
                if(p.dis(zero) > 1)
                {
                    posInTool(p,tool,getPosition(degree));
                }
                pos_tool.push_back(p);
            }
            wData.AllSeamPathMap[seamId].swap(pos_tool);
        }
        cout<<"deal image end ...."<<endl;
        m_imgCounts = 0;

        updateSeamStatus(seamId,m_iCurrGoodSid,IMAGEPROCESSING);
        cout<<"[dealImages][OVER]"<<endl;
        return RESULT_OK;
    }catch(ReturnStatus error_code)
    {
        m_imgCounts = 0;
        m_bTaking = false;
        deque<cv::Mat>().swap(mat_que);
        wData.release_pair_vec();
        if(error_code == ERR_MODULE_GET_UV_FAILED||error_code == ERR_MODULE_ADD_IMAGES_FAILED||error_code == ERR_MODULE_CONFIRM_IMAGES_FAILED)
            return ERR_FUNC_DEAL_IMAGES_FAILED;
        else
            return error_code;
    }
    catch(...){
        m_bTaking = false;
        deque<cv::Mat>().swap(mat_que);
        m_imgCounts = 0;
        wData.release_pair_vec();
        return ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

void SeamHandler::getSeamList()
{
    try
    {
        _ERRTAG_;
        vector<seamMark>().swap(m_vWeldList);
        vector<int> data(100,0);
        //_mb_ptr->getData(data);
        vector<int> vec_cad(data.begin(),data.begin()+20);
        string cadid = AscToString(vec_cad);
        //        string cadid = "shell";
        m_sCADID = cadid;
        wData.m_sCADID = cadid;
        m_strCADID = "'"+ m_sCADID+"'";
        cout<<"[cadid]:"<<m_sCADID<<endl;

        _sqlite_ptr->selectSQLOrder({"SEAMINFO"},{"SEAMID","SEAMNAME","ORDERID"},{"CADID","'"+ cadid+"'","ENABLED","1"},{"ORDERID","asc"});
        if(_sqlite_ptr->m_iRows == 0)
            throw ERR_DB_NO_DATA_FOUND;
        auto vec = _sqlite_ptr->m_vResult;
        vector<seamMark>().swap(m_vWeldList);
        std::map<int,string>().swap(seamMap);
        for(uint i = 0 ; i< vec.size() ; i++)
        {
            seamMark sm;
            sm.orderID = stoi(vec[i]["ORDERID"]);
            sm.seamID  = stoi(vec[i]["SEAMID"]);
            sm.name = _sqlite_ptr->m_vResult[0]["SEAMNAME"];
            m_vWeldList.push_back(sm);
            seamMap[sm.seamID] = sm.name;
            cout<<"seamid : "<<sm.seamID<<" | "<<"orderid : "<<sm.orderID<<" | "<<"seam : "<<sm.name<<endl;
        }
        wData.seamMap = seamMap;
        weldList_sort();
        cout<<"[getSeamList][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        throw ;
    }
    catch(...)
    {
        throw ERR_FUNC_GET_SEAMS_LIST_FAILED;
    }
}

void SeamHandler::checkSeamInfo()
{
    try
    {
        _ERRTAG_;
        for(int i = 0; i< m_vWeldList.size(); i++)
        {
            _sqlite_ptr->selectSQL1({"SCANEPOS"},{"SEAMID",to_string(m_vWeldList[i].seamID),"ENABLED","1"});
            if(_sqlite_ptr->m_iRows < 2)
                throw ERR_DB_DATA_ERROR;

            _sqlite_ptr->selectSQL1({"WELDPARAMS"},{"SEAMID",to_string(m_vWeldList[i].seamID)});
            if(_sqlite_ptr->m_iRows == 1)
            {
                double zero = 0;
                auto m = _sqlite_ptr->m_vResult[0];
                if(equald(stringToNum(m["S_W"]),zero)||equald(stringToNum(m["I_W"]),zero)||equald(stringToNum(m["V_W"]),zero))
                    throw ERR_DB_DATA_ERROR;
            }
            else{
                throw ERR_DB_DATA_ERROR;
            }
        }
        cout<<"[checkSeamInfo][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        throw ;
    }
    catch(...)
    {
        throw ERR_CHECK_SEAMINFO_FAILED;
    }
}

void SeamHandler::getSelectedSeamList()
{
    try
    {
        _ERRTAG_;
        vector<int> data(100,0);
        //_mb_ptr->getData(data);
        vector<int> vec_cad(data.begin(),data.begin()+20);
        //获取CAD工件名
        m_sCADID = AscToString(vec_cad);
        wData.m_sCADID = m_sCADID;
        m_strCADID = "'"+ m_sCADID+"'";
        cout<<"[cadid]:"<<m_sCADID<<endl;
        //获取PLC界面选中的焊缝列表
        vector<int> vec_seamNums(data.begin()+20,data.begin()+80);
        vector<seamMark>().swap(m_vWeldList);
        std::map<int,string>().swap(seamMap);
        for(int i = 0; i < vec_seamNums.size(); i++)
        {
            if(vec_seamNums[i] != 0)
            {
                string seamName;
                IntToSeamName(vec_seamNums[i],seamName);
                _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMID","SEAMNAME","ORDERID"},{"CADID",m_strCADID,"SEAMNAME","'" + seamName + "'","MAIN","1","ENABLED","1"});
                if(_sqlite_ptr->m_iRows == 0)
                {
                    cout<<"SEAMINFO 设置有错"<<endl;
                    continue;
                }
                seamMark sm;
                sm.orderID = stoi(_sqlite_ptr->m_vResult[0]["ORDERID"]);
                sm.seamID  = stoi(_sqlite_ptr->m_vResult[0]["SEAMID"]);
                sm.name = _sqlite_ptr->m_vResult[0]["SEAMNAME"];
                map<string,string> externInfos;
                getExternInfo(sm.seamID,externInfos);
                if(externInfos.size() > 0)  /// 叶轮定制需求，需要自动延展其他三条缝
                {
                    for(int i=1;i<=3;i++)
                    {
                        int id1 = atoi(externInfos["RELATE"+to_string(i)].c_str());
                        map<string,string> sInfo1;
                        if(id1 == 0)
                            continue;
                        getSeamInfo(id1,sInfo1);
                        if(sInfo1.size() > 0)
                        {
                            seamMark s1;
                            s1.orderID = stoi(sInfo1["ORDERID"]);
                            s1.seamID = stoi(sInfo1["SEAMID"]);
                            s1.name = sInfo1["SEAMNAME"];
                            m_vWeldList.push_back(s1);
                            seamMap[s1.seamID] = s1.name;
                        }
                    }
                }
                m_vWeldList.push_back(sm);
                seamMap[sm.seamID] = sm.name;
                cout<<"seamid : "<<sm.seamID<<" | "<<"orderid : "<<sm.orderID<<" | "<<"seam : "<<sm.name<<endl;
                cout<<"OK..."<<endl;
            }
        }
        wData.seamMap = seamMap;
        weldList_sort();
        cout<<"[getSelectedSeamList][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        throw ;
    }
    catch(...)
    {
        throw ERR_FUNC_GET_SEAMS_LIST_FAILED;
    }
}

void SeamHandler::autoScanWelding()
{
    if(isScanWelding.load()) {
        qWarning()<<"["<<__FUNCTION__<<"]"<<"当前正在焊接!";
        return;
    }else{
        isScanWelding.exchange(true);
    }
    try
    {
        _LOGINFO_;
        Config _sys_config(SYS);
        double movespeed = _sys_config.get<double>("robot.moveSpeed",300);
        _sys_config.put("robot.speed", movespeed) ;
        _sys_config.sync();
        int seamId = _mb_ptr->get_seamId();
        map<string,string> sInfo;
        getSeamInfo(seamId,sInfo);
        m_sCADID = sInfo["CADID"];
        wData.m_sCADID = m_sCADID;
        m_strCADID = "'"+ m_sCADID+"'";

        seamMark sm;
        sm.orderID = stoi(_sqlite_ptr->m_vResult[0]["ORDERID"]);
        sm.seamID  = stoi(_sqlite_ptr->m_vResult[0]["SEAMID"]);
        sm.name = _sqlite_ptr->m_vResult[0]["SEAMNAME"];

        map<string,string> externInfos;
        getExternInfo(sm.seamID,externInfos);
        if(externInfos.size() > 0)  /// 叶轮定制需求，需要自动延展其他三条缝
        {
            for(int i=1;i<=3;i++)
            {
                int id1 = atoi(externInfos["RELATE"+to_string(i)].c_str());
                map<string,string> sInfo1;
                if(id1 == 0)
                    continue;
                getSeamInfo(id1,sInfo1);
                if(sInfo1.size() > 0)
                {
                    seamMark s1;
                    s1.orderID = stoi(sInfo1["ORDERID"]);
                    s1.seamID = stoi(sInfo1["SEAMID"]);
                    s1.name = sInfo1["SEAMNAME"];
                    seamMap[s1.seamID] = s1.name;
                }
            }
        }

        seamMap[sm.seamID] = sm.name;
        wData.seamMap = seamMap;

        try
        {
            wData.release_map_pair();
            wData.sMark = sm;
            wData.sCurrTime = getCurrentTime();
            m_sCurrMark = sm;
            m_sCurrTime = wData.sCurrTime;
            m_bWeldSuccess = false;
            //主焊缝部分
            map<string,string> sInfo;///存放焊缝的属性及相关信息
            getSeamInfo(seamId,sInfo);
            //先补全关联焊缝信息
            CompletionRelateInfo(seamId);
            if(stoi(sInfo["TRACING"]) == 1 || stoi(sInfo["TRACING"]) == 3)//是否是跟蹤？1是 ， 0 否
            {
                if(initParams(to_string(wData.sMark.seamID),relateMap[wData.sMark.seamID],m_iAction==1))
                    workTracing();
                wData.AllSeamPathMap[wData.sMark.seamID].swap(to_corra);
            }
            else/**/
            {
                getSeamPath(seamId);//扫描主焊缝
                optimizePath();//关联得到新的路径数据
                startWelding();//焊缝焊接
            }

            cleanTorch();//判断是否需要清枪
            m_bWeldSuccess = true;
            wData.release_map_pair();
        }
        catch(ReturnStatus error_code)
        {
            vector<RobotPos>().swap(tempPath);
            m_bWeldSuccess = false;
            wData.release_map_pair();
            if(error_code != ERR_FUNC_DEAL_IMAGES_FAILED &&  error_code != ERR_FUNC_CORRELATION_FAILED )
                throw;
        }
        catch(...)
        {
            vector<RobotPos>().swap(tempPath);
            m_bWeldSuccess = false;
            wData.release_map_pair();
            throw ERR_FUNC_DEAL_SEAM_FAILED;
        }
        updateProgress();//更新焊接进度
        reset();//复位
        cout<<"[autoScanWelding][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
    }catch(...)
    {
    }
    isScanWelding.exchange(false);
}

void SeamHandler::CompletionRelateInfo(int seamId)
{
    try
    {
        _LOGINFO_;
        auto & rif = relateMap[seamId];
        getRelatedSeamsInfo(seamId,rif);
        if(rif.beginId != 0)
        {
            getSeamPath(rif.beginId);
            map<string,string> sInfo;
            getSeamInfo(rif.beginId,sInfo);
            rif.beginRank = stoi(sInfo["RANK"])+1;
            vector<RobotPos> vec_pos1(wData.AllSeamPathMap[rif.beginId]);
            rif.beginPath.swap(vec_pos1);
        }
        else
        {
            rif.beginRank = 0;
        }
        if(rif.endId != 0)
        {
            getSeamPath(rif.endId);
            map<string,string> sInfo;
            getSeamInfo(rif.endId,sInfo);
            rif.endRank = stoi(sInfo["RANK"])+1;
            vector<RobotPos> vec_pos2(wData.AllSeamPathMap[rif.endId]);
            rif.endPath.swap(vec_pos2);
        }
        else
        {
            rif.endRank = 0;
        }
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        qCritical() << getErrString(ERR_FUNC_DEAL_SEAM_FAILED) <<endl;
        throw ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

void SeamHandler::updateProgress()
{
    try
    {
        _ERRTAG_;
        int totalCounts = m_vWeldList.size();
        int successCounts = m_vWelded_ok.size();
        string currSeam = m_sCurrMark.name;
        int num = 0;
        if(!m_bWeldSuccess)
            num = SeamNameToInt(currSeam);
        //_mb_ptr->updateProgress(totalCounts,successCounts,num);
        cout<<"[updateProgress][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        map<string,string>().swap(mSeamInfo);
        throw;
    }catch(...)
    {
        map<string,string>().swap(mSeamInfo);
        throw ERR_FUNC_GET_SEAMINFO_FAILED;
    }
}

void SeamHandler::setActionType(int act)
{
    _ERRTAG_;
    m_iAction = act;
    cout<<"[setActionType][OVER]"<<endl;
}

/// 需要延展的焊缝使用刚开始的变位机状态
void SeamHandler::externPath(int seamId,int tracing, vector<RobotPos> & path)
{
    _ERRTAG_;
    if(path.size() < 2)
        return;
    std::vector<RobotPos> keelKeel;  /// 龙骨及轨迹对
    /// 先准备龙骨数据
    getFramePos(seamId,keelKeel);
    mergePath(path,tracing,keelKeel);
}

void SeamHandler::cleanTorch()
{
    _ERRTAG_;
    //    _mb_ptr->cleanTorch();
    //    getComInfo(mComInfo);
    //    double currWeldingLen = stringToNum(mComInfo["currentTotalLen"]);
    //    double totalLen = stringToNum(mComInfo["totalLen"]);

    //    if(currWeldingLen>totalLen)
    //    {
    //        //        RobotAxle cur = _robot->getCurrAxle();
    //        //        _robot->absoluteMove(currSeamMidAxle) ;
    //        //        cleanTorch();//清枪
    //        //        _robot->absoluteMove(currSeamMidAxle) ;
    //        //        _robot->absoluteMove(cur);
    //    }
}

void SeamHandler::getRelatedSeamsPath(int seamId)
{
    try
    {
        _ERRTAG_;
        auto & rif = relateMap[seamId];
        getRelatedSeamsInfo(seamId,rif);
        if(rif.beginId != 0)
        {
            if(!findSeamPath(rif.beginId))//没有找到关联焊缝的路径数据
            {
                getSeamPath(rif.beginId);
            }//关联焊缝有路径数据
            map<string,string> mSeamInfo;
            getSeamInfo(rif.beginId,mSeamInfo);
            rif.beginRank = stoi(mSeamInfo["RANK"])+1;
            vector<RobotPos> vec_pos1(wData.AllSeamPathMap[rif.beginId]);
            rif.beginPath.swap(vec_pos1);
        }
        else
        {
            rif.beginRank = 0;
        }
        if(rif.endId != 0)
        {
            if(!findSeamPath(rif.endId))//没有找到关联焊缝的路径数据
            {
                getSeamPath(rif.endId);
            }//关联焊缝有路径数据
            map<string,string> mSeamInfo;
            getSeamInfo(rif.endId,mSeamInfo);
            rif.endRank = stoi(mSeamInfo["RANK"])+1;
            vector<RobotPos> vec_pos2(wData.AllSeamPathMap[rif.endId]);
            rif.endPath.swap(vec_pos2);
        }
        else
        {
            rif.endRank = 0;
        }
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        qCritical() << getErrString(ERR_FUNC_DEAL_SEAM_FAILED) <<endl;
        throw ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

void SeamHandler::getSeamPath(int seamId)
{
    try
    {
        _LOGINFO_;
        auto iter = wData.AllSeamPathMap.find(seamId);
        if(iter != wData.AllSeamPathMap.end() && iter->second.size() > 3)
            return; /// 已有数据
        moveToMidPoint(seamId);//机器人到该焊缝对应的中间点
        rotateMotor(seamId);//旋转变位机到该焊缝起始位置
        dealSeamImages(seamId);//扫描关联焊缝
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        qCritical() << getErrString(ERR_FUNC_DEAL_SEAM_FAILED) <<endl;
        throw ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

int SeamHandler::scanSeam(int seamId)
{
    Config _sys_config(SYS);
    try
    {
        _LOGINFO_;
        auto moveSpeed = _sys_config.get<double>("robot.moveSpeed",0);
        auto scanSpeed = _sys_config.get<double>("robot.scanSpeed",0);

        getMidPointById(seamId,m_sCurrMidPoint,AMidPointAlex);

        std::vector<RobotPos> vec_pos;
        getScanPos(seamId,vec_pos);

        if(moveSpeed == 0||scanSpeed == 0|| vec_pos.size() < 0)
            throw ERR_SYSINFO_DATA_ERROR;

        _sqlite_ptr->selectSQL2({"CAD"},{"BASEPOINT"},{"CADID","'"+m_sCADID+"'"});
        RobotPos tool = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
        map<string,string> sInfo;
        getSeamInfo(seamId,sInfo);
        MOTORDEGRE degree;
        degree.angle1 = stringToNum(sInfo["MOTORPOSS"]);
        degree.angle2 = stringToNum(sInfo["MOTORANGLE1S"]);
        degree.angle3 = stringToNum(sInfo["MOTORANGLE2S"]);
        double departLen = stringToNum(sInfo["DEPARTLEN"]);
        vector<RobotPos> poslst;
        for(int i = 0; i < vec_pos.size(); i++)
        {
            RobotPos pos = vec_pos[i];
            posInBase(pos,tool,getPosition(degree));
            pos = pos>CAMERA;
            poslst.push_back(pos);//arg2
        }
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        RobotPos begin = poslst[0];
        _mb_ptr->closeAirValve();
        retractMove(begin,departLen);
        _robot->absoluteMove(begin,true,true);
        //打开激光
        _mb_ptr->openLaser();
        //移动到扫描起点
        _robot->absoluteMove(poslst[0]) ;
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.scanSpeed",0)) ;
        _sys_config.sync();

        m_bCanStart = true;
        //        cond_pic.notify_all();
        //        unlck.unlock();
        //移动到扫描结束点
        _robot->absoluteMove(poslst);
        //结束拍照，扫描结束
        m_bTaking = false;
        _sys_config.put("robot.speed",  _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        int scanPosCount = poslst.size();
        // 回缩100mm
        RobotPos end = poslst[scanPosCount-1];
        retractMove(end,departLen);
        _robot->absoluteMove(end);
        _mb_ptr->openAirValve();


        //移动到絕對安全點
        _robot->absoluteMove(currSeamMidAxle);

        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        cout<<"[dealSeamImages][OVER]"<<endl;

    }
    catch(ReturnStatus error_code)
    {
        m_imgCounts = 0;
        __LINE__;
        m_bTaking = false;
        deque<cv::Mat>().swap(mat_que);
        wData.release_pair_vec();
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        if(error_code!= ERR_ROBOT_MOVEING_EXCEPTION)
            _robot->absoluteMove(currSeamMidAxle);//絕對安全點
        throw;
    }
    catch(...){
        m_imgCounts = 0;
        m_bTaking = false;
        deque<cv::Mat>().swap(mat_que);
        wData.release_pair_vec();
        _sys_config.put("robot.speed", _sys_config.get<double>("robot.moveSpeed",0)) ;
        _sys_config.sync();
        _robot->absoluteMove(currSeamMidAxle);//絕對安全點
        throw ERR_FUNC_DEAL_SEAM_FAILED;
    }
}

void SeamHandler::optimizePath()
{
    try
    {
        _LOGINFO_;
        if(correlation())//关联函数,得到新的路径数据
        {
            map<string,string> sInfo;
            getSeamInfo(m_sCurrMark.seamID,sInfo);
            _sqlite_ptr->selectSQL2({"CAD"},{"BASEPOINT"},{"CADID","'"+m_sCADID+"'"});
            RobotPos tool = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
            MOTORDEGRE degree;
            degree.angle1 = stringToNum(sInfo["MOTORPOSS"]);
            degree.angle2 = stringToNum(sInfo["MOTORANGLE1S"]);
            degree.angle3 = stringToNum(sInfo["MOTORANGLE2S"]);
            vector<RobotPos> path;
            for(auto p : wData.AllSeamPathMap[m_sCurrMark.seamID])
            {
                posInBase(p,tool,getPosition(degree));
                path.push_back(p);
            }
            int tps = tempPath.size();
            if(tps > 0 && path.size() >0)
            {
                RobotPos ts = tempPath[0];
                RobotPos te = tempPath[tps-1];
                RobotPos ps = path[0];
                RobotPos pe = path[path.size()-1];
                //*****rSeamsInfo2取值需要确认!!?
                double slen = abs(relateMap[wData.sMark.seamID].endExtern);
                vector<RobotPos> gdPath;
                if(slen > 5)
                {
                    Insertion_point(ts,te,ps,pe,slen,gdPath);
                    for(auto g : gdPath)
                    {
                        tempPath.push_back(g); /// 添加过度点
                    }
                }
            }
            tempPath.insert(tempPath.end(),path.begin(),path.end());
            Config _sys_config(SYS);
            if(_sys_config.get<int>("saveImageLevel",0))
                wData.saveImagesAll();

            updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,STARTWELDING);
        }else{
            wData.saveImagesAll();
        }
        cout<<"[optimizePath][OVER]"<<endl;
    }catch(ReturnStatus error_code)
    {
        vector<RobotPos>().swap(tempPath);
        wData.release_map_pair();
        updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,WELDINGFAILED);
        throw ERR_FUNC_CORRELATION_FAILED;
    }
    catch(...)
    {
        updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,WELDINGFAILED);
        vector<RobotPos>().swap(tempPath);
        wData.release_map_pair();
        throw ERR_FUNC_CORRELATION_FAILED;
    }
}

void SeamHandler::startWelding()
{
    try
    {
        _LOGINFO_;
        // 0 空走 1 焊接 2 空走后焊接
        switch(m_iAction)
        {
        case 0:
            m_bWelding = false;
            welding();
            break;
        case 1:
            m_bWelding = true;
            welding();
            break;
        case 2:
            m_bWelding = false;
            welding();
            if(m_bWelding)
            {
                welding();
            }
            break;
        }
        updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,WELDINGSUCCESS);
        vector<RobotPos>().swap(tempPath);
        cout<<"[start][OVER]"<<endl;
    }catch(ReturnStatus error_code)
    {
        vector<RobotPos>().swap(tempPath);
        updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,WELDINGFAILED);
        throw;
    }
    catch(...)
    {
        updateSeamStatus(m_sCurrMark.seamID,m_iCurrGoodSid,WELDINGFAILED);
        vector<RobotPos>().swap(tempPath);
        throw ERR_FUNC_WELDING_FAILED;
    }
}

void SeamHandler::welding()
{
    try
    {
        _LOGINFO_;
        auto path = tempPath;
        int ps = path.size();
        if(ps<2)
            throw -1;
        Config _sys_config(SYS);

        map<string,string> sInfo;
        getSeamInfo(m_sCurrMark.seamID,sInfo);
        int dr = stoi(sInfo["HEIGHT"]);
        int tracing = stoi(sInfo["TRACING"]); /// CASE 2?
        int autoFit = stoi(sInfo["AUTOFIT"]);
        double departLen = stringToNum(sInfo["DEPARTLEN"]);

        getSeamWeldInfo(m_sCurrMark.seamID,weldConf);

        auto moveSpeed = _sys_config.get<int>("robot.moveSpeed",200);
        auto simulateSpeed = _sys_config.get<int>("robot.simulateSpeed",100);

        for(int i = 0; i <weldConf.size(); i++)
        {
            if(m_bWelding)
            {
                _robot->setWeldingTrue(weldConf[i].WI);
            }
            RobotPos start = path[0];
            retractMove(start,departLen);
            _robot->absoluteMove(start,true,true);
            RobotPos boffset = weldConf[i].base_offset;
            RobotPos offset = weldConf[i].offset;
            RobotPos p0 = boffset<<(path[0])<<offset;
            _robot->absoluteMove(p0);
            _sys_config.put("robot.speed", simulateSpeed) ;
            _sys_config.sync();
            _robot->absoluteMove(path,weldConf[i].offset,weldConf[i].base_offset,dr,autoFit,weldConf[i].WI);
            if(tracing == 2)
            {
                vector<RobotPos> backPath;
                auto it = path.end();
                do{
                    it--;
                    RobotPos boff = RobotPos::instance();
                    boff.z = -3;
                    boff = (*it)<<boff;
                    backPath.push_back(boff);
                }while(it != path.begin());
                _robot->absoluteMove(backPath,weldConf[i].offset,weldConf[i].base_offset,dr,autoFit,weldConf[i].WI);
            }
        }
        _sys_config.put("robot.speed", moveSpeed) ;
        _sys_config.sync();

        /// 回缩100mm
        RobotPos end = _robot->getCurrPos( RobotTCP::UPLOAD );;
        retractMove(end,departLen);
        _robot->absoluteMove(end);
        _robot->absoluteMove(currSeamMidAxle);
        cout<<"[welding][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        qCritical()<<getErrString(ERR_FUNC_WELDING_FAILED)<<endl;
        throw ERR_FUNC_WELDING_FAILED;
    }
}

void SeamHandler::weldList_sort() // TODO
{
    sort(m_vWeldList.begin(),m_vWeldList.end(),comp);
}

/////////////////上方代码为跟踪准备部分，下方代码是跟踪的具体逻辑实现//////////////////////
/// 跟踪开始前的参数、容器初期化
bool SeamHandler::initParams(string seamId,RelateInfo rInfo,bool wflag)
{
    Config _sys_config(SYS);
    lastEnd = _sys_config.get<int>("weld.lastEnd",0);
    waitMode = false;
    weldFlag = wflag;
    m_bTaking = false;
    scaneEnd = RobotPos::instance();
    std::vector<RobotPos>().swap(oriPath);
    std::vector<RobotPos>().swap(seamKeel);
    std::vector<RobotPos>().swap(weldVec);
    vector<RobotPos>().swap(backPath);
    std::queue<SensorDataPack> empty2;
    dataSet.swap(empty2);
    std::vector<RobotPos>().swap(to_corra);
    curSeam = seamId;
    getSeamInfo(stoi(curSeam),seamInfo);
    curCAD = seamInfo["CADID"];
    trackMode = atoi(seamInfo["TRACING"].c_str());
    string stype = seamInfo["SEAMTYPE"];
    string option = seamInfo["OPTION"];
    default_w = stringToNum(seamInfo["WIDTH"]);
    default_h = stringToNum(seamInfo["HEIGHT"]);
    autoFit = stoi(seamInfo["AUTOFIT"]);
    angleInfo.rotateTime = 0;
    angleInfo.motors.angle1 =  stringToNum(seamInfo["MOTORPOSS"]);
    angleInfo.motors.angle2 =  stringToNum(seamInfo["MOTORANGLE1S"]);
    angleInfo.motors.angle3 =  stringToNum(seamInfo["MOTORANGLE2S"]);
    angleInfo.motore.angle1 =  stringToNum(seamInfo["MOTORPOSE"]);
    angleInfo.motore.angle2 =  stringToNum(seamInfo["MOTORANGLE1E"]);
    angleInfo.motore.angle3 =  stringToNum(seamInfo["MOTORANGLE2E"]);
    angleInfo.motor1speed =  stringToNum(seamInfo["MOTOR1SPEED"]);
    angleInfo.motor2speed =  stringToNum(seamInfo["MOTOR2SPEED"]);
    angleInfo.motor3speed =  stringToNum(seamInfo["MOTOR3SPEED"]);
    cameraMode = 0;
    rInfos = rif;
    _gxA = NULL;
    try
    {
        ///构建算法类对象
        int tc = 0;
        for(uint i=0;i<option.length();i++)
        {
            tc *= 2;
            if(option.substr(i,1) == "1")
            {
                tc+=1;
            }
        }
        if(stype == "V")
        {
            _gxA = new GetSBuvVW(LT_VW);
            lineType = LT_VW;
        }
        else if(stype == "J")
        {
            _gxA = new GetSBuvJ(LT_J);
            lineType = LT_J;
        }
        else if(stype == "S")
        {
            _gxA = new GetSBuvS(LT_S);
            lineType = LT_S;
        }
        else if(stype == "EE")
        {
            _gxA = new GetSBuvE(LT_EE);
            lineType = LT_EE;
        }
        else if(stype == "M")
        {
            _gxA = new GetSBuvM(LT_M);
            lineType = LT_M;
        }
        else if(stype == "W")
        {
            _gxA = new GetSBuvW(LT_WW);
            lineType = LT_WW;
        }
        else if(stype == "WC")
        {
            _gxA = new GetSBuvWC(LT_WC);
            lineType = LT_WC;
        }
        if(_gxA != NULL)
        {
            _gxA->set_weld_off();
            if(tc&0x01)
                _gxA->SetSide(true);
            else
                _gxA->SetSide(false);
        }
        else
        {
            cout<<"算法类型错误，请检查SEAMINFO的SEAMTYPE"<<endl;
            return false;
        }
        std::map<string,string> cadInfo;
        getCadInfo(curCAD,cadInfo,tool);
        loc = stringToNum(cadInfo["SAFELOC"]);
        safeA1 = stringToNum(cadInfo["SAFEANGLE1"]);
        vector<WELDINFO> weldInfo;
        getSeamWeldInfo(stoi(curSeam),weldInfo);

        offset = weldInfo[0].offset;
        base_offset = weldInfo[0].base_offset;

        wf.S_W = weldInfo[0].WI.S_W;
        wf.I_W = weldInfo[0].WI.I_W;
        wf.V_W = weldInfo[0].WI.V_W;
        wf.T_END = weldInfo[0].WI.T_END;
        wf.I_END = weldInfo[0].WI.I_END;
        wf.V_END = weldInfo[0].WI.V_END;
        wf.range = weldInfo[0].WI.range;
        wf.frequency = weldInfo[0].WI.frequency;
        wf.serialNumber = weldInfo[0].WI.serialNumber;
        end_err_limit = d_round(10*stringToNum(seamInfo["ENDCHECKLEN"])*6/wf.S_W); // 每秒10张
        // double defh = weldInfo[0].WI.axis_x; // 基准焊高
        double defw = weldInfo[0].WI.axis_y; // 基准焊宽
        /// 先准备龙骨数据
        getFramePos(stoi(curSeam),seamKeel);
        for(auto& pos : seamKeel){
            pos.dw = defw;
        }
    }catch(...)
    {
        __LINE__;
        cout<<"数据库缺失夹具基本点信息"<<endl;
        return false;
    }
    /// 先准备龙骨数据
    getFramePos(stoi(curSeam),seamKeel);
//    if(lastEnd > 0)
//    {
//        vector<RobotPos> temp;
//        for(int i=lastEnd;i<seamKeel.size();i++)
//        {
//            temp.push_back(seamKeel[i]);
//        }
//        int ts = temp.size();
//        int ss = seamKeel.size();
//        angleInfo.motors.angle1    +=  (angleInfo.motore.angle1-angleInfo.motors.angle1)*ts/ss;
//        angleInfo.motors.angle2    +=  (angleInfo.motore.angle2-angleInfo.motors.angle2)*ts/ss;
//        angleInfo.motors.angle3    +=  (angleInfo.motore.angle3-angleInfo.motors.angle3)*ts/ss;
//        seamKeel.swap(temp);
//    }
    /// 跟踪日志目录
    logPath = "testLog/"+curCAD+"_" + seamInfo["SEAMNAME"] + "_" + getCurrentTime()  ;
    if ( !boost::filesystem::exists(logPath)) {
        boost::filesystem::create_directories(logPath) ;
    }
    return true;
}

/// 图像 时间 位置采集至dataset
int SeamHandler::takeTracePic()
{
    ofstream ofs( logPath + "/path0.log") ;//打开记录机器人坐标位置文件
    try
    {
        if(!_mb_ptr || !_camera_ptr)
        {
            throw ERR_CAMERA_TAKE_PICTURES_FAILED;
        }
        ofs<<"照片序号	机器人反馈的pos	拍照时间 相机状态"<<endl;
        int count = 0;
        std::queue<SensorDataPack>().swap(dataSet);/// 清空图像位置容器
        m_bTaking = true;  /// 进入拍照状态
        while(m_bTaking)
        {
            if(waitMode)  /// 起头有一个TCP运动到焊接起点的过程，这个过程无需拍照
            {
                std::this_thread::sleep_for(10_ms);
                continue;
            }
            Mat mat;/// 当前图像
            _camera_ptr->takePicture().copyTo(mat);
            auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
            auto time_pic = getMicTime();
            SensorDataPack sdp = {mat,cur,time_pic,cameraMode};
            {
                std::lock_guard<std::mutex> lck(_mtx);
                dataSet.push(sdp);
            }
            std::this_thread::sleep_for(100_ms);
            char str[256];
            ::sprintf(str,"%d\t%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\ttime\t%.3f\t"
                      ,count,cur.x,cur.y,cur.z,cur.a,cur.b,cur.c,time_pic);
            ofs <<str<<cameraMode<<endl;
            count++;
        }
        char msg[256];
        sprintf(msg,"takePictures end：%.3f",getMicTime());
        ofs<<msg<<endl;
        ofs.close();//关闭坐标位置文件
        return RESULT_OK;
    }catch(ReturnStatus error_code)
    {
        throw;
    }
    catch(...)
    {
        cout<<__FUNCTION__<<endl;
        char msg[256];
        sprintf(msg,"takePictures err end：%.3f",getMicTime());
        ofs<<msg<<endl;
        m_bTaking = false;
        return ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
    }
}

void SeamHandler::posCheck(SensorDataPack mpa,double & x)
{
    x = -1;
    uint ws = weldVec.size();
    if(ws < 1)
        return;
    RobotPos wE = weldVec[ws-1];
    Matrix4d mWe;
    pos2Matrix(wE,mWe,zyz);
    RobotPos r;
    UV2Point3D(mpa.r,ct[mpa.cameraMode],r,mpa.cameraMode);

    MOTORDEGRE magl = getAngleByTime(mpa.time, angleInfo);
    posInTool(r,tool,getPosition(magl));
    Vector4d vr = {r.x,r.y,r.z,1};
    vr = mWe.inverse()*vr;
    x = vr(0,0);
}

int Tracing::imgProcess()
{
    Config _sys_config(SYS);
    double endlen = _sys_config.get<double>("fit.endlen",10);
    double c2tx = _sys_config.get<double>("fit.c2tx",33);
    double fitps = _sys_config.get<double>("fit.fitps",3);
    ofstream imglog(logPath + "/path2.log") ;//打开记录机器人坐标位置文件
    imglog<<"照片序号	uv值	时间	UV23D计算出的pos	变位机角度变化（弧度）	已知原始点容器大小/待处理容器大小	原始点数据用于拟合的序号区间"<<endl;
    int count = 0;
    int exitCount = 0;
    int IMG_ERR_LIMIT = d_round(10*maxGap*6/wf.S_W); // 每秒10张
    int ss = seamKeel.size();
    static double preTime = getMicTime();
    bool saveFlag = false;
    UV pre_uv = {0,0};  /// 前一次算法的uv
    try
    {
        while(m_bTaking || !dataSet.empty())
        {
            double inTime = getMicTime();
            std::this_thread::sleep_for(1_ms);
            Mat m;
            if(dataSet.empty())
                continue;
            SensorDataPack mpa;
            {
                std::lock_guard<std::mutex> lck(_mtx);
                mpa = dataSet.front();
                mpa.m.copyTo(m);
                dataSet.pop();
            }
            cv::Point2d puv;
            cout<<"img algo in!"; //cv::flip(m,m,1);
            if(weldStatus)
            {
                _gxA->set_weld_on();
            }
            else
            {
                _gxA->set_weld_off();
            }
            switch(imageLevel)
            {
            case 0:
                break;
            case 1:
                imwrite(logPath+"/"+to_string(count)+".png",m);
                break;
            }
            if(getMicTime() > preTime+1)
            {
                preTime = getMicTime();
                if(imageLevel == 2)
                {
                    imwrite(logPath+"/"+to_string(count)+".png",m);
                    saveFlag = true;
                }
            }
            _gxA->AddImg(m);
            puv = _gxA->getuv(); //puv.x= m.cols-puv.x;
            cout<<"img algo ok!"<<endl;
            char stime[256];
            ::sprintf(stime,"inTime:%.3fuse%.3fs ",inTime,getMicTime()-inTime);
            imglog<<stime<<" "<<count;
            UV tar = {puv.x,puv.y};
            RobotPos r;
            int ws = weldVec.size();
            imglog<<" 当前点数"<<ws<<"龙骨点数"<<ss<<" ";
            if(abs(ss-ws) <= 1 && rInfos.endPath.size() > 0)/// 进入结束点关联逻辑，停止拍照
            {
                if(tracing_end_correlation(logPath+"/correlation_end.log",oriPath,
                                           rInfos.endPath,rInfos.endExtern,rInfos.endDistance,weldVec,p2p_len))
                {
                    m_bTaking = false;
                    break;
                }
            }
            double x;
            MOTORDEGRE magl = getAngleByTime(mpa.time, angleInfo);
            imglog<<mpa.cameraMode<<"angle:"<<magl.angle1<<"/"<<magl.angle2<<"/"<<magl.angle3<<" ";
            posCheck(mpa,x);
            if(x > 0)
            {
                imglog<<x<<"\tscaned pos!"<<endl;
            }
            bool uvOk = checkUV(tar,ct[1]);
            int ks = seamKeel.size();
            bool toEnd = ws >= ks-endKeelIndex;
            imglog<<toEnd<<"/"<<end_err_limit<<"/"<<IMG_ERR_LIMIT<<"/"<<exitCount<<"/";
            vector<double> lruv;
            _gxA->GetWhatYouWant(lruv);
            if(lineType == LT_WW)
            {
                if(lruv.size() < 4)
                {
                    uvOk = false;
                }
                else
                {
                    float py,pz,ly,lz,ry,rz;
                    float fu = (float)ct[1].u;
                    float fv = (float)ct[1].v;
                    laser_yz((float)(puv.x)-fu,(float)(puv.y)-fv,py,pz,1);
                    laser_yz((float)lruv[0]-fu,(float)lruv[1]-fv,ly,lz,1);
                    laser_yz((float)lruv[2]-fu,(float)lruv[3]-fv,ry,rz,1);
                    double dw = (ly-py)*(ly-py)+(lz-pz)*(lz-pz);
                    double dh = (ry-py)*(ry-py)+(rz-pz)*(rz-pz);
                    if(dw > 4*default_w*default_w
                            || dh > 4*default_h*default_h
                            || dw < 0.25*default_w*default_w
                            || dh < 0.25*default_h*default_h)
                    {
                        imglog<<"dw:"<<dw<<"/dh:"<<dh<<" "
                             <<puv.x << "," <<puv.y << "/" <<lruv[0] << "," <<lruv[1]
                              << "/" <<lruv[2] << "," <<lruv[3]<< endl;
                        uvOk = false;
                    }
                }
            }
            if(!uvOk) /// 跟踪过程中图像异常
            {
                exitCount++;
                imglog<<count<<"\terr"<<endl;
                int limit = toEnd?end_err_limit:IMG_ERR_LIMIT; /// 靠近结尾图像异常相对严格控制
                if(exitCount > limit) /// 连续IMG_ERR_LIMIT张图像异常，退出跟踪逻辑
                {
                    vector<RobotPos> cleanPath;
                    int os = oriPath.size();
                    for(int e=0;e<os-exitCount;e++)
                    {
                        cleanPath.push_back(oriPath[e]);
                    }
                    fitTracingErrEnd(imglog,cleanPath,seamKeel,weldVec,p2p_len, endlen);
                    throw -1;
                }
                if(abs(pre_uv.u) < 1 || abs(pre_uv.v)<1)
                {
                    count ++;
                    continue;
                }
                /// getPosFromKeel(r,oriPath,weldVec,seamKeel,p2p_len); 从龙骨序列中提取未来的pos
                tar = pre_uv;
            }
            /// 图像无异常
            UV2Point3D(mpa.r,tar,r,mpa.cameraMode);
            emit printUV(tar.u,tar.v);
            char str[256];
            ::sprintf(str,"%duv\t%.3f,%.3f\tt\t%.3f 23d ",count,tar.u,tar.v,mpa.time);
            imglog <<str<<r.toStr();
            RobotPos rTool = r;
            posInTool(rTool,tool,getPosition(magl));
            imglog<<" "<<oriPath.size()<<"/"<<dataSet.size()<<" ";
            if(appendWeldPath(logPath,rTool,imglog,oriPath,seamKeel,weldVec,p2p_len, rInfos,endlen,c2tx,fitps))
            {
                pre_uv = tar;
                if(uvOk)
                {
                    exitCount --;
                    exitCount = exitCount<0?0:exitCount;
                }
            }
            else
            {
                if(uvOk)
                {
                    exitCount ++;
                }
                uvOk = false;
                int ws = weldVec.size();
                int ks = seamKeel.size();
                int limit = ws >= ks-endKeelIndex?end_err_limit:IMG_ERR_LIMIT; /// 靠近结尾图像异常相对严格控制
                if(exitCount > limit) /// 连续IMG_ERR_LIMIT张图像异常，退出跟踪逻辑
                {
                    vector<RobotPos> cleanPath;
                    int os = oriPath.size();
                    for(int e=0;e<os-exitCount;e++)
                    {
                        cleanPath.push_back(oriPath[e]);
                    }
                    fitTracingErrEnd(imglog,cleanPath,seamKeel,weldVec,p2p_len, endlen);
                    throw -1;
                }
                /// getPosFromKeel(r,oriPath,weldVec,seamKeel,p2p_len); 从龙骨序列中提取未来的pos
                tar = pre_uv;
            }
            if(imageLevel == 2 && !uvOk && !saveFlag)
                imwrite(logPath+"/"+to_string(count)+".png",m);
            imglog<<endl;
            count++;
        }
        delete _gxA;
        char msg[64];
        sprintf(msg,"imgProcess end：%.3f",getMicTime());
        imglog<<msg<<endl;
    }
    catch(...)
    {
        char msg[64];
        sprintf(msg,"imgProcess err end：%.3f",getMicTime());
        imglog<<msg<<endl;
        m_bTaking = false;
        return ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
    }
    imglog.close();
    return RESULT_OK;
}

/// 只负责机器人运动控制,记录:
/// 拟合转换到base的pos及速度序列
/// 记录变位机坐标系下的原始pos序列
/// udp反馈的pos序列及时间
int SeamHandler::tracking()
{
    static uint weldPoints = 0;  /// 已焊接点数
    int ret = RESULT_OK;
    /// 处理原始点列,准备给机器人发送位置运动指令
    ofstream track_ofs( logPath + "/path5.log") ;//给机器人发送的点列信息
    track_ofs<<"跟踪时发送给机器人运动的pos序列	运动速度	变位机转动角度（弧度）	期望到达的时间"<<endl;
    try
    {
        Config _sys_config("etc/sys.info");
        double maxSpeed = _sys_config.get<double>("tracing.maxSpeed",20);
        cout<<"发送变位机转动信号，待转模式,angle:"<<angleInfo.motore.angle3<<endl;
        double scanspeed = _sys_config.get<double>("robot.tracingScan",10);
        _sys_config.put<double>("robot.speed",scanspeed);
        _sys_config.sync();
        _robot->absoluteMove(scaneEnd);  /// 运动到扫描终点
        waitMode = true; /// 等待，直到机器人运动到焊接起点位置了
        _mb_ptr->openAirValve();   /// 打开气阀，传感器进入跟踪状态
        int wvCheck = 3000;
        while(wvCheck--)
        {
            std::this_thread::sleep_for(1_ms);  /// 等待3秒
            if(weldVec.size()>=3)
                break;
        }
        if(weldVec.size() < 3)  /// 3秒内无焊接轨迹，起头失败，跟踪逻辑异常退出
        {
            track_ofs<<"起头3秒内无轨迹!"<<endl;
            throw -1;
        }
        _mb_ptr->rotateNew(angleInfo.motore.angle1,abs(angleInfo.angleSpeed1),
                           angleInfo.motore.angle2,abs(angleInfo.angleSpeed2),
                           angleInfo.motore.angle3,abs(angleInfo.angleSpeed3),2);  /// 发送变位机转动信号，待转模式
        //_mb_ptr->rotateNew(-35,25,-5,8,-8,8,2);  /// 发送变位机转动信号，待转模式
        auto p = weldVec[0];
        posInBase(p,tool,getPosition(angleInfo.motors));
        double curTime = 0;
        _sys_config.put("robot.speed",START_POINT_SPEED) ; /// 运动到焊接起点，固定速度
        _sys_config.sync();
        char str[256];
        _robot->absoluteMove(p);
        _robot->pathLog = true;
        waitMode = false; /// 到达起点，无需等待
        cameraMode = 1; /// 跟踪状态
        weldStatus = false;
        weldPoints = 0;
        double p2pTime = p2p_len*6/(wf.S_W);   /// 根据焊接速度，确定机器人跟踪点到点的运动时间
        int rotateIndex = 2;   /// 从准备发送第3个点开始转动
        int cp = 2; /// cp move point mis = 2
        uint udp12 = 0;
        while(1)
        {
            std::this_thread::sleep_for(1_ms);  /// 等待
            if(weldVec.size()<=weldPoints+1)
                break;
            if(weldPoints == rotateIndex)
            {
                _robot->setRotateTrue();
                angleInfo.rotateTime = getMicTime()+0.01;
                track_ofs<<"开始转动时间:"<<angleInfo.rotateTime<<endl;
            }
            auto p = weldVec[weldPoints];
            curTime = getMicTime();
            MOTORDEGRE dd = {0,0,0};
            if(angleInfo.rotateTime > 1)
            {
                dd.angle1 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed1;
                dd.angle2 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed2;
                dd.angle3 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed3;
                if(abs(dd.angle1)>abs(angleInfo.motore.angle1-angleInfo.motors.angle1))
                {
                    dd.angle1 = angleInfo.motore.angle1-angleInfo.motors.angle1;
                }
                if(abs(dd.angle2)>abs(angleInfo.motore.angle2-angleInfo.motors.angle2))
                {
                    dd.angle2 = angleInfo.motore.angle2-angleInfo.motors.angle2;
                }
                if(abs(dd.angle3)>abs(angleInfo.motore.angle3-angleInfo.motors.angle3))
                {
                    dd.angle3 = angleInfo.motore.angle3-angleInfo.motors.angle3;
                }
            }
            double targetTime;
            if(weldPoints >= rotateIndex)
                targetTime = angleInfo.rotateTime+p2pTime*(weldPoints-rotateIndex+cp);
            else
                targetTime = 0;
            MOTORDEGRE degree = angleInfo.motors;
            degree.angle1 += dd.angle1;
            degree.angle2 += dd.angle2;
            degree.angle3 += dd.angle3;
            posInBase(p,tool,getPosition(degree));
            auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
            double distence = cur.dis(p);
            if(targetTime < 1)
            {
                p.v = wf.S_W/6;
            }
            else
            {
                if(targetTime-curTime > 0.01)
                {
                    p.v = distence/(targetTime-curTime);
                }
                else
                {
                    p.v = maxSpeed;
                }
            }
            p.v = p.v<maxSpeed?p.v:maxSpeed;
            p.v = p.v>1?p.v:1;
            if(weldFlag)
            {
                p.v *= 6;   /// 焊接状态下，速度单位是cm/min 空走时单位 mm/sec
            }
            vector<RobotPos> vmi;
            vmi.push_back(p);
            auto w = weldVec[weldPoints];
            ::sprintf(str,"%.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.1f angle3 %.1f cur %.3f tar %.3f "
                      ,w.x,w.y,w.z,w.a,w.b,w.c,p.x,p.y,p.z,p.a,p.b,p.c,p.v,degree.angle3,curTime,targetTime);
            track_ofs <<str;
            if(Pos2Joint2(vmi))   /// 检查出目标位置机器人运动不到
            {
                ret = 1;
                track_ofs<<"机器人位置不可达"<<endl;
                throw -1;
            }
            auto next = weldVec[weldPoints+1];
            if(weldFlag)
            {
                if(udp12 == 0 && next.weld)
                {
                    if(!weldStatus)
                    {
                        _robot->setWeldingTrue(wf);
                        weldStatus = true;
                    }
                }
                if((weldStatus && next.weld == false) || (!weldStatus && next.weld == true)) /// 状态切换
                {
                    udp12 = 9999;
                }
                _robot->sendWeldPoss(udp12++,p,offset,base_offset,default_h,autoFit,wf,(distence > 0.1));
                if(udp12 == 10000)
                {
                    udp12 = 0;
                    if(weldStatus)
                    {
                        _robot->setWeldingFalse();
                        weldStatus = false;
                    }
                }
            }
            else
            {
                _robot->sendWeldPoss(udp12++,p,offset,base_offset,default_h,autoFit,wf,(distence > 0.1));
            }
            backPath.push_back(p);
            track_ofs<<_robot->weldIndex;
            curTime = getMicTime();
            double timewait = targetTime-curTime-p2pTime;
            track_ofs<<" "<<timewait<<endl;
            if(timewait>0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(d_round(timewait*1000)));
            }
            weldPoints++;
        }
        auto endp = weldVec[weldPoints];
        auto endpre = weldVec[weldPoints-1];
        double endp2p = endp.dis(endpre)/p2p_len;
        curTime = getMicTime();
        MOTORDEGRE dd = angleInfo.motors;
        dd.angle1 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed1;
        dd.angle2 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed2;
        dd.angle3 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed3;
        if(abs(dd.angle1)>abs(angleInfo.motore.angle1-angleInfo.motors.angle1))
        {
            dd.angle1 = angleInfo.motore.angle1-angleInfo.motors.angle1;
        }
        if(abs(dd.angle2)>abs(angleInfo.motore.angle2-angleInfo.motors.angle2))
        {
            dd.angle2 = angleInfo.motore.angle2-angleInfo.motors.angle2;
        }
        if(abs(dd.angle3)>abs(angleInfo.motore.angle3-angleInfo.motors.angle3))
        {
            dd.angle3 = angleInfo.motore.angle3-angleInfo.motors.angle3;
        }
        MOTORDEGRE degree = angleInfo.motors;
        degree.angle1 += dd.angle1;
        degree.angle2 += dd.angle2;
        degree.angle3 += dd.angle3;
        posInBase(endp,tool,getPosition(degree));
        auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
        double targetTime = angleInfo.rotateTime+p2pTime*(weldPoints-1-rotateIndex+cp)+p2pTime*endp2p;
        if(targetTime-curTime > 0.01)
        {
            endp.v = cur.dis(endp)/(targetTime-curTime);
        }
        else
        {
            endp.v = maxSpeed;
        }
        endp.v = endp.v<maxSpeed?endp.v:maxSpeed;
        endp.v = endp.v>1?endp.v:1;
        if(weldFlag)
        {
            endp.v *= 6;   /// 焊接状态下，速度单位是cm/min 空走时单位 mm/sec
        }
        auto ew = weldVec[weldPoints];
        vector<RobotPos> vmi;
        vmi.push_back(endp);
        ::sprintf(str,"%.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.1f angle3 %.1f cur %.3f tar %.3f end"
                  ,ew.x,ew.y,ew.z,ew.a,ew.b,ew.c,endp.x,endp.y,endp.z,endp.a,endp.b,endp.c,endp.v,degree.angle3,curTime,targetTime);
        track_ofs <<str;
        if(Pos2Joint2(vmi))
        {
            ret = 2;
            track_ofs<<"机器人位置不可达"<<endl;
            throw -1;
        }
        _robot->setRotateFalse();
        _robot->sendWeldPoss(9999,endp,offset,base_offset,default_h,autoFit,wf);
        track_ofs<<_robot->weldIndex<<endl;
        char msg[64];
        sprintf(msg,"tracking end：%.3f",getMicTime());
        track_ofs<<msg<<endl;
    }catch(...)
    {
        ret = ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
        char msg[64];
        sprintf(msg,"tracking err end：%.3f",getMicTime());
        track_ofs<<msg<<endl;
        /// 跟踪异常退出
    }
    if(weldStatus)
        _robot->setWeldingFalse();
    if(weldVec.size() < seamKeel.size()-10)
    {
        Config _sys_config(SYS);
        _sys_config.put<int>("weld.lastEnd",weldVec.size());
        _sys_config.sync();
    }
    m_bTaking = false;
    _robot->pathLog = false;
    track_ofs.close();
    return ret;
}

/// 开始跟踪逻辑
void SeamHandler::workTracing()
{
    Config _sys_config(SYS);
    imageLevel = _sys_config.get<double>("imageLevel",1);
    maxGap =  _sys_config.get<double>("fit.gap",50);
    endKeelIndex = _sys_config.get<double>("fit.endKeelIndex",10);
    double movespeed = _sys_config.get<double>("robot.moveSpeed",300);
    double scanspeed = _sys_config.get<double>("robot.tracingScan",10);
    double motor1steps = _sys_config.get<double>("motor.motor1steps",10);
    double motor2steps = _sys_config.get<double>("motor.motor2steps",11100);
    double motor3steps = _sys_config.get<double>("motor.motor3steps",12100);
    double departLen = stringToNum(seamInfo["DEPARTLEN"]);
    angleInfo.angleSpeed1 = angleInfo.motor1speed/motor1steps;
    angleInfo.angleSpeed2 = 360*angleInfo.motor2speed/motor2steps;
    angleInfo.angleSpeed3 = 360*angleInfo.motor3speed/motor3steps;
    string posName;
    RobotAxle midpos;
    getMidPointById(stoi(curSeam),posName,midpos);
    p2p_len = 0;
    for(uint i=0;i<seamKeel.size()-1;i++)
    {
        point3dd k0 = {seamKeel[i].x,seamKeel[i].y,seamKeel[i].z};
        point3dd k1 = {seamKeel[i+1].x,seamKeel[i+1].y,seamKeel[i].z};
        p2p_len += k1.dist(k0); ///  按约定，龙骨都是等间距的
    }
    p2p_len /= (seamKeel.size()-1);
    _robot->setRotateFalse();  /// 通过机器人，关闭变位机转动IO口
    _sys_config.put("robot.speed", movespeed) ;
    _sys_config.sync();
    _robot->absoluteMove(midpos);  /// 机器人运动到安全位置
    if(abs(gMotorposs-angleInfo.motors.angle1) > 0.1
     ||abs(gMotorangle1s-angleInfo.motors.angle2) > 0.1
     ||abs(gMotorangle2s-angleInfo.motors.angle3) > 0.1)
    {
        if(gMotorposs > loc)
        {
            _mb_ptr->rotateNew(loc,30,gMotorangle1s,10,gMotorangle2s,10,1);
        }
        if(gMotorangle1s < safeA1)
        {
            _mb_ptr->rotateNew(gMotorposs,30,safeA1,10,gMotorangle2s,10,1);
        }
        _mb_ptr->rotateNew(angleInfo.motors.angle1,30,angleInfo.motors.angle2,10,angleInfo.motors.angle3,10,1);
    }
    _sqlite_ptr->selectSQLOrder({"SCANEPOS"},{"INDEX1","POS","ENABLED"},{"SEAMID",curSeam},{"INDEX1","ASC"});
    if(_sqlite_ptr->m_iRows <= 0)
    {
        cout<<"焊缝信息缺失,请检查DB!"<<endl;
        _robot->absoluteMove(midpos);//回到安全位置
        return;
    }
    vector<string> pindex;
    vector<RobotPos> poslst;
    needPick = _sqlite_ptr->m_vResult[0]["ENABLED"] == "2"?true:false;
    for(auto& v:_sqlite_ptr->m_vResult)
    {
        RobotPos r = RobotPos::instance(v["POS"]);
        posInBase(r,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
        poslst.push_back(r);
        pindex.push_back(v["INDEX1"]);
    }
    _mb_ptr->closeAirValve();   /// 关闭气阀，传感器变成寻位模式
    bool errEnd = false;
    switch(trackMode)  /// 焊缝的tracing属性1表示跟踪，3直接走龙骨
    {
    case 1:
    {
        /// 跟踪逻辑
        _mb_ptr->openLaser();  /// 打开激光
        if(poslst.size() < 2)
        {
            cout<<"初始图像采集路径缺失!"<<endl;
            _robot->absoluteMove(midpos);//回到安全位置
            return;
        }
        RobotPos p0 = poslst[0];
        p0 = p0>CAMERA;  /// 转成相机中心点对准焊缝位置
        _robot->absoluteMove(p0,true,true); /// 扫描起点位置
        _sys_config.put("robot.speed", scanspeed) ;
        _sys_config.sync();
        RobotPos p1 = poslst[1];
        if(_gxA && needPick)
        {
            RobotPos mis;
            findRealStart(p0,mis);
            p0 = CAMERA>p0;
            p1 = p1<<mis;
            RobotPos p0_ = p0;
            posInTool(p0_,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
            _sqlite_ptr->updateSQL({"SCANEPOS"},{"POS","'"+p0_.toStr()+"'","ENABLED","1"},{"SEAMID",curSeam,"INDEX1",pindex[0]});
            RobotPos p1_ = p1;
            posInTool(p1_,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
            _sqlite_ptr->updateSQL({"SCANEPOS"},{"POS","'"+p1_.toStr()+"'","ENABLED","1"},{"SEAMID",curSeam,"INDEX1",pindex[1]});
            p0 = p0>CAMERA;
            _robot->absoluteMove(p0,true,true); /// 扫描起点位置
        }
        scaneEnd = p1>CAMERA;  /// 转成相机中心点对准焊缝位置
        std::future<int> f1 = std::async(std::launch::async,&SeamHandler::takeTracePic,this);           /// 拍照线程
        std::future<int> f12 = std::async(std::launch::async,&SeamHandler::imgProcess,this);            /// 图像处理线程
        std::future<int> f13 = std::async(std::launch::async,&SeamHandler::tracking,this); /// 机器人跟踪控制线程
        try
        {
            if(f1.get())
            {
                cout<<"图像采集异常结束!"<<endl;
            }
            if(f12.get())
            {
                cout<<"图像算法及轨迹拟合异常结束!"<<endl;
            }
            if(f13.get())
            {
                cout<<"机器人跟踪程序异常结束!"<<endl;
            }
            if(oriPath.size() > 0)
            {
                auto oriIter = oriPath.end()-1;
                while(oriIter>=oriPath.begin() && to_corra.size()<50)    /// 将原始点列最后50个点转到base坐标系下，放入to_corra容器，用于其他焊缝的拟合关联逻辑
                {
                    auto p = *oriIter;
                    posInBase(p,tool,getPosition(angleInfo.motore));
                    to_corra.push_back(p);
                    oriIter--;
                }
            }
        }
        catch(...)
        {
            errEnd = true;
        }
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        RobotPos curpos = _robot->getCurrPos(RobotTCP::UPLOAD);
        retractMove(curpos,departLen);
        curpos.v = movespeed;
        _robot->absoluteMove(curpos);
        _robot->absoluteMove(midpos);//回到安全位置
//        if(abs(gMotorposs-angleInfo.motors.angle1) > 0.1
//         ||abs(gMotorangle1s-angleInfo.motors.angle2) > 0.1
//         ||abs(gMotorangle2s-angleInfo.motors.angle3) > 0.1)
//        {
//            if(gMotorposs > loc)
//            {
//                _mb_ptr->rotateNew(loc,30,gMotorangle1s,10,gMotorangle2s,10,1);
//            }
//            if(gMotorangle1s < safeA1)
//            {
//                _mb_ptr->rotateNew(gMotorposs,30,safeA1,10,gMotorangle2s,10,1);
//            }
//            _mb_ptr->rotateNew(angleInfo.motors.angle1,30,angleInfo.motors.angle2,10,angleInfo.motors.angle3,10,1);
//        }
        ofstream origin_ofs( logPath + "/path3.log") ;//变位机坐标系内的原始pos及角度
        origin_ofs<<"变位机坐标系内的原始pos"<<endl;   /// 打印变位机坐标系下的原始位置信息
        for(auto & p:oriPath)
        {
            origin_ofs<<p.toStr()<<endl;
        }
        origin_ofs.close();
        _mb_ptr->closeLaser();
    }
        break;
    case 3:/// 直接走龙骨
    {
        _mb_ptr->openLaser();  /// 打开激光
        auto s0 = seamKeel[0];
        posInBase(s0,tool,getPosition(angleInfo.motors));
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        scaneEnd = s0;
        _robot->absoluteMove(s0>CAMERA);
        weldVec = seamKeel;
        std::future<int> f13 = std::async(std::launch::async,&SeamHandler::tracking,this); /// 机器人跟踪控制线程
        try
        {
            int result3 = f13.get();
            if(result3)
            {
                cout<<"机器人跟踪程序异常结束!"<<endl;
            }
        }
        catch(...)
        {
            errEnd = true;
        }
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        _mb_ptr->closeLaser();
        RobotPos curpos = _robot->getCurrPos(RobotTCP::UPLOAD);
        retractMove(curpos,departLen);
        _robot->absoluteMove(curpos);//back 200
        _robot->absoluteMove(midpos);//回到安全位置
    }
        break;
    }
}

bool SeamHandler::findRealStart(RobotPos & p0,RobotPos & mis)
{
    RobotPos tar = p0;
    ofstream ofs("testLog/mo/findRealStart.txt");
    ofs<<getMicTime()<<endl;
    ofs<<"in pos :"<<p0.x<<","<<p0.y<<","<<p0.z<<endl;
    _robot->absoluteMove(tar);
    for(int t=0;t<=2;t++)
    {
        vector<RobotPos> p3d;
        for(int i=0;i<=4;i++)
        {
            for(int j=0;j<=4;j++)
            {
                RobotPos offset = RobotPos::instance();
                offset.x = -t*5;
                if(i%2)
                    offset.y = 16-j*8;
                else
                    offset.y = j*8-16;
                offset.z = 16-i*8;
                RobotPos ti = tar<<offset;
                _robot->absoluteMove(ti);
                Mat mat;/// 当前图像
                _camera_ptr->takePicture().copyTo(mat);
                _gxA->AddImg(mat);
                imwrite("testLog/mo/"+to_string(i)+"_"+to_string(j)+".png",mat);
                cv::Point2d p2d = _gxA->getuv();
                UV tz = {p2d.x,p2d.y};
                ofs<<ti.toStr()<<" "<<tz.u<<","<<tz.v;
                if(checkUV(tz,ct[0]))
                {
                    RobotPos pi;
                    UV2Point3D(ti,tz,pi,0);
                    p3d.push_back(pi);
                    ofs<<" "<<pi.toStr();
                }
                ofs<<endl;
            }
        }
        RobotPos realPos = p0;
        if(getCenter(p3d,realPos))
        {
            tar = realPos>CAMERA;
            ofs<<"real pos :"<<tar.x<<","<<tar.y<<","<<tar.z<<endl;
            mis = (!p0)<<tar;
            p0 = tar;
            break;
        }
    }
    ofs.close();
    return true;
}
