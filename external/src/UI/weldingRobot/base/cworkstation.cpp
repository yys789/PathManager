#include "cworkstation.h"
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

#include "fitCorrelation.h"
#include "base.h"
#include "base/data_types.h"
#include "base/errorcode.h"
#include "tracing.h"
#include "common/dbinterface.h"

using namespace std;
using namespace cv;
using namespace walgo;


#define _LOGINFO_ cout<<"["<<std::this_thread::get_id()<<"]"<<"["<<getCurrentTime().c_str()<<"]["<<__FILE__<<"]["<<__LINE__<<"]["<<__FUNCTION__<<"]"<<endl;

extern void Insertion_point(RobotPos p0, RobotPos p1, RobotPos p2, RobotPos p3, double len, vector<RobotPos> & pOut);
extern Tracing * traceLogic;

double gMotorposs;
double gMotorangle1s;
double gMotorangle2s;

CWorkStation::CWorkStation():
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

CWorkStation::~CWorkStation()
{
    /// save seamInfo to db
    sync();
}

void CWorkStation::loading_cad()
{

}

void CWorkStation::unloading_cad()
{

}

void CWorkStation::syncSeamInfo()
{

}

void CWorkStation::transferDBData()
{
    try
    {
        _LOGINFO_;
        //        int cadId = 3;
        int cadId = _mb_ptr->get_cadId();

        _sqlite_ptr->selectSQL1({"CAD"},{"CADCODE",to_string(cadId),"ENABLED","1"});
        if(_sqlite_ptr->m_iRows < 1) throw ERR_DB_NO_DATA_FOUND;
        string cadname = _sqlite_ptr->m_vResult[0]["CADID"];
        cadname = "'"+cadname+"'";
        vector<uint16_t> DB;
        _sqlite_ptr->selectSQLAllOrder({"SEAMINFO"},{"CADID",cadname,"MAIN","1","ENABLED","1"},{"ORDERID","ASC"});
        if(_sqlite_ptr->m_iRows < 1) throw ERR_DB_NO_DATA_FOUND;
        auto vec = _sqlite_ptr->m_vResult;
        auto seamCnt = vec.size();
        cout<<"******** total seams count : "<<seamCnt<<" *********"<<endl;
        for(auto& v:vec)
        {
            DB.push_back(stoi(v["SEAMID"]));

            string sn = v["SEAMNAME"];
            int seam = SeamNameToInt(sn);
            DB.push_back(seam);

            DB.push_back(stoi(v["ORDERID"]));

            int seamtype = 0;
            charToAsc(seamtype,v["SEAMTYPE"]);
            DB.push_back(seamtype);

            DB.push_back(stoi(v["RANK"]));
            DB.push_back(stoi(v["TRACING"]));

            double angle = 0;
            angle = stringToNum(v["MOTORPOSS"]);
            angle = angle*10;
            DB.push_back(angle);

            angle = 0;
            angle = stringToNum(v["MOTORANGLE1S"]);
            angle = angle*10;
            DB.push_back(angle);

            angle = 0;
            angle = stringToNum(v["MOTORANGLE2S"]);
            angle = angle*10;
            DB.push_back(angle);

            angle = 0;
            angle = stringToNum(v["MOTORPOSE"]);
            angle = angle*10;
            DB.push_back(angle);

            angle = 0;
            angle = stringToNum(v["MOTORANGLE1E"]);
            angle = angle*10;
            DB.push_back(angle);

            angle = 0;
            angle = stringToNum(v["MOTORANGLE2E"]);
            angle = angle*10;
            DB.push_back(angle);


            _sqlite_ptr->selectSQL1({"SEAMRELATE"},{"SEAMID",v["SEAMID"],"ENABLED","1"});
            if(_sqlite_ptr->m_iRows > 0)
            {
                auto seamrelate = _sqlite_ptr->m_vResult;
                int seam1 = 0;
                int seam2 = 0;
                if(seamrelate[0]["RELATE1"] != "0")
                {
                    _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMNAME"},{"SEAMID",seamrelate[0]["RELATE1"]});
                    if(_sqlite_ptr->m_iRows == 0)
                        throw ERR_DB_NO_DATA_FOUND;
                    string sn = _sqlite_ptr->m_vResult[0]["SEAMNAME"];
                    seam1 = SeamNameToInt(sn);
                }

                int snum2 = 0;
                if(seamrelate[0]["RELATE2"] != "0")
                {
                    _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMNAME"},{"SEAMID",seamrelate[0]["RELATE2"]});
                    if(_sqlite_ptr->m_iRows == 0)
                        throw ERR_DB_NO_DATA_FOUND;
                    string sn = _sqlite_ptr->m_vResult[0]["SEAMNAME"];
                    seam2 = SeamNameToInt(sn);
                }
                DB.push_back(seam1);
                DB.push_back(stoi(seamrelate[0]["SPACING1"])*10);
                DB.push_back(stoi(seamrelate[0]["DISTANCE1"])*10);

                DB.push_back(seam2);
                DB.push_back(stoi(seamrelate[0]["SPACING2"])*10);
                DB.push_back(stoi(seamrelate[0]["DISTANCE2"])*10);
                DB.push_back(stoi(seamrelate[0]["SEFLAG"]));
            }else{
                for(int i = 0; i<9; i++)
                    DB.push_back(0);
            }

            //焊缝焊接参数表
            _sqlite_ptr->selectSQL1({"SEAMWELDINFO"},{"SEAMID",v["SEAMID"]});
            if(_sqlite_ptr->m_iRows != 0)
            {
                auto seamweldInfo = _sqlite_ptr->m_vResult[0];
                RobotPos offset = RobotPos::instance(seamweldInfo["OFFSET"]);
                DB.push_back(offset.x*10);//offset
                DB.push_back(offset.y*10);
                DB.push_back(offset.z*10);
                DB.push_back(offset.a*10);
                DB.push_back(offset.b*10);
                DB.push_back(offset.c*10);
            }else{
                for(int i = 0; i < 6;i++)
                    DB.push_back(0);
            }

            //焊接参数表
            _sqlite_ptr->selectSQL1({"WELDPARAMS"},{"SEAMID",v["SEAMID"]});
            if(_sqlite_ptr->m_iRows != 0)
            {
                auto weldparams = _sqlite_ptr->m_vResult[0];
                DB.push_back(stoi(weldparams["S_W"])*10);
                DB.push_back(stoi(weldparams["I_W"])*10);
                DB.push_back(stoi(weldparams["V_W"])*10);
                DB.push_back(stoi(weldparams["T_END"])*10);
                DB.push_back(stoi(weldparams["I_END"])*10);
                DB.push_back(stoi(weldparams["V_END"])*10);
                DB.push_back(stoi(weldparams["RANGE"])*10);
                DB.push_back(stoi(weldparams["FREQUENCY"])*10);
                DB.push_back(stoi(weldparams["SERIALNUMBER"]));
                DB.push_back(stoi(weldparams["AXIS_X"])*10);
                DB.push_back(stoi(weldparams["AXIS_Y"])*10);
            }else{
                for(int i = 0; i < 11;i++)
                    DB.push_back(0);
            }
        }

        for(int i = seamCnt+1; i<= 50;i++)
        {
            for(int i= 0; i<36 ;i++)
                DB.push_back(0);
        }
        int msglen = DB.size();
        uint16_t iSendData[msglen] = {0};
        memset(iSendData,0,sizeof(iSendData));
        for(int i = 0; i< msglen; i++)
            iSendData[i] = DB[i];
        _mb_ptr->write_Seam(iSendData,msglen);
    }
    catch(ReturnStatus error_code )
    {
        cout<<getErrString(error_code);
        throw;
    }
    catch(...)
    {
        cout << getErrString(ERR_FUNC_DISPLAY_SEAMINFO_FAILED) <<endl;
        throw ERR_FUNC_DISPLAY_SEAMINFO_FAILED;
    }
}

void CWorkStation::back_home()
{
    try
    {
        _robot->absoluteMove(AMidPointAlex);//絕對安全點
        updateComInfo("'currentMidPoint'","'midpoint_a'");
        if(currentPosSafe(AMidPointAlex))//比较安全点和中间点位置差距
            _mb_ptr->robotbackDone();
        else
            throw ERR_ROBOT_POS_NOT_SAFE;
    }catch(ReturnStatus error_code ){
        qCritical()<<getErrString(error_code);
        throw;
    }catch(...){
        cout<<"error: robot back to safe position failed." <<endl;
        throw ERR_ROBOT_POS_NOT_SAFE;
    }
}

bool CWorkStation::beenSensorChecked()
{
    try
    {
        _LOGINFO_;
        bool isDone = false;
        return isDone;
    }
    catch(...)
    {
        qCritical() << getErrString(ERR_CHECK_BOOL_SENSOR_FAILED) <<endl;
        throw ERR_CHECK_BOOL_SENSOR_FAILED;
    }
}

void CWorkStation::send_cad_list()
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
        for(int i = vec_cadcode.size()+1;i<=15;i++)
        {
            for(int j = 0; j<21; j++){
                tempDate.push_back(0);
            }
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

double CWorkStation::getFeature(Mat m)
{
}

void CWorkStation::adjustPlane()
{
}


void CWorkStation::reset()
{
}

void CWorkStation::syncMotorAngle()
{
    try{
        _LOGINFO_;
//        //        _mb_ptr->writeOver();
//        vector<int> data(100,0);
//        //_mb_ptr->getData(data);

//        gMotorposs = data[95]/10.0;//地轨
//        gMotorangle2s= data[96]/10.0;//旋转
//        gMotorangle1s = data[97]/10.0;//翻转
//        cout<<"current currmotorpos : "<<gMotorposs<<endl;
//        cout<<"current currmotorangle1 : "<<gMotorangle1s<<endl;
//        cout<<"current currmotorangle2 : "<<gMotorangle2s<<endl;

//        updateComInfo("'CURMOTORPOS'",to_string(gMotorposs));
//        updateComInfo("'CURMOTORANGLE2'",to_string(gMotorangle2s));
//        updateComInfo("'CURMOTORANGLE1'",to_string(gMotorangle1s));

    }catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }catch(...)
    {
        throw ERR_MOTOR_SYNC_FAILED;
    }
}

void CWorkStation::NewSyncMotorAngle()
{
    try{
        _LOGINFO_;
        auto data = _mb_ptr->obj.msgBody;
        gMotorposs = data[5]/10.0;//地轨
        gMotorangle2s= data[6]/10.0;//旋转
        gMotorangle1s = data[7]/10.0;//翻转
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

bool CWorkStation::currentPosSafe(RobotAxle midPos)
{
    _LOGINFO_;
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

void CWorkStation::sensorCheck()
{
    try
    {
        _LOGINFO_;
        if(!_mb_ptr || !_camera_ptr)
            throw ERR_DEVICE_DISCONNECTED;

        Config _sys_config(SYS);
        map<string,string> mComInfo;
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

void CWorkStation::checkSensor()
{
    try
    {
        int result = 1;
        _mb_ptr->feedbackResult(result);
    }
    catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        qCritical()<<getErrString(ERR_CHECK_SENSOR_FAILED)<<endl;
        throw ERR_CHECK_SENSOR_FAILED;
    }
}


void CWorkStation::checkBrightness(int code)
{
    try
    {
        _LOGINFO_;
        m_bCheckSuccess = true;
        m_bRobotException = false;
        if(!_mb_ptr || !_camera_ptr )
            throw ERR_DEVICE_DISCONNECTED;
        map<string,string> mComInfo;
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

void CWorkStation::checkTcp()
{
    try
    {
        _LOGINFO_;
        if(!_mb_ptr)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        //        RobotAxle pos0;
        //        _sqlite_ptr->selectSQL2({"COMPARAMS"},{"COMVALUE"},{"COMNAME","'checkTcp.pos0'"});
        //        if(_sqlite_ptr->m_iRows == 0)
        //            throw ERR_INVALID_DATA;

        //        getComInfo();
        //        auto value = mComInfo["checkTcp.pos0"];
        //        RobotAxle checkPos ;
        //        sPosTodPos(value,checkPos);
        //        _robot->absoluteMove(checkPos);
        _mb_ptr->checkTcp();
        //        _robot->absoluteMove(AMidPointAlex);
        cout<<"[checkTcp][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        qCritical()<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        qCritical()<<getErrString(ERR_CHECK_TCP_FAILED)<<endl;
        throw ERR_CHECK_TCP_FAILED;
    }
}

bool CWorkStation::findSeamPath(int seamId)
{
    _LOGINFO_;
    bool result = false;
    auto iter = wData.AllSeamPathMap.find(seamId);
    if(iter != wData.AllSeamPathMap.end())
        result = true;
    cout<<"[findSeamPath][OVER]"<<endl;
    return result;
}

void CWorkStation::moveToMidPoint(int seamId)
{
    try
    {
        _LOGINFO_;
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

void CWorkStation::autoScanWelding()
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
        _sys_config.put("robot.speed", movespeed);
        _sys_config.sync();
        int seamId = _mb_ptr->get_seamId();
        //        int seamId = 580;

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
            sHandler.CompletionRelateInfo(seamId);
            if(stoi(sInfo["TRACING"]) == 1 || stoi(sInfo["TRACING"]) == 3)//是否是跟蹤？1是 ， 0 否
            {
                if(traceLogic->initParams(to_string(wData.sMark.seamID),relateMap[wData.sMark.seamID],m_iAction==1))
                    traceLogic->workTracing();
                wData.AllSeamPathMap[wData.sMark.seamID].swap(traceLogic->to_corra);
            }
            else/**/
            {
                sHandler.getSeamPath(seamId);//扫描主焊缝
                sHandler.optimizePath();//关联得到新的路径数据
                sHandler.startWelding();//焊缝焊接
            }

            cleanTorch();//判断是否需要清枪
            m_bWeldSuccess = true;
            wData.release_map_pair();
        }
        catch(ReturnStatus error_code)
        {
            vector<RobotPos>().swap(tempPath);
            m_bWeldSuccess = false;
            updateProgress();//更新焊接进度
            wData.release_map_pair();
            if(error_code != ERR_FUNC_DEAL_IMAGES_FAILED &&  error_code != ERR_FUNC_CORRELATION_FAILED )
                throw;
        }
        catch(...)
        {
            vector<RobotPos>().swap(tempPath);
            m_bWeldSuccess = false;
            updateProgress();//更新焊接进度
            wData.release_map_pair();
            throw ERR_FUNC_DEAL_SEAM_FAILED;
        }
    }
    catch(ReturnStatus error_code)
    {
        _mb_ptr->feedbackErrCode((int)error_code);
    }catch(...)
    {
        _mb_ptr->feedbackErrCode(32);
    }
    reset();//复位
    isScanWelding.exchange(false);
    updateProgress();//更新焊接进度
    cout<<"[autoScanWelding][OVER]"<<endl;
}

void CWorkStation::updateProgress()
{
    try
    {
        _LOGINFO_;
        _mb_ptr->updateProgress(m_sCurrMark.seamID,m_bWeldSuccess);
    }
    catch(ReturnStatus error_code)
    {
        throw;
    }catch(...)
    {
        throw ERR_FUNC_GET_SEAMINFO_FAILED;
    }
}

void CWorkStation::setActionType(int act)
{
    _LOGINFO_;
    m_iAction = act;
    cout<<"[setActionType][OVER]"<<endl;
}

void CWorkStation::cleanTorch()
{
    _LOGINFO_;

}
