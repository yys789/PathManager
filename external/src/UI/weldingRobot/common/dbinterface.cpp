#include "common/dbinterface.h"
#include <iostream>
#include "device/mysqliteapi.h"
#include "tools/singleton.h"
#include "base/errorcode.h"
#include "tools.h"
#include <thread>

using namespace std;

#define _LOGINFO_ cout<<"["<<std::this_thread::get_id()<<"]"<<"["<<getCurrentTime().c_str()<<"]["<<__FILE__<<"]["<<__LINE__<<"]["<<__FUNCTION__<<"]"<<endl;


void getSeamInfo(int seamID,map<string,string>& mSeamInfo)
{
    try
    {
        _LOGINFO_;
        string seamId = to_string(seamID);
        mSeamInfo.clear();
        ///焊缝信息表
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"SEAMINFO"},{"SEAMID",seamId,"ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        auto v = _sqlite_ptr->m_vResult[0];
        auto it = v.begin();
        while(it != v.end()){
            mSeamInfo.insert(make_pair(it->first,it->second));
            it++;
        }
        cout<<"[getSeamInfo][OVER]"<<endl;
    }
    catch(ReturnStatus error_code){
        mSeamInfo.clear();
        throw;
    }catch(...){
        mSeamInfo.clear();
        throw ERR_FUNC_GET_SEAMINFO_FAILED;
    }
}

/// 定制需求，一条边扩展出其他3条
void getExternInfo(int seamID, map<string,string>& mExternInfo)
{
    try
    {
        _LOGINFO_;
        string seamId = to_string(seamID);
        map<string,string>().swap(mExternInfo);
        ///焊缝信息表
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"AUTOEXTERN"},{"SEAMID",seamId,"ENABLED","1"});
        if(_sqlite_ptr->m_iRows == 0)
            return;
        auto v = _sqlite_ptr->m_vResult[0];
        auto it = v.begin();
        while(it != v.end()){
            mExternInfo.insert(make_pair(it->first,it->second));
            it++;
        }
        cout<<"[getExternInfo][OVER]"<<endl;
    }
    catch(ReturnStatus error_code){
        mExternInfo.clear();
        throw;
    }catch(...){
        mExternInfo.clear();
        throw ERR_FUNC_GET_SEAMINFO_FAILED;
    }
}


void getComInfo(map<string,string>& mComInfo)
{
    try
    {
        _LOGINFO_;
        map<string,string>().swap(mComInfo);
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL("COMPARAMS");
        auto vec = _sqlite_ptr->m_vResult;
        for(auto& v : vec){
            string key,value;
            auto it = v.begin();
            while(it != v.end()){
                if(it->first == "COMNAME")
                    key = it->second;
                if(it->first == "COMVALUE")
                    value = it->second;
                it++;
            }
            mComInfo.insert(make_pair(key,value));
        }
        cout<<"[getComInfo][OVER]"<<endl;
    }
    catch(ReturnStatus error_code){
        mComInfo.clear();
        throw;
    }catch(...){
        mComInfo.clear();
        throw ERR_FUNC_GET_COMINFO_FAILED;
    }
}

void getScanPos(int seamId,vector<RobotPos>& scanPath)
{
    try
    {
        _LOGINFO_;
        scanPath.clear();
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQLOrder({"SCANEPOS"},{"INDEX1","POS"},{"SEAMID",to_string(seamId),"ENABLED","1"},{"INDEX1","ASC"});
        auto vec = _sqlite_ptr->m_vResult;
        for(auto& v: vec){
            scanPath.push_back(RobotPos::instance(v["POS"]));
        }

        cout<<"[getScanPos][OVER]"<<endl;
    }
    catch(ReturnStatus error_code){
        scanPath.clear();
        throw;
    }catch(...){
        scanPath.clear();
        throw ERR_FUNC_GET_COMINFO_FAILED;
    }
}

void getFramePos(int seamId,vector<RobotPos>& framePos)
{
    try
    {
        _LOGINFO_;
        framePos.clear();
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQLOrder({"FRAMEPOS"},{"INDEX1","POS","WELD","HEIGHT"},{"SEAMID",to_string(seamId),"ENABLED","1"},{"INDEX1","ASC"});
        auto vec = _sqlite_ptr->m_vResult;

        for(auto& v: vec){
            RobotPos r = RobotPos::instance(v["POS"]);
            r.weld = v["WELD"]=="1"?true:false;
            r.dh = stringToNum(v["HEIGHT"]);
            framePos.push_back(r);
        }
        cout<<"[getFramePos][OVER]"<<endl;
    }
    catch(ReturnStatus error_code){
        framePos.clear();
        throw;
    }catch(...){
        framePos.clear();
        throw ERR_FUNC_GET_COMINFO_FAILED;
    }
}


void getRelatedSeamsInfo(int seamId, RelateInfo & rif)
{
    try
    {
        _LOGINFO_;
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"SEAMRELATE"},{"SEAMID",to_string(seamId),"ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        auto mResult = _sqlite_ptr->m_vResult[0];
        rif.beginId = stoi(mResult["RELATE1"]);
        rif.beginExtern = stringToNum(mResult["SPACING1"]);
        rif.beginDistance = stringToNum(mResult["DISTANCE1"]);
        int id2 = stoi(mResult["RELATE2"]);
        if(id2 > 0)
        {
            rif.endId = stoi(mResult["RELATE2"]);
            rif.endExtern = stringToNum(mResult["SPACING2"]);
            rif.endDistance = stringToNum(mResult["DISTANCE2"]);
        }
        else
        {
            rif.endId = 0;
            rif.endExtern = 0;
            rif.endDistance = 0;
            rif.L = stringToNum(mResult["SPACING2"]);
        }
        cout<<"[getRelatedSeamsInfo][OVER]"<<endl;
    }catch(...){
        throw ERR_FUNC_GET_RELATED_SEAMS_FAILED;
    }
}

void getMidPointById(int seamId,string& midpoint,RobotAxle& axle)
{
    try
    {
        _LOGINFO_;
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        //查询焊缝对应的中间点的posid
        _sqlite_ptr->selectSQL1({"SEAMINFO"},{"SEAMID",to_string(seamId),"ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;

        auto posId = _sqlite_ptr->m_vResult[0]["MIDPOINT"];
        //根据posid查询对应的posname和pos
        _sqlite_ptr->selectSQL2({"SAFEPOS"},{"POSNAME","POS"},{"POSID",posId,"ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;

        midpoint = _sqlite_ptr->m_vResult[0]["POSNAME"];

        string sPos = _sqlite_ptr->m_vResult[0]["POS"];
        sPosTodPos(sPos,axle);
    }
    catch(ReturnStatus error_code){
        throw;
    }catch(...){
        throw ERR_FUNC_GET_SEAMINFO_FAILED;
    }
}


void updateSeamStatus(int seamId,int goodSid,SEAMSTATUS st)
{
    _LOGINFO_;
    shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
    _sqlite_ptr->selectSQL1({"SEAMINFO"},{"SEAMID",to_string(seamId),"ENABLED","1"});
    if(_sqlite_ptr->m_iRows != 1)
        throw ERR_DB_NO_DATA_FOUND;
    auto orderId = _sqlite_ptr->m_vResult[0]["ORDERID"];

    string status = to_string((int)(st));

    _sqlite_ptr->updateSQL({"SEAMSTATUS"},{"STATUS",status},{\
                               "ORDERID",orderId,\
                               "GOODSID",to_string(goodSid)});
}


void updateComInfo(string columnName,string value)
{
    _LOGINFO_;
    shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
    _sqlite_ptr->updateSQL({"COMPARAMS"},{"COMVALUE",value},{\
                               "COMNAME",columnName});
}

void getSeamWeldInfo(int seamId,vector<WELDINFO>& weldInfo)
{
    try
    {
        _LOGINFO_;
        weldInfo.clear();
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        ///焊缝焊接信息表
        _sqlite_ptr->selectSQLOrder({"SEAMWELDINFO"},{"OFFSET","BASEOFFSET"},{"SEAMID",to_string(seamId),"ENABLED","1"},{"WELDORDER","ASC"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        auto v1 = _sqlite_ptr->m_vResult;

        ///焊接参数表
        _sqlite_ptr->selectSQL1({"WELDPARAMS"},{"SEAMID",to_string(seamId)});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        auto v2 = _sqlite_ptr->m_vResult;

        if(v1.size() != v2.size())
            throw ERR_DB_DATA_ERROR;

        auto count = v1.size();
        for(int i = 0; i< count; i++ ){
            WELDINFO w;
            w.offset=RobotPos::instance(v1[i]["OFFSET"]);
            w.base_offset=RobotPos::instance(v1[i]["BASEOFFSET"]);
            w.WI.S_W = stoi(v2[i]["S_W"]);
            w.WI.I_W = stoi(v2[i]["I_W"]);
            w.WI.V_W = stoi(v2[i]["V_W"]);
            w.WI.T_END = stoi(v2[i]["T_END"]);
            w.WI.I_END = stoi(v2[i]["I_END"]);
            w.WI.V_END = stoi(v2[i]["V_END"]);
            w.WI.range = stoi(v2[i]["RANGE"]);
            w.WI.frequency = stoi(v2[i]["FREQUENCY"]);
            w.WI.serialNumber = stoi(v2[i]["SERIALNUMBER"]);
            w.WI.axis_x = stoi(v2[i]["AXIS_X"]);
            w.WI.axis_y = stoi(v2[i]["AXIS_Y"]);
            weldInfo.push_back(w);
        }
    }
    catch(ReturnStatus error_code){
        throw;
    }catch(...){
        throw ERR_DB_OPEN_FAILED;
    }
}

void getSeamMotorAngleInfo(int seamId,WELDANGLEINFO& angleInfo)
{
    try
    {
        _LOGINFO_;
        ///焊缝信息表
        Config _sys_config("sys.info");
        double motor2steps = _sys_config.get<double>("motor.motor2steps",10800);
        double motor3steps = _sys_config.get<double>("motor.motor3steps",12100);

        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"SEAMINFO"},{"SEAMID",to_string(seamId),"ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        auto vec = _sqlite_ptr->m_vResult[0];
        //变位机角度\速度
        angleInfo.motors.angle1    =  stringToNum(vec["MOTORPOSS"]);
        angleInfo.motors.angle2 =  stringToNum(vec["MOTORANGLE1S"]);
        angleInfo.motors.angle3 =  stringToNum(vec["MOTORANGLE2S"]);
        angleInfo.motore.angle1 =  stringToNum(vec["MOTORPOSE"]);
        angleInfo.motore.angle2 = stringToNum(vec["MOTORANGLE1E"]);
        angleInfo.motore.angle3 =  stringToNum(vec["MOTORANGLE2E"]);
        angleInfo.motor1speed =  stringToNum(vec["MOTOR1SPEED"]);
        angleInfo.motor2speed =  stringToNum(vec["MOTOR2SPEED"]);
        angleInfo.motor3speed =  stringToNum(vec["MOTOR3SPEED"]);
        angleInfo.angleSpeed1 = angleInfo.motor1speed/10;
        angleInfo.angleSpeed2 = 360*angleInfo.motor2speed/motor2steps;
        angleInfo.angleSpeed3 = 360*angleInfo.motor3speed/motor3steps;
    }
    catch(ReturnStatus error_code){
        throw;
    }catch(...){
        throw ERR_DB_OPEN_FAILED;
    }
}


void getCadInfo(std::string cadId,std::map<std::string,std::string>& cadInfo,RobotPos& tool)
{
    try{
        _LOGINFO_;
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"CAD"},{"CADID","'"+cadId+"'","ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;

        cadInfo.clear();
        auto vec = _sqlite_ptr->m_vResult[0];

        cadInfo.insert(make_pair("MIDPOINT",vec["MIDPOINT"]));
        cadInfo.insert(make_pair("SAFELOC",vec["SAFELOC"]));
        cadInfo.insert(make_pair("SAFEANGLE1",vec["SAFEANGLE1"]));
        cadInfo.insert(make_pair("BASEPOINT",vec["BASEPOINT"]));
        tool = RobotPos::instance(vec["BASEPOINT"]);
    }catch(ReturnStatus error_code){
        throw;
    }catch(...){
        throw ERR_DB_OPEN_FAILED;
    }
}


void getCadInfo(std::string cadId,RobotPos& pos)
{
    try{
        _LOGINFO_;
        shared_ptr<mySqliteApi> _sqlite_ptr = Singleton<mySqliteApi>::get();
        _sqlite_ptr->selectSQL1({"CAD"},{"CADID","'"+cadId+"'","ENABLED","1"});
        if(_sqlite_ptr->m_iRows != 1)
            throw ERR_DB_NO_DATA_FOUND;
        pos = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
    }catch(ReturnStatus error_code){
        throw;
    }catch(...){
        throw ERR_DB_OPEN_FAILED;
    }
}

