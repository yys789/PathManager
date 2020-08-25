#include "mbControl.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <QString>

#include "base.h"
#include "base/config.h"
#include <thread>
#include "base/errorcode.h"
#include <QDebug>
#include "tools/singleton.h"

using namespace std;


mbControl::mbControl():
    _laser_ptr(Singleton<RobotLaser>::get()),
    _mb(nullptr)
{
    try{
        init();
        connectDevice();
    }catch(...){};
}

mbControl::~mbControl()
{
    disconnectDevice();
}

void mbControl::init()
{
    address = 0;
    iSignal = 0;
    checkCode = 0;
    oldCheckCode = 0;
    errCnt = 0;
    bOpenOver = false;
    bGetDone = false;
    bRotateOver = false;
    memset(iRecvData,0,sizeof(iRecvData));
    memset(iSendData,0,sizeof(iSendData));
    while(!msgQueue.empty())
        msgQueue.pop();
}

void mbControl::connectDevice()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try {
        _LOGINFO_;
        if(!_mb){
            Config _sys_config("etc/sys.info");
            _mb = modbus_new_rtu(_sys_config.get<string>("motor.serialPort","/dev/ttyS1").c_str(),115200,'N',8,1);
            if(_mb == NULL){
                cout<<"Unable to create the libmodbus context , errInfo :"<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_CREATE_CONTEXT_FAILED;
            }
            int rc = modbus_set_slave(_mb, 1);
            if (rc == -1){
                cout<<" Invalid salve ID , errInfo :"<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_INVALID_SLAVEID;
            }

            struct timeval t;
            t.tv_sec= 10;
            t.tv_usec=0;
            //rc = modbus_set_response_timeout(_mb,0,1000000);
            //modbus_set_debug(_mb,true);
            modbus_rtu_set_serial_mode(_mb,MODBUS_RTU_RS232);
            if (modbus_connect(_mb) == -1) {
                qCritical() << "Connection failed: " << modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_CONNECT_FAILED;
            }
        }
        _FUNC_END_
    }catch(ReturnStatus error_code){
        modbus_free(_mb);
        qCritical()<<getErrString(error_code)<<endl;
        throw ERR_DEVICE_MODBUS_DISCONNECTED;
    }catch (...) {
        modbus_free(_mb);
        throw ERR_DEVICE_MODBUS_DISCONNECTED;
    }
}

void mbControl::disconnectDevice()
{
    if(_mb){
        modbus_close(_mb);
        modbus_free(_mb);
    }
    _FUNC_END_
}

void mbControl::reConnectDevice()
{
    connectDevice();
    disconnectDevice();
    _FUNC_END_
}

void mbControl::ReceiveReply(int sig)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[MSGLEN]= {0};
        memset(iSendData,0,sizeof(iSendData));
        switch(sig)
        {
        case 1://auto welding
            iSendData[0] = 101;
            break;
        case 2://kz
            iSendData[2] = 102;
            break;
        case 3://welding
            iSendData[2] = 103;
            break;
        case 4:
            iSendData[2] = 104;
            break;
        case 5://sync seam info
            iSendData[3] = 105;
            break;
        case 6://clean torch
            iSendData[4] = 106;
            break;
        case 7:
            iSendData[5] = 107;
            break;
        case 10:
            iSendData[10] = 110;
            break;
        case 11:
            iSendData[12] = 111;
            break;
        case 14:
            iSendData[14] = 114;
            break;
        case 15:
            iSendData[22] = 115;
            break;
        case 16:
            iSendData[23] = 116;
            break;
        case 17:
            iSendData[24] = 117;
            break;
        default:
            break;
        }
        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<getErrString(error_code)<<endl;
    }catch (...) {
        cout<<"error :Receive Reply failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::write(uint16_t (&iSendData)[MSGLEN])
{
    try
    {
        _LOGINFO_;
        _msgData msgData;
        msgData.bArray = true;
        msgData.bSeamData = false;
        msgData.bCadData = false;
        memcpy(msgData.array,iSendData,MSGLEN);
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        msgQueue.push(msgData);
        _FUNC_END_
    }catch (...) {
        cout<<"error :write msg to queue failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::write_cad(uint16_t (&iSendData)[MSGCADLEN])
{
    try
    {
        _LOGINFO_;
        _msgData msgData;
        msgData.bArray = false;
        msgData.bSeamData = false;
        msgData.bCadData = true;
        memcpy(msgData.cadArray,iSendData,MSGCADLEN);
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        msgQueue.push(msgData);
        _FUNC_END_
    }catch (...) {
        cout<<"error : write cad ,push msg to queue failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::write_Seam(uint16_t *iSendData,int msglen)
{
    try
    {
        _LOGINFO_;
        _msgData msgData;
        msgData.bArray = false;
        msgData.bSeamData = true;
        msgData.bCadData = false;
        msgData.seamMsgLen = msglen;
        memcpy(msgData.seamArray,iSendData,msglen);
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        msgQueue.push(msgData);
        _FUNC_END_
    }catch (...) {
        cout<<"error:write seam ,push msg to queue failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

//void mbControl::reset()
//{
//    try{
//        _LOGINFO_;
//        if(!_mb)
//            throw ERR_DEVICE_MODBUS_DISCONNECTED;

//        uint16_t iSendData[MSGLEN]= {0};
//        iSendData[0] = 1;
//        iSendData[3] = 1;
//        write(iSendData);
//        _FUNC_END_
//    }catch(ReturnStatus error_code){
//        cout<<"error: "<<getErrString(error_code)<<endl;
//        throw ;
//    }catch (...) {
//        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
//    }
//}

void mbControl::openAirValve()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGLEN]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 3;//打開
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        cout<<"error: open airValue failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::closeAirValve()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGLEN]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 4;//打開
        write(iSendData);
        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        cout<<"error: close airValue failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::robotbackDone()
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[MSGLEN]= {0};
        iSendData[11] = 12;
        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        cout<<"error: robot back safe position failed."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void mbControl::openLaser()
{
    Config _sys_config("etc/sys.info");
    string control = _sys_config.get<string>("laser.control");
    if(control != "PLC"){
        _laser_ptr->alwaysLight();
        this_thread::sleep_for(10_ms) ;
        return;
    }
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGLEN]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 1;//打開
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...){
        cout<<"error: open laser faield ."<<endl;
        throw ERR_MODBUS_LASER_CONTROL_FAILED;
    }
}

//need correct
void mbControl::closeLaser()
{
    Config _sys_config("etc/sys.info");
    string control = _sys_config.get<string>("laser.control");
    if(control != "PLC"){
        _laser_ptr->alwaysClose();
        return;
    }
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        int iResult = 0;

        uint16_t iSendData[MSGLEN]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 2;
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...){
        cout<<"error: close laser failed ."<<endl;
        throw ERR_MODBUS_LASER_CONTROL_FAILED;
    }
}

void mbControl::send_cad_list(std::vector<uint16_t>& vec_cad)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGCADLEN] = {0};
        memset(iSendData,0,sizeof(iSendData));
        for(int i = 0; i< MSGCADLEN; i++)
            iSendData[i] = vec_cad[i];
        multi_write_registers(CADADDRESS,MSGCADLEN,iSendData);
    }catch(ReturnStatus error_code){
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...){
        cout <<"error: write cad list to plc failed ."<<endl;
        throw ERR_FUNC_DISPLAY_CADS_LIST_FAILED;
    }
}

void mbControl::checkTcp()
{

}

void mbControl::checkSensor()
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        //        uint16_t iSendData[NUM]= {0};
        //        iSendData[5] = 107;
        //        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        qCritical() <<"error :check sensor failed ."<<endl;
        throw ERR_CHECK_SENSOR_FAILED;
    }
}

void mbControl::feedbackResult(int result)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGLEN]= {0};
        iSendData[6] = 7;
        iSendData[15] = result;
        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"feed back result failed, "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        qCritical() <<"error : feed back result failed ."<<endl;
        throw ERR_MODBUS_FEEDBACK_RESULT_FAILED;
    }
}

void mbControl::feedbackErrCode(int errCode)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[MSGLEN]= {0};
        iSendData[57] = errCode;
        cout<<"error code : "<<errCode<<endl;
        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"feed back error code ,"<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        cout<<"error :feed back error code failed ."<<endl;
        throw ERR_MODBUS_FEEDBACK_ERRORCODE_FAILED;
    }
}

void mbControl::updateProgress(int seamNum,int result)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[MSGLEN]= {0};
        iSendData[13] = 13;
        iSendData[15] = seamNum;
        iSendData[16] = result;
        write(iSendData);
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"update progress failed , "<<getErrString(error_code)<<endl;
        throw ;
    }catch (...) {
        cout<<"error:update progress failed ."<<endl;
        throw ERR_FUNC_UPDATE_PROGRESS_FAILED;
    }
}


void mbControl::rotateMotor(MOTORTYPE eMotorType,double angle,int speed,int mode)
{
    try
    {
        _LOGINFO_;
        if(!_mb )
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        rotate(eMotorType,angle,speed,mode);
        double currPos = 0;
        while(mode==1)
        {
            switch(eMotorType)
            {
            case GROUNDRAIL:
                currPos = gMotorposs;
                break;
            case FLIPDISK:
                currPos = gMotorangle1s;
                break;
            case ROTATINGDISK:
                currPos = gMotorangle2s;
                break;
            }

            if(equald(angle,currPos))
                break;
        }
        _FUNC_END_
    }catch(ReturnStatus error_code){
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }catch(...){
        qCritical() << getErrString(ERR_MOTOR_ROTATE_FAILED) <<endl;
        throw ERR_MOTOR_ROTATE_FAILED;
    }
}


void mbControl::rotate(MOTORTYPE eMotorType,double angle,int speed,int mode)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[MSGLEN] = {0};
        angle *= 10;

        if(eMotorType == GROUNDRAIL){
            iSendData[3] = 1;
            iSendData[98] = speed;
            iSendData[105] = angle;
        }
        if(eMotorType == ROTATINGDISK){
            iSendData[3] = 2;
            iSendData[97] = speed;
            iSendData[106] = angle;
        }
        if(eMotorType == FLIPDISK){
            iSendData[3] = 3;
            iSendData[96] = speed;
            iSendData[107] = angle;
        }
        if(eMotorType == ALLRESET){
            iSendData[3] = 4;
        }
        iSendData[5] = mode;
        write(iSendData);
        bRotateOver = false;

        if(1 == mode){
            std::unique_lock <std::mutex> lck(_MTmtx);
            while (!bRotateOver)
                mtCV.wait(lck);
            bRotateOver = false;
        }
        _FUNC_END_
    }catch(ReturnStatus error_code){
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }catch(...){
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}


void mbControl::rotateNew(double Groundpos,double speed1,
                          double FlipAngle,double speed2,
                          double RotateAngle3,double speed3,
                          int mode)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        cout<<"rotate :"<<Groundpos<<"|"<<speed1<<"|"<<FlipAngle<<"|"<<speed2<<"|"<<RotateAngle3<<"|"<<speed3<<endl;
        uint16_t iSendData[MSGLEN] = {0};

        iSendData[9] = 9;
        iSendData[15] = mode;
        iSendData[16] = Groundpos * 10;
        iSendData[17] = speed1 * 100;
        iSendData[18] = FlipAngle * 10;
        iSendData[19] = speed2 * 100;
        iSendData[20] = RotateAngle3 * 10;
        iSendData[21] = speed3 * 100;

        write(iSendData);
        bRotateOver = false;

        if(1 == mode){
            std::unique_lock <std::mutex> lck(_MTmtx);
            while (!bRotateOver)
                mtCV.wait(lck);
            bRotateOver = false;
        }
        _FUNC_END_
    }catch(ReturnStatus error_code){
        cout<<"rotate new failed, " <<getErrString(error_code)<<endl;
        throw ;
    }catch(...){
        cout<<"error:motor rotate failed"<<endl;
        throw ERR_MODBUS_MOTOR_CONTROL_FAILED;
    }
}


void mbControl::recv_msg()
{
    try{
        memset(iRecvData,0,sizeof(iRecvData));
        read_registers(READADDRESS,MSGLEN,iRecvData);
    }catch(ReturnStatus error_code){
        cout<<"recv message failed , "<<getErrString(error_code)<<endl;
        throw;
    }catch (...) {
        cout <<"throw exception : receive message exception"<<endl;
        throw ERR_MODBUS_READ_REGISTERS_FAILED;
    }
}

void mbControl::send_msg()
{
    try
    {
        int i = 0;
        while(1){
            if(msgQueue.size() != 0){
                break;
            }else{
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                i++;
            }
            if(i > 10000){
                cout<<"error , PC msg queue is empty for 10s !!!"<<endl;
                throw ERR_MODBUS_MSGQUEUE_SEND_IS_EMPTY;
            }
        }
        if(msgQueue.size() > 1)
            cout<<"current queue size = "<<msgQueue.size()<<endl;

        auto &msg = msgQueue.front();
        if(msg.bArray)
        {
            std::lock_guard<std::recursive_mutex> lck(_mtx) ;
            oldCheckCode = checkCode;
            updateCheckCode();
            msg.array[MSGLEN-1] = checkCode;
            write_registers(WRITEADDRESS,MSGLEN,msg.array);
            msgQueue.pop();
        }else{
            bool bSend = false;
            while(!bSend)
            {
                int i = 0;
                while(1){
                    if(msgQueue.size() != 0){
                        break;
                    }else{
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        i++;
                    }
                    if(i > 10000){
                        cout<<"error , PC msg queue is empty for 10s !!!"<<endl;
                        throw ERR_MODBUS_MSGQUEUE_SEND_IS_EMPTY;;
                    }
                }
                auto &msg = msgQueue.front();
                //                if(msg.bCadData)
                //                {
                //                    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
                //                    int count = sizeof(msg.cadArray)/sizeof(msg.cadArray[1]);
                //                    write_registers(msg.cadArray,count,CADADDRESS);
                //                    msgQueue.pop();
                //                    continue;
                //                }
                if(msg.bSeamData)
                {
                    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
                    int count = msg.seamMsgLen;
                    multi_write_registers(SEAMADDRESS,count,msg.seamArray);
                    msgQueue.pop();
                    continue;
                }
                if(msg.bArray)
                {
                    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
                    oldCheckCode = checkCode;
                    updateCheckCode();
                    msg.array[MSGLEN-1] = checkCode;
                    write_registers(WRITEADDRESS,MSGLEN,msg.array);
                    msgQueue.pop();
                    bSend = true;
                }
            }
        }
        if(msgQueue.size() != 0)
            cout<<msgQueue.size()<<" message left ,next cycle to send "<<endl;
    }catch(ReturnStatus error_code){
        cout<<"send message failed, "<<getErrString(error_code)<<endl;
        throw;
    }catch (...) {
        cout <<"throw exception : send message exception"<<endl;
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}

int mbControl::deal_msg()
{
    try
    {
        bRotateOver = false;
        bOpenOver = false;
        obj.init();
        obj.load(iRecvData);
        iSignal = obj.msgCmd;

        if(obj.msgCheckCode != 85 && obj.msgCheckCode != 170){
            cout<<"check code is not between 85 or 170, error !!!"<<endl;
            errCnt++;
            return -1;
        }
        if(oldCheckCode == obj.msgCheckCode){
            cout<<"twice check code same"<<endl;
            errCnt++;
            return -1;
        }
        if(errCnt >20){
            errCnt = 0;
            cout<<"check code error times more than 20, "<<endl;
            throw ERR_MODBUS_CHECKCODE_ERROR;
        }
        checkCode = obj.msgCheckCode;

        if(obj.bHandMsg)//hand
        {
            //todo cout<<"this is hand msg"<<endl;
            memset(iSendData,0,sizeof(iSendData));
            _msgData msgData;
            memcpy(msgData.array,iSendData,MSGLEN);
            std::lock_guard<std::recursive_mutex> lck(_mtx) ;
            msgQueue.push(msgData);
        }else{
            cout<<"this is command ***"<<endl;
            if(iRecvData[18] == 108){
                std::unique_lock <std::mutex> lck(mtx);
                bOpenOver = true;
                myCV.notify_all();
            }
            if(iRecvData[5] == 10 ){
                std::unique_lock <std::mutex> lck(_MTmtx);
                bRotateOver = true;
                mtCV.notify_all();
            }
        }
        _FUNC_END_;
        return iSignal;
    }catch(ReturnStatus error_code){
        cout<<"deal message failed ,"<<getErrString(error_code)<<endl;
        throw;
    }catch (...) {
        cout <<"throw exception : deal message exception"<<endl;
        throw ERR_MODBUS_INVALID_DATA;
    }
}

void mbControl::read_registers(int address,int count,int16_t* iRecvData)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t temp[count];
        memset(temp,0,sizeof(temp)/sizeof(temp[1]));
        int rc = modbus_flush(_mb);
        rc = modbus_read_registers(_mb,address,count,temp);
        if(rc == -1){
            int errCnt = 0;
            bool bRead = false;
            for(int i =0; i< 100; i++)
            {
                errCnt++;
                cout<<" read register error times : "<<errCnt<<" , try read again !"<<endl;
                memset(temp,0,sizeof(temp)/sizeof(temp[1]));
                address = READADDRESS;
                int rc = modbus_flush(_mb);
                rc = modbus_read_registers(_mb,address,MSGLEN,temp);

                if(rc == -1)
                    continue;
                else{
                    bRead = true;
                    cout<<"try read register success !"<<endl;
                    for(int i = 0; i< MSGLEN; i++)
                        iRecvData[i] = (int16_t)(temp[i]);
                    break;
                }
            }
            if(!bRead){
                cout<<" read error, "<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_READ_REGISTERS_FAILED;
            }
        }else{
            for(int i = 0; i< MSGLEN; i++)
                iRecvData[i] = (int16_t)(temp[i]);
        }
        print_recv_Data(iRecvData,count);
    }catch(ReturnStatus error_code){
        cout<<"read register failed ,"<<getErrString(error_code)<<endl;
        throw;
    }catch (...) {
        cout <<"throw exception : readRegisters exception"<<endl;
        throw ERR_MODBUS_READ_REGISTERS_FAILED;
    }
}

void mbControl::write_registers(int address,int count,uint16_t* iSendData)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        print_send_Data(iSendData,count);
        int rc = modbus_flush(_mb);
        rc = modbus_write_registers(_mb,address,count,iSendData);
        if(rc == -1){
            cout<<" write error, "<< modbus_strerror(errno)<<endl;
            int errCnt = 0;
            bool bSend = false;
            for(int i =0; i< 100; i++)
            {
                errCnt++;
                cout<<" write register error times : "<<errCnt<<" , try write again !"<<endl;
                rc = modbus_flush(_mb);
                rc = modbus_write_registers(_mb,address,count,iSendData);
                if(rc == -1)
                    continue;
                else{
                    bSend = true;
                    cout<<"try write register success !"<<endl;
                    break;
                }
            }
            if(!bSend)
                throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
        }else{
            //cout<<"write register success ."<<endl;
        }
    }catch(ReturnStatus error_code){
        cout<<getErrString(error_code)<<endl;
    }catch (...) {
        qCritical() <<"throw exception : write registers exception"<<endl;
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}


void mbControl::multi_write_registers(int address,int count,uint16_t* data)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        std::lock_guard<std::recursive_mutex> lck(_mtx) ;

        int num = MODBUS_MAX_WRITE_REGISTERS;
        int quotient = count/num;
        int remainder = count%num;
        cout<<"quotient = "<< quotient <<"|remainder = "<<remainder<<endl;

        for(int i = 0; i< quotient; i++ ){
            uint16_t iSendData[num] = {0};
            for(int j = 0; j < num; j++){
                iSendData[j] = data[j + num*i];
            }
            int addr = address + num*i;
            write_registers(addr,num,iSendData);
            cout<<"write "<< i <<" index success ."<<endl;
        }

        cout<<"write  quotient  success ."<<endl;
        if(!(quotient > 0 && quotient*num == count)){
            uint16_t iSendData[remainder] = {0};
            for(int j = 0; j < remainder; j++){
                iSendData[j] = data[j + num*quotient];
            }
            int addr = address + num*quotient;
            write_registers(addr,remainder,iSendData);
            cout<<"write  remainder  success ."<<endl;
        }
    }catch(ReturnStatus error_code){
        cout<<getErrString(error_code)<<endl;
    }catch (...) {
        cout <<"throw exception : write registers exception"<<endl;
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}

void mbControl::updateCheckCode()
{
    if(checkCode == 0){
        checkCode = 85;
    }else if(checkCode == 85){
        checkCode = 170;
    }else if(checkCode == 170){
        checkCode = 85;
    }
}


//void mbControl::cleanTorch()
//{
//    try{
//        //        _LOGINFO_;
//        //        if(!_mb)
//        //            throw ERR_DEVICE_MODBUS_DISCONNECTED;
//        //        uint16_t iSendData[NUM]= {0};
//        //        iSendData[4] = 106;
//        //        write(iSendData);
//    }catch(ReturnStatus error_code){
//        cout<<getErrString(error_code)<<endl;
//    }catch (...) {

//    }
//}

int mbControl::get_cadId() const
{
    return obj.msgBody[11];
}

int mbControl::get_seamId() const
{
    return obj.msgBody[10];
}


void mbControl::test()
{
    //    uint16_t iReadData[2]={0};

    //    modbus_read_registers(_mb,0,2,iReadData);
    //    auto temp1 = modbus_get_float_dcba(iReadData);
    //    cout<<temp1<<endl;

    //    modbus_read_registers(_mb,2,2,iReadData);
    //    auto temp2 = modbus_get_float_dcba(iReadData);
    //    cout<<temp2<<endl;

    //    modbus_read_registers(_mb,4,2,iReadData);
    //    auto temp3 = modbus_get_float_dcba(iReadData);
    //    cout<<temp3<<endl;

    int address = 0;
    vector<uint16_t> vec(800,5);
    //write_registers(vec,0);
}
