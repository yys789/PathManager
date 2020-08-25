#include "mbSlave.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <QString>
#include <time.h>
#include <thread>
#include "base/errorcode.h"
#include <QDebug>
#include "base/config.h"

using namespace std;


slave::slave():
    _mb(nullptr),
    mb_mapping{nullptr},
    req_len{0},
    checkCode{0},
    oldCheckCode{0},
    handShakeDate{0},
    iSignal{0},
    bGetDone{false},
    bRotateOver{false},
    bDataUpdate{true}
{
    try
    {
        connectDevice();
    }catch(...){};
}

slave::~slave()
{
    disconnectDevice();
}

void slave::connectDevice()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try {
        _LOGINFO_;
        Config _sys_config("etc/sys.info");
        _mb = modbus_new_rtu("/dev/ttyS1", 9600, 'N', 8, 1);
        if(_mb == NULL){
            cout<<"Unable to create the libmodbus context , errInfo :"<< modbus_strerror(errno)<<endl;
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        }
        int rc = modbus_set_slave(_mb, 1);
        if(rc == -1){
            cout<<" Invalid salve ID , errInfo :"<< modbus_strerror(errno)<<endl;
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        }
        struct timeval t;
        t.tv_sec=1;
        t.tv_usec=1000000;
        modbus_set_response_timeout(_mb,t.tv_sec,t.tv_usec);

        if(modbus_connect(_mb) == -1){
            cout<<" Connection failed , errInfo :"<< modbus_strerror(errno)<<endl;
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        }
        modbus_set_debug(_mb,true);
        //寄存器map初始化
        mb_mapping = modbus_mapping_new_start_address(0,0,0,0,0,60,0,60);
        if (mb_mapping == NULL) {
            cout<<"Failed to allocate the mapping: "<< modbus_strerror(errno);
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        }
        cout<<"[connectDevice][OVER]"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        modbus_free(_mb);
        qCritical()<<getErrString(error_code)<<endl;
        throw ERR_DEVICE_MODBUS_DISCONNECTED;
    }
    catch (...) {
        modbus_free(_mb);
        throw ERR_DEVICE_MODBUS_DISCONNECTED;
    }
}


void slave::disconnectDevice()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    if(_mb)
    {
        modbus_mapping_free(mb_mapping);
        modbus_close(_mb);
        modbus_free(_mb);
    }
    cout<<"[disconnectDevice][OVER]"<<endl;
}

void slave::reConnectDevice()
{
    connectDevice();
    disconnectDevice();
    cout<<"[reConnectDevice][OVER]"<<endl;
}


void slave::write(uint16_t (&iSendData)[NUM])
{
    try
    {
        _LOGINFO_;
        for(int i = 0; i < NUM; i++)
        {
            if(iSendData[i] != 0)
                cout<<"[address]: "<<i*2 <<" ,arg["<<i<<"]: "<<iSendData[i]<<endl;
        }

        SENDMESSAGE sendmsg;
        memcpy(sendmsg.array,iSendData,sizeof(iSendData));
        {
            std::lock_guard<std::recursive_mutex> lck(_mtx);
            msgQueue.push(sendmsg);
        }
        cout<<"push over ."<<endl;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::reset()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[0] = 1;
        iSendData[3] = 1;

        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::openAirValve()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 3;//打開
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
    }
    catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::closeAirValve()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 4;//打開
        write(iSendData);
        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
    }
    catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

//need correct
void slave::displaySeamScanPosInfo(vector<int>& vec)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try
    {
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        int count = 8;
        uint16_t iCmd[count]= {0};
        for(int i = 0; i<vec.size(); i++)
        {
            iCmd[i] = vec[i];
        }
        int address = 400;
        address = address/2;
        int rc = modbus_flush(_mb);
        rc = modbus_write_registers(_mb,address,count,iCmd);
        if(rc == -1)
        {
            qCritical() <<" [getSelectedSeam]: error, "<< modbus_strerror(errno)<<endl;
            throw ERR_MODBUS_READ_REGISTERS_FAILED;
        }
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }

}

//need correct
void slave::currPosSafe()
{
    try{
        cout<<__FUNCTION__<<endl;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iCmd[NUM]= {0};
        iCmd[0] = iSignal;
        int address = 14;
        address = address/2;
        int rc = modbus_flush(_mb);
        rc = modbus_write_registers(_mb,address,2,iCmd);
        if(rc == -1)
        {
            qCritical() <<"[currPosSafe] write error, "<< modbus_strerror(errno)<<endl;
            throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
        }
        while(1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            address = 4;
            address = address/2;
            iCmd[1]= {0};
            rc = modbus_read_registers(_mb,address,1,iCmd);
            if(rc == -1)
            {
                qCritical() <<"[currPosSafe] write error, "<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_READ_REGISTERS_FAILED;
            }else{
                if(iCmd[0] == 3)
                {
                    break;
                    cout<<"[currPosSafe] PLC done !"<<endl;
                }else{
                    cout<<"wait ... !"<<endl;
                }
            }
        }
        address = 4;
        address = address/2;
        iCmd[1]= 0;
        rc = modbus_write_registers(_mb,address,1,iCmd);
        if(rc == -1)
        {
            qCritical() <<"[currPosSafe] write error, "<< modbus_strerror(errno)<<endl;
            throw ERR_MODBUS_READ_REGISTERS_FAILED;
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

//need correct
void slave::openLaser()
{
    Config _sys_config("etc/sys.info");
    string control = _sys_config.get<string>("laser.control");
    if(control != "PLC")
    {
        _laser_ptr->alwaysLight();
        this_thread::sleep_for(10_ms) ;
        return;
    }
    try{
        _LOGINFO_
                if(!_mb)
                throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 1;//打開
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        cout<<"[openLaser][OVER]"<<endl;

    }
    catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...)
    {
        qCritical() <<" write cad list to plc error"<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;

    }
}

//need correct
void slave::closeLaser()
{
    Config _sys_config("etc/sys.info");
    string control = _sys_config.get<string>("laser.control");
    if(control != "PLC")
    {
        _laser_ptr->alwaysClose();
        return;
    }
    try{
        _LOGINFO_
                if(!_mb)
                throw ERR_DEVICE_MODBUS_DISCONNECTED;
        int iResult = 0;

        uint16_t iSendData[NUM]= {0};
        iSendData[7]= 8;//打開
        iSendData[8]= 2;
        write(iSendData);

        std::unique_lock <std::mutex> lck(mtx);
        while (!bOpenOver)
            myCV.wait(lck);
        bOpenOver = false;
        cout<<"[closeLaser][OVER]"<<endl;

    }catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...)
    {
        qCritical() <<" write cad list to plc error"<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;

    }
}

void slave::writePLCCAD(std::vector<int>& vec_cad,int index,int done)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};

        iSendData[0] = iSignal;
        if(done == 1)
            iSendData[4] = 1;
        iSendData[10] = index;
        if(vec_cad.size() != 0){
            for(int i = 11 ; i<= vec_cad.size()+11 ; i++)
                iSendData[i] = vec_cad[i-11];
        }

        write(iSendData);

        if(done != 1)
        {
            std::unique_lock <std::mutex> lck(mtx);
            while (!bGetDone)
                myCV.wait(lck);
            bGetDone = false;
        }
        cout<<"[writePLCCAD][OVER]"<<endl;

    }
    catch(ReturnStatus error_code)
    {
        cout<<"error: "<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...)
    {
        qCritical() <<" write cad list to plc error"<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::writePLCMidPoint(std::vector<int>& vec_midpoint,int index,int done)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};

        iSendData[0] = iSignal;

        if(done == 1)
            iSendData[4] = 1;

        iSendData[10] = index;

        if(vec_midpoint.size() != 0){
            for(int i = 11 ; i<= vec_midpoint.size()+11 ; i++)
                iSendData[i] = vec_midpoint[i-11];
        }

        write(iSendData);

        if(done != 1)
        {
            std::unique_lock <std::mutex> lck(mtx);
            while (!bGetDone)
                myCV.wait(lck);
            bGetDone = false;
        }
        cout<<"[writePLCMidPoint][OVER]"<<endl;

    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        qCritical() <<" write cad list to plc error"<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::writePLCSeamList(std::vector<int>& vec_seamList)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[4] = 1;
        if(vec_seamList.size()!= 0){
            for(int i = 10 ; i < vec_seamList.size()+10; i++)
                iSendData[i] = vec_seamList[i-10];
        }
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        qCritical() <<"write seam list to plc error"<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::writePLCSeamInfo(vector<int>&vec_seamInfo)
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[4] = 1;
        for(int i = 10 ; i < vec_seamInfo.size()+10; i++)
            iSendData[i] = vec_seamInfo[i-10];
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        qCritical() <<"[writePLCSeamInfo]:write plc seam info failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::checkTcp()
{

    //    try{
    //        _LOGINFO_
    //                if(!_mb)
    //                throw ERR_DEVICE_MODBUS_DISCONNECTED;

    //        uint16_t iSendData[NUM]= {0};
    //        iSendData[0] = 3;

    //        write(iSendData);

    //        while(1)
    //        {
    //            this_thread::sleep_for(std::chrono::milliseconds(10));
    //            if(!bDataUpdate)
    //                continue;
    //            vector<int> temp(100,0);
    ////            getData(temp);

    //            cout<<"temp[94] = "<<temp[94]<<endl;
    //            if(temp[94] == 27 ||temp[94] == 28)
    //            {
    //                break;
    //            }
    //        }
    //        cout<<"[checkTcp][OVER]"<<endl;
    //    }
    //    catch(ReturnStatus error_code)
    //    {
    //        cout<<getErrString(error_code)<<endl;
    //        throw ;
    //    }
    //    catch (...) {
    //        qCritical() <<"[checkTcp]:write plc failed ."<<endl;
    //        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    //    }
}

void slave::checkSensor(bool bSuccess)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[4] = 1;
        if(bSuccess)
            iSendData[108] = 29;
        else
            iSendData[108] = 30;

        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        qCritical() <<"[checkTcp]:write plc failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

//need correct
void slave::updateProgress(int totals,int successCnt,int seamNum)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[99] = seamNum;
        iSendData[100] = successCnt;
        iSendData[100] = totals;
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}


void slave::rotateMotor(MOTORTYPE eMotorType,double angle,int speed,int mode)
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

            //            if(equald(angle,currPos))
            //                break;
        }
        cout<<"[rotate][OVER]"<<endl;
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


void slave::rotate(MOTORTYPE eMotorType,double angle,int speed,int mode)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM] = {0};
        angle *= 10;

        if(eMotorType == GROUNDRAIL)
        {
            iSendData[3] = 1;
            iSendData[98] = speed;
            iSendData[105] = angle;
        }
        if(eMotorType == ROTATINGDISK)
        {
            iSendData[3] = 2;
            iSendData[97] = speed;
            iSendData[106] = angle;
        }
        if(eMotorType == FLIPDISK)
        {
            iSendData[3] = 3;
            iSendData[96] = speed;
            iSendData[107] = angle;
        }
        if(eMotorType == ALLRESET)
        {
            iSendData[3] = 4;
        }
        iSendData[5] = mode;
        write(iSendData);
        bRotateOver = false;

        if(1 == mode)
        {
            std::unique_lock <std::mutex> lck(_MTmtx);
            while (!bRotateOver)
                mtCV.wait(lck);
            bRotateOver = false;
        }
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::rotateNew(double Groundpos,int speed1,
                          double FlipAngle,int speed2,
                          double RotateAngle3,int speed3,
                          int mode)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM] = {0};

        iSendData[9] = 9;
        iSendData[15] = mode;
        iSendData[16] = Groundpos;
        iSendData[17] = speed1;
        iSendData[18] = FlipAngle;
        iSendData[19] = speed2;
        iSendData[20] = RotateAngle3;
        iSendData[21] = speed3;

        write(iSendData);
        bRotateOver = false;

        if(1 == mode)
        {
            std::unique_lock <std::mutex> lck(_MTmtx);
            while (!bRotateOver)
                mtCV.wait(lck);
            bRotateOver = false;
        }
    }
    catch(ReturnStatus error_code)
    {
        qCritical() <<getErrString(error_code)<<endl;
        throw ;
    }
    catch(...)
    {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}



void slave::recv_msg()
{
    _LOGINFO_;
    memset(query,0,sizeof(query));
    //轮询接收数据，并做相应处理5
    req_len = 0;
    func_Code = 0;
    req_len = modbus_receive(_mb, query);
    if (req_len > 0)
    {
        func_Code = uint16_t(query[1]);
        cout<<"func code : "<<func_Code<<endl;

        int head_len = modbus_get_header_length(_mb);
        cout<<"head_len = "<<head_len<<endl;

        uint16_t slaveAddr = uint16_t(query[0]);
        cout<<"slave address : "<<slaveAddr<<endl;


        uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query,2);
        cout<<"start addr : "<<startAddr<<endl;

        uint16_t rgtNums = MODBUS_GET_INT16_FROM_INT8(query,4);
        cout<<"register nbs : "<<rgtNums<<endl;

        if(func_Code == 16)//write
        {
            uint16_t byteNums = uint16_t(query[6]);
            cout<<"byte nbs : "<<byteNums<<endl;
            recvData.clear();
            for(int i = 0; i< rgtNums; i++){
                uint16_t temp1 = MODBUS_GET_INT16_FROM_INT8(query,7+i*2);
                int16_t temp = (int16_t)(temp1);
                recvData.push_back(temp);
                if(temp != 0)
                    cout<<"[recv] address: "<<i <<" ,arg["<<i<<"] : "<<temp<<endl;
            }
            if(recvData.size() != NUM)
                throw ERR_MODBUS_INVALID_DATA;
            //modbus_reply(_mb, query, req_len, mb_mapping);
        }
    } else if (req_len  == -1) {
        throw ERR_MODBUS_INVALID_DATA;
    }
}

void slave::send_msg()
{
    _LOGINFO_;
    //    while(1)
    //    {
    //        if(msgQueue.size() != 0){
    //            break;
    //        }
    //    }

    //        std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    //        auto &msg = msgQueue.front();

    //        updateCheckCode();
    //        msg.array[59] = checkCode;
    //        for(int i = 0; i<NUM;i++)
    //        {
    //            mb_mapping->tab_input_registers[i] = msg.array[i];
    //        }
    //        msgQueue.pop();

    //    bDataUpdate = false;
    //    for(int i = 0; i<60; i++){
    //        if(mb_mapping->tab_input_registers[i] != 0)
    //            cout<<"i = "<<i <<",value = "<<mb_mapping->tab_input_registers[i]<<endl;
    //    }
    int checkcode = 0;
    if(recvData[59] == 85)
        checkcode = 170;
    else if(recvData[59] == 170)
        checkcode = 85;
    mb_mapping->tab_input_registers[59] = checkcode;
    modbus_reply(_mb, query, req_len, mb_mapping);

}

int slave::deal_msg()
{
    _LOGINFO_;
    //    bRotateOver = false;
    //    bOpenOver = false;
    //    recvMessage.init();
    //    recvMessage.load(recvData);
    //    checkCode = recvMessage.msgCheckCode;
    //    iSignal = recvMessage.msgCmd;
    //    if(checkCode != 85 && checkCode != 170){
    //        cout<<"check code is not between 85 or 170, error !!!"<<endl;
    //        return -1;
    //    }

    //    if(oldCheckCode == checkCode){
    //        bDataUpdate = false;
    //        cout<<"twice check code same"<<endl;
    //        return -1;
    //    }else{
    //        bDataUpdate = true;
    //    }
    //    oldCheckCode = checkCode;
    //    bool bHandCmd = true;
    //    if(recvMessage.msgCmd != 0)
    //        bHandCmd = false;

    //    if(bHandCmd)//hand
    //    {
    //        for(int i = 0; i<43; i++){
    //            if(recvMessage.msgBody[i] != 0){
    //                bHandCmd = false;
    //                break;
    //            }
    //        }
    //        if(!bHandCmd)//not hand
    //        {
    //            cout<<"this is not hand msg , is ack "<<endl;
    //            if(recvData[18] == 108){
    //                std::unique_lock <std::mutex> lck(mtx);
    //                bOpenOver = true;
    //                myCV.notify_all();
    //            }
    //        }else{
    //    cout<<"this is hand msg"<<endl;
    //    uint16_t iSendData[NUM] = {0};
    //    SENDMESSAGE sendmsg;
    //    memcpy(sendmsg.array,iSendData,sizeof(iSendData));
    //    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    //    msgQueue.push(sendmsg);
    //        }
    //    }
    //    else
    //    {
    //        cout<<"this is command ***"<<endl;
    //        if(recvData[5] == 10 ){
    //            std::unique_lock <std::mutex> lck(_MTmtx);
    //            bRotateOver = true;
    //            mtCV.notify_all();
    //        }
    //    }
    //    cout<<"signal = "<<iSignal<<endl;
    return iSignal;
}


void slave::readRegisters()
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        recvData.clear();
        //address = READADDRESS/2;
        address = 0;

        int rc = modbus_flush(_mb);
        uint16_t iRecvData[100] = {0};
        rc = modbus_read_registers(_mb,address,100,iRecvData);
        if(rc == -1)
        {
            throw ERR_MODBUS_READ_REGISTERS_FAILED;
        }else{
            for(int i = 0; i< NUM; i++)
            {
                int16_t data = (int16_t)(iRecvData[i]);
                recvData.push_back(data);
                if(data != 0)
                    qDebug()<<"[read] address: "<<0+i*2 <<" ,arg["<<i<<"] : "<<data;

            }
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        qDebug()<<" read error, "<< modbus_strerror(errno)<<endl;
    }
    catch (...) {
        qCritical() <<"throw exception : readRegisters exception"<<endl;
        throw ERR_MODBUS_READ_REGISTERS_FAILED;
    }
}

void slave::writeRegisters(uint16_t iSendData[60])
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        for(int i = 0; i< NUM; i++)
        {
            if(iSendData[i] != 0)
            {
                qDebug()<<"[write] address: "<<i*2 <<" ,arg["<<i<<"] : "<<iSendData[i];
                //                                cout<<"[write] address: "<<WRITEADDRESS+i*2 <<" ,arg["<<i<<"] : "<<iSendData[i]<<endl;
            }
        }
        address = WRITEADDRESS/2;

        int rc = modbus_flush(_mb);
        rc = modbus_write_registers(_mb,address,NUM,iSendData);
        if(rc == -1)
        {
            throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        errCnt++;
        qDebug()<<" write error, "<< modbus_strerror(errno)<<endl;
        qDebug()<<"error count : "<<errCnt<<endl;
        qDebug()<<"read again : "<<endl;
        for(int i = 0; i< 100; i++)
        {
            modbus_flush(_mb);
            int rc = modbus_write_registers(_mb,address,NUM,iSendData);
            if(rc == -1)
            {
                errCnt++;
                qDebug()<<" write error, "<< modbus_strerror(errno)<<endl;
                qDebug()<<"error count : "<<errCnt<<endl;
                if(i < 99)
                    continue;
                else
                    throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
            }else{
                return;
            }
        }
    }
    catch (...) {
        qCritical() <<"throw exception : writeRegisters exception"<<endl;
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}

void slave::writeRegisters(struct SENDMESSAGE* pMsg)
{
    //    _LOGINFO_
    //std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        address = WRITEADDRESS/2;

        uint16_t iSendData[NUM] = {0};
        for(int i = 0; i< NUM; i++)
        {
            iSendData[i] = pMsg->array[i];
            //                        if(pMsg->array[i] != 0)
            //                            qDebug()<<"WM_address: "<<WRITEADDRESS+i*2 <<" ,arg["<<i<<"] : "<<pMsg->array[i]<<endl;
        }
        int rc = modbus_flush(_mb);
        rc = modbus_write_registers(_mb,address,NUM,iSendData);
        if(rc == -1)
        {
            cout<<" write error, "<< modbus_strerror(errno)<<endl;
            throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
        }else{
            cout<<" write success"<<endl;
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}

void slave::writeRegisters(vector<uint16_t>&vec,int startAddr)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        int done = 0;
        int index = 1;
        int num = 123;
        int quotient = vec.size()/num;
        int remainder = vec.size()%num;
        cout<<"quotient = "<< quotient <<"|remainder = "<<remainder<<endl;


        for(int i = 0; i< quotient; i++ )
        {
            uint16_t iSendData[num] = {0};
            for(int j = 0; j < num; j++)
            {
                iSendData[j] = vec[j + num*i];
            }
            int rc = modbus_flush(_mb);

            int addr = startAddr + num*i;
            rc = modbus_write_registers(_mb,addr,num,iSendData);
            if(rc == -1)
            {
                cout<<" write error, "<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
            }
            cout<<"write "<< i <<" index success ."<<endl;
        }

        cout<<"write  quotient  success ."<<endl;


        if(!(quotient > 0 && quotient*num == vec.size())){
            uint16_t iSendData[remainder] = {0};
            for(int j = 0; j < remainder; j++)
            {
                iSendData[j] = vec[j + num*quotient];
            }
            int rc = modbus_flush(_mb);
            int addr = startAddr + num*quotient;
            rc = modbus_write_registers(_mb,addr,remainder,iSendData);
            if(rc == -1)
            {
                cout<<" write error, "<< modbus_strerror(errno)<<endl;
                throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
            }
            cout<<"write  remainder  success ."<<endl;
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        cout<<" write error, "<< modbus_strerror(errno)<<endl;
    }
    catch (...) {
        cout <<"throw exception : writeRegisters exception"<<endl;
        throw ERR_MODBUS_WRITE_REGISTERS_FAILED;
    }
}


void slave::robotbackDone()
{
    try{
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[11] = 12;
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::feedbackResult(int result)
{
    try
    {
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};
        iSendData[6] = 7;
        iSendData[15] = result;
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        qCritical() <<"[checkTcp]:write plc failed ."<<endl;
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}
void slave::feedbackErrCode(int errCode)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[57] = errCode;
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw ;
    }
    catch (...) {
        throw ERR_MODBUS_PUSH_MESSAGE_FAILD;
    }
}

void slave::updateCheckCode()
{
    if(checkCode == 0)
    {
        checkCode = 85;
    }
    else if(checkCode == 85)
    {
        checkCode = 170;
    }
    else if(checkCode == 170)
    {
        checkCode = 85;
    }
}

void slave::writeOver()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[4] = 1;

        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
    }
    catch (...) {
    }
}
void slave::writeOver1()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[12] = 111;
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
    }
    catch (...) {
    }
}
void slave::feedback()
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;
        uint16_t iSendData[NUM]= {0};
        iSendData[0] = iSignal;
        iSendData[4] = 0;

        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
    }
    catch (...) {
    }
}

void slave::ReceiveReply(int sig)
{
    try{
        _LOGINFO_;
        if(!_mb)
            throw ERR_DEVICE_MODBUS_DISCONNECTED;

        uint16_t iSendData[NUM]= {0};

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
        default:
            break;
        }
        write(iSendData);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
    }
    catch (...) {
    }
}



bool slave::getCmd(vector<int>& vec)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try
    {
        auto v = recvData;
        if(v.size() != NUM)
            throw ERR_MODBUS_INVALID_DATA;
        for(int i = 0; i< 10; i++)
            vec[i] = v[i];
        return true;
    }
    catch(...)
    {
        throw;
    }
}

void slave::getData(vector<int>& vec)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try
    {
        if(recvData.size()!= NUM)
            throw ERR_MODBUS_INVALID_DATA;
        auto v = recvData;
        for(int i = 10; i< v.size()-1; i++)
            vec[i-10]= v[i];
    }
    catch(...)
    {
        throw;
    }
}

int slave::getCheckData()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try
    {
        auto v = recvData;
        if(v.size() != NUM)
            throw ERR_MODBUS_INVALID_DATA;
        return v[NUM-1];
    }
    catch(...)
    {
        throw;
    }
}

void slave::setCmd(vector<int>& vec)
{

}

void slave::setData(vector<int>& vec)
{

}

void slave::setCheck(vector<int>& vec)
{

}
