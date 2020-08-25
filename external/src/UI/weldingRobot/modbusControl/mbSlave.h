#ifndef MBSLAVE_H
#define MBSLAVE_H

#include "modbus/modbus.h"
#include <boost/shared_ptr.hpp>
#include "device/robotlaser.h"
#include <mutex>
#include <queue> 
#include <condition_variable>
#include <iostream>
#include "mbCommon.h"


using namespace std;

class slave
{
public:
    slave();
    ~slave();
public:

    void feedbackErrCode(int errCode);

    void write(uint16_t (&iSendData)[NUM]);
    ///接收消息
    void recv_msg();
    ///发送消息
    void send_msg();
    ///处理消息
    int deal_msg();

    void reset();
    ///傳感器教研
    void checkSensor(bool);
    ///tcp校驗
    void checkTcp();
    ///更新校验码
    void updateCheckCode();

    void writeOver();

    void feedback();
    ///顯示焊縫掃描位置信息
    void displaySeamScanPosInfo(std::vector<int>& vec);
    ///连接设备
    void connectDevice();
    ///断开连接
    void disconnectDevice();
    ///判断当前机器人位置是否安全
    void currPosSafe();

    void reConnectDevice();
    ///清槍
    void cleanTorch();
    ///剪絲
    void cutWire();
    ///噴油
    void spraying();
    ///打開激光
    void openLaser();
    ///關閉激光
    void closeLaser();
    ///傳輸中間點
    void writePLCMidPoint(std::vector<int>& vec,int index,int done);
    ///发送工件列表
    void writePLCCAD(std::vector<int>& vec,int index,int done);
    ///发送焊缝列表
    void writePLCSeamList(std::vector<int>& vec);
    ///发送焊缝信息
    void writePLCSeamInfo(std::vector<int>&vec);

    void updateProgress(int,int,int);
    void rotateMotor(MOTORTYPE eMotorType,double angle,int speed = 200,int mode = 1);
    ///电机旋转
    void rotate(MOTORTYPE ,double,int speed = 200,int mode = 1);
    void rotateNew(double Groundpos,int speed1,
                   double FlipAngle,int speed2,
                   double RotateAngle3,int speed3,
                   int mode);
    ///写寄存器
    void writeRegisters(uint16_t iSendData[NUM]);
    ///都寄存器
    void writeRegisters(struct SENDMESSAGE* pMsg);

    void writeRegisters(std::vector<uint16_t>&vec,int addr);

    void readRegisters();

    void robotbackDone();

    void feedbackResult(int result);

    ///打開氣閥
    void openAirValve();
    ///關閉氣閥
    void closeAirValve();

    bool getCmd(std::vector<int>& vec);

    void getData(std::vector<int>& vec);

    int getCheckData();

    void setCmd(std::vector<int>& vec);

    void setData(std::vector<int>& vec);

    void setCheck(std::vector<int>& vec);

    void writeOver1();

    void ReceiveReply(int sig);

private:

    modbus_t* _mb;
    modbus_mapping_t *mb_mapping;
    uint8_t query[MODBUS_MAX_READ_REGISTERS];
    std::shared_ptr<RobotLaser> _laser_ptr;///激光指针
    /// 线程同步锁
    std::recursive_mutex _mtx ;
    std::mutex mtx;
    std::mutex _MTmtx;
public:
    int func_Code;
    int req_len;
    int address;
    int iSignal;
    int checkCode;
    int oldCheckCode;
    bool bDataUpdate;
    bool bOpenOver ;
    bool bGetDone ;
    bool bRotateOver;
    std::queue<_msgObj> msgQueue;
    std::vector<int> recvData;
    std::vector<int> sendData;
    std::condition_variable myCV;
    std::condition_variable mtCV;

    int errCnt;
    int lastCheckCode;

};

#endif // MOTORCONTROL_H
