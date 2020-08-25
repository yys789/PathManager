#ifndef MBCONTROL_H
#define MBCONTROL_H

#include "modbus/modbus.h"
#include <boost/shared_ptr.hpp>
#include "device/robotlaser.h"
#include <mutex>
#include <queue> 
#include <condition_variable>
#include "mbCommon.h"

class mbControl
{
public:
    mbControl();
    ~mbControl();
public:
    void init();
    void connectDevice();
    void disconnectDevice();
    void reConnectDevice();

    void updateCheckCode();
    void updateProgress(int,int);

    void ReceiveReply(int sig);
    void feedbackErrCode(int errCode);
    void feedbackResult(int result);
    void robotbackDone();

    void reset();
    void checkSensor();
    void checkTcp();
    //    void cleanTorch();
    void cutWire();
    void spraying();
    void openLaser();
    void closeLaser();
    void openAirValve();
    void closeAirValve();

    void send_cad_list(std::vector<uint16_t>& vec);

    void rotateMotor(MOTORTYPE eMotorType,double angle,int speed = 200,int mode = 1);
    void rotate(MOTORTYPE ,double,int speed = 200,int mode = 1);
    void rotateNew(double Groundpos,double speed1,
                   double FlipAngle,double speed2,
                   double RotateAngle3,double speed3,
                   int mode);

    void recv_msg();
    void send_msg();
    int deal_msg();

    void write(uint16_t (&)[MSGLEN]);
    void write_Seam(uint16_t *iSendData,int msglen);
    void write_cad(uint16_t (&)[MSGCADLEN]);

    void read_registers(int address,int count,int16_t *);
    void write_registers(int address,int count,uint16_t *);
    void multi_write_registers(int address,int count,uint16_t *);

    /////test/////
    int get_cadId()const;
    int get_seamId()const;

    void test();

private:
    modbus_t* _mb;
    std::shared_ptr<RobotLaser> _laser_ptr;///激光指针
    /// 线程同步锁
    std::recursive_mutex _mtx ;
    std::mutex mtx;
    std::mutex _MTmtx;
public:
    int address;
    int iSignal;
    int checkCode;
    int oldCheckCode;
    int errCnt;
    bool bOpenOver ;
    bool bGetDone ;
    bool bRotateOver;
    std::int16_t iRecvData[MSGLEN];
    std::uint16_t iSendData[MSGLEN];
    std::queue<_msgData> msgQueue;
    _msgObj obj;
    //_msgData recv;
    std::condition_variable myCV;
    std::condition_variable mtCV;

};

#endif // MOTORCONTROL_H
