#ifndef MBCOMMON_H
#define MBCOMMON_H
#include <iostream>
#include <vector>
#include <cstring>
#include "base/errorcode.h"

using namespace std;

enum STATUS{READY,RUN,FINISHED,SUSPEND,FAILED};
enum MOTORTYPE{GROUNDRAIL,ROTATINGDISK,FLIPDISK,ALLRESET};

extern string getCurrentTime();
extern string getDate();
extern double gMotorposs;
extern double gMotorangle1s;
extern double gMotorangle2s;

#define _LOGINFO_ cout<<"["<<getCurrentTime().c_str()<<"]["<<__FILE__<<"]["<<__LINE__<<"]["<<__FUNCTION__<<"]"<<endl;
#define _FUNC_END_ cout<<"";// cout<<"End of "<<__FUNCTION__<<" execution ."<<endl;

#define READADDRESS 0
#define WRITEADDRESS 60
#define CADADDRESS 120
#define SEAMADDRESS 435

#define MSGLEN 60
#define MSGHEADLEN 15
#define MSGBODYLEN 44
#define MSGCHECKLEN 1
#define MSGCADLEN 315
#define MSGSEAMLEN 1800

typedef struct MESSAGEDATA
{
    MESSAGEDATA(){
        bCadData = false;
        bSeamData = false;
        bArray = true;

        memset(array,0,sizeof(array));
        memset(cadArray,0,sizeof(cadArray));
        memset(seamArray,0,sizeof(seamArray));
        seamMsgLen = 0;
    }
    uint16_t array[MSGLEN];
    uint16_t cadArray[MSGCADLEN];
    uint16_t seamArray[MSGSEAMLEN];
    int seamMsgLen;
    bool bCadData;
    bool bSeamData;
    bool bArray;

}_msgData;

typedef struct MESSAGEOBJECT
{

public:
    MESSAGEOBJECT()
    {
        init();
    }
    void init()
    {
        msgCmd = 0;
        msgCheckCode = 0;
        memset(msgHead,0,sizeof(msgHead));
        memset(msgBody,0,sizeof(msgBody));
        bHandMsg = true;
    }
    void load(int16_t (&recvData)[MSGLEN])
    {
        try{
            for(int i = 0; i < MSGHEADLEN; i++){
                msgHead[i] = recvData[i];
                if(recvData[i] != 0){
                    msgCmd = recvData[i];
                }
            }
            for(int i = 0; i < MSGBODYLEN; i++)
                msgBody[i] = recvData[i+MSGHEADLEN];
            for(int i = 0; i < MSGLEN-1; i++){
                if(recvData[i] != 0)
                    bHandMsg = false;
            }
            msgCheckCode =recvData[MSGLEN-1];
        }catch(...){
            cout<<"error: load data failed ."<<endl;
            throw ERR_MODBUS_INVALID_DATA;
        }
    }

public:
    int16_t msgCmd;
    int16_t msgCheckCode;
    std::int16_t msgHead[MSGHEADLEN];
    std::int16_t msgBody[MSGBODYLEN];
    bool bHandMsg;

}_msgObj;


void print_recv_Data(int16_t*array, int count,bool bAll = false);
void print_send_Data(uint16_t*array, int count,bool bAll = false);
#endif // MBCOMMON_H
