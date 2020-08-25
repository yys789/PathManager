#ifndef AS_UDP_H
#define AS_UDP_H


#include <sys/socket.h>
#include <atomic>
#include "walgo/lasermodel3.h"
#include "math.h"
#include <vector>
#include <queue>
#include <thread>
#include "tools.h"
#include "base.h"
#include "base/wzerror.h"
#include "base/data_types.h"
#include "base/errorcode.h"

#include <queue>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "tools/matrix.h"
#include "tools/singleton.h"
#include <netinet/in.h>

using namespace std ;
using namespace Eigen;
using namespace chrono;

#define DEP 10
#define MAXLINE 1024
#define PORT 10111    /// afeng
#define AUTO1 10000   /// autostart
#define AUTO2 10112   /// autostart2

#define TIMESEC (duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000.0)

class as_udp
{
public:
    as_udp();
    /// 多点运动点数序号
    uint weldIndex;
    /// 是否存udp记录
    bool pathLog;
    RobotPos getCurrPos(RobotTCP tcp=UPLOAD) const;
    RobotAxle getCurrAxle() const;
    void relativeMove(RobotPos relativePos, bool bBlock=true, bool bWelding=false);
    void relativeMove_axis(vector<RobotPos> axis_vec, bool bBlock=true, bool bWelding=false);
    void relativeMove_axle(RobotAxle axle, bool bBlock=true);
    void absoluteMove(RobotAxle absoluteAxle, bool bBlock=true);
    void absoluteMove(RobotPos absolutePos, bool bBlock=true, bool bWelding=false);
    void setPointWeldTrue(int count);
    void setPointWelding(RobotPos absolutePos);
    void setPointWelding2(RobotPos start, RobotPos end, std::vector<double> params);
    void setWeldingTrue(weldInfo wf, bool bBlock = true);
    void setPointWeldFalse();
    void setRotateTrue();
    void setRotateFalse();
    void setWeldingFalse();
    void absoluteMove(vector<RobotPos> absolutePath,RobotPos offset=RobotPos::instance(),
                      RobotPos base_offset=RobotPos::instance(),double dr=6,int autoFit=0,
                      weldInfo wf={0,0,0,0,0,0,0,0,0,0,0});
    void absoluteMove(vector<RobotAxle> absolutePath);
    void weldAutoSet(double h, weldInfo & wf);
    void weldAutoSet2(RobotPos p, weldInfo & wf);
    void sendWeldPoss(int index, RobotPos absolutePos,
                              RobotPos offset,RobotPos base_offset,double dh,
                              int autoFit,weldInfo wf,bool needMove = true);
    void setInTCP( RobotTCP tcp ) ;
    void setSpeed(double spd=100, double acc=80);
    RobotTCP getInTCP() const ;
private:
    void sendUdpCmd(char * cmd);
    void afeng();
    void autoStart();
    void autoStart2();
    char _sendCmd[512];
    char up10000[256] ;
    char up10111[256] ;
    char up10112[256] ;
    /// 机器人TCP点的选取
    RobotTCP _tcp ;
    /// 私服状态
    int rgsoStatus;
    /// 更新焊接参数标志
    bool updateparams;
    /// 机器人状态
    RobotStatus _status ;
    /// 机器人错误信息
    RobotError _error ;
    /// 线程同步锁
    mutable std::recursive_mutex _mtx ;
    /// 条件变量使用的锁
    std::mutex _mtxStoped ;
    /// 通知等待机器人停止的条件变量
    std::condition_variable _condStoped ;
    /// 是否焊接
    bool _bWeldingOn ;
    /// 记录机器人当前机器轴的坐标
    RobotAxle _currAxle;
    /// 机器人当前位置，用于线程间交互
    RobotPos _currPos;
    Config _sys_config;
    bool _bweldOn ;
};

#endif // AS_UDP_H
