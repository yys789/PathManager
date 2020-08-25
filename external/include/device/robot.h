#ifndef _ROBOT_H
#define _ROBOT_H


#include <chrono>
#include <queue>
#include <condition_variable>

#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>

#include "base/data_types.h"
#include "base/config.h"
#include "device/socketConnect.h"

class Robot
{
public:
    /// Constroctor
    Robot();

    /// 析构函数
    virtual ~Robot() ;

    /// 获取机器人当前位置 [线程安全]
    /// @return 机器人当前位置
    RobotPos getCurrPos(RobotTCP = RobotTCP::UPLOAD) const ;

    /// 获取机器人当前机器轴的位置
    /// @return 机器人当前机器轴的位置
    RobotAxle getCurrAxle() const ;

    /// 获取机器人当前状态
    /// @return 机器人当前状态
    RobotStatus getStatus() const ;

    /// 获取机器人错误信息
    RobotError getLastError() const ;

    /// 相对路徑运动
    /// @param [in] relativePos 相对于机器人当前位置的位移量
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void relativeMove( RobotPos relativePos, bool bBlock = true, bool bWelding = false ) = 0 ;

    /// 相对路徑运动[非欧位角的方式运动]
    /// @param [in] relativePos 相对于机器人当前位置的位移量
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void relativeMove_axis(std::vector<RobotPos> axis_vec, bool bBlock = true, bool bWelding = false) = 0 ;

    /// 相对路徑轴运动
    /// @param [in] axle 相对于机器人当前机器轴坐标的旋转量
    /// @param [in] bBlock 是否等待运动完成再返回
    virtual void relativeMove_axle(RobotAxle axle, bool bBlock = true) = 0 ;

    /// 绝对路徑轴运动
    /// @param [in] absoluteAxle 目标位置的轴坐标
    /// @param [in] bBlock 是否等待运动完成再返回
    virtual void absoluteMove( RobotAxle absoluteAxle, bool bBlock = true ) = 0 ;

    /// 绝对路徑运动
    /// @param [in] absolutePos 目标位置的绝对坐标
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void absoluteMove( RobotPos absolutePos, bool bBlock = true, bool bWelding = false ) = 0 ;

    virtual void fitPath( std::vector<RobotPos> &){}

    /// 绝对路徑运动
    /// @param [in] absolutePath 绝对坐标规划的路徑
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void absoluteMove( std::vector<RobotPos> absolutePath,
                               RobotPos offset=RobotPos::instance(),
                               RobotPos base_offset=RobotPos::instance(),
                               double dr=6,int autoFit=0,
                               weldInfo wf={0,0,0,0,0,0,0,0,0,0,0},
                               bool bBlock = true, bool bWelding = false, bool bFitPath = true ) = 0 ;

    /**
     * @brief absoluteMove
     * @param absolutePath
     * @param bBlock
     * @details 连续轴运动
     */
    virtual void absoluteMove( std::vector<RobotAxle> absolutePath,bool bBlock = true) = 0;
    /// 从中间焊接
    /// @param [in] absolutePath 绝对坐标规划的路徑
    /// @param [in] safePos 安全位置
    virtual void weldFromMiddle(std::vector<RobotPos> absolutePath,
                                RobotPos offset=RobotPos::instance(),
                                RobotPos base_offset=RobotPos::instance(),
                                double dr=6,int autoFit=0,
                                weldInfo wf={0,0,0,0,0,0,0,0,0,0,0}, int speed=100, double len=100, bool weld = false) = 0;

    //    virtual void setPointWeldTrue(std::vector<double> &_weldingRatio,int count =1) = 0;
    virtual void setPointWeldTrue(int count =1) = 0;

    virtual void setPointWelding(RobotPos absolutePos, bool bBlock = true) = 0;

    virtual void setPointWelding2(RobotPos start, RobotPos end, std::vector<double> params, bool bBlock = true) = 0;
    virtual void setPointWeldFalse() = 0;

    virtual void setWeldingTrue(weldInfo wf, bool bBlock = true) = 0;

    virtual void setWeldingFalse() = 0;

    virtual void setRotateTrue() = 0;

    virtual void setRotateFalse() = 0;

    virtual void setWeldingSpeed() = 0;

    virtual void resetSockFlag(int sockFlag=0) = 0 ;

    virtual void resetSpeed(int) = 0;
    virtual void sendWeldPoss(int index, RobotPos absolutePos,RobotPos offset,RobotPos base_offset,double dr,int autoFit,weldInfo wf,bool needMove = true) = 0;
    virtual void cmove(std::vector<RobotPos> poslst) = 0;
    /// 等待机器人停止运动
    /// @param [in] ms 等待的时间
    /// @return 表示是否停止运动
    bool waitForMoveStoped(std::chrono::milliseconds ms = std::chrono::milliseconds(0)) ;

//    /// 开始焊接 [机器人下次运动时生效,运动完则停止]
//    void startWelding() ;

    /// 强制焊接
    void forceWelding() ;

    /// 停止焊接
    void stopWelding() ;

    /// 设置内部的TCP点
    void setInTCP( RobotTCP tcp ) ;

    /// 设置外部的TCP点
    void setOutTCP( RobotTCP tcp ) ;

    /// 获取内部的TCP点
    RobotTCP getInTCP() const ;

    /// 获取外部的TCP点
    RobotTCP getOutTCP() const ;

    /// UDP 服务关闭
    void stop() ;

    /// UDP 服务开启
    void run() ;

    /// 停止運動
    /// @param [in] e 设定停止错误码
    virtual void stopMove(RobotError e) ;

    /// 是否到达目标位置
    virtual bool isArrived() const = 0 ;


    virtual void unlockBlock() = 0;

    virtual void resetRobotStatus() = 0;

    /// 是否记录位置
    bool pathLog;
public:
    /// 机器人位置变化时发送通知消息
    /// @param [in] newPos 最新的位置
    boost::signals2::signal<void (RobotPos newPos)> posChanged_sig ;

    boost::signals2::signal<void ()> weldStoped_sig ;

    /// 通知焊机已经停止
    boost::signals2::signal<void ()> weldingStoped_sig ;

    /// 机器人开始移动时发送通知
    boost::signals2::signal<void ()> moveStarted_sig ;

    /// 停止運動
    boost::signals2::signal<void ()> moveStoped_sig ;

    /**
     * @brief successived_sig
     * @details 多点运动情况下反馈执行结果 [arg]当前循环在第几阶段
     * @warning 当前仅在多轴连续运动情况下开启了进度反馈
     */
    boost::signals2::signal<void (int)> successived_sig ;

    /// 机器轴发生变化时发送通知
    /// @param [in] newAxle 最新的机器轴位置
    boost::signals2::signal<void (RobotAxle newAxle)> axleChanged_sig ;
    /// 私服状态
    int rgsoStatus;
    /// 更新焊接参数标志
    bool updateparams;
    /// 多点运动点数序号
    uint weldIndex;
protected:
    /// 相对运动
    /// @param [in] q 机器人运动的相对位移量
    /// @param [in] bWelding 本次运动是否焊接
    virtual void relativeMove( std::shared_ptr<std::queue<RobotPos>> q, bool bWelding = false ){}

    virtual void relativeMove(RobotPos relativePos, double speed, double acc, bool bWelding){}

    // virtual void relativeMove(const std::string &cmd, bool bWelding){}

    /// 發送机器人指令
    //virtual void sendCmd( std::shared_ptr<boost::asio::ip::udp::endpoint> _ep_ptr, const boost::system::error_code &ec, std::size_t s ){
    //   std::cout << "sendCmd ep_tr 调用" << std::endl ;
    //}

    virtual void sendCmd(const boost::system::error_code &ec, std::size_t s) = 0 ;

protected:
    /// 机器人通信服务
    SocketConnect _socketConnect ;
    /// 通信类
    /// 通信缓冲
    char _recvBuffer[4096] = { 0 } ;

    /// 机器人运动路徑，为空则机器人停止
    std::queue<RobotPos> _queuePos ;
    /// 机器轴旋转
    std::queue<RobotAxle> _queueAxle ;
    /// 机器人当前位置，用于线程间交互
    std::atomic<RobotPos> _currPos;
    /// 机器人当前位置，用于外部接口调用
    std::atomic<RobotPos> _outPos;
    /// 记录机器人初始位置
    RobotPos _firstPos ;
    /// 记录机器人上一次的位置
    RobotPos _lastPos ;
    /// 记录机器人当前机器轴的坐标
    std::atomic<RobotAxle> _currAxle;
    /// 是否焊接
    bool _bWeldingOn ;
    /// 机器人运动是否停止
    bool _bStoped ;
    /// 线程同步锁
    mutable std::recursive_mutex _mtx ;
    /// 通知等待机器人停止的条件变量
    std::condition_variable _condStoped ;
    /// 条件变量使用的锁
    std::mutex _mtxStoped ;
    /// 是否强制起弧
    bool _bForceWelding;
    /// 配置
    Config _sys_config ;

    Config _calibration_config ;
    /// 机器人状态
    RobotStatus _status ;
    /// 机器人错误信息
    RobotError _error ;
    /// 机器人TCP点的选取
    RobotTCP _tcp ;
    /// RSITCP点的选取
    RobotTCP _rsiTcp ;
protected:
    /// 获取当前TCP点的坐标
    RobotPos in2OutPos(RobotTCP tcp) const ;
    /// 获取当前TCP点的坐标
    RobotPos in2OutPos(RobotTCP tcp, RobotPos) const ;

    /// 外部TCP坐标转为内部当前TCP坐标
    /// @param [in] pos 外部TCP坐标
    /// @param [in] tcp 外部选择的TCP点
    RobotPos out2InPos( RobotPos pos ) ;

} ;

#endif // _ROBOT_H
