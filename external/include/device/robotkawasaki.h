#ifndef _ROBOTKAWASAKI_H
#define _ROBOTKAWASAKI_H


#include <queue>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <boost/format.hpp>

#include "base/config.h"
#include "tools/matrix.h"
#include "device/socketConnect.h"
#include "device/robot.h"
#include "tools/singleton.h"

/// 提供一个机器人控制类 [线程安全]
class RobotKAWASAKI : public Robot
{
public:
    /// Constroctor
    RobotKAWASAKI() ;

    ~RobotKAWASAKI() ;

    virtual void stopMove(RobotError e) override ;

    virtual void resetSockFlag(int sockFlag=0) override ;

    /// 相对路徑运动
    /// @param [in] relativePos 相对于机器人当前位置的位移量
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void relativeMove( RobotPos relativePos, bool bBlock = true, bool bWelding = false ) override;

    /// 相对路徑运动[非欧位角的方式运动]
    /// @param [in] relativePos 相对于机器人当前位置的位移量
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void relativeMove_axis(std::vector<RobotPos> axis_vec, bool bBlock = true, bool bWelding = false) override;

    /// 相对路徑运动[非欧位角的方式运动]
    /// @param [in] axle 相对于机器人当前机器轴坐标的旋转量
    /// @param [in] bBlock 是否等待运动完成再返回
    virtual void relativeMove_axle(RobotAxle axle, bool bBlock = true) override ;

    /// 绝对路徑运动
    /// @param [in] absoluteAxle 目标位置的轴坐标
    /// @param [in] bBlock 是否等待运动完成再返回
    virtual void absoluteMove( RobotAxle absoluteAxle, bool bBlock = true ) override ;

    /// 绝对路徑运动
    /// @param [in] absolutePos 目标位置的绝对坐标
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void absoluteMove( RobotPos absolutePos, bool bBlock = true, bool bWelding = false ) override ;

    /// 绝对路徑运动
    /// @param [in] absolutePath 绝对坐标规划的路徑
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void absoluteMove( std::vector<RobotPos> absolutePath,
                               RobotPos offset=RobotPos::instance(),
                               RobotPos base_offset=RobotPos::instance(),
                               double dr=6,int autoFit=0,
                               weldInfo wf={0,0,0,0,0,0,0,0,0,0,0},
                               bool bBlock = true, bool bWelding = false, bool bFitPath = true ) override ;

    /**
     * @brief absoluteMove
     * @param absolutePath
     * @param bBlock 是否反馈当前进度
     * @details 连续轴运动
     */
    virtual void absoluteMove( std::vector<RobotAxle> absolutePath,bool bBlock = true) override;

    /// 从中间焊接
    /// @param [in] absolutePath 绝对坐标规划的路徑
    /// @param [in] safePos 安全位置
    virtual void weldFromMiddle(std::vector<RobotPos> absolutePath,
                                RobotPos offset=RobotPos::instance(),
                                RobotPos base_offset=RobotPos::instance(),
                                double dr=6,int autoFit=0,
                                weldInfo wf={0,0,0,0,0,0,0,0,0,0,0}, int speed=100, double len=100, bool weld= false) override;

    virtual void cmove(std::vector<RobotPos> poslst) ;
    /// 解除阻塞
    virtual void unlockBlock();
    void sendWeldPoss(int index, RobotPos absolutePos,RobotPos offset,RobotPos base_offset,double dh,int autoFit,weldInfo wf,bool needMove = true);

private :
    /// 相对运动
    /// @param [in] q 机器人运动的相对位移量
    /// @param [in] bWelding 本次运动是否焊接
    virtual void relativeMove( RobotPos relativePos, double speed, double acc, bool bWelding ) override;
    /// 發送机器人指令
    virtual void sendCmd(const boost::system::error_code &ec, std::size_t s) override ;

    /// 是否到达目标位置
    virtual bool isArrived() const override ;

    //    virtual void setPointWeldTrue(std::vector<double> &_weldingRatio,int count =1) override;
    virtual void setPointWeldTrue(int count =1) override;

    virtual void setPointWelding(RobotPos absolutePos, bool bBlock = true) override;
    virtual void setPointWelding2(RobotPos start, RobotPos end, std::vector<double> params, bool bBlock = true) override;

    virtual void setPointWeldFalse() override;

    virtual void setWeldingTrue(weldInfo wf, bool bBlock = true) override;

    virtual void setWeldingFalse() override;

    virtual void setRotateTrue() override;

    virtual void setRotateFalse() override;

    virtual void setWeldingSpeed()override;

    void recvPos(const boost::system::error_code &ec, std::size_t s) ;

    void stopBrake(const boost::system::error_code &ec, std::size_t s) ;

    virtual void resetSpeed(int);

    virtual void resetRobotStatus();

    void sendCMovePos(RobotPos absolutePos);

    void sendCMoveEnd(RobotPos absolutePos);

    void sendUdpCmd(boost::format cmd);

    void runMainControl();
    void runAutoStart2();
private:
    boost::format _moveCmd ;
    SocketConnect socketStop ;
    SocketConnect _socketPos ;
    char _replyCmd[4096] ;
    char _replyPos[4096] ;
    char _replyBrake[4096] ;
    char _sendCmd[256];
    int _currSpeed ;
    int _currAcc ;
    size_t _double ;
    bool _bweldOn ;
    int errCounts;
};

#endif // _ROBOTKAWASAKI_H
