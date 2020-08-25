#ifndef RobotKUKA_H
#define RobotKUKA_H

#include <queue>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "walgo/point.h"
#include "base/config.h"
#include "tools/matrix.h"
#include "device/socketConnect.h"
#include "device/robot.h"

#include "tools/singleton.h"

/// 提供一个机器人控制类 [线程安全]
class RobotKUKA : public Robot
{
public:
    RobotKUKA() ;

    ~RobotKUKA() ;
    /// Constroctor

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

    virtual void fitPath( std::vector<RobotPos> &absolutePath ) override ;

    /// 绝对路徑运动
    /// @param [in] absolutePath 绝对坐标规划的路徑
    /// @param [in] bWelding true 表示在下一次移动时进行焊接
    virtual void absoluteMove( std::vector<RobotPos> absolutePath,
                               RobotPos offset={0,0,0,0,0,0,UPLOAD},
                               RobotPos base_offset={0,0,0,0,0,0,UPLOAD},
                               double dr=6,int autoFit=0, weldInfo wf={0,0,0,0,0,0,0,0,0},
                               bool bBlock = true, bool bWelding = false, bool bFitPath = true ) override ;
private :
    /// 相对运动
    /// @param [in] q 机器人运动的相对位移量
    /// @param [in] bWelding 本次运动是否焊接
    virtual void relativeMove( std::shared_ptr<std::queue<RobotPos>> q, bool bWelding = false) override;

    /// 發送机器人指令
    virtual void sendCmd(const boost::system::error_code &ec, std::size_t s) override ;

    /// 是否到达目标位置
    virtual bool isArrived() const override ;
};

#endif // RobotKUKA_H
