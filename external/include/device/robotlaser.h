#ifndef ROBOTLASER_H
#define ROBOTLASER_H

#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/timer.hpp>

/// 控制激光类 [线程安全]
class RobotLaser
{
    /// 激光设备状态
    enum Status { LIGHT, FLICKER, CLOSE } ;
public:
    /// Constructor
    RobotLaser();
    ~RobotLaser() ;
public:
    /// 连接设备
    void connectDevice() ;
    /// 断开设备
    void disconnectDevice() ;
    /// 重连设备
    void reconnectDevice() ;
    /// 关闭激光
    void alwaysClose() ;
    /// 常亮
    void alwaysLight( ) ;
    /// 闪烁
    void flicker() ;
    /// 打开LED
    void openLED() ;
    /// 关闭LED
    void closeLED() ;

    void initLight();
private:
    /// 存放激光控制命令
    char _cmd[16] ;

    /// 存放应答指令
    char _readBuffer[16] ;

    /// 线程同步锁
    std::recursive_mutex _mtx ;
    /// 条件变量
    std::condition_variable _condReply ;
    /// 条件变量锁
    std::mutex _mtxReply ;
    /// 是否已经应答
    bool _bReply ;
    /// 记录激光状态
    Status  _laserStatus ;
    /// asio 服务管理
    boost::asio::io_service _service ;

    /// 激光控制通信管理
    std::shared_ptr<boost::asio::serial_port> _serialPort_ptr ;

    bool _bOpenLED ;

    boost::timer _timeout ;

    std::thread _thService ;

    std::size_t _status ;
private:
    /// 设备应答
    void recvReply( const boost::system::error_code &ec, size_t ) ;
    /// 等待设备应答
    void waitReply() ;
};

#endif // ROBOTLASER_H
