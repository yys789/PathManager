// 机器人控制类

#ifndef EXPCHECK_H
#define EXPCHECK_H

#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include "device/robotmanager.h"
#include <QtCore>

class expCheck: public QObject
{
    Q_OBJECT
public:
    explicit expCheck();
    ~expCheck();
    /// 查询异常
    void getError(QString & msg);

    bool checkStatus(std::vector<int>& ,QString&);
    /// 自動鏈接機器人
    bool startRobot();
    /// sockflag
    bool resetSockFlag();
    /// 重新鏈接
    bool reconnect();
    /// 断开链接
    bool disconnect();
    void sendUdpCmd(boost::format cmd);
    QByteArray   readBuf;   //UDP接收缓存区
private slots:
private:
    bool stopFlag;
    std::thread udpThread;
    QUdpSocket * us;
    /// 鏈接狀態
    bool connectflag;
    bool         sendFlag;  // false 不发送 true 发送
    QByteArray   sendBuf;   //UDP发送缓存区
    QHostAddress robotAddr; //UDP机器人地址
    quint16      robotPort; //UDP机器人端口
    /// 配置
    Config _sys_config ;
    /// 机器人管理
    RobotManager _robot ;
    /// udp反饋字符串
    char _replyCmd[4096] ;
    /// udp待發送字符串
    char _sendCmd[256];
    /// 机器人错误信息
    RobotError _error ;
    /// 通知等待机器人停止的条件变量
    std::condition_variable _condStoped ;
    /// 机器人状态
    RobotStatus _status ;
    /// 线程同步锁
    mutable std::recursive_mutex _mtx ;
};

#endif // EXPCHECK_H
