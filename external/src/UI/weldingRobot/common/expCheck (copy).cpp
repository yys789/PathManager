#include "common/expCheck.h"
#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>

using namespace std ;
using namespace Eigen;

#define ERRPORT 10017

expCheck::expCheck()
{
    sendFlag = false;
    connectflag = false;
    udpThread = std::thread([this]{
        try
        {
            int sockIn;
            uint server_addr_length;
            // 使用函数socket()，生成套接字文件描述符；
            if( (sockIn = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
                perror("socket() error");
                exit(1);
            }
            // 通过struct sockaddr_in 结构设置服务器地址和监听端口；
            struct sockaddr_in serveraddr;
            bzero(&serveraddr,sizeof(serveraddr));
            serveraddr.sin_family = AF_INET;
            serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
            serveraddr.sin_port = htons(ERRPORT);
            server_addr_length = sizeof(serveraddr);
            // 使用bind() 函数绑定监听端口，将套接字文件描述符和地址类型变量（struct sockaddr_in ）进行绑定；
            if( bind(sockIn, (struct sockaddr *) &serveraddr, server_addr_length) < 0){
                perror("bind() error");
                exit(1);
            }
            struct sockaddr_in in_addr;
            uint in_len;
            // 接收客户端的数据，使用recvfrom() 函数接收客户端的网络数据；
            while(1)
            {
                in_len = sizeof(sockaddr_in);
                int recv_length = 0;
                recv_length = recvfrom(sockIn, _replyCmd, sizeof(_replyCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                if(recv_length > 0)
                    break;
            }
            int sockOut = socket(AF_INET, SOCK_DGRAM, 0);
            //IPV4  SOCK_DGRAM 数据报套接字（UDP协议）
            if(sockOut < 0)
            {
                throw;
            }
            struct sockaddr_in out_addr;
            out_addr.sin_family = AF_INET;
            out_addr.sin_port=htons(ERRPORT);
            out_addr.sin_addr.s_addr = in_addr.sin_addr.s_addr;
            socklen_t out_len = sizeof(out_addr);
            // 向客户端发送数据，使用sendto() 函数向服务器主机发送数据；
            timeval tv = {0, 200};
            setsockopt(sockIn, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
            while(1)
            {
                std::this_thread::sleep_for(1_ms);  /// 等待
                recvfrom(sockIn, _replyCmd, sizeof(_replyCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                if(strcmp( "", _replyCmd) && strcmp( "heart", _replyCmd))
                    cout<<"_replyCmd:"<<_replyCmd<<endl;
                if(!strcmp( "running", _replyCmd))
                {
                    _status = RUNNING ;
                    cout << "进入running\n" << flush ;
                }
                else if(strcmp( "", _replyCmd) && (_status == RUNNING))
                {
                    _status = STOPPED ;
                    _condStoped.notify_all() ;
                    cout << "进入stopped\n" << flush ;
                }
                ::memset(_replyCmd,0,sizeof(_replyCmd));
                lock_guard<recursive_mutex> lck(_mtx) ;
                if(strcmp(_sendCmd,""))
                {
                    int ret = sendto(sockOut, _sendCmd, sizeof(_sendCmd), 0, (struct sockaddr *) &out_addr, out_len);
                    cout << "sendto:"<<ret<<endl;
                }
                memset(_sendCmd,0,sizeof(_sendCmd));
            }
        }
        catch(...)
        {
            cout<<"機器人異常控制线程异常退出!!!!!!"<<endl;
        }
    });

//            std::thread([this]{
//        us = new QUdpSocket;
//        us->bind(QHostAddress("192.168.1.120"),10017);
//        stopFlag = false;

//        while(!stopFlag)
//        {
//            int psize = us->pendingDatagramSize();
//            if(psize <= 0)
//                continue;
//            QByteArray datagram;
//            datagram.resize(us->pendingDatagramSize());
//            if(sendFlag)
//            {
//                us->readDatagram(datagram.data(),datagram.size(),&robotAddr,&robotPort);
//                us->writeDatagram(sendBuf.data(),sendBuf.size(),robotAddr,10017);
//                cout<<"send data 2 autostart2.pc:"<<QString(sendBuf).toStdString()<<endl;
//                sendFlag = false;
//            }
//            else
//            {
//                us->readDatagram(datagram.data(),datagram.size(),&robotAddr,&robotPort);
//            }
//            if(datagram.size() > 3)
//            {
//                QString QD = QString(datagram);
//                if(QD == "MC OK")
//                {
//                    readBuf = datagram;
//                }
//                cout<<"接收到数据:"<<QD.toLocal8Bit().toStdString()<<endl;
//                QStringList runInfos = QD.split(",");
//                if(runInfos.size() >= 6)
//                {
//                    readBuf = datagram;
//                }
//            }
//        }
//    });
    udpThread.detach();
}

expCheck::~expCheck()
{
    cout<<"~expCheck"<<endl;
    stopFlag = true;
    udpThread.join();
    us->close();
    delete us;
}

void expCheck::sendUdpCmd(boost::format cmd)
{
//    _error = NORMAL ;
//    _status = RUNNING;
    {
        //lock_guard<recursive_mutex> lck(_mtx) ;
        strcpy(_sendCmd,cmd.str().c_str());
    }
//    waitForMoveStoped();
}

void expCheck::getError(QString & msg)
{
    int rec = 3;
    while(rec > 0)
    {
        readBuf = "";
        sendBuf = "error";
        sendFlag = true;
        int times = 10;
        while(readBuf == "" && times >=0)
        {
            std::this_thread::sleep_for(300_ms);
            times--;
        }
        if(readBuf != "")
        {
            break;
        }
        rec--;
    }
    msg = QString(readBuf);
    readBuf = "";
}

bool expCheck::checkStatus(std::vector<int>& sts_vec, QString& msg)
{
    int result = false;
    int rec = 3;
    sts_vec.clear();
    while(rec > 0)
    {
        readBuf = "";
        sendBuf = "error";
        sendFlag = true;
        int times = 10;
        while(readBuf == "" && times >=0)
        {
            std::this_thread::sleep_for(300_ms);
            times--;
        }
        if(readBuf != "")
        {
            break;
        }
        rec--;
    }
    msg = QString(readBuf);
    readBuf = "";
    if(msg == "")
    {
        msg = "机器人通讯重连失败！";
        qCritical() <<msg<<endl;
        return result;
    }else{
        QStringList stsList = msg.split(",");
        for(auto sts : stsList)
            sts_vec.push_back(sts.toInt());
        if(sts_vec.size() < 6)
        {
            sts_vec.clear();
            msg = "获取异常信息失败！";
            qCritical() <<msg<<endl;
            return result;
        }else{
            msg = "机器人";
            if(sts_vec[0] != -1)
                msg += "[处于HOLD状态]";
            if(sts_vec[1] != -1)
                msg += "[控制器处于REPEAT模式]";
            if(sts_vec[2] != 0)
                msg += "[处于急停状态]";
            /// 通知等待机器人停止的条件变量
            std::condition_variable _condStoped ;
            if(sts_vec[3] != 0)
                msg += "[示教器处于REPEAT模式]";
            if(sts_vec[4] != 0)
                msg += "[系统处于异常,见示教器上显示的错误信息]";
            result = true;
        }
    }
    return result;
}

bool expCheck::resetSockFlag()
{
    readBuf = "";
    sendBuf = "sockflag";
    sendFlag = true;
    int times = 10;
    while(readBuf != "sockflag" && times >=0)
    {
        std::this_thread::sleep_for(500_ms);
        times--;
    }
    if(readBuf != "sockflag")
    {
        readBuf = "";
        return false;
    }
    else
    {
        readBuf = "";
        return true;
    }
}

bool expCheck::reconnect()
{
    if(_robot->rgsoStatus == -2)
        return true;
    int rec = 3;
    while(rec > 0)
    {
        readBuf = "";
        sendBuf = "reconnect";
        sendFlag = true;
        int times = 10;
        while(readBuf != "MC OK" && times >=0)
        {
            std::this_thread::sleep_for(500_ms);
            times--;
        }
        if(readBuf == "MC OK")
        {
            break;
        }
        rec--;
    }
    if(readBuf != "MC OK")
    {
        readBuf = "";
        return false;
    }
    else
    {
        readBuf = "";
        return true;
    }
}



bool expCheck::startRobot()
{
    std::this_thread::sleep_for(2000_ms);
    int rec = 3;
    while(rec > 0)
    {
        readBuf = "";
        sendBuf = "start";
        sendFlag = true;
        int times = 10;
        while(readBuf != "MC OK" && times >=0)
        {
            std::this_thread::sleep_for(300_ms);
            times--;
        }
        if(readBuf == "MC OK")
        {
            break;
        }
        rec--;
    }
    if(readBuf != "MC OK")
    {
        readBuf = "";
        connectflag = true;
        return false;
    }
    else
    {
        readBuf = "";
        return true;
    }
}

bool expCheck::disconnect()
{
    std::this_thread::sleep_for(2000_ms);
    int rec = 3;
    while(rec > 0)
    {
        readBuf = "";
        sendBuf = "close";
        sendFlag = true;
        int times = 10;
        while(readBuf != "UDP_CLOSE OK" && times >=0)
        {
            std::this_thread::sleep_for(300_ms);
            times--;
        }
        if(readBuf == "UDP_CLOSE OK")
        {
            break;
        }
        rec--;
    }
    if(readBuf != "UDP_CLOSE OK")
    {
        readBuf = "";
        connectflag = true;
        return false;
    }
    else
    {
        readBuf = "";
        return true;
    }
}
