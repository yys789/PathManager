#include "device/robotkawasaki.h"
#include "walgo/lasermodel3.h"
#include "math.h"
#include <vector>
#include <queue>
#include <thread>

#include "tools.h"
#include "base.h"
#include "base/wzerror.h"
#include "base/data_types.h"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "base/errorcode.h"

using namespace boost::asio ;
using boost::asio::ip::udp ;
using namespace std ;
using namespace Eigen;
using namespace chrono;

#define DEP 10
#define MAXLINE 1024
#define PORT 10111
#define AUTO 10112

#define TIMESEC (duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000.0)

void RobotKAWASAKI::sendUdpCmd(boost::format cmd)
{
    _error = NORMAL ;
    _status = RUNNING;
    {
        lock_guard<recursive_mutex> lck(_mtx) ;
        strcpy(_sendCmd,cmd.str().c_str());
    }
    waitForMoveStoped();
}

RobotKAWASAKI::RobotKAWASAKI()
    :_replyCmd{},
      _replyPos{},
      _bweldOn{false}
{
    memset(_sendCmd,0,sizeof(_sendCmd));
    pathLog = false;
    _double = 0;
    _tcp = (RobotTCP)_sys_config.get<int>("robot.currentTCP", RobotTCP::UPLOAD) ;
    _rsiTcp = (RobotTCP)_sys_config.get<int>("robot.currentRSITCP", RobotTCP::FLANGE) ;
    udp::endpoint ep(udp::v4(), _sys_config.get<int>("robot.port")) ;
    std::cout << "ip port:" << _sys_config.get<std::string>("robot.IPAddr") << ' '
              << _sys_config.get<int>("robot.port") << "\n" << std::flush ;


    udp::endpoint eppp(udp::v4(), 10019) ;
    socketStop.reset(SocketConnectType::UDPCLIENT, eppp) ;
    socketStop.async_read(buffer(_replyBrake),boost::bind(&RobotKAWASAKI::stopBrake, this,_1,_2)) ;
    socketStop.run() ;

    // 位置更新
    _socketPos.reset(SocketConnectType::UDPSERVER, ep);
    _socketPos.async_read(buffer(_replyPos), boost::bind(&RobotKAWASAKI::recvPos, this, _1, _2)) ;
    _socketPos.run() ;

    // UDP通信运动控制主线程
    runMainControl();

    // autostart2.pc 服务
    runAutoStart2();
}

RobotKAWASAKI::~RobotKAWASAKI()
{
}

void RobotKAWASAKI::runMainControl()
{
    std::thread([this]{
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
            serveraddr.sin_port = htons(PORT);
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
            out_addr.sin_port=htons(PORT);
            out_addr.sin_addr.s_addr = in_addr.sin_addr.s_addr;
            socklen_t out_len = sizeof(out_addr);
            // 向客户端发送数据，使用sendto() 函数向服务器主机发送数据；
            timeval tv = {0, 200};
            setsockopt(sockIn, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
            while(1)
            {
                std::this_thread::sleep_for(1_ms);  /// 等待
                recvfrom(sockIn, _replyCmd, sizeof(_replyCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                if(!strcmp( "running", _replyCmd))
                {
                    _status = RUNNING ;
                    cout << "进入running\n" << flush ;
                }
                else if((strcmp( "heart", _replyCmd) == 0 || strcmp( "stopped", _replyCmd) ==0) && ( _status == RUNNING ))
                {
                    _status = STOPPED ;
                    _condStoped.notify_all() ;
                    cout<<_replyCmd << "进入stopped\n" << flush ;
                }
                ::memset(_replyCmd,0,sizeof(_replyCmd));
                lock_guard<recursive_mutex> lck(_mtx) ;
                if(strcmp(_sendCmd,""))
                {
                    sendto(sockOut, _sendCmd, sizeof(_sendCmd), 0, (struct sockaddr *) &out_addr, out_len);
                }
                memset(_sendCmd,0,sizeof(_sendCmd));
            }
        }
        catch(...)
        {
            cout<<"UDP通信运动控制主线程异常退出!!!!!!"<<endl;
        }
    }).detach() ;
}

void RobotKAWASAKI::runAutoStart2()
{
    std::thread([this]{
        try
        {
            int sockIn = socket(AF_INET, SOCK_DGRAM, 0);
            if(sockIn < 0)
                exit(1);
            sockaddr_in serveraddr;
            bzero(&serveraddr,sizeof(serveraddr));
            serveraddr.sin_family = AF_INET;
            serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
            serveraddr.sin_port = htons(AUTO);
            uint server_addr_length = sizeof(serveraddr);
            if( bind(sockIn, (struct sockaddr *) &serveraddr, server_addr_length) < 0){
                exit(1);
            }
            sockaddr_in in_addr;
            uint in_len;
            char _autoCmd[8];
            while(1)
            {
                in_len = sizeof(sockaddr_in);
                int recv_length = recvfrom(sockIn, _autoCmd, sizeof(_autoCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                if(recv_length > 0)
                    break;
            }
            int sockOut = socket(AF_INET, SOCK_DGRAM, 0);
            if(sockOut < 0)
                exit(1);
            sockaddr_in out_addr;
            out_addr.sin_family = AF_INET;
            out_addr.sin_port = htons(AUTO);
            out_addr.sin_addr.s_addr = in_addr.sin_addr.s_addr;
            socklen_t out_len = sizeof(out_addr);
            // 向客户端发送数据，使用sendto() 函数向服务器主机发送数据；
            timeval tv = {0, 1000};
            setsockopt(sockIn, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
            char _autoReplyCmd[64];
            while(1)
            {
                std::this_thread::sleep_for(1_ms);  /// 等待10ms
                recvfrom(sockIn, _autoReplyCmd, sizeof(_autoReplyCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                if(!strcmp(_autoReplyCmd,""))/// 未收到反馈
                    continue;
                if(!strcmp( "heart", _autoReplyCmd)){}
                else if(!strcmp( "MC OK", _autoReplyCmd))
                {
                    cout<<_autoReplyCmd<<endl;
                    _status = STOPPED;
                }else if(!strcmp( "MC ERR", _autoReplyCmd))
                {
                    cout<<_autoReplyCmd<<endl;
                }
                else
                {
                    // cout<<"RUN,REPEAT,EMERGENCY,TEACH_LOCK,ERROR,TIMER(10):"<<_autoReplyCmd<<endl;
                    stringstream rretet(_autoReplyCmd);
                    int run,repeat,emergency,teach,errs,timer;
                    rretet>>run>>repeat>>emergency>>teach>>errs>>timer;
                    if(run == 1 && repeat == 1 && emergency == 0 && teach == 0 && errs == 0)
                    {
                        strcpy(_autoCmd,"1");
                    }
                }
                if(_status == ERROR && strcmp(_autoCmd,"") == 0)
                {
                    strcpy(_autoCmd,"0");
                }
                if(strcmp(_autoCmd,""))
                {
                    sendto(sockOut, _autoCmd, sizeof(_autoCmd), 0, (struct sockaddr *) &out_addr, out_len);
                    memset(_autoCmd,0,sizeof(_autoCmd));
                }
                memset(_autoReplyCmd,0,sizeof(_autoReplyCmd));
            }
        }
        catch(...)
        {
            cout<<"UDP 自启动、状态查询 异常结束!!!!!!"<<endl;
        }
    }).detach() ;
}

void RobotKAWASAKI::stopMove(RobotError e)
{
    string strStop = "brake" ;
    //udp::endpoint epStop(boost::asio::ip::address::from_string("192.168.1.2"), 49153);
    //    SocketConnect socketStop(SocketConnectType::UDPSERVER, "192.168.1.2", 49153) ;
    cout << strStop << "\n" << flush ;
    socketStop.write(buffer(strStop)) ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( SERVICESTOP == _error ) return ;
    _error = e ;
    _condStoped.notify_all();
}

void RobotKAWASAKI::relativeMove(RobotPos relativePos, bool bBlock, bool bWelding )
{
    try
    {
        cout<<"relativeMove 1"<<endl;
        auto targetPos = in2OutPos(_tcp, _currPos) + relativePos ;
        absoluteMove(targetPos, bBlock, bWelding) ;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::relativeMove_axis(vector<RobotPos> axis_vec, bool bBlock, bool bWelding)
{
    try
    {
        RobotPos pointInFlange = getPosInFlange(_tcp) ;
        RobotPos targetPos{} ;
        pointRotate(in2OutPos(RobotTCP::UPLOAD, _currPos), pointInFlange, axis_vec, targetPos);
        cout << "target:" << targetPos.toStr() << endl ;
        // 生成路徑
        absoluteMove( targetPos, bBlock, bWelding ) ;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::relativeMove_axle(RobotAxle axle, bool bBlock)
{
    try
    {
        if ( RUNNING == _status ) {
            BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
        }
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        // 设置速度加速度
        short iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            sendUdpCmd(_moveCmd);//6,
        }
        _moveCmd = boost::format( "4,%1%,%2%,%3%,%4%,%5%,%6%," ) ;
        _moveCmd % axle.a1 % axle.a2 % axle.a3 % axle.a4 % axle.a5 % axle.a6 ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]轴相对运动:" << _moveCmd.str() << "\n" << flush ;
        sendUdpCmd(_moveCmd);//4,

    }catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::absoluteMove(RobotAxle absoluteAxle, bool bBlock)
{
    if(abs(absoluteAxle.a1) < 0.1 && abs(absoluteAxle.a2) < 0.1
            && abs(absoluteAxle.a3) < 0.1 && abs(absoluteAxle.a4) < 0.1
            && abs(absoluteAxle.a5) < 0.1 && abs(absoluteAxle.a6) < 0.1)
        return;
    cout << "轴旋转:" << absoluteAxle.toStr() << std::endl ;
    try
    {
        if ( RUNNING == _status ) {
            BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
        }
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        // 设置速度加速度
        short iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            //_socketConnect.write(buffer(_moveCmd.str())) ;
            sendUdpCmd(_moveCmd);//6,
        }
        _moveCmd = boost::format( "3,%1%,%2%,%3%,%4%,%5%,%6%," ) ;
        _moveCmd % absoluteAxle.a1 % absoluteAxle.a2 % absoluteAxle.a3
                % absoluteAxle.a4 % absoluteAxle.a5 % absoluteAxle.a6 ;
        sendUdpCmd(_moveCmd);//3,
    }catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::absoluteMove(RobotPos absolutePos, bool bBlock, bool bWelding)
{
    try
    {
        cout<<"absoluteMove 1"<<endl;
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _error = NORMAL ;
        _status = RUNNING ;
        cout<<"bWelding = "<<bWelding <<endl;
        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        // 生成路徑
        int iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
        cout<<"iSpeed = "<<iSpeed <<endl;
        cout<<"iAcc = "<<iAcc <<endl;
        cout<<"_currSpeed = "<<_currSpeed <<endl;
        cout<<"_currAcc = "<<_currAcc <<endl;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            cout << "设置速度加速度:" << _moveCmd.str() << "\n" << flush ;
            sendUdpCmd(_moveCmd); //6,
        }
        if(!bWelding)
            _moveCmd = boost::format( "1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        else
            _moveCmd = boost::format( "91,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z % absolutePos.a % absolutePos.b % absolutePos.c ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]绝对运动:" << _moveCmd.str() << "\n" << flush ;
        _error = NORMAL ;
        _status = RUNNING ;
        sendUdpCmd(_moveCmd); //1,
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

/// 解除阻塞
void RobotKAWASAKI::unlockBlock()
{
    char msg[64];
    sprintf(msg,"%.3f",getMicTime());
    cout<<msg<<" ";
    __LINE__;
    _status = STOPPED;
    _condStoped.notify_all();
}

void RobotKAWASAKI::resetRobotStatus()
{
    char msg[64];
    sprintf(msg,"%.3f",getMicTime());
    cout<<msg<<" ";
    __LINE__;
    _error = NORMAL ;
    unlockBlock();
}

void RobotKAWASAKI::setPointWeldTrue(int count)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        Config comminfo("etc/common.info");
        string objectname = comminfo.get<string>("object");
        Config _cad_config(objectname);
        auto currMotorType = _cad_config.get<string>("curr_motor");
        auto currSeam = _cad_config.get<string>(currMotorType + ".startSeam");
        struct weldOpt
        {
            double S_W ;
            double I_W ;
            double V_W ;
            double t_end ;
            double I_end ;
            double V_end ;
        } weldInfo;
        weldInfo.S_W = 0;
        weldInfo.I_W = 0;
        weldInfo.V_W = 0;
        weldInfo.t_end = _cad_config.get<double>(currMotorType + "_" + currSeam + "_part0.pointWeldingArg" + to_string(count) + ".t_w",1);
        weldInfo.I_end = _cad_config.get<double>(currMotorType + "_" + currSeam + "_part0.pointWeldingArg" + to_string(count) + ".I_w",150);
        weldInfo.V_end = _cad_config.get<double>(currMotorType + "_" + currSeam + "_part0.pointWeldingArg" + to_string(count) + ".V_w",15);

        _moveCmd = boost::format( "15,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % weldInfo.t_end % weldInfo.I_end % weldInfo.V_end % weldInfo.S_W % weldInfo.I_W % weldInfo.V_W ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]打开点焊开关（不运动）:" << _moveCmd.str() << "\n" << flush ;
        sendUdpCmd(_moveCmd);//15
        _error = NORMAL ;
        _status = RUNNING ;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setPointWelding(RobotPos absolutePos, bool bBlock)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _error = NORMAL ;
        _status = RUNNING ;
        // 做TCP点的转换
        auto currPos = in2OutPos(_tcp, _currPos);
        auto moveSpeed = _sys_config.get<double>("robot.speed");
        auto time=  sqrt(pow((absolutePos.x-currPos.x),2)+pow((absolutePos.y-currPos.y),2)+pow((absolutePos.z-currPos.z),2))/moveSpeed;
        time=ceil(time+3);
        _moveCmd = boost::format( "15,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z % absolutePos.a % absolutePos.b % absolutePos.c ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]绝对运动:" << _moveCmd.str() << "\n" << flush ;
        sendUdpCmd(_moveCmd);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}


void RobotKAWASAKI::setPointWelding2(RobotPos start, RobotPos end, std::vector<double> params,  bool bBlock)
{
    if(params.size() < 9)
        return;
    try
    {
        _error = NORMAL ;
        _status = RUNNING ;
        _moveCmd = boost::format( "16,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\
%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\
%.3f,%.3f,%.3f,\
%.3f,%.3f,%.3f,\
%.3f,%.3f,%.3f," ) ;
        _moveCmd % start.x % start.y % start.z % start.a % start.b % start.c
                 % end.x % end.y % end.z % end.a % end.b % end.c
                % params[0] % params[1] % params[2]
                % params[3] % params[4] % params[5]
                % params[6] % params[7] % params[8];
        sendUdpCmd(_moveCmd);//16,
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setPointWeldFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _error = NORMAL ;
        _status = RUNNING ;
        RobotPos currPos = getPosInFlange(_tcp);
        _moveCmd = boost::format( "8,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % currPos.x % currPos.y % currPos.z % currPos.a % currPos.b % currPos.c ;
        sendUdpCmd(_moveCmd);//8,
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]关闭焊接开关（不运动）:" << _moveCmd.str() << "\n" << flush ;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setWeldingTrue(weldInfo wf, bool bBlock)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _moveCmd = boost::format( "7,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % wf.S_W % wf.I_W % wf.V_W % wf.T_END % wf.I_END % wf.V_END
                % wf.range % wf.frequency % wf.serialNumber ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__
             << "]打开焊接开关（不运动）:" << _moveCmd.str() << "\n" << flush ;
        sendUdpCmd(_moveCmd);//7,
        _bweldOn = true;
        updateparams = false;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setRotateTrue()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        sendUdpCmd(boost::format( "51,1" ));//51,
        cout<<"rotate start comd send"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setRotateFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        sendUdpCmd(boost::format( "52,1" ));//52,
        cout<<"rotate end comd send"<<endl;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::setWeldingFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _moveCmd = boost::format( "8,0" ) ;
        sendUdpCmd(_moveCmd);//8,
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]关闭焊接开关（不运动）:" << _moveCmd.str() << "\n" << flush ;
        _bweldOn = false;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }

}

void RobotKAWASAKI::setWeldingSpeed()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }

        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        // 生成路徑
        int iSpeed = _sys_config.get<double>("robot.speed");
        int iAcc = _sys_config.get<double>("robot.acc") ;

        _currSpeed = 80 ;
        _currAcc = 50 ;
        _moveCmd = boost::format("6,%1%,%2%,") ;
        _moveCmd % _currSpeed % _currAcc ;
        cout << "设置速度加速度:" << _moveCmd.str() << "\n" << flush ;
        //_socketConnect.write(buffer(_moveCmd.str())) ;
        sendUdpCmd(_moveCmd);//6,
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::absoluteMove(vector<RobotPos> absolutePath,RobotPos offset,RobotPos base_offset,double dr,int autoFit,
                                 weldInfo wf, bool bBlock, bool bWelding, bool bFitPath)
{
    try
    {
        int vecsize = absolutePath.size();
        if(vecsize < 2)
        {
            cout<<"点数太少!"<<endl;
            return;
        }
        if(vecsize < 3)
        {
            absoluteMove(absolutePath[1]);
            return;
        }
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }

        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        // 生成路徑
        int iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
        cout<<"iSpeed = "<<iSpeed <<endl;
        cout<<"iAcc = "<<iAcc <<endl;
        cout<<"_currSpeed = "<<_currSpeed <<endl;
        cout<<"_currAcc = "<<_currAcc <<endl;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            cout << "设置速度加速度:" << _moveCmd.str() << "\n" << flush ;
            //_socketConnect.write(buffer(_moveCmd.str())) ;
            sendUdpCmd(_moveCmd);//6,
        }
        if(_bweldOn)
        {
            bool wStatus = absolutePath[0].weld; /// 起始点需不需要焊接
            int psize = absolutePath.size();
            int i=0;
            while(i<psize)
            {
                int d=i;
                vector<RobotPos> pathi;
                for(;d<psize;d++)
                {
                    auto p = absolutePath[d];
                    pathi.push_back(p);
                    if(p.weld != wStatus)
                    {
                        i=d+1;
                        wStatus = p.weld;
                        break;
                    }
                }
                i=d;
                if(i<psize)
                {
                    if(wStatus) /// 说明之前是空走
                    {
                        setWeldingFalse();
                    }
                    else
                    {
                        setWeldingTrue(wf);
                    }
                    pathi[0].z_x = pathi[1].x-pathi[0].x;  /// 前进方向
                    pathi[0].z_y = pathi[1].y-pathi[0].y;
                    pathi[0].z_z = pathi[1].z-pathi[0].z;
                    sendWeldPoss(0,pathi[0],offset,base_offset,dr,autoFit,wf);
                    double dep = 0;
                    for(int p=1;p<pathi.size()-1;p++)
                    {
                        point3d<double> ps = {pathi[p-1].x,pathi[p-1].y,pathi[p-1].z};
                        point3d<double> pe = {pathi[p].x,pathi[p].y,pathi[p].z};
                        dep += pe.dist(ps);
                        if(dep < DEP)
                            continue;
                        pathi[p].z_x = pathi[p+1].x-pathi[p].x;  /// 前进方向
                        pathi[p].z_y = pathi[p+1].y-pathi[p].y;
                        pathi[p].z_z = pathi[p+1].z-pathi[p].z;
                        sendWeldPoss(p,pathi[p],offset,base_offset,dr,autoFit,wf);
                        dep = 0;
                    }
                    pathi[pathi.size()-1].z_x = pathi[pathi.size()-1].x-pathi[pathi.size()-2].x;  /// 前进方向
                    pathi[pathi.size()-1].z_y = pathi[pathi.size()-1].y-pathi[pathi.size()-2].y;
                    pathi[pathi.size()-1].z_z = pathi[pathi.size()-1].z-pathi[pathi.size()-2].z;
                    sendWeldPoss(9999,pathi[pathi.size()-1],offset,base_offset,dr,autoFit,wf);
                    if(wStatus) /// 说明之前是空走
                    {
                        setWeldingTrue(wf);
                    }
                    else
                    {
                        setWeldingFalse();
                    }
                }
                else
                {
                    pathi[0].z_x = pathi[1].x-pathi[0].x;  /// 前进方向
                    pathi[0].z_y = pathi[1].y-pathi[0].y;
                    pathi[0].z_z = pathi[1].z-pathi[0].z;
                    sendWeldPoss(0,pathi[0],offset,base_offset,dr,autoFit,wf);
                    double dep = 0;
                    for(int p=1;p<pathi.size()-1;p++)
                    {
                        point3d<double> ps = {pathi[p-1].x,pathi[p-1].y,pathi[p-1].z};
                        point3d<double> pe = {pathi[p].x,pathi[p].y,pathi[p].z};
                        dep += pe.dist(ps);
                        if(dep < DEP)
                            continue;
                        pathi[p].z_x = pathi[p+1].x-pathi[p].x;  /// 前进方向
                        pathi[p].z_y = pathi[p+1].y-pathi[p].y;
                        pathi[p].z_z = pathi[p+1].z-pathi[p].z;
                        sendWeldPoss(p,pathi[p],offset,base_offset,dr,autoFit,wf);
                        dep = 0;
                    }
                    pathi[pathi.size()-1].z_x = pathi[pathi.size()-1].x-pathi[pathi.size()-2].x;  /// 前进方向
                    pathi[pathi.size()-1].z_y = pathi[pathi.size()-1].y-pathi[pathi.size()-2].y;
                    pathi[pathi.size()-1].z_z = pathi[pathi.size()-1].z-pathi[pathi.size()-2].z;
                    sendWeldPoss(9999,pathi[pathi.size()-1],offset,base_offset,dr,autoFit,wf);
                }
                if(d == psize)
                {
                    break;
                }
            }
            setWeldingFalse();
        }
        else
        {
            absolutePath[0].z_x = absolutePath[1].x-absolutePath[0].x;  /// 前进方向
            absolutePath[0].z_y = absolutePath[1].y-absolutePath[0].y;
            absolutePath[0].z_z = absolutePath[1].z-absolutePath[0].z;
            sendWeldPoss(0,absolutePath[0],offset,base_offset,dr,autoFit,wf);
            double dep = 0;
            for(int p=1;p<vecsize-1;p++)
            {
                point3d<double> ps = {absolutePath[p-1].x,absolutePath[p-1].y,absolutePath[p-1].z};
                point3d<double> pe = {absolutePath[p].x,absolutePath[p].y,absolutePath[p].z};
                dep += pe.dist(ps);
                if(dep < DEP)
                    continue;
                absolutePath[p].z_x = absolutePath[p+1].x-absolutePath[p].x;  /// 前进方向
                absolutePath[p].z_y = absolutePath[p+1].y-absolutePath[p].y;
                absolutePath[p].z_z = absolutePath[p+1].z-absolutePath[p].z;
                sendWeldPoss(p,absolutePath[p],offset,base_offset,dr,autoFit,wf);
                dep = 0;
            }
            absolutePath[absolutePath.size()-1].z_x = absolutePath[absolutePath.size()-1].x-absolutePath[absolutePath.size()-2].x;  /// 前进方向
            absolutePath[absolutePath.size()-1].z_y = absolutePath[absolutePath.size()-1].y-absolutePath[absolutePath.size()-2].y;
            absolutePath[absolutePath.size()-1].z_z = absolutePath[absolutePath.size()-1].z-absolutePath[absolutePath.size()-2].z;
            sendWeldPoss(9999,absolutePath[vecsize-1],offset,base_offset,dr,autoFit,wf);
        }
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"absoluteMove vector err!"<<endl;
    }
}

void RobotKAWASAKI::absoluteMove(std::vector<RobotAxle> absolutePath, bool bBlock)
{
    static int index = 0;
    for(auto absoluteAxle:absolutePath){
        if(abs(absoluteAxle.a1) < 0.1 && abs(absoluteAxle.a2) < 0.1
                && abs(absoluteAxle.a3) < 0.1 && abs(absoluteAxle.a4) < 0.1
                && abs(absoluteAxle.a5) < 0.1 && abs(absoluteAxle.a6) < 0.1)
            return;
        cout << "轴旋转:" << absoluteAxle.toStr() << std::endl ;
        try
        {
            if ( RUNNING == _status ) {
                BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
            }
            if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
            {
                throw ERR_ROBOT_MOVEING_EXCEPTION;
            }
            // 设置速度加速度
            short iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
            if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
                _currSpeed = iSpeed ;
                _currAcc = iAcc ;
                _moveCmd = boost::format("6,%1%,%2%,") ;
                _moveCmd % _currSpeed % _currAcc ;
                sendUdpCmd(_moveCmd);//6,
            }
            _moveCmd = boost::format( "3,%1%,%2%,%3%,%4%,%5%,%6%," ) ;
            _moveCmd % absoluteAxle.a1 % absoluteAxle.a2 % absoluteAxle.a3
                    % absoluteAxle.a4 % absoluteAxle.a5 % absoluteAxle.a6 ;
            sendUdpCmd(_moveCmd);//3,
             _error = NORMAL ;
             _status = RUNNING ;
             if(bBlock)
                successived_sig(index);
             index++;
        }catch(ReturnStatus error_code)
        {
            cout<<getErrString(error_code)<<endl;
            throw;
        }
        catch(...)
        {
            cout<<"连续关节运动: 未知异常 "<<endl;
        }
    }
    index = 0;

}

void RobotKAWASAKI::weldFromMiddle(vector<RobotPos> absolutePath,RobotPos offset,RobotPos base_offset,double dr,int autoFit,weldInfo wf, int speed, double back, bool weld)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        int vecsize = absolutePath.size();
        if(vecsize < 5)
        {
            cout<<"点数太少!"<<endl;
            return;
        }
        int mid = vecsize/2;
        auto safePos = absolutePath[mid];
        retractMove(safePos,back);
        absoluteMove(safePos);
        sendWeldPoss(0,absolutePath[mid],offset,base_offset,dr,autoFit,wf);
        double dep = 0;
        for(int p=mid-1;p>0;p--)
        {
            point3d<double> ps = {absolutePath[p+1].x,absolutePath[p+1].y,absolutePath[p+1].z};
            point3d<double> pe = {absolutePath[p].x,absolutePath[p].y,absolutePath[p].z};
            dep += pe.dist(ps);
            if(dep < DEP)
                continue;
            sendWeldPoss(p,absolutePath[p],offset,base_offset,dr,autoFit,wf);
            dep = 0;
        }
        sendWeldPoss(9999,absolutePath[0],offset,base_offset,dr,autoFit,wf);
        dep = 0;
        /// set speed
        _moveCmd = boost::format("6,%1%,%2%,") ;
        _moveCmd % speed % _currAcc ;
        //_socketConnect.write(buffer(_moveCmd.str())) ;
        sendUdpCmd(_moveCmd);//6,
        /// back
        auto end1 = absolutePath[0];
        retractMove(end1,back);
        absoluteMove(end1);
        absoluteMove(safePos);
        if(weld)
            setWeldingTrue(wf,true);
        _moveCmd = boost::format("6,%1%,%2%,") ;
        _moveCmd % _currSpeed % _currAcc ;
        //_socketConnect.write(buffer(_moveCmd.str())) ;
        sendUdpCmd(_moveCmd);//6,
        sendWeldPoss(0,absolutePath[mid],offset,base_offset,dr,autoFit,wf);
        for(int p=mid+1;p<vecsize;p++)
        {
            point3d<double> ps = {absolutePath[p-1].x,absolutePath[p-1].y,absolutePath[p-1].z};
            point3d<double> pe = {absolutePath[p].x,absolutePath[p].y,absolutePath[p].z};
            dep += pe.dist(ps);
            if(dep < DEP)
                continue;
            sendWeldPoss(p,absolutePath[p],offset,base_offset,dr,autoFit,wf);
            dep = 0;
        }
        sendWeldPoss(9999,absolutePath[vecsize-1],offset,base_offset,dr,autoFit,wf);
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: weldFromMiddle unknown exception "<<endl;
    }
}

void weldAutoSet(double h, weldInfo & wf)  /// E 型缝根据焊高度确定焊接参数
{

}

void weldAutoSet2(RobotPos p, weldInfo & wf)
{
    point3dd px = {p.z_x,p.z_y,p.z_z};
    Matrix3d mp;
    pos2Matrix(p,mp,zyz);
    Vector3d vz = {-sin(16*PI/180),0,cos(16*PI/180)};
    vz = mp*vz;
    point3dd pz = {vz(0,0),vz(1,0),vz(2,0)};
    if(px.norm() > 0.1)
    {
        double xita = acos(px.inner(pz)/px.norm());
        if(xita > 90)
        {
            wf.V_W *= (1+0.1*(xita-90)/(120-90));
        }
    }
}

void RobotKAWASAKI::sendWeldPoss(int index, RobotPos absolutePos,RobotPos offset,RobotPos base_offset,double dh,int autoFit,weldInfo wf,bool needMove)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        // 生成路徑
        int iSpeed = _sys_config.get<double>("robot.speed"), iAcc = _sys_config.get<double>("robot.acc") ;
        cout<<"iSpeed = "<<iSpeed <<endl;
        cout<<"iAcc = "<<iAcc <<endl;
        cout<<"_currSpeed = "<<_currSpeed <<endl;
        cout<<"_currAcc = "<<_currAcc <<endl;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            cout << "设置速度加速度:" << _moveCmd.str() << "\n" << flush ;
            sendUdpCmd(_moveCmd); //6,
        }
        if(updateparams && _bweldOn)
        {
            // setWeldingFalse();
            cout<<"reset weld params!"<<endl;
            setWeldingTrue(wf);
            updateparams = false;
        }
        switch(autoFit)
        {
        case 1:
            // offset.y *= (absolutePos.dh+0.7)/dh;  泵机壳 w焊缝
            break;
        case 2:
            weldAutoSet(absolutePos.dh, wf);  /// E 型缝根据焊高度确定焊接参数
            break;
        case 3:
            weldAutoSet2(absolutePos, wf);  /// E 型缝根据焊高度确定焊接参数
            break;
        }
        if(autoFit > 0)
        {

        }
        Matrix4d mbst;
        pos2Matrix(base_offset,mbst,zyx);
        Matrix4d mcur;
        pos2Matrix(absolutePos,mcur,zyz);
//        point3d<double> my = {mcur(0,1),mcur(1,1),mcur(2,1)};
//        if(my.inner({0,0,1}) < 0)
//            offset.y *= -1;
        Matrix4d mb2b;
        pos2Matrix(offset,mb2b,zyx);
        mcur = mbst*mcur*mb2b;
        Matrix2pos(mcur,absolutePos,zyz);
        absolutePos.tcp = UPLOAD;
        double weldspeed = absolutePos.v > 0.01?absolutePos.v:wf.S_W;
        if(needMove)
        {
            if(weldspeed > 0.09 && wf.I_W > 1&& wf.V_W > 1)
            {
                if(index == 0)  // 起始点
                {
                    _moveCmd = boost::format("12,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f," ) ;
                }
                else if(index == 9999) // 终点
                {
                    _moveCmd = boost::format("14,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f," ) ;
                }
                else   // 中间点
                {
                    _moveCmd = boost::format("13,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f," ) ;
                }
                _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z
                        % absolutePos.a % absolutePos.b % absolutePos.c
                        % weldspeed % wf.I_W % wf.V_W;
                _currSpeed = weldspeed;
            }
            else
            {
                if(index == 0)  // 起始点
                {
                    _moveCmd = boost::format("12,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f" ) ;
                }
                else if(index == 9999) // 终点
                {
                    _moveCmd = boost::format("14,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f" ) ;
                }
                else   // 中间点
                {
                    _moveCmd = boost::format("13,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f" ) ;
                }
                _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z
                        % absolutePos.a % absolutePos.b % absolutePos.c;
            }
        }
        else
        {
            _moveCmd = boost::format("23,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f," ) ;
            _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z
                    % absolutePos.a % absolutePos.b % absolutePos.c
                    % weldspeed % wf.I_W % wf.V_W;
            _currSpeed = weldspeed;
        }
        cout<<"UDPSENT:"<<_moveCmd<<endl;
        sendUdpCmd(_moveCmd);//13,
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::relativeMove(RobotPos relativePos, double speed, double acc, bool bWelding)
{
    throw "error move" ;
#if 0
    {
        _error = NORMAL ;
        // 设置速度加速度
        udp::endpoint epCmd(boost::asio::ip::address::from_string("192.168.1.2"), 49152);
        int iSpeed = speed, iAcc = acc ;
        if ( !(iSpeed == _currSpeed && iAcc == _currAcc) ) {
            _currSpeed = iSpeed ;
            _currAcc = iAcc ;
            _moveCmd = boost::format("6,%1%,%2%,") ;
            _moveCmd % _currSpeed % _currAcc ;
            cout << "设置速度加速度:" << _moveCmd.str() << "\n" << flush ;
            _socketConnect.write(buffer(_moveCmd.str()), epCmd) ;

        }
        _moveCmd = boost::format( "1,%1%,%2%,%3%,%4%,%5%,%6%," ) ;
        auto absolutePos = relativePos + _currPos ;

        _moveCmd % absoldStoped.notify_all(utePos.x % absolutePos.y % absolutePos.z % absolutePos.a % absolutePos.b % absolutePos.c ;
                if ( bWelding ) {
            _bWeldingOn = bWelding ;
            _welder_ptr->setWelding(true) ;
        }
        _socketConnect.write(buffer(_moveCmd.str()), epCmd) ;
        cout << "[" << __FILE__ << "]" << "[" << __LINE__ << "]相对运动:" << _moveCmd.str() << "\n" << flush ;
    }
#endif
}


void RobotKAWASAKI::cmove(vector<RobotPos> poslst)//modify
{
}

void RobotKAWASAKI::sendCMovePos(RobotPos absolutePos)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _moveCmd = boost::format( "11,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        double x_ = absolutePos.x;
        _moveCmd % x_ % absolutePos.y % absolutePos.z % absolutePos.a % absolutePos.b % absolutePos.c ;
        //_socketConnect.write(buffer(_moveCmd.str())) ;
        sendUdpCmd(_moveCmd);//11,
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::sendCMoveEnd(RobotPos absolutePos)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        _moveCmd = boost::format( "111,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," ) ;
        _moveCmd % absolutePos.x % absolutePos.y % absolutePos.z % absolutePos.a % absolutePos.b % absolutePos.c ;
        sendUdpCmd(_moveCmd);//111,
        //_socketConnect.write(buffer(_moveCmd.str())) ;
        _error = NORMAL ;
        _status = RUNNING ;
    }
    catch(ReturnStatus error_code)
    {
        cout<<getErrString(error_code)<<endl;
        if(error_code == ERR_ROBOT_MOVEING_EXCEPTION)
            throw;
    }
    catch(...)
    {
        cout<<"throw exception: unknown exception "<<endl;
    }
}

void RobotKAWASAKI::sendCmd(const boost::system::error_code &ec, size_t)
{
//    _socketConnect.async_read(buffer(_replyCmd), boost::bind(&RobotKAWASAKI::sendCmd, this, _1, _2)) ;
//    if ( ec ) return ;
//    try {
//        std::cout << _replyCmd << endl << flush ;
//        if ( !strcmp( "running", _replyCmd))  {
//            bFlag = true ;
//            bRecord = true ;
//            _status = RobotStatus::RUNNING ;
//        }
//        else if ( !strcmp( "stopped", _replyCmd)) {
//            bRecord = false ;
//            ++intRecord;
//            if ( 6 == intRecord ) bFlag = false ;
//            _status = RobotStatus::STOPPED ;
//            if ( _bWeldingOn ) {
//                _bWeldingOn = false ;
//            }

//            cout << "进入stopped\n" << flush ;
//            _condStoped.notify_all() ;
//        }
//    }
//    catch ( const boost::exception &e ) {
//        DUMPERROR(e) ;
//        cout << "cmdReceive:" << _replyCmd<< "\n" << flush ;
//    }
//    catch ( const boost::system::system_error e ) {
//        cout << "systemError:" << e.what() << endl ;
//        cout << "cmdReceive:" << _replyCmd << "\n" << flush ;
//    }
//    memset( _replyCmd, 0, sizeof(_replyCmd) ) ;
}

void RobotKAWASAKI::recvPos(const boost::system::error_code &ec, size_t)
{
    _socketPos.async_read(buffer(_replyPos), boost::bind(&RobotKAWASAKI::recvPos, this, _1, _2)) ;
    if ( ec ) return ;
    _error = RobotError::NORMAL ;
    std::vector<string> pos_vec ;
    try {
        char timechar[32];
        ::sprintf(timechar,"udp/%.3f.txt",TIMESEC);
        static ofstream ofs(timechar) ;
        boost::algorithm::split(pos_vec, _replyPos, boost::algorithm::is_any_of(","));
        _currPos.store(RobotPos{stod(boost::trim_copy(pos_vec[0])),
                                stod(boost::trim_copy(pos_vec[1])),
                                stod(boost::trim_copy(pos_vec[2])),
                                stod(boost::trim_copy(pos_vec[3])),
                                stod(boost::trim_copy(pos_vec[4])),
                                stod(boost::trim_copy(pos_vec[5])), RobotTCP::UPLOAD}) ;
        posChanged_sig(in2OutPos(_tcp));
        if(pathLog)
        {
            char str[32];
            ::sprintf(str,"%.3f,",TIMESEC);
            ofs<<str<<pos_vec[0]<<","<<pos_vec[1]<<","<<pos_vec[2]
              <<","<<pos_vec[3]<<","<<pos_vec[4]<<","<<pos_vec[5]<<endl;
            ofs << flush ;
        }
        if(pos_vec.size() > 12)
        {
            rgsoStatus = ::atoi(pos_vec[12].c_str());
            switch(rgsoStatus)
            {
            case 0:
                errCounts = 0;
                updateparams = true;
                break;
            case -1:
                errCounts++;
                break;
            case -2:
                errCounts = 0;
                break;
            }
            if(errCounts > 1000)
            {
                //cout<<"robot rgso err _status = RobotStatus::STOPPED _condStoped.notify_all()"<<endl;
                _error = RobotError::FORCESTOPMOVING;
//                char msg[64];
//                sprintf(msg,"%.3f",getMicTime());
//                cout<<msg<<" ";
                __LINE__;
                _status = ERROR ;
                _condStoped.notify_all();
                errCounts = 0;
            }
        }
        if(pos_vec.size() > 13)
        {
            weldIndex = ::atoi(pos_vec[13].c_str());
        }
        _currAxle.store(RobotAxle{stod(pos_vec[6]),
                                  stod(pos_vec[7]),
                                  stod(pos_vec[8]),
                                  stod(pos_vec[9]),
                                  stod(pos_vec[10]),
                                  stod(pos_vec[11])}) ;
        axleChanged_sig( _currAxle.load()) ;
    }
    catch ( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
    catch ( const boost::system::system_error e ) {
        cout << "systemError:" << e.what() << endl ;
        cout << "posreceive:" << _replyPos << "$$$size:" << pos_vec.size() << endl ;
    }
    catch ( const std::exception &e ) {
    }
    memset( _replyPos, 0, sizeof(_replyPos ) ) ;
}

bool RobotKAWASAKI::isArrived() const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    return RobotStatus::STOPPED == _status ;
}
void RobotKAWASAKI::stopBrake(const boost::system::error_code &ec, size_t s)
{
    socketStop.async_read(buffer(_replyBrake),boost::bind(&RobotKAWASAKI::stopBrake, this,_1,_2)) ;
    if ( ec ) return ;
    try {
        if ( !strncmp( "v1", _replyBrake, 1) )  {

        }
    }
    catch ( const boost::exception &e ) {
        DUMPERROR(e) ;
        cout << "stopBrakeReceive:  " << _replyBrake<< "\n" << flush ;
    }
    catch ( const boost::system::system_error e ) {
        cout << "systemVisualError:  " << e.what() << endl ;
        cout << "cmdVisualReceive:  " << _replyBrake << "\n" << flush ;
    }
    memset( _replyBrake, 0, sizeof(_replyBrake) ) ;
}
void RobotKAWASAKI::resetSockFlag(int sockFlag)
{
    string strFlag;
    if( sockFlag )
        strFlag = "sockflag1" ;
    else
        strFlag = "sockflag0" ;

    cout <<strFlag<< "\n" << flush ;
    socketStop.write(buffer(strFlag)) ;
}

void RobotKAWASAKI::resetSpeed(int iSpeed)
{
    _currSpeed = iSpeed;
}
