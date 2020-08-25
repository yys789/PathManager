#include "device/as_udp.h"

as_udp::as_udp()
    :up10000{},
      _sendCmd{},
      _sys_config(Config()),
      pathLog(false)
{
    weldIndex = 0;
    // UDP通信运动控制主线程
    afeng();
    // autostart.pc 服务
    autoStart();
    // autostart2.pc 服务
    autoStart2();
}

RobotPos as_udp::getCurrPos(RobotTCP tcp) const
{
    return _currPos;//in2OutPos(tcp) ;
}

RobotAxle as_udp::getCurrAxle() const
{
    return _currAxle ;
}

void as_udp::sendUdpCmd(char * cmd)
{
    cout<<cmd<<endl;
    {
        lock_guard<recursive_mutex> lck(_mtx) ;
        strcpy(_sendCmd,cmd);
        _error = NORMAL ;
        _status = RUNNING;
    }
    try
    {
        unique_lock<mutex> lck(_mtxStoped) ;
        cout<<__FILE__<<" "<<__FUNCTION__<<endl;
        _condStoped.wait(lck, [this]()->bool{
                             lock_guard<recursive_mutex> lck(_mtx) ;
                             return STOPPED== _status || SERVICESTOP == _error || FORCESTOPMOVING == _error;}) ;
        if ( FORCESTOPMOVING == _error || SERVICESTOP == _error || MOVETIMEOUT == _error)
            throw ERR_ROBOT_MOVEING_EXCEPTION;
    }
    catch(ReturnStatus error_code)
    {
        cout <<getErrString(error_code)<<endl;
        _status = STOPPED;
        //throw ERR_ROBOT_MOVEING_EXCEPTION;
    }
    catch(...)
    {
        cout<<"throw exception : unknown exception."<<endl;
        //throw ERR_ROBOT_MOVEING_EXCEPTION;
    }
}

void as_udp::afeng()
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
                recv_length = recvfrom(sockIn, up10111, sizeof(up10111), 0, (struct sockaddr *) &in_addr, &in_len);
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
                recvfrom(sockIn, up10111, sizeof(up10111), 0, (struct sockaddr *) &in_addr, &in_len);
                if(!strcmp( "running", up10111))
                {
                    _status = RUNNING ;
                    cout << "进入running\n" << flush ;
                }
                else if((strcmp( "heart", up10111) == 0 || strcmp( "stopped", up10111) ==0) && ( _status == RUNNING ))
                {
                    _status = STOPPED ;
                    _condStoped.notify_all() ;
                    cout<<up10111 << "进入stopped\n" << flush ;
                }
                ::memset(up10111,0,sizeof(up10111));
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

void as_udp::autoStart()
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
            serveraddr.sin_port = htons(AUTO1);
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
            // 向客户端发送数据，使用sendto() 函数向服务器主机发送数据；
            timeval tv = {0, 1000};
            setsockopt(sockIn, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
            char _autoReplyCmd[64];
            int errCounts;
            while(1)
            {
                _error = RobotError::NORMAL ;
                try {
                    char timechar[32];
                    ::sprintf(timechar,"udp/%.3f.txt",TIMESEC);
                    static ofstream ofs(timechar) ;
                    static double preTime = getMicTime();

                    std::this_thread::sleep_for(1_ms);  /// 等待10ms
                    recvfrom(sockIn, _autoReplyCmd, sizeof(_autoReplyCmd), 0, (struct sockaddr *) &in_addr, &in_len);
                    stringstream doubles(_autoReplyCmd);
                    doubles>>_currPos.x>>_currPos.y>>_currPos.z>>_currPos.a>>_currPos.b>>_currPos.c
                            >>_currAxle.a1>>_currAxle.a2>>_currAxle.a3>>_currAxle.a4>>_currAxle.a5>>_currAxle.a6>>rgsoStatus>>weldIndex;
                    if(pathLog && getMicTime() > preTime+1)
                    {
                        preTime = getMicTime();
                        char str[32];
                        ::sprintf(str,"%s,%.3f",TIMESEC,_currPos.toStr().c_str());
                        ofs<<str<<endl<<flush;
                    }
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
                        char msg[64];
                        sprintf(msg,"%.3f",getMicTime());
                        cout<<msg<<" ";
                        __LINE__;
                        _status = ERROR ;
                        _condStoped.notify_all();
                        errCounts = 0;
                    }
                }
                catch ( const std::exception &e ) {
                }
                memset( _autoReplyCmd, 0, sizeof(_autoReplyCmd ) ) ;
            }
        }
        catch(...)
        {
            cout<<"UDP 自启动、状态查询 异常结束!!!!!!"<<endl;
        }
    }).detach() ;
}

void as_udp::autoStart2()
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
            serveraddr.sin_port = htons(AUTO2);
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
            out_addr.sin_port = htons(AUTO2);
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

void as_udp::setSpeed(double spd=100, double acc=80)
{
    /// 设置速度 加速度
    char cmd[256];
    sprintf(cmd,"6,%.3f,%.3f",spd,acc);
    _currPos.v = spd;
    _currPos.acc = acc;
    sendUdpCmd(cmd);
}

void as_udp::relativeMove(RobotPos relativePos, bool bBlock, bool bWelding )
{
    try
    {
        cout<<"relativeMove:"<<relativePos.toStr().c_str()<<endl;
        auto targetPos =  _currPos + relativePos ;
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

void as_udp::relativeMove_axis(vector<RobotPos> axis_vec, bool bBlock, bool bWelding)
{
    try
    {
        RobotPos pointInFlange = getPosInFlange(_tcp) ;
        RobotPos targetPos{} ;
        pointRotate(_currPos, pointInFlange, axis_vec, targetPos);
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

void as_udp::relativeMove_axle(RobotAxle axle, bool bBlock)
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
        ::sprintf(cmd,"4,%s",axle.toStr().c_str());
        sendUdpCmd(cmd);//4,

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

void as_udp::absoluteMove(RobotAxle absoluteAxle, bool bBlock)
{
    if(abs(absoluteAxle.a1) < 0.1 && abs(absoluteAxle.a2) < 0.1
            && abs(absoluteAxle.a3) < 0.1 && abs(absoluteAxle.a4) < 0.1
            && abs(absoluteAxle.a5) < 0.1 && abs(absoluteAxle.a6) < 0.1)
        return;
    cout << "轴旋转:" << absoluteAxle.toStr().c_str() << std::endl ;
    try
    {
        if ( RUNNING == _status ) {
            BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
        }
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        char cmd[256];
        ::sprintf(cmd,"3,%s",absoluteAxle.toStr().c_str());
        sendUdpCmd(cmd);//3,
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

void as_udp::absoluteMove(RobotPos absolutePos, bool bBlock, bool bWelding)
{
    try
    {
        cout<<"absoluteMove 1"<<endl;
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        cout<<"bWelding = "<<bWelding <<endl;
        char cmd[256];
        if(!bWelding)
            ::sprintf(cmd,"1,%s",absolutePos.toStr().c_str());
        else
            ::sprintf(cmd,"91,%s",absolutePos.toStr().c_str());
        sendUdpCmd(cmd); //1,
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

void as_udp::setPointWeldTrue(int count)
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
        char cmd[256];
        sprintf(cmd,"15,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",weldInfo.I_end, weldInfo.V_end, weldInfo.S_W, weldInfo.I_W, weldInfo.V_W);
        sendUdpCmd(cmd);//15
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

void as_udp::setPointWelding(RobotPos absolutePos)
{
        auto moveSpeed = _sys_config.get<double>("robot.speed");
        auto time=  absolutePos.dis(_currPos)/moveSpeed;
        time=ceil(time+3);
        char cmd[256];
        sprintf(cmd,"15,%s",absolutePos.toStr().c_str());
        sendUdpCmd(cmd);
}

void as_udp::setPointWelding2(RobotPos start, RobotPos end, std::vector<double> params)
{
    if(params.size() < 9)
        return;
    char cmd[256];
    sprintf(cmd,"16,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",start.toStr().c_str(),end.toStr().c_str(),
            params[0],params[1],params[2],params[3],params[4],params[5],params[6],params[7],params[8]);
    sendUdpCmd(cmd);//16,
}

void as_udp::setPointWeldFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        RobotPos currPos = getPosInFlange(_tcp);
        char cmd[256];
        sprintf(cmd,"8,%s",currPos.toStr().c_str());
        sendUdpCmd(cmd);
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

void as_udp::setWeldingTrue(weldInfo wf, bool bBlock)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        char cmd[256];
        sprintf(cmd,"7,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
                ,wf.S_W,wf.I_W,wf.V_W,wf.T_END,wf.I_END,wf.V_END,wf.range,wf.frequency,wf.serialNumber);
        sendUdpCmd(cmd);//7,
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

void as_udp::setRotateTrue()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        sendUdpCmd("51,1");//51,
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

void as_udp::setRotateFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        sendUdpCmd("52,1");//52,
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

void as_udp::setWeldingFalse()
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        sendUdpCmd("8,0");//8,
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
void as_udp::absoluteMove(vector<RobotPos> absolutePath,RobotPos offset,RobotPos base_offset,double dr,int autoFit,
                                 weldInfo wf)
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


void as_udp::absoluteMove(vector<RobotAxle> absolutePath)
{
    for(auto axle : absolutePath){
        if(abs(axle.a1) < 0.1 && abs(axle.a2) < 0.1 && abs(axle.a3) < 0.1
                && abs(axle.a4) < 0.1 && abs(axle.a5) < 0.1 && abs(axle.a6) < 0.1)
            continue;
        try
        {
            char cmd[256];
            sprintf(cmd,"3,%s,",axle.toStr().c_str());
            sendUdpCmd(cmd);//3,
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
}

void as_udp::weldAutoSet(double h, weldInfo & wf)  /// E 型缝根据焊高度确定焊接参数
{

}

void as_udp::weldAutoSet2(RobotPos p, weldInfo & wf)
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

void as_udp::sendWeldPoss(int index, RobotPos absolutePos,
                          RobotPos offset,RobotPos base_offset,double dh,
                          int autoFit,weldInfo wf,bool needMove)
{
    try
    {
        if( FORCESTOPMOVING == _error || SERVICESTOP == _error)
        {
            throw ERR_ROBOT_MOVEING_EXCEPTION;
        }
        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        char cmd[256];
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
        absolutePos = base_offset<<absolutePos;
        absolutePos = absolutePos<<offset;
        double weldspeed = absolutePos.v > 0.01?absolutePos.v:wf.S_W;
        if(needMove)
        {
            if(weldspeed > 0.09 && wf.I_W > 1&& wf.V_W > 1)
            {
                if(index == 0)  // 起始点
                {
                    sprintf(cmd,"12,%s,%.3f,%.3f,%.3f,",absolutePos.toStr().c_str(),weldspeed, wf.I_W , wf.V_W);
                }
                else if(index == 9999) // 终点
                {
                    sprintf(cmd,"14,%s,%.3f,%.3f,%.3f,",absolutePos.toStr().c_str(),weldspeed , wf.I_W , wf.V_W);
                }
                else   // 中间点
                {
                    sprintf(cmd,"13,%s,%.3f,%.3f,%.3f,",absolutePos.toStr().c_str(),weldspeed , wf.I_W , wf.V_W);
                }
                _currPos.v = weldspeed;
            }
            else
            {
                if(index == 0)  // 起始点
                {
                    sprintf(cmd,"12,%s,",absolutePos.toStr().c_str());
                }
                else if(index == 9999) // 终点
                {
                    sprintf(cmd,"14,%s,",absolutePos.toStr().c_str());
                }
                else   // 中间点
                {
                    sprintf(cmd,"13,%s,",absolutePos.toStr().c_str());
                }
                _currPos.v = weldspeed;
            }
        }
        else
        {
            sprintf(cmd,"23,%s,%.1f,%.1f,%.1f,",absolutePos.toStr().c_str(),weldspeed , wf.I_W , wf.V_W);
            _currPos.v = weldspeed;
        }
        sendUdpCmd(cmd);//13,
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

void as_udp::setInTCP(RobotTCP tcp)
{
    if ( _tcp == tcp ) return ;
    _tcp = tcp ;
    _sys_config.put("robot.currentTCP", _tcp);
    _sys_config.sync();
}

RobotTCP as_udp::getInTCP() const
{
    return _tcp ;
}
