#include "device/robotkuka.h"
#include "walgo/lasermodel3.h"

#include "math.h"
#include <vector>
#include <queue>
#include <thread>
#include <functional>
#include "tools/robot_move.h"
#include "tools/coord.h"
#include "eigen3/Eigen/Dense"

#include "tools.h"
#include "base/wzerror.h"
#include "base/data_types.h"

#include <boost/property_tree/xml_parser.hpp>


using namespace boost::asio ;
using boost::asio::ip::udp ;
using namespace std ;

RobotKUKA::RobotKUKA()
{
    //using namespace std::placeholders ;
    _tcp = (RobotTCP)_sys_config.get<int>("robot.currentTCP") ;
    _rsiTcp = (RobotTCP)_sys_config.get<int>("robot.currentRSITCP", RobotTCP::FLANGE) ;
    _socketConnect.reset(SocketConnectType::UDPSERVER, "", _sys_config.get<int>("robot.port")) ;
    _socketConnect.async_read(buffer(_recvBuffer), boost::bind(&RobotKUKA::sendCmd, this, _1, _2)) ;
    _socketConnect.run();
    std::cout << "robot.port:" << _sys_config.get<int>("robot.port") << std::endl ;
}

RobotKUKA::~RobotKUKA()
{
    cout << "断开KUKA连接\n" << flush ;
}

void RobotKUKA::relativeMove(RobotPos relativePos, bool bBlock, bool bWelding )
{
    shared_ptr<queue<RobotPos>> path_ptr(new queue<RobotPos>) ;
    auto targetPos = out2InPos(in2OutPos(_tcp) + relativePos) ;
    // 坐标系旋转
    relativePos = targetPos - _currPos;
    cout << "target:" << targetPos.toStr() << endl ;
    cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
    // 生成路徑
    relative2Eular( _currPos, relativePos,
                   _sys_config.get<double>("robot.speed"),
                   _sys_config.get<double>("robot.acc"),
                   _sys_config.get<double>("robot.angleSpeed"),
                   _sys_config.get<double>("robot.angleAcc"),
             *path_ptr ) ;

    relativeMove( path_ptr, bWelding ) ;

    if ( bBlock )
        waitForMoveStoped() ;
}

void RobotKUKA::relativeMove_axis(vector<RobotPos> axis_vec, bool bBlock, bool bWelding)
{
    {
        shared_ptr<queue<RobotPos>> path_ptr(new queue<RobotPos>) ;
        RobotPos pointInFlange = getPosInFlange(_tcp) ;
        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        RobotPos targetPos ;
        pointRotate(in2OutPos(RobotTCP::FLANGE), pointInFlange, axis_vec, targetPos);
        targetPos = out2InPos(targetPos) ;
        auto relativePos = targetPos - _currPos;
        cout << "target:" << targetPos.toStr() << endl ;
        // 生成路徑
        relative2Eular( _currPos, relativePos,
                       _sys_config.get<double>("robot.speed"),
                       _sys_config.get<double>("robot.acc"),
                       _sys_config.get<double>("robot.angleSpeed"),
                       _sys_config.get<double>("robot.angleAcc"),
                 *path_ptr ) ;
        relativeMove( path_ptr, bWelding ) ;
    }
    if ( bBlock )
        waitForMoveStoped() ;
}

void RobotKUKA::relativeMove_axle(RobotAxle axle, bool bBlock)
{
    {
        auto axleSpeed = _sys_config.get<double>("robot.axleSpeed", 0.01) ;
        auto axleAcc = _sys_config.get<double>("robot.axleAcc", 0.01) ;
        std::queue<RobotAxle> queueAxle ;
        axleRelative(axle, axleSpeed, axleAcc, queueAxle) ;
        lock_guard<recursive_mutex> lck(_mtx) ;
        if ( RUNNING == _status ) {
            BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
        }
        if ( SERVICESTOP == _error ) {
            BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人服务停止")) ;
        }
        _error = NORMAL ;
        _status = RUNNING;
        if ( queueAxle.empty() ) {
            _status = STOPPED;
            _condStoped.notify_all();
        }
        else {
            _queueAxle = std::move(queueAxle) ;
        }
    }
    if ( bBlock )
        waitForMoveStoped() ;
}

void RobotKUKA::absoluteMove(RobotAxle absoluteAxle, bool bBlock)
{
    std::cout << "轴旋转:" << absoluteAxle.toStr() << std::endl ;
    RobotAxle cur = _currAxle;
    std::cout << "当前轴:" << cur.toStr() << std::endl ;
    auto rel = absoluteAxle - cur;
    std::cout << "轴旋转相对:" << rel.toStr() << std::endl << std::flush ;
    relativeMove_axle(absoluteAxle - _currAxle, bBlock);
}

void RobotKUKA::absoluteMove(RobotPos absolutePos, bool bBlock, bool bWelding)
{
    {

        if ( absolutePos == getCurrPos( ) ) return ;
        shared_ptr<queue<RobotPos>> path_ptr(new queue<RobotPos>) ;
        // 做TCP点的转换
        auto targetPos = out2InPos(absolutePos);
        cout << "当前速度：" << _sys_config.get<double>("robot.speed") << endl ;
        // 生成路徑
        absolute2Eular( _currPos, targetPos,
                       _sys_config.get<double>("robot.speed"),
                       _sys_config.get<double>("robot.acc"),
                       _sys_config.get<double>("robot.angleSpeed"),
                       _sys_config.get<double>("robot.angleAcc"),
                 *path_ptr ) ;
        relativeMove( path_ptr, bWelding ) ;
    }
    if ( bBlock )
        waitForMoveStoped() ;
}

void RobotKUKA::fitPath(std::vector<RobotPos> &absolutePath)
{
    // 坐标系旋转
    //cout << "orisize:" << absolutePath.size() << endl ;
    std::ofstream ofs2( "原始(未转换).txt", std::ios::binary ) ;
    for ( auto & pos : absolutePath ) {
        ofs2 << pos.toStr() << "\n" ;
    }
    ofs2.close() ;
    for ( auto &pos : absolutePath ) {
        pos = out2InPos(pos) ;
    }

    std::ofstream ofs1( "原始(转换).txt", std::ios::binary ) ;
    for ( auto & pos : absolutePath ) {
        ofs1 << pos.toStr() << "\n" ;
    }
    ofs1.close() ;
    std::vector<RobotPos> path_out ;
    pathFit(absolutePath, path_out, _sys_config.get<double>("robot.multipleSpeed")) ;
    std::ofstream ofs( "转换后拟合.txt", std::ios::binary ) ;
    for ( auto & pos : path_out ) {
        // 坐标转换
        RobotPos robotCoord = _sys_config.get<RobotPos>("robot.coord", RobotPos{}) ;
        coordinateRotate(pos, robotCoord, pos);
        ofs << pos.toStr() << "\n" ;
    }
    ofs.close() ;
    std::swap(absolutePath, path_out) ;
}

void RobotKUKA::absoluteMove(vector<RobotPos> absolutePath,RobotPos offset,RobotPos base_offset,double dr,int autoFit,
                             weldInfo wf,  bool bBlock, bool bWelding, bool bFitPath)
{
    {
       absoluteMove(absolutePath[0], bBlock, bWelding) ;
        shared_ptr<queue<RobotPos>> path_ptr(new queue<RobotPos>) ;
        for ( auto &pos : absolutePath ) {
           pos = out2InPos(pos) ;
        }
        if ( bFitPath ) {
            fitPath(absolutePath);
        }
        for ( size_t i = 0; i < absolutePath.size()-1; ++i ) {
            queue<RobotPos> path ;
            absolute2Eular(absolutePath[i], absolutePath[i+1],
                    0,0,0,0, path) ;

            while ( !path.empty() ) {
                path_ptr->push(path.front()) ;
                path.pop() ;
            }
        }
        relativeMove( path_ptr, bWelding ) ;
    }
    if ( bBlock )
        waitForMoveStoped() ;
}

void RobotKUKA::relativeMove(shared_ptr<queue<RobotPos>> q, bool bWelding)
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( RUNNING == _status ) {
        BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人处于运动状态，请稍后再试")) ;
    }
    if ( SERVICESTOP == _error ) {
        BOOST_THROW_EXCEPTION(WZError() << err_robot("机器人服务停止")) ;
    }
    _error = NORMAL ;
    _status = RUNNING;
    if ( !q->empty() ) {
        _queuePos.swap(*q);
        if ( bWelding ) {
            // TODO TRANS startWelding();
        }
    }
    else {
        _status = STOPPED;
        _condStoped.notify_all();
    }
}

void RobotKUKA::sendCmd(const boost::system::error_code &ec, size_t)
{
    //static ofstream ofs("robotTime.txt") ;
    //Millsecond millSecond = chrono::steady_clock::now() ;
    //static Millsecond lastTime = millSecond ;
    //cout << (millSecond - lastTime).count() << "\n" ;
    //cout << flush ;
    //lastTime = millSecond ;
    //std::cout << "sendCmd" << std::flush ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    using namespace boost::property_tree ;
    _socketConnect.async_read(buffer(_recvBuffer), boost::bind(&RobotKUKA::sendCmd, this, _1, _2)) ;
    //using namespace std::placeholders ;
    if ( ec ) return ;
    _error = RobotError::NORMAL ;
    try {
        ptree ptXml ;
        stringstream iostr(_recvBuffer) ;
        memset( _recvBuffer, 0, sizeof(_recvBuffer)) ;
        read_xml(iostr, ptXml) ;
        //write_xml(cout, ptXml) ;
        //cout << flush ;

        double x, y, z, a, b, c, a1, a2, a3, a4, a5, a6 ;
        string strIPOC ;
        struct RobotPos nextPos, tempPos ;
        struct RobotAxle nextAxle ;
        // 读取机器人当前坐标
        auto posAttr = ptXml.get_child("Rob.RIst.<xmlattr>") ;
        x = posAttr.get<double>("X") ;
        y = posAttr.get<double>("Y") ;
        z = posAttr.get<double>("Z") ;
        a = posAttr.get<double>("A") ;
        b = posAttr.get<double>("B") ;
        c = posAttr.get<double>("C") ;
        strIPOC = ptXml.get<string>("Rob.IPOC") ;
        tempPos = {x,y,z,a,b,c};
        if ( RobotTCP::FLANGE == _rsiTcp) {
            auto m = _sys_config.get<Eigen::Matrix4d>("RSIWelding2Flange") ;
            rotateMatrix2Pos(tempPos, m, tempPos);
        }
        tempPos.tcp = RobotTCP::FLANGE ;
        _currPos.store(tempPos);
        // 读取当前机器轴坐标
        auto axleAttr = ptXml.get_child("Rob.AIPos.<xmlattr>") ;
        a1 = axleAttr.get<double>("A1") ;
        a2 = axleAttr.get<double>("A2") ;
        a3 = axleAttr.get<double>("A3") ;
        a4 = axleAttr.get<double>("A4") ;
        a5 = axleAttr.get<double>("A5") ;
        a6 = axleAttr.get<double>("A6") ;
        _currAxle.store(RobotAxle{a1, a2, a3, a4, a5, a6}) ;

        posChanged_sig(in2OutPos(_tcp));
        axleChanged_sig(_currAxle) ;

        // 实时更新坐标
        if ( RUNNING == _status ) {
            if ( isArrived() ) {
                cout << "arrived" ;
                _status = STOPPED;
                moveStoped_sig() ;
                _condStoped.notify_all();
            }
        }

        // 向机器人发送指令
        bool bEmptyPos = true ;
        if ( _queuePos.empty() ) {
            //_welder_ptr->setWelding(_bForceWelding) ;
            if ( _bWeldingOn ) {
               weldingStoped_sig() ;
               _bWeldingOn = false ;
            }
            // 相对运动
            nextPos = RobotPos::instance();
        }
        else {
            nextPos = _queuePos.front() ;
            _queuePos.pop();
            bEmptyPos = false ;
        }
        bool bEmptyAxle = true ;
        // 轴旋转指令
        if ( _queueAxle.empty() ) {
            // 相对运动
            nextAxle = {0, 0, 0, 0, 0, 0} ;
        }
        else {
            bEmptyAxle = false ;
            //_welder_ptr->setWelding(false) ;
            //stopWelding();
            nextAxle = _queueAxle.front() ;
            _queueAxle.pop();
        }

        // 准备发送报文
        ptree ptSend ;

        ptSend.put("Sen.<xmlattr>.Type", "ImFree") ;
        //if ( !bEmptyPos ) {
            ptSend.put("Sen.RKorr.<xmlattr>.X", nextPos.x);
            ptSend.put("Sen.RKorr.<xmlattr>.Y", nextPos.y);
            ptSend.put("Sen.RKorr.<xmlattr>.Z", nextPos.z);
            ptSend.put("Sen.RKorr.<xmlattr>.A", nextPos.a);
            ptSend.put("Sen.RKorr.<xmlattr>.B", nextPos.b);
            ptSend.put("Sen.RKorr.<xmlattr>.C", nextPos.c);
        //}
        //else if ( !bEmptyAxle ){
            ptSend.put("Sen.AK.<xmlattr>.A1", nextAxle.a1);
            ptSend.put("Sen.AK.<xmlattr>.A2", nextAxle.a2);
            ptSend.put("Sen.AK.<xmlattr>.A3", nextAxle.a3);
            ptSend.put("Sen.AK.<xmlattr>.A4", nextAxle.a4);
            ptSend.put("Sen.AK.<xmlattr>.A5", nextAxle.a5);
            ptSend.put("Sen.AK.<xmlattr>.A6", nextAxle.a6);
        //}

        ptSend.put("Sen.DIO1", 2) ;
        ptSend.put("Sen.DIO2", 0) ;
        ptSend.put("Sen.DIO3", 0) ;
        ptSend.put("Sen.DIO4", 0) ;
        ptSend.put("Sen.DIO5", 0) ;
        ptSend.put("Sen.DIO6", 0) ;
        ptSend.put("Sen.IPOC", strIPOC) ;

        stringstream ostr ;
        write_xml(ostr, ptSend) ;
        //write_xml(cout, ptSend) ;
        //cout << flush ;
        _socketConnect.write(buffer(ostr.str())) ;
        _lastPos = _currPos.load() ;
    }
    catch ( const boost::exception &e ) {
        DUMPERROR(e) ;
        cout << _recvBuffer << endl ;
        memset( _recvBuffer, 0, sizeof(_recvBuffer)) ;
    }
    catch ( const boost::system::system_error e ) {
        cout << e.what() << endl ;
        cout << _recvBuffer << endl ;
        memset( _recvBuffer, 0, sizeof(_recvBuffer)) ;
    }
}

bool RobotKUKA::isArrived() const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    static int count = 0 ;
    if ( _queuePos.empty() && _queueAxle.empty() ) {
        return true ;
        //if ( _currPos == _lastPos) {
        //    if ( ++count >= 200 ) {
        //        count = 0 ;
        //        return true ;
        //    }
        //}
        //else {
        //    count = 0 ;
        //    return false ;
        //}
    }
    return false ;
}

