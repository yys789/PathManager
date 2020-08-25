#include "device/robot.h"
#include "base.h"
#include "tools.h"
#include "tools/matrix.h"
#include "base/wzerror.h"
#include "base/data_types.h"
#include "base/errorcode.h"

using namespace std ;

Robot::Robot()
    : _bWeldingOn(false),
      _currPos{},
      _bStoped(true),
      _firstPos{},
      _lastPos{},
      _currAxle{},
      _bForceWelding(false),
      _sys_config(Config()),
      _calibration_config("etc/calibration.info"),
      _status{STOPPED},
      _error{SERVICESTOP},
      updateparams{false}
{
    weldIndex = 0;
    _tcp = (RobotTCP)_sys_config.get<int>("robot.currentTCP") ;
    _rsiTcp = (RobotTCP)_sys_config.get<int>("robot.currentRSITCP", RobotTCP::FLANGE) ;
}

Robot::~Robot()
{
    stop();
}

void Robot::stop()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( SERVICESTOP == _error ) return ;
    if ( _socketConnect.is_open() ) {
        stopMove(SERVICESTOP) ;
        // 如果有线程等待运动结束则通知它们结束
        _socketConnect.close();
    }
    _condStoped.notify_all() ;
}

void Robot::run()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _socketConnect.run();
    _status = STOPPED;
    _error = NORMAL ;
}

RobotPos Robot::getCurrPos(RobotTCP tcp) const
{
    return _currPos;//in2OutPos(tcp) ;
}

RobotAxle Robot::getCurrAxle() const
{
    return _currAxle ;
}

RobotStatus Robot::getStatus() const
{
    return _status ;
}

RobotError Robot::getLastError() const
{
    return _error ;
}

bool Robot::waitForMoveStoped(chrono::milliseconds ms)
{
    try
    {
        cout<<__FILE__<<" "<<__FUNCTION__<<endl;
        unique_lock<mutex> lck(_mtxStoped) ;
        bool ret = true ;
        if ( chrono::milliseconds(0) == ms ) {
            _condStoped.wait(lck, [this]()->bool{
                                 lock_guard<recursive_mutex> lck(_mtx) ;
                                 return STOPPED== _status || SERVICESTOP == _error || FORCESTOPMOVING == _error;}) ;
        }
        else {
            ret = _condStoped.wait_for(lck, ms, [this]()->bool{
                                           lock_guard<recursive_mutex> lck(_mtx) ;
                                           return STOPPED== _status || SERVICESTOP == _error || FORCESTOPMOVING == _error;}) ;
            if(ret)
            {
                cout<<"ret = true"<<endl;
                cout<<"_status = "<< _status <<endl;
                cout<<"_error = "<< _error <<endl;
            }else{
                cout<<"ret = false"<<endl;
                if(rgsoStatus == 0)//hold
                {
                    while(_error == HOLDING)
                    {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }
                else
                {
                    _error = MOVETIMEOUT;
                }
            }
        }
        if ( FORCESTOPMOVING == _error || SERVICESTOP == _error || MOVETIMEOUT == _error)
            throw ERR_ROBOT_MOVEING_EXCEPTION;

        if ( ret )
            _status = STOPPED;
        return ret ;
    }
    catch(ReturnStatus error_code)
    {
        cout <<getErrString(error_code)<<endl;
        _status = STOPPED;
        throw ERR_ROBOT_MOVEING_EXCEPTION;
    }
    catch(...)
    {
        cout<<"throw exception : unknown exception."<<endl;
        throw ERR_ROBOT_MOVEING_EXCEPTION;
    }
}

RobotPos Robot::in2OutPos(RobotTCP tcp) const
{
    return in2OutPos(tcp, _currPos) ;
}

RobotPos Robot::in2OutPos(RobotTCP tcp, RobotPos pos) const
{
    return pos>tcp;
}

RobotPos Robot::out2InPos(RobotPos pos)
{
    auto inPos = pos ;
    auto tcp = pos.tcp ;
    posConversion( tcp, pos, RobotTCP::UPLOAD, inPos );
    inPos.tcp = RobotTCP::UPLOAD ;
    return inPos ;
}

void Robot::setInTCP(RobotTCP tcp)
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( _tcp == tcp ) return ;
    _tcp = tcp ;
    _sys_config.put("robot.currentTCP", _tcp);
    _sys_config.sync();
}

void Robot::setOutTCP(RobotTCP tcp)
{
    {
        lock_guard<recursive_mutex> lck(_mtx) ;
        if ( _rsiTcp == tcp ) return ;
        _rsiTcp = tcp ;
        _sys_config.put("robot.currentRSITCP", _rsiTcp);
        _sys_config.sync();
    }
    this_thread::sleep_for(10_ms) ;
}

RobotTCP Robot::getInTCP() const
{
    return _tcp ;
}

RobotTCP Robot::getOutTCP() const
{
    return _rsiTcp ;
}

void Robot::stopMove(RobotError e)
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( SERVICESTOP == _error ) return ;
    _error = e ;
    if (! _queuePos.empty() ) {
        queue<RobotPos> em ;
        _queuePos.swap(em) ;
        _status = STOPPED;
        _condStoped.notify_all();
    }
}
