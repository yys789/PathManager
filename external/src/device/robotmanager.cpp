#include "device/robotmanager.h"
#include "base/data_types.h"
using namespace std ;

std::shared_ptr<as_udp> RobotManager::_robot_ptr ;
RobotType RobotManager::_t ;

RobotManager::RobotManager()
{
    _t = RobotType::KAWASAKI;
    updateConfig();
}

void RobotManager::selectRobot(RobotType t)
{
    if ( _t != t ) {
        _t = t ;
        updateConfig();
    }
//    _robot_ptr.reset() ;
    _robot_ptr.reset( new as_udp ) ;
}

std::shared_ptr<as_udp> RobotManager::operator->()
{
    return _robot_ptr ;
}

void RobotManager::updateConfig()
{
    Config::changeRobot("kawasaki") ;
}
