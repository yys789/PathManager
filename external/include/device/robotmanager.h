#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H
#include "as_udp.h"
#include <string>

enum class RobotType{KUKA, TURIN, KAWASAKI} ;
//const std::string RobotTypeString[] = {"KUKA", "TURIN", "KAWASAKI"} ;

class RobotManager
{
public:
    RobotManager();

    void selectRobot( RobotType t ) ;

    std::shared_ptr<as_udp> operator->() ;

private:
    static std::shared_ptr<as_udp> _robot_ptr ;
    static RobotType _t ;
private:
    void updateConfig() ;
};

#endif // ROBOTMANAGER_H
