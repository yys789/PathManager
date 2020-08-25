#ifndef LOGGER_H
#define LOGGER_H
#include <iostream>
#include "device/loggerFolder.h"
#include <memory>

using namespace std;

#define LOG(LOG_MODULE)  logger(LOG_MODULE).start(LOG_MODULE,__FILE__,__LINE__,__FUNCTION__)

typedef enum log_modbule
{
    LASER = 1,
    SENSOR,
    TCP,
    UDP,
    PLC,
    SYSTEM,
    SEAM,
    IMAGE,
    SCANPATH,
    UV,
    CORRECTION,
    TEST

}LOG_MODULE;

void initLogger();
void initLogger(const string& path);

class logger
{
public:
    logger();
    ~logger();

    logger(LOG_MODULE module): m_LogModule(module){}

    friend void initLogger();
    friend void initLogger(LOG_MODULE module,const string& path);

    static std::ostream& start(LOG_MODULE module,
                               const std::string& cppFile,
                               const int line,
                               const std::string& function);

private:

    static std::ostream& getStream(LOG_MODULE module);

    static std::ofstream m_LaserLog;
    static std::ofstream m_SensorLog;
    static std::ofstream m_TcpLog;
    static std::ofstream m_UdpLog;
    static std::ofstream m_PlcLog;
    static std::ofstream m_SysRunLog;
    static std::ofstream m_ScanPathLog;
    static std::ofstream m_UVLog;
    static std::ofstream m_TestLog;

    LOG_MODULE m_LogModule;

};


#endif // LOGGER_H
