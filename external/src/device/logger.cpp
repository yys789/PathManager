#include "device/logger.h"
#include <fstream>
#include <time.h>
#include <map>
#include <boost/filesystem.hpp>


std::ofstream logger::m_LaserLog;
std::ofstream logger::m_SensorLog;
std::ofstream logger::m_TcpLog;
std::ofstream logger::m_UdpLog;
std::ofstream logger::m_PlcLog;
std::ofstream logger::m_SysRunLog;
std::ofstream logger::m_ScanPathLog;
std::ofstream logger::m_UVLog;
std::ofstream logger::m_TestLog;

extern string getDate();

string getTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return tmp;
}

void initLogger()
{
    loggerFolder::Ptr _fd_ptr{loggerFolder::getInstance()};
    logger::m_LaserLog.open(_fd_ptr->mLogFilePath["LASER"],ios::app);
    logger::m_SensorLog.open(_fd_ptr->mLogFilePath["SENSOR"],ios::app);
    logger::m_TcpLog.open(_fd_ptr->mLogFilePath["TCP"],ios::app);
    logger::m_UdpLog.open(_fd_ptr->mLogFilePath["UDP"],ios::app);
    logger::m_PlcLog.open(_fd_ptr->mLogFilePath["PLC"],ios::app);
    logger::m_SysRunLog.open(_fd_ptr->mLogFilePath["SYSTEM"],ios::app);
}

void initLogger(LOG_MODULE module,const string& path)
{
    loggerFolder::Ptr _fd_ptr{loggerFolder::getInstance()};
    if(module == SCANPATH){
        string seamPath = _fd_ptr->mLogDirPath["SEAM"];
        seamPath += "/" + path;

        string file_scanPath = "scanPath.txt";
        string temp = seamPath + "/" + file_scanPath;
        if(!fs::exists(temp))
            _fd_ptr->createFile(temp);
        logger::m_ScanPathLog.close();
        logger::m_ScanPathLog.open(temp,ios::app);
    }
    if(module == UV){
        string seamPath = _fd_ptr->mLogDirPath["SEAM"];
        string file_UV = "uv.txt";
        string temp = seamPath + "/" + file_UV;
        if(!fs::exists(temp))
            _fd_ptr->createFile(temp);
        logger::m_UVLog.close();
        logger::m_UVLog.open(temp,ios::app);
    }
    if(module == TEST){
        if(!fs::exists(path))
            _fd_ptr->createFile(path);
        logger::m_TestLog.close();
        logger::m_TestLog.open(path,ios::app);
    }
}


std::ostream& logger::getStream(LOG_MODULE module)
{
    switch(module)
    {
    case LASER:
        return m_LaserLog;
        break;
    case SENSOR:
        return m_SensorLog;
        break;
    case TCP:
        return m_TcpLog;
        break;
    case UDP:
        return m_UdpLog;
        break;
    case PLC:
        return m_PlcLog;
        break;
    case SYSTEM:
        return m_SysRunLog;
        break;
    case SCANPATH:
        return m_ScanPathLog;
        break;
    case UV:
        return m_UVLog;
        break;
    case TEST:
        return m_TestLog;
        break;
    }
}

std::ostream& logger::start(LOG_MODULE module,
                            const std::string& cppFIle,
                            const int line,
                            const std::string& function) {

    string time = getTime();
    if(module == SYSTEM){
        return getStream(module) << "["<<time<<"]"
                                 << "[FILE]:"<<cppFIle
                                 << ",[function]:(" << function << ")"
                                 << ",[line]:" << line
                                 << ",[content]:"<<std::flush;
    }else{
        return getStream(module) <<std::flush;
    }

}

logger::logger()
{

}


logger::~logger()
{
    getStream(m_LogModule) <<std::endl<< std::flush;
}
