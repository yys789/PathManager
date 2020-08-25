#ifndef DECODECONTROL_H
#define DECODECONTROL_H

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>

#include "Frame.h"

class DecodeControl
{
public:
    DecodeControl();
    /// 从本地加载一个数据源
    bool readFromLocal(std::string filename);
    bool readFromLocalTxt(std::string filename);
    bool readFromLocalFd(std::string filename);

    bool insertProtoData();
    bool insert2SQL(std::string );

    void closeDB();

    QStringList     log;
    double progress;
    frame::Frame  root;
private:

    QSqlDatabase dbT1;

};

#endif // DECODECONTROL_H
