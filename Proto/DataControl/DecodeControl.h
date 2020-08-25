#ifndef DECODECONTROL_H
#define DECODECONTROL_H

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QQueue>
#include <QDateTime>
#include <atomic>
#include <future>

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

    QQueue<QString> log;
    // QStringList     log;
    double progress;
    frame::Frame  root;

    std::atomic<int> new_seamid = {-1};

    std::mutex g_mtx;
    std::condition_variable g_CV;
private:
    int connectName = 0;
    QString conName;
    QScopedPointer<QSqlQuery> queryP;
    QSqlDatabase dbT1;
    QMap<QString,QString> mapInsertQuery;   //插入专用
    QMap<QString,QString> mapUpdateQuery;   //更新专用
    QMap<QString,QString> mapGetQuery;      //查询专用
    // QScopedPointer<QSqlQuery> queryP;

};

#endif // DECODECONTROL_H
