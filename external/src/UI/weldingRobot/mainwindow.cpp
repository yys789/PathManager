#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QDate>
#include <QDir>
#include <QFileDialog>
#include <QTimer>
#include <iostream>
#include <thread>
#include "base/wzerror.h"
#include "walgo/utils.h"
#include "tools.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "base/robotui.h"
#include "base/sensorui.h"
#include "base/settingui.h"
#include "base/griddev.h"
#include "base/bladeui.h"

#define LOG_FILE_KEEP_NUM 7

using namespace boost::gregorian;
using boost::asio::ip::udp ;
using namespace std ;
using namespace walgo;

extern string getDate();

int eSTATE = systemState::IDLE;
std::mutex _logmtx;

Tracing * traceLogic;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _bAutoSaveImg(false)
{
    ui->setupUi(this);
    using namespace boost::property_tree ;

    tabWidget = ui->weldingRobot ;
    _robotUI = new RobotUI;
    _sensorUI = new SensorUI;
    _settingUI = new SettingUI;
    rsWid = new QWidget;
    rslayout = new QHBoxLayout;
    rslayout->addWidget(_sensorUI);
    rslayout->addWidget(_robotUI);
    rsWid->setLayout(rslayout);
    _bladeUI = new bladeui;
    traceLogic = new Tracing;
    _gridWid = new GridDev;
    initLaserYZ("etc/grid.txt",0);
    initLaserYZ("etc/grid1.txt",1);
    updateMenu();
}

void MainWindow::closeEvent(QCloseEvent *)
{

}


MainWindow::~MainWindow()
{
    delete _settingUI;
    delete rsWid;
    delete _bladeUI;
    delete ui;
}

void MainWindow::updateMenu()
{
    cout<<"updateMenu"<<endl;
    auto tabCount = tabWidget->count();
    cout<<"before ,tabCount"<<tabCount<<endl;
    tabWidget->clear();
    tabWidget->addTab(_settingUI,"设置");
    tabWidget->addTab(rsWid, "硬件控制") ;
    tabWidget->addTab(traceLogic, "跟踪测试") ;
    tabWidget->addTab(_gridWid,"纠正");
    tabWidget->addTab(_bladeUI,"PLC控制");
    cout<<"after ,tabCount"<< tabWidget->count()<<endl;
}

void MainWindow::closeWindow()
{
    this->close();
}

void myMessageOutPut(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    std::lock_guard<std::mutex> lck(_logmtx);
    Config _sys_config("etc/sys.info");
    static int level = _sys_config.get<int>("level",1);
    if(level == 1 && type == QtDebugMsg)
    {
        return;
    }
    QDir dir("testLog");
    if (!dir.exists())
    {
        QDir dir;
        dir.mkdir("testLog");
    }

    QString currentDate = QDateTime::currentDateTime().toString("yyyyMMdd");
    QString logName = "runlog_" + currentDate + ".txt";
    QString logFileName = "testLog/" + logName;
    if (!QFile::exists(logFileName))
    {
        QFileInfoList fileList = dir.entryInfoList(QStringList() << "*.txt", QDir::NoFilter, QDir::Time);
        if (fileList.size() >= LOG_FILE_KEEP_NUM)
        {
            int i = 1;
            foreach(QFileInfo fileInfo , fileList)
            {
                if (i >= LOG_FILE_KEEP_NUM)
                {
                    QString fileName = fileInfo.absoluteFilePath();
                    QFile::remove(fileName);
                }
                i++;
            }
        }
    }

    QFile file(logFileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append))
    {
        file.close();
        return ;
    }

    QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMsg;

    switch (type)
    {
    case QtDebugMsg:
//        logMsg = QString("%1: [Debug]: %2  Function: %3  File: %4  Line: %5\n").arg(currentDateTime).arg(msg).arg(context.function).arg(context.file).arg(context.line);
        logMsg = QString("%1: [Debug]: %2  \n").arg(currentDateTime).arg(msg);

        break;
    case QtInfoMsg:
        logMsg = QString("%1: [Info]: %2  Function: %3  File: %4  Line: %5\n").arg(currentDateTime).arg(msg).arg(context.function).arg(context.file).arg(context.line);
        break;
    case QtWarningMsg:
        logMsg = QString("%1: [Warning]: %2 Function: %3 Line: %4 File: %5\n").arg(currentDateTime).arg(msg).arg(context.function).arg(context.file).arg(context.line);
        break;
    case QtCriticalMsg:
        logMsg = QString("%1: [Critical]: %2 Function: %3 Line: %4 File: %5\n").arg(currentDateTime).arg(msg).arg(context.function).arg(context.file).arg(context.line);
        break;
    case QtFatalMsg:
        logMsg = QString("%1: [Fatal]: %2 Function: %3 Line: %4 File: %5\n").arg(currentDateTime).arg(msg).arg(context.function).arg(context.file).arg(context.line);
        break;
    default:
        break;
    }
    //cout<<logMsg.toStdString().c_str()<<endl;
    QTextStream ts(&file);
    ts << logMsg;

    file.close();
}

