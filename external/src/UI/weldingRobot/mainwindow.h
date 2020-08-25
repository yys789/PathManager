#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <QHBoxLayout>
#include "device/camera.h"
#include "device/robotmanager.h"
#include "tools/singleton.h"
#include "modbusControl/mbControl.h"
//#include "modbusControl/mbSlave.h"
//#include "common/expCheck.h"
#include "device/mysqliteapi.h"
#include "base/tracing.h"

using namespace std;
namespace Ui {
class MainWindow;
}

enum systemState{BUSY=0,IDLE};
enum role{consumer=0,admin,root};
extern int user;
extern string getCurrentRole();
extern int getEqualUser(const QString&);
/// UI界面类
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event);

public slots:
    void updateMenu();
    void closeWindow();
signals:
    void signal_showMenu();
private:
    /// 初始化界面配置
    void initUI() ;

public:
    void welderAdjust();
    void cameraAdjust();
public:
    bool TOMIN;

private:
    Ui::MainWindow *ui;
    Singleton<Camera> _camera ;
    Singleton<RobotLaser> _laser ;
    Singleton<mbControl> _mb_ptr;
//    Singleton<slave> _salve_ptr;
//    Singleton<expCheck> _expCheck;
    Singleton<mySqliteApi> _sqlite_ptr;
    /// 系统配置
    Config _sys_config ;

    /// 是否自动保存图片
    bool _bAutoSaveImg ;
    /// 当前选择的焊件
    std::string _currWeldment ;
    QTabWidget *tabWidget;
    QWidget *_robotUI;
    QWidget *_sensorUI;
    QWidget *_settingUI;
    QWidget * rsWid;
    QWidget * _bladeUI;
    QWidget * _gridWid;
    QHBoxLayout *rslayout;

public:
    RobotManager _robot ;

};

void myMessageOutPut(QtMsgType type, const QMessageLogContext &context, const QString &msg);

#endif // MAINWINDOW_H
