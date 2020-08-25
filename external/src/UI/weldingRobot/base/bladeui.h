#ifndef BLADEUI_H
#define BLADEUI_H

#include <QWidget>
#include "device/robotmanager.h"
#include "modbusControl/mbControl.h"
//#include "modbusControl/mbSlave.h"
#include "base/cworkstation.h"


namespace Ui {
class bladeui;
}

class bladeui:public QWidget
{
    Q_OBJECT

public:
    explicit bladeui(QWidget *parent = 0);
    ~bladeui();


public slots:

    void run_master();
    void func(int);

signals:
    void sig_operate(int);

private slots:

    void on_openValve_clicked();

    void on_closeValve_clicked();

    void on_openLaser_clicked();

    void on_closeLaser_clicked();

    void on_send_clicked();

    void on_transferData_clicked();

    void on_rotate_clicked();

    void on_openIO_clicked();

    void on_closeIO_clicked();

    void on_testUpdateProgress_clicked();

    void on_feedbackErrCode_clicked();

    void on_rotate1_clicked();

    void on_startMaster_clicked();

    void on_stopMaster_clicked();

    void on_send_single_clicked();

    void on_cadList_currentTextChanged(const QString &arg1);

    void on_read_clicked();

    void on_test_clicked();

    void on_test_seamlist_clicked();

    void on_write_test_clicked();

private:
    Ui::bladeui *ui;
    RobotManager _robot;
    CWorkStation cs;
    std::shared_ptr<mySqliteApi> _sqlite_ptr;
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
//    std::shared_ptr<slave> _slave_ptr;
    bool m_flag;//master run flag
    bool s_flag;//slave run flag
    bool bRun;
};

#endif // BLADEUI_H
