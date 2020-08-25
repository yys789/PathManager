// 机器人控制类

#ifndef ROBOTUI_H
#define ROBOTUI_H

#include <QWidget>
#include "device/robotmanager.h"
#include <QTimer>

//#include "common/expCheck.h"
namespace Ui {
class RobotUI;
}

class RobotUI : public QWidget
{
    Q_OBJECT

public:
    explicit RobotUI(QWidget *parent = 0);
    ~RobotUI();
private slots:
    void on_relativeMove_clicked();

    void on_getCurrPos_clicked();

    void on_stopMove_clicked();

    void on_relativeMoveAxle_clicked();
    // 更新界面机器人位置
    void updateRobotPosAndAxle() ;

    void on_currentTCP_currentIndexChanged(const QString &arg1);

//    void on_currentRSITCP_currentIndexChanged(const QString &arg1);

    void on_clear_clicked();

    void on_abMove_clicked();

    void on_RxyzMove_clicked();

    void on_absoluteMoveAxle_clicked();
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_btndiscon_clicked();

//    void on_btnSockflag_clicked();

    void on_cur_axis_clicked();

    void on_btnOpenFile_clicked();

    void on_btnMove_clicked();

private:
    Ui::RobotUI *ui;

    /// 配置
    Config _sys_config ;
    /// 机器人管理
    RobotManager _robot ;
    //std::shared_ptr<expCheck> _expCheck;
    /// 焊接管理
    bool _bWeldingOn{false} ;
    /// 更新界面位置
    QTimer _timerPos ;
    RobotAxle _robotAxle{} ;
    bool logFlag;
private:
    void on_robotMoveStarted() ;
    void on_robotMoveStoped() ;
};

#endif // ROBOTUI_H
