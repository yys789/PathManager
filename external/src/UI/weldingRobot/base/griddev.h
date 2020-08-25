#ifndef GRIDDEV_H
#define GRIDDEV_H

#include <QWidget>
#include "device/camera.h"
#include "device/robotmanager.h"
#include "tools.h"
#include "ui_griddev.h"
#include "modbusControl/mbControl.h"

using namespace  cv;
using namespace std;

struct Angle
{
    double a1,a2,a3,a4,a5,a6;
};

class GridDev : public QWidget
{
    Q_OBJECT
public:
    explicit GridDev(QWidget *parent = 0);
    ~GridDev();
    /// 相机
    std::shared_ptr<Camera> _camera_ptr ;
    /// 激光
    std::shared_ptr<RobotLaser> _laser_ptr ;
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
    /// 系统配置
    Config _sys_config ;
    /// 机器人通信类
    RobotManager _robot;
    /// 纠正面用的圆心UV数组
    double xita;   /// 激光面绕x轴转的角度
    uint mode;    /// 迭代模式
    bool autoMode;  /// 自动纠正模式
    UV ct[2];           /// 中心点
    RobotPos cpos;   /// 迭代用.
    int calMode;   /// 纠正对象
    double maxGridMis_y; /// 最大格点误差Y
    double maxGridMis_z; /// 最大格点误差Z
    /// 收缩后的球半径
    double minR;
    int cindex;
    map<uint,UV> cCenter;
    double minC2Line();
    void calSum(RobotPos pos, double & sumLen);
    bool calSumP3(RobotPos pos, UV & cuv, vector<UV>& luv);
    void calGrid(RobotPos pos, int row,UV & uv);
    void ajustCycleAndLine(RobotPos & r,point3d<double> & dp, double & minf,int & counts);
    /// 理论pos列表
    map<int, string> lrposLst;
    void initUI();
private:
    Ui::GridForm *ui;
private slots:
    void on_btnBasePos_clicked();
    void on_btnface_clicked();
    void on_btn4p_clicked();
    void on_btn4p2abc_clicked();
    void on_initGrid_clicked();
    void on_btnGrid_clicked();
    void openFlower();
    void on_btnxz_clicked();
    void on_btnnihe_clicked();
    void on_btnsave_clicked();
    void on_btn4p2abc_2_clicked();

    void on_btnnihe_2_clicked();

    void on_btnLocation_clicked();

    void on_checkGrid_clicked();

    void on_mathBtn_clicked();

    void on_pushButton_8_clicked();

    void on_autoWork_clicked();

    void on_testAir_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);
};

#endif // GRIDDEV_H
