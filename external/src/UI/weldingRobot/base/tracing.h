#ifndef TRACING_H
#define TRACING_H

#include <QWidget>
#include "device/mysqliteapi.h"
#include "device/camera.h"
#include "device/robotlaser.h"
#include "device/robotmanager.h"
#include "modbusControl/mbControl.h"
//#include "common/expCheck.h"
#include "base/config.h"
#include <queue>
#include <QTimer>
#include "fitCorrelation.h"
#include "getuvonshell/getsbuva.h"
#include <chrono>
#include "cworkpiecedata.h"

#define SYS "etc/sys.info"

using namespace std;
using namespace chrono;
using namespace Eigen;

namespace Ui {
class Tracing;
}

/// 跟踪控制算法类
class Tracing : public QWidget
{
    Q_OBJECT
public:
    explicit Tracing(QWidget *parent = 0);
    ~Tracing();
    void initUI();
    /// 初期化参数列表
    bool initParams(string seamId,RelateInfo rif, bool wflag = false);
    /// 开始跟踪逻辑
    void workTracing();
    void workTracing_hg(); /// 華光項目的跟蹤入口
    std::vector<RobotPos> to_corra;     /// 跟踪结束相位角下pos序列
private:
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
    std::shared_ptr<Camera> _camera_ptr;
    std::shared_ptr<mySqliteApi> _sqlite_ptr;
    map<string,string> seamMap;      /// 焊缝名与焊缝ID对应MAP
    RobotManager _robot;   /// 机器人控制对象
    Ui::Tracing *ui;
    std::mutex _mtx;       /// 多线程锁
    bool weldFlag;         /// 空走还是焊接
    bool weldStatus;       /// 断焊状态false 空走　true　焊接
    bool m_bTaking;        /// 是否正在拍照标志位
    string curCAD;            /// 当前工件CAD型号
    string curSeam;        /// 当前焊缝ID
    string seamTime;       /// 跟踪逻辑開始时间
    RobotPos tool;         /// 夹具基本点pos
    double loc;            /// 地轨极限位置
    double safeA1;         /// 翻转极限位置
    int trackMode;
    /// 当前焊缝处理时间（三线程开启时间）
    weldInfo wf;              /// 当前焊缝的焊接参数
    int autoFit;              /// 是否自适应
    double default_w;         /// 默认宽度
    double default_h;         /// 焊缝高度
    RobotPos offset;          /// tool下的offset
    RobotPos base_offset;     /// base下的offset

    double p2p_len;           /// 工件世界，点到点距离间隔
    RobotPos scaneEnd;        /// 起始一小段扫描结束POS
    WELDANGLEINFO angleInfo;  /// 焊缝的变位机对应角度信息
    std::vector<RobotPos> seamKeel;  /// DB龙骨数据
    std::vector<RobotPos> weldVec;    /// 给机器人发送的点序列,抽象（变位机）坐标系下的
    std::queue<SensorDataPack> dataSet;   /// 图像采集跟踪数据集合
    std::vector<RobotPos> backPath;   /// back path
    std::vector<RobotPos> oriPath;   /// 变位机坐标系下的原始3d序列

    int lastEnd; ///
    bool needPick; /// 需要摸排定起点
    int cameraMode;  /// 初始扫描相机模式 0 寻位模式 1 跟踪模式
    bool waitMode;  /// 跟踪起头一段，寻位扫描结束的时候，运动到焊接起点需要一个不拍照的等待过程
    RelateInfo rInfos;  /// 关联信息
    LINE_TYPE lineType; /// 线形
    GetSBuvA* _gxA;  /// 图像算法类对象
    UV ct[2];  /// 两个相机中心点
    int imageLevel; /// 存储图像级别
    double maxGap; /// 最大容忍图像异常距离
    uint end_err_limit; /// end最大容忍图像异常距离
    uint endKeelIndex; /// end keel index
    string logPath; /// 日志路径
    map<string,string> seamInfo; /// 焊縫配置信息
    /// start check
    void posCheck(SensorDataPack mpa,double & x);
    /// 线程1：拍照及机器人位置采集线程
    int takePictures();
    /// 线程2：图像及轨迹计算处理线程
    int imgProcess();
    /// 线程2：图像及轨迹计算处理线程 华光项目
    int imgProcess_hg(string seamId);
    /// 线程3：机器人运动控制线程
    int tracking();
    /// 线程3：机器人运动控制线程  华光demo
    int tracking_hg(string seamId);
    /// 跟踪起头摸工件，直到起头位置确认清楚
    bool findRealStart(RobotPos & p, RobotPos & mis);
private slots:
    /// UI:选择工具类型
    void on_cbxCAD_currentIndexChanged(const QString &arg1);
    /// UI:变位机回位
    void on_home_clicked();
    /// UI:跟踪测试入口
    void on_rotateTrack_clicked();
    /// UI:地轨参数拟合
    void on_btnPosition1_clicked();
    /// UI:RX转动的变位机参数拟合
    void on_btnPosition2_clicked();
    /// UI:RZ转动的变位机参数拟合
    void on_btnPosition3_clicked();
    /// UI:关闭机器人signal 5信号
    void on_home_2_clicked();
    /// UI:临时性功能 测试io信号是否正常发送
    void on_home_3_clicked();
    void on_signal51_clicked();
    void on_verifiKeel_clicked();
    void on_fitBase_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_testRotate_clicked();
    void on_fitHGBase_clicked();
    void on_testFit_clicked();
    void on_simuFit_clicked();
    void on_testRotate_2_clicked();
signals:
    void printUV(double u,double v);
};

#endif // TRACING_H
