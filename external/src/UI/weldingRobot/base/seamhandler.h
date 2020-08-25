#ifndef SEAMHANDLER_H
#define SEAMHANDLER_H

#include "getuvonshell/getsbuva.h"
#include "base/data_types.h"
#include <queue>
#include <QTimer>
#include <chrono>
#include <atomic>
#include "device/camera.h"
#include "device/robotmanager.h"
#include "device/robotlaser.h"

#include "cworkpiecedata.h"

#include "base/config.h"
#include "device/mysqliteapi.h"
#include "modbusControl/mbControl.h"
#include "device/mysqliteapi.h"
#include "device/camera.h"
#include "device/robotlaser.h"
#include "device/robotmanager.h"
#include "modbusControl/mbControl.h"
#include "base/config.h"
#include <queue>
#include <QTimer>
#include "fitCorrelation.h"
#include "getuvonshell/getsbuva.h"
#include <chrono>
#include <condition_variable>
#include "cworkpiecedata.h"
#define SYS "etc/sys.info"

using namespace std;
using namespace chrono;
using namespace Eigen;
using namespace cv;
using namespace walgo;

enum HandlerStatus{FREE=0, PHOTO, CALCU, MOVE, ERR};

extern UV algoUV;
typedef struct MOTORANGLE
{
    double motorPos;
    double motorAngle1;
    double motorAngle2;
}MotorAngle;

typedef struct MOTORSPEED
{
    double speed1;  /// mm/s
    double speed2;  /// 度/s
    double speed3;  /// 度/s
}MotorSpeed;

typedef struct SCANPATH
{
    vector<RobotPos> scan;    /// DB scanepos 记录的
    int cameraMode;           /// 相机　0 寻位模式 1 跟踪模式
    bool needExplore;         /// 需要先摸索起点真实位置
    vector<RobotPos> ori_Vec; /// 拍照采集的原始列表
    vector<RobotPos> pos_23d; /// 23d后的序列
}ScanPath; /// 机器人坐标系内的

typedef struct WELDPATH
{
    vector<RobotPos> keelVec; /// 龙骨 夹具坐标系内的
    vector<RobotPos> ori_Vec; /// 原始数据 夹具坐标系FREE内的
    vector<RobotPos> weldVec; /// 拟合后的 夹具坐标系内的
    vector<RobotPos> sendVec; /// 发送给机器人的BASE下的序列
}WeldPath;

typedef struct WELDSETS
{
    RobotPos offset; ///
    RobotPos base_offset; ///
    weldInfo wInfo;/// 焊接参数
}WeldSet;  /// 焊接设置

typedef struct SEAMPACK
{
    int main;            /// 1主焊缝 0 辅助
    string seamName;     /// 焊缝名 记log需要
    string cadId;        /// cad 型号
    RobotAxle safeAxle;  /// CAD安全轴位置
    RobotPos fixture;    /// 夹具相对于变位机的坐标变换　随ＣＡＤ变化
    int orderId;         /// 焊接次序
    int rank;            /// 焊缝形态 1 直线 2 一般平面曲线 3...
    LINE_TYPE algo_type; /// 算法类型
    uint option;         /// 图像算法动态参数
    double departLen;    /// 回撤距离
    RobotPos midPos;     /// 安全pos
    RobotAxle midAxle;   /// 安全轴位置
    bool middleS;        /// 中间开始标志
    double width;        /// 焊缝高度
    double height;       /// 焊缝宽度
    MotorAngle mAngles;  /// 变位机起头角度
    MotorAngle mAnglee;  /// 变位机结尾角度
    MotorSpeed mSpeed;   /// 变位机转动速度
    double cornerlen;    /// 包边长度
    double cornerAngle;  /// 包边角度
    int autoFit;         /// 0 不自适应 1\2\3各类自适应方案
    int tracing;         /// 0 寻位 1 跟踪 2 半寻位　3 走龙骨
    bool enabled;        /// 0 弃用 1 使用
    ScanPath scan;       /// 扫描路径    dynamic
    WeldPath weld;       /// 焊接路径    dynamic
    RelateInfo relateS;  /// 关联信息
    vector<WeldSet> wSet; /// 焊接设置 多层参数可能不一致，所以用vector,平时只用第一个
    HandlerStatus status; /// 处理状态   dynamic
} SeamPack;

/// 焊缝处理中心
class SeamHandler
{
public:
    SeamHandler();
    ~SeamHandler();
    void weld(int seamid);
    /// 初期化参数列表
    bool initParams(string seamId, RelateInfo rInfo, bool wflag = false);
    /// 开始跟踪逻辑
    void workTracing();
    std::vector<RobotPos> to_corra;     /// 跟踪结束相位角下pos序列
    void CompletionRelateInfo(int);
    //获取焊缝的路径数据
    void getSeamPath(int seamID);
    //路径优化
    void optimizePath();
    //关联焊接流程
    void startWelding();
private:
    map<int,SeamPack> seamCenter; /// 焊缝中心
    ofstream path0;            /// 用于记录拍照时的位置 图像 时间
    ofstream path1;            /// 用于记录寻位的拟合逻辑
    ofstream path2;            /// 用于记录跟踪过程中的算法过程
    ofstream path3;            /// 跟踪结束后，记录抽象原始序列
    ofstream path4;            /// 跟踪结束后，记录抽象拟合点
    ofstream path5;            /// 跟踪过程中，记录发送给机器人的点序列
    ofstream path6;            /// 寻位过程中的算法数据
    ofstream path7;            /// 半寻位算法数据
    void init();  /// 事务开始时的准备
    void end();   /// 事务结束后的清理
    /// 将焊缝设置信息发送给plc
    void writeSeamSettingRegisters();
    /// 将数据库中的焊缝列表发送给plc
    void displaySeamList();
    /// 将数据库所有工件类型信息发送给PLC
    void send_cad_list();
    /// 将数据库所有安全位置信息发送给PLC
    void displayMidPointList();
    /// 发送plc转动命令，至变位机到指定焊缝的状态
    void rotateMotor(int seamId);
    /// 将plc反馈的三个变位机状态信息写入DB
    void syncMotorAngle();
    /// 保存新的工件类型安全位置到DB
    void saveMidPoint();
    /// 加载DB中所有焊缝信息
    void getSeamList();
    /// 加载DB中选中的焊縫列表
    void getSelectedSeamList();
    /// 读取DB焊接配置信息 TODO
    void checkSeamInfo();

    /// 相机中心点校验 TODO
    void checkSensor();
    double getFeature(Mat m);
    //复位
    void reset();
    //当前机器人位置是否安全
    bool currentPosSafe(RobotAxle midPos);
    //电机复位
    //    void motorReset();
    //是否需要检测传感器
    bool beenSensorChecked();
    //检查激光亮暗度
    void checkBrightness(int code = -1);
    //传感器校验
    void sensorCheck();
    //清枪 TODO
    void cleanTorch();
    //自动扫描焊接流程
    void autoScanWelding();
    //获取焊缝的关联焊缝的路径数据
    void getRelatedSeamsPath(int seamID);
    //查看是否有焊缝的路径数据
    bool findSeamPath(int seamID);
    //移动机器人到中间点
    void moveToMidPoint(int seamId);
    //扫描焊缝及处理焊缝图像流程
    int dealSeamImages(int seamId);
    //扫描焊缝
    int scanSeam(int seamId);
    //拍取焊缝图像
    int takePictures(int seamId);
    //处理焊缝图像
    int dealImages(int seamId);
    //更新焊接进度给PLC
    void updateProgress();
    //设置操作流程
    void setActionType(int act);
    //关联函数
    bool correlation();
    //焊接
    void welding();
    //跟踪部分
    //焊缝列表按ordrId排序
    void weldList_sort();
    void NewSyncMotorAngle();
private:
    //设备指针
    std::shared_ptr<Camera> _camera_ptr;///相机指针
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
    std::shared_ptr<mySqliteApi> _sqlite_ptr;///数据库智能指针
    RobotManager _robot;///机器人类对象
    //焊接完成结果
    std::vector<int> m_vWelded_ok;///焊縫焊接完成列表
    std::vector<int> m_vWelded_error;///焊縫焊接失敗列表
    //焊缝列表\标识结构体
    seamMark m_sCurrMark;///当前焊缝的标识结构体
    std::map<int,string> seamMap;///seamid 和seamName的对应关系
    vector<seamMark> m_vWeldList;///焊接列表,按orderid排序
    //工件ID情况
    int m_iCurrGoodSid;///当前工件ID
    bool m_bLastGoodsidDone;///上一工件是否已经完成
    string m_sCADID;///当前工件名
    string m_strCADID;
    //当前时间
    string m_sCurrTime;
    //焊缝路径信息
    CWorkPieceData wData;///工件掃描信息類
    //扫描的焊缝信息（位置、图像）
    int m_imgCounts;///图片的张数
    std::deque<cv::Mat> mat_que;///存放照片的deque的队列容器
    vector<RobotPos> tempPath; /// 连续焊接的缓冲区
    vector<pair<string,string>> tempSeam;   /// 连续焊接焊缝列表
    //焊缝的属性及相关信息
    map<string,string> mSeamInfo;///存放焊缝的属性及相关信息
    vector<WELDINFO> weldConf;              /// 当前焊缝的焊接参数
    map<string,string> mComInfo;///全局數據
    WELDANGLEINFO angleInfo;///焊缝的变位机对应角度信息
    //关联焊缝信息
    std::vector<int> rSeamsList;///存放关联焊缝的容器
    std::vector<relatedInfo> rSeamsInfo2;///存放当前焊缝的关联焊缝的所有信息
    //中间点
    RobotAxle AMidPointAlex;///絕對安全點位置信息
    std::string m_sCurrMidPoint;///中间点名字
    RobotAxle currSeamMidAxle;///当前焊缝中间点轴数据
    //标志位
    int m_iAction;///空走、焊接、空走后焊接
    bool m_bTaking;///是否正在拍照标志位
    bool m_bWeldSuccess;
    bool m_bCheckSuccess;///校验是否成功
    bool m_bRobotException;///机器人是否出异常
    bool m_bCanStart;
    /// 触发拍照条件变量
    std::condition_variable cond_pic;
    bool m_bWelding;///

    std::mutex _mtx;
    std::mutex _ImgMtx;

private:
    // 当前焊缝处理时间（三线程开启时间）
    vector<WELDINFO> wInfos;              /// 当前焊缝的焊接参数
    std::atomic<bool> isScanWelding = ATOMIC_FLAG_INIT;        /// 当前是否焊接状态

    //龙骨数据扩展
    void externPath(int seamId,int tracing, vector<RobotPos> & path);
public:
    LINE_TYPE lineType; /// 线形
    int lastEnd; ///
    double default_w;         /// 默认宽度
    double default_h;         /// 焊缝高度
    bool weldFlag;         /// need weld
    bool weldStatus;       /// 断焊状态false 空走　true　焊接
    string curCAD;            /// 当前工件CAD型号
    string curSeam;        /// 当前焊缝ID
    string seamTime;       /// 跟踪逻辑開始时间
    RobotPos tool;         /// 夹具基本点pos
    double loc;            /// 地轨极限位置
    double safeA1;         /// 翻转极限位置
    RelateInfo rInfos;  /// 关联信息
    int trackMode;
    /// 当前焊缝处理时间（三线程开启时间）
    int autoFit;              /// 是否自适应
    RobotPos offset;          /// tool下的offset
    RobotPos base_offset;     /// base下的offset

    double p2p_len;           /// 工件世界，点到点距离间隔
    double rotateTime;        /// 开启转动的时间 用于推算各个变位机角度，以后换成变位机编码值，可以不用记录这个时间
    RobotPos scaneEnd;        /// 起始一小段扫描结束POS
    double motorposs;         /// 跟踪开始时地轨位置
    double motorangle1s;      /// 跟踪开始时翻转电机的相位角
    double motorangle2s;      /// 跟踪开始时旋转电机的相位角
    double motorpose;         /// 跟踪end时地轨位置
    double motorangle1e;      /// 跟踪end时翻转电机的相位角
    double motorangle2e;      /// 跟踪end时旋转电机的相位角
    double angleSpeed1;        /// 变位机转速1(MM/秒) 计算
    double angleSpeed2;        /// 变位机转速2(度/秒) 计算
    double angleSpeed3;        /// 变位机转速3(度/秒) 计算
    double motor1speed;        /// 地轨转速(脉冲/s) DB
    double motor2speed;        /// 翻转转速(脉冲/s) DB
    double motor3speed;        /// 旋转转速(脉冲/s) DB
    std::vector<RobotPos> seamKeel;  /// DB龙骨数据
    std::vector<RobotPos> weldVec;    /// 给机器人发送的点序列,抽象（变位机）坐标系下的
    std::queue<SensorDataPack> dateStart;  /// 图像采集起头数据集合
    std::queue<SensorDataPack> dataSet;   /// 图像采集跟踪数据集合
    std::vector<RobotPos> backPath;   /// back path
    std::vector<RobotPos> oriPath;   /// 变位机坐标系下的原始3d序列

    bool needPick; /// 需要摸排定起点
    int cameraMode;  /// 初始扫描相机模式 0 寻位模式 1 跟踪模式
    bool waitMode;  /// 跟踪起头一段，寻位扫描结束的时候，运动到焊接起点需要一个不拍照的等待过程
    map<int, RelateInfo> relateMap;  /// 关联信息
    weldInfo wf;
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
    int takeTracePic();
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

};

#endif // SEAMHANDLER_H
