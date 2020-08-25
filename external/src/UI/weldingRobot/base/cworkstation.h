#ifndef CWORKSTATION_H
#define CWORKSTATION_H

#include <QTimer>
#include <chrono>
#include <atomic>

#include "device/camera.h"
#include "device/robotmanager.h"
#include "device/robotlaser.h"
#include "cworkpiecedata.h"
#include "device/mysqliteapi.h"
#include "base/tracing.h"
#include "modbusControl/mbControl.h"
#include <condition_variable>
#include "seamhandler.h"

class CWorkStation
{
public:
    CWorkStation();
    ~CWorkStation();

    void loading_cad();

    void unloading_cad();

    void syncSeamInfo();

    void transferDBData();

    void displayALLCAD();

    void syncMotorAngle();

    void getSeamList();

    void getSelectedSeamList();

    void checkTcp();

    void checkSensor();

    void adjustPlane();

    double getFeature(Mat m);

    void reset();

    bool currentPosSafe(RobotAxle midPos);

    bool beenSensorChecked();

    void checkBrightness(int code = -1);

    void sensorCheck();

    void cleanTorch();

    void autoScanWelding();

    bool findSeamPath(int seamID);

    void moveToMidPoint(int seamId);

    void updateProgress();

    void setActionType(int act);

    void back_home();

    void NewSyncMotorAngle();

    void send_cad_list();

private:
    //设备指针
    std::shared_ptr<Camera> _camera_ptr;///相机指针
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
    std::shared_ptr<mySqliteApi> _sqlite_ptr;///数据库智能指针
    RobotManager _robot;///机器人类对象
    //焊缝列表\标识结构体
    seamMark m_sCurrMark;///当前焊缝的标识结构体
    std::map<int,string> seamMap;///seamid 和seamName的对应关系
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
    WELDANGLEINFO angleInfo;///焊缝的变位机对应角度信息
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

    SeamHandler sHandler; /// 焊缝处理类对象
private:
    // 当前焊缝处理时间（三线程开启时间）
    std::atomic<bool> isScanWelding = ATOMIC_FLAG_INIT;        /// 当前是否焊接状态
public:
    map<int, RelateInfo> relateMap; /// 关联关系
};

#endif // CWorkStation_H
