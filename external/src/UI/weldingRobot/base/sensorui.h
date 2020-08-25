// 相机控制类

#ifndef SENSORUI_H
#define SENSORUI_H

#include <QWidget>
#include <memory>
#include "device/camera.h"
#include "device/robotmanager.h"
#include "modbusControl/mbControl.h"
#include "device/mysqliteapi.h"

#define GRIDR 9

class MyData : public QObject
{
    Q_OBJECT
public:
    explicit MyData(QObject *parent = nullptr);
    virtual ~MyData();

    MyData(const int iId,
           const map<string,string> kv,
           const QString strName,
           QObject *parent = nullptr)
        : QObject(parent),
          m_iId(iId),
          codes(kv),
          m_strName(strName)
    {
    }

    MyData(const MyData &data, QObject *parent = nullptr)
        : QObject(parent)
    {
        m_iId = data.GetId();
        codes = data.GetCodes();
        m_strName = data.GetName();
    }

    MyData &operator = (const MyData &data)
    {
        if (this == &data)
        {
            return *this;
        }
        m_iId = data.GetId();
        m_strName = data.GetName();
        codes=data.GetCodes();
        return *this;
    }
    friend bool operator == (const MyData &lhs, const MyData &rhs)
    {
        return (lhs.GetId() == rhs.GetId()
                && lhs.GetName() == rhs.GetName());
    }
    friend bool operator != (const MyData &lhs, const MyData &rhs)
    {
        return !(lhs.GetId() == rhs.GetId()
                && lhs.GetName() == rhs.GetName());
    }
private:
    int m_iId;
    QString m_strName;
    map<string,string> codes;
public:
    int GetId() const { return m_iId; }
    void SetId(const int iId) { m_iId = iId; }
    map<string,string> GetCodes() const { return codes; }
    void SetCodes(const map<string,string> empty) { codes = empty;}
    QString GetName() const { return m_strName; }
    void SetName(const QString &strName) { m_strName = strName; }
};

Q_DECLARE_METATYPE(MyData)


namespace Ui {
class SensorUI;
}

class SensorUI : public QWidget
{
    Q_OBJECT

public:
    explicit SensorUI(QWidget *parent = 0);
    ~SensorUI();
    void initUPDOWN();
    static void initMap(int mode=0);
private slots:
    //    void on_reconnectCamera_clicked();

    void on_openCamera_clicked();

    void on_takePicture_clicked();

    /// 显示相机图片
    void showPicture() ;
    /// 显示文件图片
    void showPic();

    void on_autoSaveImg_stateChanged(int arg1);

    void on_reconnectLaser_clicked();

    void on_alwaysLight_clicked();

    void on_laserClose_clicked();

    void mousePressEvent(QMouseEvent * event);

    void mouseMoveEvent(QMouseEvent * event);

    void mouseReleaseEvent(QMouseEvent *event);

    void keyPressEvent(QKeyEvent *event);

    void paintEvent(QPaintEvent *event);

    //void on_laserCorner_clicked();

    void on_btnMoveTo_clicked();

    void on_btnSave_clicked();

    void on_btnHome_clicked();

    void on_seamList_currentTextChanged(const QString &arg1);

    void on_btnObject_currentTextChanged(const QString &arg1);

    void on_openPic_clicked();
private:
    QRect enlargeR;
    QRect enSmallR;
    // 放大倍數
    int largeMode;
    UV pressuv;
    // 拖動偏差
    UV offuv;
    // 特征点UV
    UV fetureuv;
    // 前一条焊缝
    MyData preseam;
    /// SAFEPOS
    map<string,MyData> safeData;
    // 图像路径
    QString imgPath;
    Ui::SensorUI *ui;
    std::shared_ptr<mbControl> _mb_ptr;///电机指针
    /// 相机
    std::shared_ptr<Camera> _camera_ptr ;
    std::shared_ptr<mySqliteApi> _sqlite_ptr;///数据库智能指针
    /// 激光
    //    std::shared_ptr<RobotLaser> _laser_ptr ;

    std::fstream ofsR ;

    RobotManager _robot ;
    size_t currCameraDN ;
private:
    bool bOpen ;
    bool normalConnect ;
signals:
    void getRecord(QString r);
    void motor();
};

#endif // SENSORUI_H
