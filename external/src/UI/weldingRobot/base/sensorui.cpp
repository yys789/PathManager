#include "base/sensorui.h"
#include "ui_sensorui.h"
#include <QTimer>
#include <QCursor>
#include <QtGui>
#include <QMessageBox>
#include <fstream>
#include <opencv2/tracking.hpp>
#include <boost/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "base/config.h"
#include "base/data_types.h"
#include "base.h"
#include "base/wzerror.h"
#include "base/imagelabel.h"
#include "tools/singleton.h"
#include "tools.h"
#include "common/tmessagebox.h"
#include <dirent.h>
#include "base/errorcode.h"
#include <QFileDialog>
#include "base/fitCorrelation.h"
#include "tools/coord.h"
#include "seamhandler.h"

#define SYS "etc/sys.info"
#define COM "etc/common.info"
#define MATW 720
#define MATH 540
using namespace std;
using namespace cv;


MyData::MyData(QObject *parent) : QObject(parent)
{
    map<string,string>().swap(codes);
}

MyData::~MyData()
{
}

UV uvMap[2][GRIDR][GRIDR];

SensorUI::SensorUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SensorUI),
    _camera_ptr(Singleton<Camera>::get()),
    _sqlite_ptr{Singleton<mySqliteApi>::get()},
    bOpen{false},
    normalConnect{false},
    //    _laser_ptr(Singleton<RobotLaser>::get()),
    _mb_ptr(Singleton<mbControl>::get())
{
    largeMode = 1;
    ui->setupUi(this);
    ui->picture->setAttribute(Qt::WA_TransparentForMouseEvents);
    currCameraDN = 0;
    initMap(0);
    initMap(1);
    initUPDOWN();
    pressuv = {0,0};
    offuv = {0,0};
    fetureuv = {360,270};
    this->setMouseTracking(true);
}

SensorUI::~SensorUI()
{
    delete ui;
}

void SensorUI::initMap(int mode)
{
    fstream fin("etc/grid"+int2str(mode)+".txt");  //打开文件
    string ReadLine;
    int index = 0;
    Config cconfig("etc/calibration.info") ;
    UV tempUV = cconfig.get<UV>("cameraCenterUV"+int2str(mode));
    while(getline(fin,ReadLine))  //逐行读取，直到结束
    {
        QString inl = QString(ReadLine.c_str());
        QStringList inLst = inl.split(" ");
        if(inLst.size()<5)
            continue;
        double u = inLst[3].toDouble()+tempUV.u;
        double v = inLst[4].toDouble()+tempUV.v;
        UV uv = {u, v};
        int row = index/GRIDR;
        int col = index%GRIDR;
        if(GRIDR == 9 && row % 2)
        {
            col = GRIDR-1-col;
        }
        uvMap[mode][row][col] = uv;
        index++;
    }
    fin.close();
}

void SensorUI::initUPDOWN()
{
    _sqlite_ptr->selectSQL({"CAD"});
    if(_sqlite_ptr->m_iRows == 0)
        throw ERR_DB_NO_DATA_FOUND;
    auto vec = _sqlite_ptr->m_vResult;
    for(auto& v:vec)
        ui->btnObject->addItem(QString::fromStdString(v["CADID"]));
    _sqlite_ptr->selectSQL({"SAFEPOS"});
    if(_sqlite_ptr->m_iRows == 0)
        throw ERR_DB_NO_DATA_FOUND;
    vec = _sqlite_ptr->m_vResult;
    for(auto v : vec)
    {
        safeData[v["POSID"]] = MyData(atoi(v["POSID"].c_str()),v,QString::fromStdString(v["POS"]));
    }
}

//void SensorUI::on_reconnectCamera_clicked()
//{
//    _camera_ptr->disconnectDevice() ;
//    _camera_ptr->connectDevice() ;
//}

void SensorUI::on_openCamera_clicked()
{

    //boost::posix_time::ptime pt = boost::posix_time::microsec_clock::local_time() ; ;
    // 以当前时间为默认文件名
    //string strName = boost::posix_time::to_iso_string(pt) + ".txt" ;
    ofsR.open("etc/cameraLog/cameraError.txt" , std::ios::app ) ;
    auto t = boost::posix_time::microsec_clock::local_time() ;
    ofsR << "\n\n new test time\n"<<t <<"\n" ;
    ofsR<<std::flush ;
    try {
        static QTimer qTimer ;
        qTimer.stop();
        if ( bOpen ) {
            QObject::disconnect(&qTimer, SIGNAL(timeout()),
                                this, SLOT(showPicture())) ;
            QObject::connect(&qTimer, SIGNAL(timeout()),
                             this, SLOT(showPic())) ;
            ui->openCamera->setText("打开相机");
            ui->cameraStatus->setText("相机状态：关闭");
            qTimer.start(100);
        }
        else {
            QObject::disconnect(&qTimer, SIGNAL(timeout()),
                                this, SLOT(showPic())) ;
            QObject::connect(&qTimer, SIGNAL(timeout()),
                             this, SLOT(showPicture())) ;
            qTimer.start(100);
            ui->openCamera->setText("关闭相机");
            ui->cameraStatus->setText("相机状态：开启");
        }
        bOpen = !bOpen ;
        normalConnect = false ;
    }
    catch ( boost::exception const &e ) {
        DUMPERROR( e ) ;
        ui->cameraStatus->setText("相机状态：出错");
    }
}

void SensorUI::on_takePicture_clicked()
{
    _mb_ptr->openLaser();
    saveImg(_camera_ptr->takePicture()) ;
    _mb_ptr->closeLaser();
}


void SensorUI::paintEvent(QPaintEvent *event)
{
    QPainter paint(this);
    QRect picR = ui->picture->geometry();
    enlargeR = QRect(picR.x()+picR.width()+10,80,40,40);
    enSmallR = QRect(picR.x()+picR.width()+10,140,40,40);
    paint.fillRect(enlargeR,QBrush(QColor(255,0,0)));
    QBrush pen = QBrush(QColor(0,0,0));
    paint.setBrush(pen);
    paint.drawLine(QPoint(enlargeR.x()+5,enlargeR.y()+enlargeR.height()/2)
                   ,QPoint(enlargeR.x()+enlargeR.width()-5,enlargeR.y()+enlargeR.height()/2));
    paint.drawLine(QPoint(enlargeR.x()+enlargeR.width()/2,enlargeR.y()+5)
                   ,QPoint(enlargeR.x()+enlargeR.width()/2,enlargeR.y()+enlargeR.height()-5));
    paint.fillRect(enSmallR,QBrush(QColor(0,255,0)));
    paint.drawLine(QPoint(enSmallR.x()+5,enSmallR.y()+enSmallR.height()/2)
                   ,QPoint(enSmallR.x()+enSmallR.width()-5,enSmallR.y()+enSmallR.height()/2));
    paint.fillRect(QRect(picR.x()+picR.width()+10,200,40,40),QBrush(QColor(255,255,255)));
    paint.drawText(QPoint(enSmallR.x(),enSmallR.y()+enSmallR.height()+50),
                   QString::number(largeMode));
    QCursor cs = cursor();
    QPoint pos = mapFromGlobal(cs.pos());
    int pw = MATW/largeMode;
    int ph = MATH/largeMode;
    QString uvstr = ui->centerUV->text();
    QStringList uvsl = uvstr.split(",");
    assert(uvsl.length() == 2);
    int centerU = uvsl[0].toInt();
    int centerV = uvsl[1].toInt();
    int centerX = (centerU-MATW/2+pw/2)*ui->picture->width()/pw+ui->picture->x();
    int centerY = (centerV-MATH/2+ph/2)*ui->picture->height()/ph+ui->picture->y();
    if(ui->picture->geometry().contains(pos))
    {
        int u = MATW/2-pw/2 + pw*(pos.x()-ui->picture->x()+offuv.u)/ui->picture->width();
        int v = MATH/2-ph/2 + ph*(pos.y()-ui->picture->y()+offuv.v)/ui->picture->height();
        paint.drawText(QPoint(enSmallR.x(),enSmallR.y()+enSmallR.height()+50),
                       QString::number(u).append(",").append(QString::number(v)));
    }
    paint.setPen(QColor(255,255,255));
    paint.drawLine(QPoint(centerX-5,centerY),QPoint(centerX+5,centerY));
    paint.drawLine(QPoint(centerX,centerY-5),QPoint(centerX,centerY+5));
}

void SensorUI::mousePressEvent(QMouseEvent * event)
{
    pressuv = {event->pos().x(), event->pos().y()};
}

void SensorUI::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Space)
    {
        offuv = {0,0};
    }
}

void SensorUI::mouseMoveEvent(QMouseEvent * event)
{
    if(pressuv.u && pressuv.v && largeMode != 1)
    {
        offuv.u -= event->pos().x()-pressuv.u;
        offuv.v -= event->pos().y()-pressuv.v;
        pressuv.u = event->pos().x();
        pressuv.v = event->pos().y();
    }
    update();
}

void SensorUI::mouseReleaseEvent(QMouseEvent *event)
{
    pressuv = {0,0};
    if(event->button() != Qt::LeftButton)
        return;
    QCursor cs = cursor();
    QPoint pos = mapFromGlobal(cs.pos());
    int pw = MATW/largeMode;
    int ph = MATH/largeMode;
    if(ui->picture->geometry().contains(pos))
    {
        int u = MATW/2-pw/2 + pw*(pos.x()-ui->picture->x()+offuv.u)/ui->picture->width();
        int v = MATH/2-ph/2 + ph*(pos.y()-ui->picture->y()+offuv.v)/ui->picture->height();
        UV temp = {u,v};

        //    float y0,z0;
        //    laser_yz(u-temp.u,v-temp.v,y0,z0);
        RobotPos flangePos = _robot->getCurrPos() ;
        static RobotPos targetPos{} ;
        if(ui->tracingMode->isChecked())
            UV2Point3D(flangePos, temp, targetPos,1);
        else
            UV2Point3D(flangePos, temp, targetPos);
        targetPos.a = flangePos.a ;
        targetPos.b = flangePos.b ;
        targetPos.c = flangePos.c ;
        targetPos.tcp = flangePos.tcp ;
        std::cout<< "(UV to point ) target pos: " ;
        std::cout<<"x = "<<targetPos.x<<" y = "<<targetPos.y<<" z = "<<targetPos.z <<std::endl;
        ui->currUVtoPos->setText(QString::fromStdString(targetPos.toStr()));
        if ( (ui->checkMove->isChecked()) && (QMessageBox::Yes == TMessageBox().information(NULL, "提示", "移动焊枪？", QMessageBox::Yes, QMessageBox::No)))
        {
            std::thread([this]{
                _robot->absoluteMove(targetPos) ;
            }).detach() ;
        }
        QString targetPosStr = QString::fromStdString(flangePos.toStr());
        targetPosStr.append(",");
        targetPosStr.append(QString::number(u,'f',3));
        targetPosStr.append(",");
        targetPosStr.append(QString::number(v,'f',3));
        if(ui->checkMove->isChecked())
            emit getRecord(targetPosStr);
    }
    if(enlargeR.contains(pos))
    {
        if(largeMode < 8)
            largeMode *=2;
        offuv = {0,0};
        update();
    }
    if(enSmallR.contains(pos))
    {
        if(largeMode > 1)
            largeMode /=2;
        offuv = {0,0};
        update();
    }

}

void SensorUI::showPicture()
{
    cv::Mat mat0 = _camera_ptr->getImg() ;
    fetureuv = algoUV;
    //cout<<"getImg time:"<<clock()<<endl;
    Mat mat = mat0.clone();
    int mode = ui->tracingMode->isChecked()?1:0;
    Config cconfig("etc/calibration.info") ;
    UV tempUV = cconfig.get<UV>("cameraCenterUV"+int2str(mode));
    ui->centerUV->setText(QString::number(tempUV.u)+","+QString::number(tempUV.v));
    if ( mat.data ) {
        normalConnect = true ;
        if(ui->checkGrid->isChecked())
        {
            // 画出格点
            for(int i=0;i<GRIDR-1;i++)
            {
                for(int j=0;j<GRIDR-1;j++)
                {
                    UV & p1 = uvMap[mode][i][j];
                    UV & p2 = uvMap[mode][i][j+1];
                    UV & p3 = uvMap[mode][i+1][j];
                    UV & p4 = uvMap[mode][i+1][j+1];
                    line(mat, Point(p1.u,p1.v), Point(p2.u,p2.v), Scalar(255, 255, 255), 0,LINE_AA);
                    line(mat, Point(p1.u,p1.v), Point(p3.u,p3.v), Scalar(255, 255, 255), 0,LINE_AA);
                    line(mat, Point(p4.u,p4.v), Point(p2.u,p2.v), Scalar(255, 255, 255), 0,LINE_AA);
                    line(mat, Point(p4.u,p4.v), Point(p3.u,p3.v), Scalar(255, 255, 255), 0,LINE_AA);
                }
            }
            UV center = uvMap[mode][GRIDR/2][GRIDR/2];
            RotatedRect rr(Point2f(center.u,center.v),Size2f(18,18),0);
            ellipse(mat,rr,Scalar(255, 255, 255),0,LINE_AA);
        }
        if(fetureuv.u > 10 && fetureuv.v > 10 && fetureuv.u <mat.cols-10 && fetureuv.v < mat.rows-10)
        {
            line(mat, Point(fetureuv.u-10,fetureuv.v), Point(fetureuv.u+10,fetureuv.v), Scalar(255, 0, 0), 0,LINE_AA);
            line(mat, Point(fetureuv.u,fetureuv.v-10), Point(fetureuv.u,fetureuv.v+10), Scalar(255, 0, 0), 0,LINE_AA);
        }
        // 可获取最新标定的图片及对应的区域
        // auto rect = ui->picture->getRect();
        // mat 为最新的图片
        // static cv::Ptr<cv::Tracker> track ;
        //  static cv::Rect2d roi ;
        // static bool bLastValid = false ;
        static size_t count = 0 ;
        cv::Mat rgb = mat.clone();
        //  bool bRGB = true ;
        //        double px = (mat.size().width) / (ui->picture->width());
        //        double py = (mat.size().height) / (ui->picture->height());
        //        static bool bRoiValid = false ;
        //        if ( !bLastValid && ui->picture->isValid() ) {
        //            if ( QRect{0, 0, 1, 1} != rect ) {
        //                roi = cv::Rect2d( rect.left() * px, rect.top() * py, px*(rect.right()-rect.left()), py*(rect.bottom()-rect.top()) ) ;
        //                if ( greaterd( roi.width, 0 ) && greaterd(roi.height, 0) ) {
        //                    track = cv::Tracker::create("MIL");
        //                    track->init(mat, roi) ;
        //                    rgb = mat;
        //                    bRoiValid = true ;
        //                }
        //                else {
        //                    bRoiValid = false ;
        //                }
        //            }
        //            bLastValid = true ;
        //        }
        //        else if ( ui->picture->isValid() && bRoiValid){
        //            track->update(mat, roi) ;
        //            cv::cvtColor(mat, rgb, CV_GRAY2RGB) ;
        //            cv::rectangle(rgb, roi, cv::Scalar(0,255,0), 2,1);
        //            bRGB = true ;
        //        }
        //        if ( !ui->picture->isValid() ) {
        //            bLastValid = false ;
        //        }
        cv::cvtColor(mat, rgb, CV_BGR2RGB) ;
        auto fmt = QImage::Format_RGB888 ;
        //        if ( !bRGB ) {
        //            fmt = QImage::Format_Indexed8 ;
        //        }

        QImage image((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, fmt) ;

        QPixmap pixmap = QPixmap::fromImage(image);
        int w = ui->picture->size().width();
        int h = ui->picture->size().height();
        QSize largeScall = QSize(w*largeMode,h*largeMode);
        QRect centerR = QRect(largeScall.width()/2-w/2+offuv.u,
                              largeScall.height()/2-h/2+offuv.v,w,h);
        QPixmap pixmapScaled = pixmap.scaled(largeScall).copy(centerR);
        ui->picture->setPixmap(pixmapScaled);
        time_t tm = time( nullptr ) ;
        static time_t tmLast = tm ;
        ++count ;
        if ( tmLast != tm ) {
            tmLast = tm ;
            count = 0 ;
        }
    }

    if (bOpen && !normalConnect )
    {
        //cout<<"Camera connect fail!"<<endl;
    }
}

void SensorUI::showPic()
{
    if(!bOpen)
    {
        cv::Mat mat0 = imread(imgPath.toStdString(),IMREAD_GRAYSCALE);
        Mat mat = mat0.clone();
        int mode = ui->tracingMode->isChecked()?1:0;
        Config cconfig("etc/calibration.info") ;
        UV tempUV = cconfig.get<UV>("cameraCenterUV"+int2str(mode));
        ui->centerUV->setText(QString::number(tempUV.u)+","+QString::number(tempUV.v));
        if ( mat.data ) {
            normalConnect = true ;
            if(ui->checkGrid->isChecked())
            {
                // 画出格点
                for(int i=0;i<GRIDR-1;i++)
                {
                    for(int j=0;j<GRIDR-1;j++)
                    {
                        UV & p1 = uvMap[mode][i][j];
                        UV & p2 = uvMap[mode][i][j+1];
                        UV & p3 = uvMap[mode][i+1][j];
                        UV & p4 = uvMap[mode][i+1][j+1];
                        line(mat, Point(p1.u,p1.v), Point(p2.u,p2.v), Scalar(255, 0, 0), 0,LINE_AA);
                        line(mat, Point(p1.u,p1.v), Point(p3.u,p3.v), Scalar(255, 0, 0), 0,LINE_AA);
                        line(mat, Point(p4.u,p4.v), Point(p2.u,p2.v), Scalar(255, 0, 0), 0,LINE_AA);
                        line(mat, Point(p4.u,p4.v), Point(p3.u,p3.v), Scalar(255, 0, 0), 0,LINE_AA);
                    }
                }
                UV center = uvMap[mode][GRIDR/2][GRIDR/2];
                RotatedRect rr(Point2f(center.u,center.v),Size2f(130,18),0);
                ellipse(mat,rr,Scalar(255, 0, 0),0,LINE_AA);
            }
            if(fetureuv.u > 10 && fetureuv.v > 10 && fetureuv.u <mat.cols-10 && fetureuv.v < mat.rows-10)
            {
                line(mat, Point(fetureuv.u-10,fetureuv.v), Point(fetureuv.u+10,fetureuv.v), Scalar(255, 0, 0), 0,LINE_AA);
                line(mat, Point(fetureuv.u,fetureuv.v-10), Point(fetureuv.u,fetureuv.v+10), Scalar(255, 0, 0), 0,LINE_AA);
            }
            cv::Mat rgb = mat.clone();
            bool bRGB = false ;
            auto fmt = QImage::Format_RGB888 ;
            if ( !bRGB ) {
                fmt = QImage::Format_Indexed8 ;
            }
            QImage image((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, fmt) ;
            QPixmap pixmap = QPixmap::fromImage(image);
            int w = ui->picture->size().width();
            int h = ui->picture->size().height();
            QSize largeScall = QSize(w*largeMode,h*largeMode);
            QRect centerR = QRect(largeScall.width()/2-w/2+offuv.u,
                                  largeScall.height()/2-h/2+offuv.v,w,h);
            QPixmap pixmapScaled = pixmap.scaled(largeScall).copy(centerR);
            ui->picture->setPixmap(pixmapScaled);
        }
    }
}

void SensorUI::on_autoSaveImg_stateChanged(int arg1)
{
    static bool bAutoSaveImg ;
    if ( 0 == arg1 ) {
        bAutoSaveImg = false ;
    }
    else if ( 2 == arg1 ) {
        bAutoSaveImg = true ;
    }
    if ( bAutoSaveImg ) {
        std::thread([this, &bAutoSaveImg](){
            string path = "picture/"+to_string(getMicTime())+"/";
            mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
            ofstream pofs;
            pofs.open(path+"pos.txt");
            int i = 0 ;
            while ( bAutoSaveImg ) {
                this_thread::sleep_for(std::chrono::milliseconds(100));
                auto mat = _camera_ptr->takePicture().clone() ;
                saveImg(mat, path, std::to_string(i++) + ".png") ;
                RobotPos p = _robot->getCurrPos() ;
                pofs<<p.toStr()<<endl;
            }
            pofs.close();
            std::cout << "停止自动拍照" << std::flush ;
        }).detach() ;
    }
}

void SensorUI::on_reconnectLaser_clicked()
{
    //    _laser_ptr->reconnectDevice();
}

void SensorUI::on_alwaysLight_clicked()
{
    //    _laser_ptr->alwaysLight();
    _mb_ptr->openLaser();
}

void SensorUI::on_laserClose_clicked()
{
    //    _laser_ptr->alwaysClose();
    _mb_ptr->closeLaser();
}

extern void sPosTodPos(string sPos,RobotAxle& axle);

void SensorUI::on_btnMoveTo_clicked()
{
    std::thread([this]{
        MyData seamData = ui->seamList->currentData().value<MyData>();
        MyData posData = ui->combme->currentData().value<MyData>();
        double seam_angle = stod(seamData.GetCodes()["MOTORANGLE2S"]);
        double curmotorangle2 = 0;
        _sqlite_ptr->selectSQL2({"COMPARAMS"},{"COMVALUE"},{"COMNAME","'CURMOTORANGLE2'"});
        if(_sqlite_ptr->m_vResult.size() > 0)
        {
            curmotorangle2 = stoi(_sqlite_ptr->m_vResult[0]["COMVALUE"]);
        }
        if(abs(curmotorangle2-seam_angle)  > 1)
        {
            QMessageBox::about(NULL,"提示","请先回到安全位置!");
            return;
        }
        if(preseam.GetId() != seamData.GetId())
        {
            string midID = seamData.GetCodes()["MIDPOINT"];
            _sqlite_ptr->selectSQL2({"SAFEPOS"},{"POS"},{"POSID",midID,"ENABLED","1"});
            if(_sqlite_ptr->m_iRows == 0)
                throw ERR_DB_NO_DATA_FOUND;
            RobotAxle ra;
            sPosTodPos(_sqlite_ptr->m_vResult[0]["POS"],ra);
            RobotAxle cur = _robot->getCurrAxle();
            if(abs(cur.a1-ra.a1) > 1
                    || abs(cur.a2-ra.a2) > 1
                    || abs(cur.a3-ra.a3) > 1
                    || abs(cur.a4-ra.a4) > 1
                    || abs(cur.a5-ra.a5) > 1
                    || abs(cur.a6-ra.a6) > 1)
            {
                QMessageBox::about(NULL,"提示","请先回到安全位置!");
                return;
            }
        }
        try
        {
            _sqlite_ptr->selectSQL2({"CAD"},{"BASEPOINT"},{"CADID","'"+seamData.GetCodes()["CADID"]+"'"});
            RobotPos tool = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
            RobotPos r = RobotPos::instance(posData.GetName().toStdString());
            MOTORDEGRE degree;
            degree.angle1 = stod(_sqlite_ptr->m_vResult[0]["MOTORPOSS"]);
            degree.angle2 = stod(_sqlite_ptr->m_vResult[0]["MOTORANGLE1S"]);
            degree.angle3 = stod(_sqlite_ptr->m_vResult[0]["MOTORANGLE2S"]);
            posInBase(r,tool,getPosition(degree));
            int dlen = atoi(seamData.GetCodes()["DEPARTLEN"].c_str());
            r = r>CAMERA;
            if(ui->combme->currentIndex() == 0)
            {
                RobotPos r100 = r;
                retractMove(r100,floor(dlen));
                _robot->absoluteMove(r100,true,true);
                _robot->absoluteMove(r);
            }
            else
                _robot->absoluteMove(r);
            preseam = seamData;
        }
        catch(...){
            QMessageBox::about(NULL,"notice","no data");
        }
    }).detach() ;
}

void SensorUI::on_btnSave_clicked()
{
    int index = ui->combme->currentIndex();
    QString text = ui->combme->currentText();
    MyData posData = ui->combme->currentData().value<MyData>();
    string posstr = ui->currUVtoPos->text().toStdString();
    RobotPos r;
    if(!str2Robot(posstr,r))
    {
        QMessageBox::about(NULL,"提示","请点击图像确认焊缝位置!");
        return;
    }
    string seamid = posData.GetCodes()["SEAMID"];
    string index1 = posData.GetCodes()["INDEX1"];
    posData.SetName(ui->currUVtoPos->text());
    ui->combme->removeItem(index);
    QVariant var;
    var.setValue(posData);
    ui->combme->insertItem(index,text,var);
    ui->combme->setCurrentIndex(index);
    _sqlite_ptr->updateSQL({"SCANEPOS"},{"POS","'"+posstr+"'"},
    {"SEAMID","'"+seamid+"'","INDEX1","'"+index1+"'"});//更新（第几个）扫描点的位置信息
    QMessageBox::about(NULL,"提示","CAD数据已更新!");
}

void SensorUI::on_btnHome_clicked()
{
    std::thread([this]()mutable{
        MyData seamData = ui->seamList->currentData().value<MyData>();
        Config _sys_config(SYS);
        // 先运行到前一个seam的安全位置
//        int rSpeed = _sys_config.get<int>("motor.speed");
//        int per360 = _sys_config.get<int>("motor.totalSteps");
//        int seamid = seamData.GetId();
        RobotPos curpos = _robot->getCurrPos(RobotTCP::UPLOAD);
        RobotAxle cur_a = _robot->getCurrAxle();
        RobotAxle ra_a;
        map<string, MyData>::iterator iter = safeData.begin();
        while(iter != safeData.end()) {
            MyData & s = iter->second;
            if(s.GetCodes()["CADID"] == "ALL")
            {
                sPosTodPos(s.GetCodes()["POS"],ra_a);
            }
            iter++;
        }
        if((abs(cur_a.a1 - ra_a.a1)>1)
                || (abs(cur_a.a2 - ra_a.a2) >1)
                || (abs(cur_a.a3 - ra_a.a3) >1)
                || (abs(cur_a.a4 - ra_a.a4) >1)
                || (abs(cur_a.a5 - ra_a.a5) >1)
                || (abs(cur_a.a6 - ra_a.a6) >1))
        {
            retractMove(curpos,100);
            _robot->absoluteMove(curpos);
        }
        try
        {
            if(preseam.GetName() != "" && seamData != preseam)
            {
                string mids_pre = preseam.GetCodes()["MIDPOINT"];
                RobotAxle ra_pre;
                sPosTodPos(safeData[mids_pre].GetName().toStdString(),ra_pre);
                if(abs(ra_pre.a2) < 0.1 && abs(ra_pre.a3) < 0.1)
                    throw -1;
                _robot->absoluteMove(ra_pre);
                string mids_cur = seamData.GetCodes()["MIDPOINT"];
                if(mids_pre != mids_cur)
                {
                    /// 回到絕對安全位置
                    _robot->absoluteMove(ra_a);
                }
                RobotAxle ra_cur;
                sPosTodPos(safeData[mids_cur].GetName().toStdString(),ra_cur);
                _robot->absoluteMove(ra_cur);
            }
            else
            {
                string mids_cur = seamData.GetCodes()["MIDPOINT"];
                RobotAxle ra_cur;
                sPosTodPos(safeData[mids_cur].GetName().toStdString(),ra_cur);
                if(abs(ra_cur.a2) < 0.1 && abs(ra_cur.a3) < 0.1)
                    throw -1;
                _robot->absoluteMove(ra_cur);
            }
            double seam_pos = stod(seamData.GetCodes()["MOTORPOSS"]);
            double seam_angle1 = stod(seamData.GetCodes()["MOTORANGLE1S"]);
            double seam_angle2 = stod(seamData.GetCodes()["MOTORANGLE2S"]);

            _mb_ptr->rotate(GROUNDRAIL,seam_pos,60,1);
            _sqlite_ptr->updateSQL({"COMPARAMS"},{"COMVALUE","'"+seamData.GetCodes()["MOTORPOSS"]+"'"},{"COMNAME","'CURMOTORPOS'"});
            std::this_thread::sleep_for(1000_ms);
            _mb_ptr->rotate(FLIPDISK,seam_angle1,60,1);
            _sqlite_ptr->updateSQL({"COMPARAMS"},{"COMVALUE","'"+seamData.GetCodes()["MOTORANGLE1S"]+"'"},{"COMNAME","'CURMOTORANGLE1'"});
            std::this_thread::sleep_for(1000_ms);
            _mb_ptr->rotate(ROTATINGDISK,seam_angle2,60,1);
            _sqlite_ptr->updateSQL({"COMPARAMS"},{"COMVALUE","'"+seamData.GetCodes()["MOTORANGLE2S"]+"'"},{"COMNAME","'CURMOTORANGLE2'"});
        }
        catch(...)
        {
            QMessageBox::about(NULL,"提示","回安全位置出錯!");
        }

    }).detach();

}

void SensorUI::on_seamList_currentTextChanged(const QString &arg1)
{
    if(arg1 == "" || !arg1.startsWith("seam"))
        return;
    int iIndex = ui->seamList->currentIndex();
    MyData mydata = ui->seamList->itemData(iIndex).value<MyData>();
    ui->combme->clear();
    _sqlite_ptr->selectSQLOrder({"SCANEPOS"},{"SEAMID","INDEX1","POS"},{"SEAMID",to_string(mydata.GetId()),"ENABLED","1"},{"INDEX1","ASC"});
    if(_sqlite_ptr->m_iRows <= 0)
    {
        QMessageBox::about(NULL,"提示","焊缝信息缺失,请检查DB!");
        return;
    }
    for(auto& v:_sqlite_ptr->m_vResult)
    {
        MyData ipos(atoi(v["INDEX1"].c_str()),v,QString::fromStdString(v["POS"]));
        QVariant var;
        var.setValue(ipos);
        string posname = "scanePos"+v["INDEX1"];
        ui->combme->addItem(QString::fromStdString(posname),var);
    }
}

void SensorUI::on_btnObject_currentTextChanged(const QString &arg1)
{
    ui->seamList->clear();
    string curCAD = arg1.toStdString();
    _sqlite_ptr->selectSQL1({"SEAMINFO"},{"CADID","'"+curCAD+"'"});
    if(_sqlite_ptr->m_iRows == 0)
        return;
    auto vecseam = _sqlite_ptr->m_vResult;
    ui->seamList->clear();
    for(auto& v:vecseam)
    {
        QVariant var;
        MyData idname(atoi(v["SEAMID"].c_str()),v,QString::fromStdString(v["SEAMNAME"]));
        var.setValue(idname);
        ui->seamList->addItem(idname.GetName(),var);
    }
}

void SensorUI::on_openPic_clicked()
{
    imgPath = QFileDialog::getOpenFileName(NULL,tr("open file"),"",tr("Image File(*.*)"));
}
