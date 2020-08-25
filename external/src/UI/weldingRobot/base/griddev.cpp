#include "griddev.h"
#include "tools.h"
#include "nlopt.h"
#include "tools/circle.h"
#include "walgo/point.h"
#include <QMessageBox>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include "base/sensorui.h"
#include "walgo/histimage.h"
#include "fitCorrelation.h"

using namespace  cv;
using namespace Eigen;
using namespace std;
#define IMGW 720
#define IMGH 540

#define MBDELAY 500_ms

#define CPINFO "etc/calibrationProcess.info"

#define UPV 3   /// UV同勸化系數

extern double dis2speed(RobotManager _robot, RobotPos p2);

struct matrixIndex
{
    Matrix4d M;
    int i;
};

#define EULER static_cast<EULERTYPE>(_sys_config.get<int>("robot.eulerType"))

/// #define KUKA // TODO

#ifdef KUKA
#define S500 1
#define S100 0.2
#define S50 0.1
#define SETSPEED "robot.speed"
#else
#define S500 500
#define S100 100
#define S50 50
#define SETSPEED "robot.speed"
#endif

static int checkindex = 500;
QString pos2Str(RobotPos r)
{
    return QString::fromStdString(r.toStr());
}

GridDev::GridDev(QWidget *parent) :
    QWidget(parent),
    _camera_ptr(Singleton<Camera>::get()),
    _laser_ptr(Singleton<RobotLaser>::get()),
    _mb_ptr(Singleton<mbControl>::get()),
    ui(new Ui::GridForm)
{
    ui->setupUi(this);
    initUI();
}

GridDev::~GridDev()
{
    delete ui;
}

void GridDev::initUI()
{
    Config jz(CPINFO);
    calMode = jz.get<int>("laser.mode",0);
    autoMode = false;
    ui->comboBox->setCurrentIndex(calMode);
    try{ui->editface0->setText(pos2Str(jz.get<RobotPos>("laserPos0"+int2str(calMode))));}catch(...){};
    try{ui->editdis0->setText(QString::number(jz.get<double>("laserPos0f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editface1->setText(pos2Str(jz.get<RobotPos>("laserPos1"+int2str(calMode))));}catch(...){};
    try{ui->editdis1->setText(QString::number(jz.get<double>("laserPos1f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editface2->setText(pos2Str(jz.get<RobotPos>("laserPos2"+int2str(calMode))));}catch(...){};
    try{ui->editdis2->setText(QString::number(jz.get<double>("laserPos2f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editface3->setText(pos2Str(jz.get<RobotPos>("laserPos3"+int2str(calMode))));}catch(...){};
    try{ui->editdis3->setText(QString::number(jz.get<double>("laserPos3f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editface4->setText(pos2Str(jz.get<RobotPos>("laserPos4"+int2str(calMode))));}catch(...){};
    try{ui->editdis4->setText(QString::number(jz.get<double>("laserPos4f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editdis5->setText(QString::number(jz.get<double>("laserPos5f"+int2str(calMode)),'f',3));}catch(...){};
    try{ui->editface->setText(pos2Str(jz.get<RobotPos>("laserPos5"+int2str(calMode))));}catch(...){};
    try{ui->editctpos->setText(pos2Str(jz.get<RobotPos>("centerPos"+int2str(calMode))));}catch(...){};
    try{ui->editcicle->setText(QString::number(jz.get<double>("laser.circle"+int2str(calMode),20000)));}catch(...){};
    try{ui->editline->setText(QString::number(jz.get<double>("laser.line"+int2str(calMode),2000)));}catch(...){};
    try{ui->editRoi->setText(QString::number(jz.get<double>("laser.roi"+int2str(calMode),120)));}catch(...){};
    try{ui->editc2f->setText(pos2Str(jz.get<RobotPos>("c2f"+int2str(calMode))));}catch(...){};
    for(int i=0;i<9;i++)
    {
        ui->twtball->setItem(i,0,new QTableWidgetItem(""));
        ui->twtball->setItem(i,1,new QTableWidgetItem(""));
        ui->twtball->setItem(i,2,new QTableWidgetItem(""));
        ui->twtball->setItem(i,3,new QTableWidgetItem(""));
    }
    for(int i=0;i<9;i++)
    {
        try{ui->twtball->item(i,0)->setText(pos2Str(jz.get<RobotPos>("pos"+to_string(i)+int2str(calMode))));}catch(...){}
        try{ui->twtball->item(i,1)->setText(pos2Str(jz.get<RobotPos>("pos"+to_string(i)+"_"+int2str(calMode))));}catch(...){}
        try{ui->twtball->item(i,2)->setText(QString::number(jz.get<double>("pos"+to_string(i)+"_f"+int2str(calMode)),'f',3));}catch(...){}
    }
    Config conf("etc/calibration.info");
    try
    {
        try{ui->edithq2fl->setText(pos2Str(conf.get<RobotPos>("TINF")));}catch(...){};
        ct[0] = conf.get<UV>("cameraCenterUV"+int2str(0));
        ct[1] = conf.get<UV>("cameraCenterUV"+int2str(1));
        ui->editcenter->setText(QString::number(ct[calMode].u,'f',1)+","+QString::number(ct[calMode].v,'f',1));
    }
    catch(...)
    {
        ui->editcenter->setText("360,270");
    }
    try{ui->editt2c->setText(pos2Str(conf.get<RobotPos>("TINC"+int2str(calMode))));}catch(...){};
    RobotPos ctpos = RobotPos::instance(ui->editface0->text().toStdString());
    xita = acos(ctpos.x/sqrt(ctpos.x*ctpos.x+ctpos.y*ctpos.y));
    /// 新建文件夹
    QDir dir1("jz/circle"),dir2("jz/line");
    if (!dir1.exists())
    {
        QDir dir;
        dir.mkdir("jz/circle");
    }
    if (!dir2.exists())
    {
        QDir dir;
        dir.mkdir("jz/line");
    }
    update();
}

void clearFloder()
{
    QDir::cleanPath("jz/circle");
    QDir::cleanPath("jz/line");
    mkdir("jz/circle", S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    mkdir("jz/line", S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
}

void GridDev::on_btnBasePos_clicked()
{
    Config jz(CPINFO);
    RobotPos curPos = _robot->getCurrPos(RobotTCP::UPLOAD);
    jz.put("laserPos0"+int2str(calMode), curPos);
    ui->editface0->setText(pos2Str(curPos));
    xita = acos(curPos.x/sqrt(curPos.x*curPos.x+curPos.y*curPos.y)); // 激光面绕x轴转动的角度
    Vector2d vy = {-sin(xita),cos(xita)};
    /// 计算其余四点的位置
    RobotPos rf1 = curPos;
    rf1.x += 16*vy(0,0);
    rf1.y += 16*vy(1,0);
    rf1.z += 16;
    jz.put("laserPos1"+int2str(calMode), rf1);
    jz.put("laserPos1f"+int2str(calMode), 999);
    ui->editface1->setText(pos2Str(rf1));
    ui->editdis1->setText("");
    RobotPos rf2 = curPos;
    rf2.x += 16*vy(0,0);
    rf2.y += 16*vy(1,0);
    rf2.z += -16;
    jz.put("laserPos2"+int2str(calMode), rf2);
    jz.put("laserPos2f"+int2str(calMode), 999);
    ui->editface2->setText(pos2Str(rf2));
    ui->editdis2->setText("");
    RobotPos rf3 = curPos;
    rf3.x += -16*vy(0,0);
    rf3.y += -16*vy(1,0);
    rf3.z += 16;
    jz.put("laserPos3"+int2str(calMode), rf3);
    jz.put("laserPos3f"+int2str(calMode), 999);
    ui->editface3->setText(pos2Str(rf3));
    ui->editdis3->setText("");
    RobotPos rf4 = curPos;
    rf4.x += -16*vy(0,0);
    rf4.y += -16*vy(1,0);
    rf4.z += -16;
    jz.put("laserPos4"+int2str(calMode), rf4);
    jz.put("laserPos4f"+int2str(calMode), 999);
    ui->editface4->setText(pos2Str(rf4));
    ui->editdis4->setText("");
    jz.sync();
    clearFloder();
    initUI();
}

void getLaserLine(Mat & m, vector<UV> & lpv)
{
    int hdj0[m.cols];
    int hdj1[m.cols];
    int hdj_[m.cols];
    int maxhd[m.cols];
    int a = m.cols;
    for(int i=0;i<m.cols;i++)
    {
        hdj1[i] = 0;
        maxhd[i] = 0;
        for(int j=0;j<m.rows;j++)
        {
            if(maxhd[i] < m.at<uchar>(j,i))
            {
                maxhd[i] = m.at<uchar>(j,i);
                hdj1[i] = j;
            }
        }
        hdj0[i] = 0;
        maxhd[i] = 0;
        for(int j=m.rows-1;j>0;j--)
        {
            if(maxhd[i] < m.at<uchar>(j,i))
            {
                maxhd[i] = m.at<uchar>(j,i);
                hdj0[i] = j;
            }
        }
        hdj_[i] = (hdj0[i]+hdj1[i])/2;
    }
    int hdh = 0;
    for(int t=0;t<m.cols;t++)
    {
        hdh += maxhd[t];
    }
    hdh /= m.cols;
//    if(hdh < 20)
//        return;
    int counts = 0;
    vector<int> xvec,yvec;
    for(int t=0;t<m.cols;t++)
    {
        if(maxhd[t] >= hdh/2)
        {
            xvec.push_back(t);
            yvec.push_back(hdj_[t]);
            counts++;
        }
    }
    MatrixXd mline = MatrixXd::Zero(counts,2);
    for(int s=0;s<xvec.size();s++)
    {
        mline(s,0) = xvec[s];
        mline(s,1) = yvec[s];
    }
    VectorXd vOnes = VectorXd::Ones(counts,1);
    Vector2d ab =mline.colPivHouseholderQr().solve(vOnes);
    double v1 = 1/ab(1,0);
    double v2 = (1-ab(0,0)*m.cols)/ab(1,0);
    lpv.push_back({0,v1});
    lpv.push_back({m.cols,v2});
    Mat cm;
    cvtColor(m,cm, CV_GRAY2BGR);
    line(cm,Point(0,v1),Point(m.cols,v2),Scalar(255,0,0),1,LINE_AA);
    m=cm;
}

///返回pix
double GridDev::minC2Line(){
    RobotPos pos0 = _robot->getCurrPos(RobotTCP::UPLOAD);
    double mis;
    calSum(pos0, mis);
    if(mis<0.5)
        return mis;
    double x=mis/50;
    RobotPos pos1 = pos0;
    pos1.x += x*cos(xita);
    pos1.y += x*sin(xita);
    _robot->absoluteMove(pos1);
    double mis1;
    calSum(pos1, mis1);
    if(mis1 < 0.5)
        return mis1;
    if(mis1 < mis-0.05)
    {
        return minC2Line();
    }
    RobotPos pos2 = pos0;
    pos2.x += -x*cos(xita);
    pos2.y += -x*sin(xita);
    _robot->absoluteMove(pos2);
    double mis2;
    calSum(pos2, mis2);
    if(mis2 < 0.5)
        return mis2;
    if(mis2 < mis-0.05)
    {
        return minC2Line();
    }
    _robot->absoluteMove(pos0);
    return mis;
}

// 微调机器人位置确定目标函数
void GridDev::calSum(RobotPos pos, double & sumLen)
{
    sumLen = 999;
    int bgcicle = ui->editcicle->text().toInt();
    int bgline = ui->editline->text().toInt();
    int roir = ui->editRoi->text().toInt();
    _mb_ptr->closeLaser();
    _camera_ptr->setExposure(bgcicle);
    _sys_config.put(SETSPEED, dis2speed(_robot,pos));
    _sys_config.sync();
    cout<<"To:"<<pos.x<<","<<pos.y<<","<<pos.z<<endl;
    _robot->absoluteMove(pos);
    std::this_thread::sleep_for(MBDELAY) ;
    auto mat10 = _camera_ptr->takePicture() ;
    Mat mat1;
    bgr2gray(mat10,mat1,2);
    //_laser_ptr->alwaysLight();
    _mb_ptr->openLaser();
    _camera_ptr->setExposure(bgline);
    std::this_thread::sleep_for(MBDELAY) ;
    auto mat20 = _camera_ptr->takePicture() ;
    Mat mat2;
    bgr2gray(mat20,mat2,2);
    UV uv = Circle::center(mat1) ;
    double dis1;                   // 圆心到中心点
    Vector2d duv = {uv.u-ct[calMode].u,UPV*(uv.v-ct[calMode].v)};
    dis1 =duv.dot(duv);
    cCenter[cindex] = uv;
    Circle circle(mat1) ;
    circle.draw(mat1);
    cout<<"circle:"<<uv.u<<","<<uv.v<<endl;
    char cs[32];
    if(mode == 0 || mode==1)
        sprintf(cs,"jz/circle/circle_b%d.png",cindex);
    else if(mode == 2)
        sprintf(cs,"jz/circle/circle_p%d.png",cindex);
    else
        sprintf(cs,"jz/circle/circle_c%d.png",cindex);
    cv::imwrite(cs,mat1);
//    mat2= cv::Mat(mat2,roi);
    vector<UV> luv;
    int ru = uv.u>=roir?uv.u-roir:0;
    int rv = uv.v>=roir?uv.v-roir:0;
    ru = ru>mat2.cols-2*roir?mat2.cols-2*roir-1:ru;
    rv = rv>mat2.rows-2*roir?mat2.rows-2*roir-1:rv;
    cv::Rect roi(ru,rv,2*roir,2*roir);
    mat2= cv::Mat(mat2,roi);
    getLaserLine(mat2,luv);
    if(luv.size() < 2)
        return;
    luv[0].u += roi.x;
    luv[0].v += roi.y;
    luv[1].u += roi.x;
    luv[1].v += roi.y;
    cout<<"line:"<<luv[0].u<<","<<luv[0].v<<";"<<luv[1].u<<","<<luv[1].v<<endl;
    double dis2;          /// 中心点到直线
    double dis3;          /// 圆心到直线
    //// (u-u0)*(v1-v0) + (v-v0)*(u0-u1)=0
    if(luv.size() >= 2)
    {
        Vector2d v1 = {ct[calMode].u-luv[0].u,UPV*(ct[calMode].v-luv[0].v)};
        Vector2d v12 = {UPV*(luv[1].v-luv[0].v), luv[0].u-luv[1].u};
        dis2 = pow(v1.dot(v12),2)/(v12.dot(v12));
        Vector2d v2 = {uv.u-luv[0].u,UPV*(uv.v-luv[0].v)};
        dis3 = pow(v2.dot(v12),2)/(v12.dot(v12));
    }
    switch(mode)
    {
    case 0:
    case 2:
    case 4:
        sumLen = sqrt(dis1+dis2);
        break;
    case 1:
        sumLen = sqrt(dis3);
        break;
    }
    cout<<"circle 2 line:"<<sumLen<<endl;
    char ls[32];
    if(mode == 0 || mode==1)
        sprintf(ls,"jz/line/line_b%d.png",cindex);
    else if(mode == 2)
        sprintf(ls,"jz/line/line_p%d.png",cindex);
    else
        sprintf(ls,"jz/line/line_c%d.png",cindex);
    cv::imwrite(ls,mat2);
}

// 微调机器人位置确定目标函数
bool GridDev::calSumP3(RobotPos pos, UV & cuv, vector<UV>& luv)
{
    int bgcicle = ui->editcicle->text().toInt();
    int bgline = ui->editline->text().toInt();
    int roir = ui->editRoi->text().toInt();
    _mb_ptr->closeLaser();
    _camera_ptr->setExposure(bgcicle);
    _sys_config.put(SETSPEED, dis2speed(_robot,pos));
    _sys_config.sync();
    cout<<"To:"<<pos.x<<","<<pos.y<<","<<pos.z<<endl;
    _robot->absoluteMove(pos);
    std::this_thread::sleep_for(MBDELAY) ;
    auto mat10 = _camera_ptr->takePicture();
    Mat mat1;
    bgr2gray(mat10,mat1,2);
    //_laser_ptr->alwaysLight();
    _mb_ptr->openLaser();
    _camera_ptr->setExposure(bgline);
    std::this_thread::sleep_for(MBDELAY) ;
    auto mat20 = _camera_ptr->takePicture() ;
    Mat mat2;
    bgr2gray(mat20,mat2,2);
    for(int i=0;i<mat1.rows;i++)
    {
        for(int j=0;j<mat1.cols;j++)
        {
            if(i>IMGH/2+roir || j>IMGW/2+roir)
            {
                mat1.at<uchar>(i,j) = 0;
            }
            if(i<IMGH/2-roir || j<IMGW/2-roir)
            {
                mat1.at<uchar>(i,j) = 0;
            }
        }
    }
    cuv = Circle::center(mat1) ;
    if(abs(cuv.u)+abs(cuv.v) < 10)
        return false;
    Circle circle(mat1) ;
    circle.draw(mat1);
    char cs[32];
    if(mode == 0 || mode==1)
        sprintf(cs,"jz/circle/circle_b%d.png",cindex);
    else if(mode == 2)
        sprintf(cs,"jz/circle/circle_p%d.png",cindex);
    else
        sprintf(cs,"jz/circle/circle_c%d.png",cindex);
    cv::imwrite(cs,mat1);
    int ru = cuv.u>=roir?cuv.u-roir:0;
    int rv = cuv.v>=roir?cuv.v-roir:0;
    ru = ru>mat2.cols-2*roir?mat2.cols-2*roir-1:ru;
    rv = rv>mat2.rows-2*roir?mat2.rows-2*roir-1:rv;
    cv::Rect roi(ru,rv,2*roir,2*roir);
    mat2= cv::Mat(mat2,roi);
    getLaserLine(mat2,luv);
    if(luv.size()<2)
        return false;
    luv[0].u += ru;
    luv[0].v += rv;
    luv[1].u += ru;
    luv[1].v += rv;
    char ls[32];
    if(mode == 0 || mode==1)
        sprintf(ls,"jz/line/line_b%d.png",cindex);
    else if(mode == 2)
        sprintf(ls,"jz/line/line_p%d.png",cindex);
    else
        sprintf(ls,"jz/line/line_c%d.png",cindex);
    cv::imwrite(ls,mat2);
    return true;
}

/// 远近距离决定机器人速度
double dis2speed(RobotManager _robot, RobotPos p2)
{
    RobotPos p1 = _robot->getCurrPos(RobotTCP::UPLOAD);
    double dis = p1.dis(p2);
#ifndef KUKA
    int speed = 5*(int)dis;
    speed = speed>5?speed:5;
    speed = speed<500?speed:500;
#else
    double speed = dis/1000;
    speed = speed>0.001?speed:0.001;
    speed = speed<0.1?speed:0.1;
#endif
    cout<<"dis2speed:"<<speed<<endl;
    return speed;
}

/// 測量格点
void GridDev::calGrid(RobotPos pos, int row, UV & uv)
{
    int bgcicle = ui->editcicle->text().toInt();
    // 激光亮度180
    //_laser_ptr->alwaysClose();
    _mb_ptr->closeLaser();
    _camera_ptr->setExposure(bgcicle);
    _sys_config.put(SETSPEED, dis2speed(_robot,pos));
    _sys_config.sync();
    _robot->absoluteMove(pos);
    std::this_thread::sleep_for(MBDELAY);
    auto mat10 = _camera_ptr->takePicture() ;
    Mat mat1;
    bgr2gray(mat10,mat1,2);
    uv = Circle::center(mat1) ;
    Circle circle(mat1) ;
    circle.draw(mat1);
    char cs[32];
    sprintf(cs,"jz/circle/circle_g%d.png",row);
    cv::imwrite(cs,mat1);
}

void GridDev::on_btnface_clicked()
{
    RobotPos pos = RobotPos::instance(ui->editface0->text().toStdString());
    QString ctstr = ui->editcenter->text();
    QStringList stpl = ctstr.split(",");
    if(stpl.size() == 2)
    {
        ct[calMode].u = stpl[0].toDouble();
        ct[calMode].v = stpl[1].toDouble();
    }
    mode = 0;// 迭代xyz
    _robot->absoluteMove(pos);
    cindex = 0;
    cpos = pos;
    point3d<double> dp = {0,0,0};
    double minf;
    int counts = 10;
    ajustCycleAndLine(pos,dp,minf,counts);
    ui->editdis0->setText(QString::number(minf,'f',1));
    Config jz(CPINFO);
    jz.put("laserPos0f"+int2str(calMode),minf);
    jz.sync();
    this->on_btnBasePos_clicked();
}

void GridDev::on_btn4p_clicked()
{
    RobotPos pos[4];
    double dis[4];
    dis[0] = ui->editdis1->text().toDouble();
    dis[1] = ui->editdis2->text().toDouble();
    dis[2] = ui->editdis3->text().toDouble();
    dis[3] = ui->editdis4->text().toDouble();
    pos[0] = RobotPos::instance(ui->editface1->text().toStdString());
    pos[1] = RobotPos::instance(ui->editface2->text().toStdString());
    pos[2] = RobotPos::instance(ui->editface3->text().toStdString());
    pos[3] = RobotPos::instance(ui->editface4->text().toStdString());
    QString ctstr = ui->editcenter->text();
    QStringList stpl = ctstr.split(",");
    RobotPos ctpos = RobotPos::instance(ui->editface0->text().toStdString());
    if(ctpos.tcp == UPLOAD)
    {
        xita = acos(ctpos.x/sqrt(ctpos.x*ctpos.x+ctpos.y*ctpos.y));
    }
    if(stpl.size() == 2)
    {
        ct[calMode].u = stpl[0].toDouble();
        ct[calMode].v = stpl[1].toDouble();
    }
    Config jz(CPINFO);
    for(int i=0;i<4;i++)
    {
        if(abs(dis[i]) < 1 && abs(dis[i]) > 0.001)
            continue;
        _sys_config.put(SETSPEED, S50);
        _sys_config.sync();
        _robot->absoluteMove(pos[i]);
        mode = 1;// 迭代x
        cpos = pos[i];
        cindex = i+1;
        double minf=minC2Line();
        pos[i] = _robot->getCurrPos(RobotTCP::UPLOAD);
        dis[i] = minf;
        string posi = "laserPos"+to_string(i+1);
        jz.put(posi+int2str(calMode), pos[i]);
        jz.put(posi+"f"+int2str(calMode),dis[i]);
        jz.put(posi+"u"+int2str(calMode),cCenter[i+1].u);
    }
    jz.sync();
    initUI();
}

void GridDev::on_btn4p2abc_clicked()
{
    vector<RobotPos> tv;
    RobotPos pos = RobotPos::instance(ui->editface1->text().toStdString());
    tv.push_back(pos);
    pos = RobotPos::instance(ui->editface2->text().toStdString());
    tv.push_back(pos);
    pos = RobotPos::instance(ui->editface3->text().toStdString());
    tv.push_back(pos);
    pos = RobotPos::instance(ui->editface4->text().toStdString());
    tv.push_back(pos);
    double c[4];
    Config jz(CPINFO);
    if(cCenter[0].u !=0)
    {
        for(int i=1;i<=4;i++)
        {
            c[i-1] = cCenter[i].u;
        }
    }
    else
    {
        try{
            c[0] = jz.get<double>("laserPos1u"+int2str(calMode));
            c[1] = jz.get<double>("laserPos2u"+int2str(calMode));
            c[2] = jz.get<double>("laserPos3u"+int2str(calMode));
            c[3] = jz.get<double>("laserPos4u"+int2str(calMode));
        }catch(...){
            double sumLen;
            cindex = 1;
            calSum(tv[0], sumLen);
            cindex = 2;
            calSum(tv[1], sumLen);
            cindex = 3;
            calSum(tv[2], sumLen);
            cindex = 4;
            calSum(tv[3], sumLen);
            for(int i=1;i<=4;i++)
            {
                c[i-1] = cCenter[i].u;
            }
        }
        if(cCenter[1].u !=0)
        {
            jz.put("laserPos1u"+int2str(calMode),cCenter[1].u);
            jz.put("laserPos2u"+int2str(calMode),cCenter[2].u);
            jz.put("laserPos3u"+int2str(calMode),cCenter[3].u);
            jz.put("laserPos4u"+int2str(calMode),cCenter[4].u);
            jz.sync();
        }
    }
    QString ctstr = ui->editcenter->text();
    QStringList stpl = ctstr.split(",");
    if(stpl.size() == 2)
    {
        ct[calMode].u = stpl[0].toDouble();
        ct[calMode].v = stpl[1].toDouble();
    }
    Vector3d abc;
    points2face(tv,abc);
    double abcm = sqrt(abc.dot(abc));
    /// 平面法向量 定义为x方向(0yz平面)
    abc = abc/abcm;
    pos = RobotPos::instance(ui->editface0->text().toStdString());
    Matrix4d mpos;
    pos2Matrix(pos,mpos,EULER);
    double n1 = (ct[calMode].u-c[0])/(c[2]-c[0]);
    double n2 = (ct[calMode].u-c[1])/(c[3]-c[1]);
    double x_ = tv[3].x*n2+tv[1].x*(1-n2)-tv[2].x*n1-tv[0].x*(1-n1);
    double y_ = tv[3].y*n2+tv[1].y*(1-n2)-tv[2].y*n1-tv[0].y*(1-n1);
    double z_ = tv[3].z*n2+tv[1].z*(1-n2)-tv[2].z*n1-tv[0].z*(1-n1);
    /// 格点Z方向
    Vector3d vz = {x_,y_,z_};
    //// (vz +nemuta*abc)dot(abc)=0
    double nemuta = -vz.dot(abc);
    /// z方向与abc正交化
    Vector3d vz0 = vz+nemuta*abc;
    /// 单位化
    vz0 = vz0/sqrt(vz0.dot(vz0));
    /// 叉积得到y轴方向
    Vector3d vy0 = vz0.cross(abc);
    /// 激光面相对于base的旋转矩阵
    Matrix4d mt2f = Matrix4d::Zero(4,4);
    mt2f.block(0,0,3,1) = abc;
    mt2f.block(0,1,3,1) = vy0;
    mt2f.block(0,2,3,1) = vz0;
    mt2f(3,3) = 1;
    Matrix4d mc2f = mpos.inverse()*mt2f;
    RobotPos rc2f;
    Matrix2pos(mc2f,rc2f,EULER);
    jz.put<RobotPos>("c2f"+int2str(calMode),rc2f);
    RobotPos pos0r = RobotPos::instance(ui->editface0->text().toStdString());
    Vector3d abc2 = {pos0r.x,pos0r.y,0};
    abc2 = abc2/sqrt(abc2.dot(abc2));
    Vector3d vz2 = {0,0,-1};
    Vector3d vy2 = vz2.cross(abc2);
    /// 希望激光面呈现的欧拉角
    Matrix4d mt2f2 = Matrix4d::Zero(4,4);
    mt2f2.block(0,0,3,1) = abc2;
    mt2f2.block(0,1,3,1) = vy2;
    mt2f2.block(0,2,3,1) = vz2;
    mt2f2(3,3) = 1;
    Matrix4d mt2b2 = mt2f2*mc2f.inverse();
    RobotPos pos_;
    Matrix2pos(mt2b2,pos_,EULER);
    pos_.x = pos.x;
    pos_.y = pos.y;
    pos_.z = pos.z;
    pos_.tcp = UPLOAD;
    jz.put<RobotPos>("laserPos0_",pos_);
    jz.sync();
    ui->editface->setText(pos2Str(pos_));
    initUI();
}

void GridDev::on_btn4p2abc_2_clicked()
{
    Config jz(CPINFO);
    RobotPos pos_ ;
    try
    {
        pos_ = jz.get<RobotPos>("laserPos0_");
    }
    catch(...)
    {
        QMessageBox::about(NULL,"提示","请先按拟合!");
        return;
    }
    _sys_config.put(SETSPEED, S50);
    _sys_config.sync();
    _robot->absoluteMove(pos_);
    mode = 0;
    cpos = pos_;
    cindex = 5;
    point3d<double> dp;
    double minf;
    int counts = 10;
    ajustCycleAndLine(pos_,dp,minf,counts);
    pos_.x+=dp._x;
    pos_.y+=dp._y;
    pos_.z+=dp._z;
    jz.put("laserPos5"+int2str(calMode),pos_);
    jz.put("laserPos5f"+int2str(calMode),minf);
    jz.put("laserPos5u"+int2str(calMode),cCenter[5].u);
    jz.sync();
    initUI();
}

void GridDev::on_initGrid_clicked()
{
    RobotPos pos;
    if(!str2Robot(ui->editface->text().toStdString(),pos))
    {
        QMessageBox::about(NULL,"提示","请先完成拟合!");
    }
    ui->twtgrid->setRowCount(81);
    ui->twtgrid->setColumnCount(6);
    ifstream ifs;
    ifs.open("etc/grid"+int2str(calMode)+".txt");
    Vector3d abc2 = {pos.x,pos.y,0};
    abc2 = abc2/sqrt(abc2.dot(abc2));
    Vector3d vz2 = {0,0,-1};
    Vector3d vy2 = vz2.cross(abc2);
    for(int i=0;i<9;i++)
    {
        for(int j=0;j<9;j++)
        {
            double dy = 2*(0.5-i%2)*(j*4-16);
            QString ijstr = QString::number(dy)+" "+QString::number(i*4-16);
            ui->twtgrid->setItem(i*9+j,0,new QTableWidgetItem(ijstr));
            RobotPos posij = pos;
            posij.x += -dy*vy2(0,0);
            posij.y += -dy*vy2(1,0);
            posij.z += i*4-16;
            ui->twtgrid->setItem(i*9+j,1,new QTableWidgetItem(pos2Str(posij)));
            ui->twtgrid->setItem(i*9+j,2,new QTableWidgetItem(""));
            ui->twtgrid->setItem(i*9+j,3,new QTableWidgetItem(""));
            ui->twtgrid->setItem(i*9+j,4,new QTableWidgetItem(""));
            ui->twtgrid->setItem(i*9+j,5,new QTableWidgetItem(""));
            try
            {
                double uvstr[5];
                ifs>>uvstr[0]>>uvstr[1]>>uvstr[2]>>uvstr[3]>>uvstr[4];
                double u0 = uvstr[3]+ct[calMode].u;
                double v0 = uvstr[4]+ct[calMode].v;
                ui->twtgrid->item(i*9+j,2)->setText(QString::number(u0));
                ui->twtgrid->item(i*9+j,3)->setText(QString::number(v0));
            }catch(...){};
        }
    }
}

void GridDev::on_btnGrid_clicked()
{
    Config grdPos("jz/grid_pos.txt");
    vector<UV> girduv;
    for(int i=0;i<81;i++)
    {
        RobotPos pos;
        QString posq = ui->twtgrid->item(i,1)->text();
        if(!str2Robot(posq.toStdString(),pos))
            continue;
        string poss = "pos"+to_string(i);
        grdPos.put(poss+int2str(calMode),posq.toStdString());
        UV uv;
        calGrid(pos,i,uv);
        girduv.push_back(uv);
        ui->twtgrid->item(i,2)->setText(QString::number(uv.u));
        ui->twtgrid->item(i,3)->setText(QString::number(uv.v));
    }
    grdPos.sync();
    Config calconf("etc/calibration.info");
    calconf.put("cameraCenterUV"+int2str(calMode)+".u",girduv[40].u);
    calconf.put("cameraCenterUV"+int2str(calMode)+".v",girduv[40].v);
    calconf.sync();
    ofstream ofs2;
    ofs2.open("etc/grid"+int2str(calMode)+".txt");
    for(int i=0;i<81;i++)
    {
        ofs2<<"0 "<<ui->twtgrid->item(i,0)->text().toStdString()<<" "<<girduv[i].u-girduv[40].u<<" "<<girduv[i].v-girduv[40].v<<endl;
    }
    ofs2.close();
    SensorUI::initMap(calMode);
}

void GridDev::openFlower()
{
    Config cali("etc/calibration.info");
    Config jz(CPINFO);
    RobotPos ctPos = jz.get<RobotPos>("centerPos"+int2str(calMode));
    Matrix4d mcurPos;  ///  焊枪抵住圆心时,法兰的基础位姿
    pos2Matrix(ctPos,mcurPos,EULER);
    RobotPos tcp = RobotPos::instance(ui->edithq2fl->text().toStdString());
    cali.put("TINF",tcp);
    cali.sync();
    Matrix4d mtcp; /// 焊枪相对与法兰的pos(即TCP)
    pos2Matrix(tcp,mtcp,EULER);
    Matrix4d mt2b = mcurPos*mtcp;
    RobotPos t2b;  /// 焊枪抵圆班圆心时,焊枪的基础位姿
    Matrix2pos(mt2b,t2b,EULER);
    RobotPos mcf2b = RobotPos::instance(ui->editface->text().toStdString());
    Matrix4d f2b;  /// 激光线经过圆心时,法兰的基础位姿
    pos2Matrix(mcf2b,f2b,EULER);
    vector<RobotPos> posvec;
    // 纠正矩阵
    Matrix4d mc2f = f2b.inverse()*mt2b; /// 初步估計
    RobotPos c2f;
    Matrix2pos(mc2f,c2f,EULER);
    auto c2f0 = jz.get<RobotPos>("c2f"+int2str(calMode));
    c2f.a = c2f0.a;
    c2f.b = c2f0.b;
    c2f.c = c2f0.c;
    jz.put("c2f"+int2str(calMode), c2f);
    jz.sync();
    double sx = 40;
    double sy = 30;
    Matrix4d Rx;
    Rx<<1,    0,        0,     0,
            0,cos(sx*CV_PI/180),-sin(sx*CV_PI/180),0,
            0,sin(sx*CV_PI/180), cos(sx*CV_PI/180),0,
            0,    0,        0,     1;
    Matrix4d Ry;
    Ry << cos(sy*CV_PI/180), 0, sin(sy*CV_PI/180), 0,
          0, 1, 0, 0,
          -sin(sy*CV_PI/180), 0, cos(sy*CV_PI/180), 0,
          0, 0, 0, 1;
    int mxys[8] = {7,6,3,0,1,2,5,8};
    RobotPos posxy0;
    Matrix2pos(f2b,posxy0,EULER);
    posvec.push_back(posxy0);
    double sxy = sx<sy?sx:sy;
    for(int e = 0;e<8;e++)
    {
        int ex = mxys[e]/3;
        int ey = mxys[e]%3;
        for(int i=0;i<sxy;i+=5)
        {
            Matrix4d Rix;
            Rix<<1,    0,        0,     0,
                    0,cos(i*CV_PI/180),-sin(i*CV_PI/180),0,
                    0,sin(i*CV_PI/180), cos(i*CV_PI/180),0,
                    0,    0,        0,     1;
            Matrix4d Riy;
            Riy << cos(i*CV_PI/180), 0, sin(i*CV_PI/180), 0,
                  0, 1, 0, 0,
                  -sin(i*CV_PI/180), 0, cos(i*CV_PI/180), 0,
                  0, 0, 0, 1;
            Matrix4d mi = f2b*mc2f;
            switch(ex)
            {
             case 0:
                mi = mi*Rx.inverse()*Rix;
                break;
             case 2:
                mi = mi*Rx*Rix.inverse();
               break;
            }
            switch(ey)
            {
             case 0:
                mi = mi*Ry.inverse()*Riy;
                break;
             case 2:
                mi = mi*Ry*Riy.inverse();
               break;
            }
            mi = mi*mc2f.inverse();
            RobotPos posxy1;
            Matrix2pos(mi,posxy1,EULER);
            vector<RobotPos> vmi;
            vmi.push_back(posxy1);
            if(!Pos2Joint2(vmi))
            {
                posvec.push_back(posxy1);
                break;
            }
        }
    }
    int posc = 0;
    for(auto pos : posvec)
    {
        jz.put("pos"+to_string(posc)+int2str(calMode), pos);
        jz.put("pos"+to_string(posc)+"_"+int2str(calMode),RobotPos::instance());
        jz.put("pos"+to_string(posc)+"_f"+int2str(calMode),0);
        posc++;
    }
    jz.sync();
    initUI();
}

void GridDev::on_btnxz_clicked()
{
    QList<QTableWidgetItem*> targetRows;
    if(autoMode)
    {
        for(int i=0;i<ui->twtball->rowCount();i++)
            targetRows.push_back(ui->twtball->item(i,0));
    }
    else
    {
        targetRows = ui->twtball->selectedItems();
        if(targetRows.size() == 0)
        {
            QMessageBox::about(NULL,"提示","请选择要修正的姿态！");
            return;
        }
        //QMessageBox::about(NULL,"提示","请将机器人再现速度调至10%！");
    }
    for(auto & sitem : targetRows)
    {
        if(!sitem) continue;
        int row = ui->twtball->row(sitem);
        int col = ui->twtball->column(sitem);
        if(col != 0)
            continue;
        RobotPos pos;
        QString poshstr = ui->twtball->item(row,1)->text();
        QString posstr = sitem->text();
        if(poshstr.split(",").length() == 6)
        {
            pos = RobotPos::instance(poshstr.toStdString());
            if(pos.dis({0,0,0,0,0,0,UPLOAD,0,0,0}) < 1)
            {
                pos = RobotPos::instance(posstr.toStdString());
            }
        }
        else
        {
            pos = RobotPos::instance(posstr.toStdString());
        }
        mode = 4;// 迭代xyz
#ifndef KUKA
        _sys_config.put(SETSPEED, S500);
        _sys_config.sync();
#endif
        _sys_config.put(SETSPEED, S500);
        _sys_config.sync();
        _robot->absoluteMove(pos,true,true);
        cindex = row;
        cpos = pos;
        point3d<double> dp;
        double minf;
        int counts = 10;
        ajustCycleAndLine(pos,dp,minf,counts);
        RobotPos cur = _robot->getCurrPos(RobotTCP::UPLOAD);
        cpos.x = cur.x;
        cpos.y = cur.y;
        cpos.z = cur.z;
        Config jz(CPINFO);
        jz.put("pos"+to_string(row)+"_"+int2str(calMode),cpos);
        jz.put("pos"+to_string(row)+"_f"+int2str(calMode),minf);
        jz.sync();
#ifndef KUKA
        _sys_config.put(SETSPEED, S500);
        _sys_config.sync();
#endif
    }
    initUI();
}

void getMIdPos(vector<RobotPos> mids, RobotPos & midpos)
{
    double minx=9999,miny=9999,minz=9999,maxx=-9999,maxy=-9999,maxz=-9999;
    for(auto p : mids)
    {
        minx = minx>p.x?p.x:minx;
        miny = miny>p.y?p.y:miny;
        minz = minz>p.z?p.z:minz;
        maxx = maxx<p.x?p.x:maxx;
        maxy = maxy<p.y?p.y:maxy;
        maxz = maxz<p.z?p.z:maxz;
    }
    midpos.x = (maxx+minx)/2;
    midpos.y = (maxy+miny)/2;
    midpos.z = (maxz+minz)/2;
}

void GridDev::on_btnnihe_clicked()
{
    vector<matrixIndex> qposs;
    for(int i=0;i<ui->twtball->rowCount();i++)
    {
        double mis = ui->twtball->item(i,2)->text().toDouble();
        if(mis <= 0 || mis > 0.2)
            continue;
        QString posir = ui->twtball->item(i,1)->text();
        RobotPos posi;
        if(!str2Robot(posir.toStdString(),posi))
            continue;
        Matrix4d mposi;
        pos2Matrix(posi,mposi,EULER);
        matrixIndex mi;
        mi.M = mposi;
        mi.i = i;
        qposs.push_back(mi);
    }
    int qpsize = qposs.size();
    //计算中心点相对于法兰的xyz
    int ms = qpsize*(qpsize-1)*3/2;
    MatrixXd M45 = MatrixXd::Zero(ms,3);
    VectorXd v45 = VectorXd::Zero(ms,1);
    int lin = 0;
    for(int i=0;i<qpsize-1;i++)
    {
        for(int j=i+1;j<qpsize;j++)
        {
            M45.block(3*lin,0,3,3) = qposs[i].M.block(0,0,3,3)-qposs[j].M.block(0,0,3,3);
            v45.block(3*lin,0,3,1) = qposs[j].M.block(0,3,3,1)-qposs[i].M.block(0,3,3,1);
            lin ++;
        }
    }
    Matrix3d mm = M45.transpose()*M45;
    Vector3d hqjz =mm.inverse()*M45.transpose()*v45;
    Config jz(CPINFO);
    RobotPos rc2f = jz.get<RobotPos>("c2f"+int2str(calMode));
    rc2f.x = hqjz(0,0);
    rc2f.y = hqjz(1,0);
    rc2f.z = hqjz(2,0);
    Config cali("etc/calibration.info");
    RobotPos hq2fl = cali.get<RobotPos>("TINF");
    Matrix4d mc2f;
    pos2Matrix(rc2f,mc2f,EULER);
    if(calMode == 0)
    {
        hq2fl.a = rc2f.a;
        hq2fl.b = rc2f.b;
        hq2fl.c = rc2f.c;
    }
    else
    {
        RobotPos c2f0 = jz.get<RobotPos>("c2f");
        hq2fl.a = c2f0.a;
        hq2fl.b = c2f0.b;
        hq2fl.c = c2f0.c;
    }
    cali.put("TINF", hq2fl);
    cali.sync();
    jz.put("c2f"+int2str(calMode), rc2f);
    Matrix4d mh2f;
    pos2Matrix(hq2fl,mh2f,EULER);
    // t2c
    Matrix4d mt2c = mc2f.inverse()*mh2f;
    RobotPos rt2c;
    Matrix2pos(mt2c,rt2c,EULER);
    Matrix4d mc2t = mt2c.inverse();
    if(calMode == 0)
    {
        rt2c.a = 0;
        rt2c.b = 0;
        rt2c.c = 0;
    }
    cali.put("TINC"+int2str(calMode), rt2c);
    RobotPos c2t;
    Matrix2pos(mc2t,c2t,EULER);
    if(calMode == 0)
    {
        c2t.a = 0;
        c2t.b = 0;
        c2t.c = 0;
    }
    cali.put("CINT"+int2str(calMode), c2t);
    cali.sync();
    // 拟合球心
    int nh = qpsize*(qpsize-1)/2;
    MatrixXd A = MatrixXd::Zero(nh,3);
    VectorXd B = VectorXd::Zero(nh,1);
    int nl = 0;
    for(int m=0;m<qpsize-1;m++)
    {
        for(int n=m+1;n<qpsize;n++)
        {
            A(nl,0) = qposs[m].M(0,3)-qposs[n].M(0,3);
            A(nl,1) = qposs[m].M(1,3)-qposs[n].M(1,3);
            A(nl,2) = qposs[m].M(2,3)-qposs[n].M(2,3);
            B(nl,0) = (qposs[m].M(0,3)*qposs[m].M(0,3)+qposs[m].M(1,3)*qposs[m].M(1,3)+qposs[m].M(2,3)*qposs[m].M(2,3)
                       -qposs[n].M(0,3)*qposs[n].M(0,3)-qposs[n].M(1,3)*qposs[n].M(1,3)-qposs[n].M(2,3)*qposs[n].M(2,3))/2,
            nl++;
        }
    }
    Matrix3d AA = A.transpose()*A;
    Vector3d C =AA.inverse()*A.transpose()*B;
    // 计算法兰和球心的距离
    double sumdis = 0;
    vector<double> disv;
    for(int l = 0;l<qpsize;l++)
    {
        Matrix4d mi = qposs[l].M;
        double dis = sqrt((mi(0,3)-C(0,0))*(mi(0,3)-C(0,0))+(mi(1,3)-C(1,0))*(mi(1,3)-C(1,0))+(mi(2,3)-C(2,0))*(mi(2,3)-C(2,0)));
        disv.push_back(dis);
        sumdis += dis;
    }
    // 平均距离
    double avgDis = sumdis/qpsize;
    // 拟合误差列表
    double bzc;
    for(int d=0;d<qpsize;d++)
    {
        bzc += pow(disv[d]-avgDis,2);
        QString  distr = QString::number(disv[d]-avgDis,'f',3);
        ui->twtball->item(qposs[d].i,3)->setText(distr);
    }
    /// 收縮求半徑
    ofstream ofs;
    ofs.open("jz/bools_info.txt");
    ofs<<"rc2f"<<rc2f.x<<","<<rc2f.y<<","<<rc2f.z<<","<<rc2f.a<<","<<rc2f.b<<","<<rc2f.c<<endl;
    ofs<<"////////////////////////////////////////////////////////"<<endl;\
    vector<RobotPos> mids; // 收縮後的點列
    for(auto mi :qposs)
    {
        Matrix4d mib = mi.M*mc2f;
        RobotPos pib;
        Matrix2pos(mib,pib,EULER);
        mids.push_back(pib);
        ofs<<pib.x<<","<<pib.y<<","<<pib.z<<","<<pib.a<<","<<pib.b<<","<<pib.c<<endl;
    }
    RobotPos midpos = {0,0,0,0,0,0,UPLOAD};
    getMIdPos(mids,midpos);
    ofs<<"////////////////////////////////////////////////////////"<<endl;
    ofs<<midpos.x<<","<<midpos.y<<","<<midpos.z<<endl;
    ofs<<"////////////////////////////////////////////////////////"<<endl;
    for(auto mi :mids)
    {
        point3d<double> dpi = {mi.x-midpos.x,mi.y-midpos.y,mi.z-midpos.z};
        double mid = dpi.dist({0,0,0});
        ofs<<"dx:"<<dpi._x
          <<","<<"dy:"<<dpi._y
          <<","<<"dz:"<<dpi._z
         <<","<<"dr:"<<mid<<endl;
    }
    ofs.close();
    initUI();
}

void GridDev::on_btnsave_clicked()
{
    Config cali("etc/calibration.info");
    Config jz(CPINFO);
    calMode = ui->comboBox->currentIndex();
    jz.put("laser.mode",calMode);
    QStringList uvl = ui->editcenter->text().split(",");
    int bgcicle = ui->editcicle->text().toInt();
    int bgline = ui->editline->text().toInt();
    int roir = ui->editRoi->text().toInt();
    if(uvl.size() == 2)
    {
        double cu = uvl[0].toDouble();
        double cv = uvl[1].toDouble();
        cali.put("cameraCenterUV"+int2str(calMode)+".u",cu);
        cali.put("cameraCenterUV"+int2str(calMode)+".v",cv);
        ct[calMode] = {cu,cv};
    }
    RobotPos hq2fl = RobotPos::instance(ui->edithq2fl->text().toStdString());
    RobotPos old_tinf = cali.get<RobotPos>("TINF");
    RobotPos old_tinC = cali.get<RobotPos>("TINC");
    RobotPos old_tinC1 = cali.get<RobotPos>("TINC1");
    old_tinC = old_tinC<<(!old_tinf)<<hq2fl;
    cali.put<RobotPos>("TINC",old_tinC);
    cali.put<RobotPos>("CINT",(!old_tinC));
    old_tinC1 = old_tinC1<<(!old_tinf)<<hq2fl;
    cali.put<RobotPos>("TINC1",old_tinC1);
    cali.put<RobotPos>("CINT1",(!old_tinC1));
    cali.put<RobotPos>("TINF",hq2fl);
    RobotPos ctpos = RobotPos::instance(ui->editctpos->text().toStdString());
    jz.put<RobotPos>("centerPos"+int2str(calMode),ctpos);
    jz.put("laser.line"+int2str(calMode),bgline);
    jz.put("laser.circle"+int2str(calMode),bgcicle);
    jz.put("laser.roi"+int2str(calMode),roir);
    jz.sync();
    cali.put<RobotPos>("FINT",!hq2fl);
    cali.sync();
    clearFloder();
}

/// 迭代使 圓斑圓心\激光線\中心點 重合
void GridDev::ajustCycleAndLine(RobotPos & r, point3d<double> & dp, double & minf,int & counts)
{
    Config conf("etc/calibration.info");
    UV ccuv = conf.get<UV>("cameraCenterUV"+int2str(calMode));
    Config jz(CPINFO);
    RobotPos c2f = jz.get<RobotPos>("c2f"+int2str(calMode));
    Matrix4d mr;
    pos2Matrix(r,mr,EULER);
    Matrix4d mc2f;
    pos2Matrix(c2f,mc2f,EULER);
    Matrix4d mc2b = mr*mc2f;
    if(dp.dist({0,0,0}) > 0.05)
    {
        Matrix4d dmp;
        dmp<<   1,0,0,dp._x,
                0,1,0,dp._y,
                0,0,1,dp._z,
                0,0,0,1;
        Matrix4d mf2b_ = mc2b*dmp*mc2f.inverse();
        Matrix2pos(mf2b_,r,EULER);
        r.tcp = UPLOAD;
        _robot->absoluteMove(r);
        dp = {0,0,0};
    }
    minf = -1;
    UV cy;
    vector<UV> line;
    if(!calSumP3(r,cy,line))
         return;
    ///計算出圓心到直線的投影uv
    UV ty;
    Matrix2d ML;
    ML<<line[1].u-line[0].u,line[1].v-line[0].v,line[1].v-line[0].v,line[0].u-line[1].u;
    Vector2d VL;
    VL<<(line[1].u-line[0].u)*cy.u+(line[1].v-line[0].v)*cy.v,(line[1].v-line[0].v)*line[0].u+(line[0].u-line[1].u)*line[0].v;
    Vector2d vty = ML.inverse()*VL;
    ty = {vty(0,0),vty(1,0)};
    double dx = sqrt(pow(ty.u-cy.u,2)+pow(ty.v-cy.v,2))/24;
    dx = cy.v<ty.v?-dx:dx;
    float dy,dz;
    laser_yz((float)(ty.u-ccuv.u),(float)(ty.v-ccuv.v),dy,dz);
    dp = {dx,dy,dz};
    minf = dp.dist({0,0,0});
    if(minf > 0.1 && counts > 0)
    {
        cout<<"cy.u"<<cy.u<<",cy.v"<<cy.v<<endl;
        counts --;
        ajustCycleAndLine(r,dp,minf,counts);
    }
    else if(minf < 0.1)
    {
        cout<<"cy.u"<<cy.u<<",cy.v"<<cy.v<<endl;
        cout<<"ty.u"<<ty.u<<",ty.v"<<ty.v<<endl;
    }
    else
    {
        //QMessageBox::about(NULL,"提示","迭代10次,仍未找到!");
    }
}

/// 多球拟合脚本
void GridDev::on_btnnihe_2_clicked()
{
    //////////////todo///////////////
    ifstream ifs;
    ifs.open("jz/bool.txt");
    string s;
    vector<vector<Matrix4d>> bools;
    int Msize = 0;
    while(getline(ifs,s))
    {
        QString posQ = QString::fromStdString(s);
        QStringList posl = posQ.split("\t");
        if(posl.size() < 4)
            continue;
        vector<Matrix4d> vpi;
        for(auto pi : posl)
        {
            RobotPos ri;
            if(!str2Robot(pi.toStdString(),ri))
                continue;
            Matrix4d mi;
            pos2Matrix(ri,mi,EULER);
            vpi.push_back(mi);
        }
        bools.push_back(vpi);
        Msize += vpi.size()*(vpi.size()-1)*3/2;
    }
    ifs.close();
    MatrixXd M89 = MatrixXd::Zero(Msize,3);
    VectorXd V89 = VectorXd::Zero(Msize,1);
    int lin = 0;
    for(auto vp : bools)
    {
        for(int i=0;i<vp.size();i++)
        {
            for(int j=i+1;j<vp.size();j++)
            {
                M89.block(3*lin,0,3,3) = vp[i].block(0,0,3,3)-vp[j].block(0,0,3,3);
                V89.block(3*lin,0,3,1) = vp[j].block(0,3,3,1)-vp[i].block(0,3,3,1);
                lin ++;
            }
        }
    }
    Matrix3d mm = M89.transpose()*M89;
    Vector3d hqjz =mm.inverse()*M89.transpose()*V89;
    RobotPos rc2f;
    Config jz(CPINFO);
    try{rc2f = jz.get<RobotPos>("c2f"+int2str(calMode));}catch(...){}
    rc2f.x = hqjz(0,0);
    rc2f.y = hqjz(1,0);
    rc2f.z = hqjz(2,0);
    //rc2f = {-17.812,95.252,650.077,99.307,13.867,177.490};
    Matrix4d mc2f;
    pos2Matrix(rc2f,mc2f,EULER);
    ofstream ofs;
    ofs.open("jz/bools_info.txt");
    ofs<<"rc2f"<<rc2f.x<<","<<rc2f.y<<","<<rc2f.z<<","<<rc2f.a<<","<<rc2f.b<<","<<rc2f.c<<endl;
    ofs<<"////////////////////////////////////////////////////////"<<endl;
    for(auto bi : bools)
    {
        vector<RobotPos> mids;
        for(auto mi :bi)
        {
            Matrix4d mib = mi*mc2f;
            RobotPos pib;
            Matrix2pos(mib,pib,EULER);
            ofs<<pib.x<<","<<pib.y<<","<<pib.z<<","<<pib.a<<","<<pib.b<<","<<pib.c<<endl;
            mids.push_back(pib);
        }
        RobotPos midpos = RobotPos::instance();
        getMIdPos(mids,midpos);
        ofs<<"////////////////////////////////////////////////////////"<<endl;
        ofs<<midpos.x<<","<<midpos.y<<","<<midpos.z<<endl;
        ofs<<"////////////////////////////////////////////////////////"<<endl;
        for(auto mi :mids)
        {
            point3d<double> dpi = {mi.x-midpos.x,mi.y-midpos.y,mi.z-midpos.z};
            ofs<<"dx:"<<dpi._x
              <<","<<"dy:"<<dpi._y
              <<","<<"dz:"<<dpi._z
             <<","<<"dr:"<<dpi.dist({0,0,0})<<endl;
        }
        ofs<<endl;
    }

    ofs.close();
    return;
    //////////////todo//////////////////
}

void GridDev::on_checkGrid_clicked()
{
    maxGridMis_y = 0;
    maxGridMis_z = 0;
    for(int i=0;i<81;i++)
    {
        QString uvs = ui->twtgrid->item(i,0)->text();
        QStringList l = uvs.split(" ");
        if(l.size()<2)
            continue;
        point3d<double> p0 = {0,l[0].toDouble(),l[1].toDouble()};
        QString u = ui->twtgrid->item(i,2)->text();
        QString v = ui->twtgrid->item(i,3)->text();
        float y,z;
        laser_yz(u.toFloat()-ct[calMode].u,v.toFloat()-ct[calMode].v,y,z, calMode);
        double mis_y = y-p0._y;
        maxGridMis_y = maxGridMis_y<abs(mis_y)?abs(mis_y):maxGridMis_y;
        double mis_z = z-p0._z;
        maxGridMis_z = maxGridMis_z<abs(mis_z)?abs(mis_z):maxGridMis_z;
        ui->twtgrid->item(i,4)->setText(QString::number(mis_y,'f',3));
        ui->twtgrid->item(i,5)->setText(QString::number(mis_z,'f',3));
    }
    /// next
    if(ui->twtgrid->rowCount() < 40)
        return;
    Config jz(CPINFO);
    for(int i=0;i<9;i++)
    {
        jz.put("pos"+to_string(i)+"_"+int2str(calMode),RobotPos::instance());
        jz.put("pos"+to_string(i)+"_f"+int2str(calMode),0);
    }
    jz.sync();
    openFlower();
    initUI();
}

/// 全自动纠正按钮
void GridDev::on_autoWork_clicked()
{
    Config conf("etc/calibration.info");
    int repeatTime = 3;
    /// 第一步：中心点修正
    do{
        on_btnface_clicked();
        on_btn4p_clicked();
        on_btn4p2abc_clicked();
        on_btn4p2abc_2_clicked();
        double mis = conf.get<double>("laserPos5f"+int2str(calMode),999);
        if(mis < 0.5)
        {
            break;
        }
        repeatTime--;
    }while(repeatTime>0);
    if(repeatTime == 0)
    {
        QMessageBox::about(NULL,"警告","传感器变化较大，请联系技术人员！");
        return;
    }
    on_initGrid_clicked();
    on_btnGrid_clicked();
    /// 验证格点
    on_checkGrid_clicked();
    if(maxGridMis_z > 1 || maxGridMis_y>0.3)
    {
        QMessageBox::about(NULL,"提示","格点测量误差过大，自动纠正失败，请重新纠正！");
        return;
    }
    autoMode = true;
    on_btnxz_clicked();
    autoMode = false;
    on_btnnihe_clicked();
    QMessageBox::about(NULL,"提示","纠正结束!");
}

void GridDev::on_comboBox_currentIndexChanged(const QString &arg1)
{
    Config conf(CPINFO);
    conf.put<int>("laser.mode",ui->comboBox->currentIndex());
    conf.sync();
    initUI();
}

///////////////////////////////////以下非纠正代码/////////////////////////////////////////
void GridDev::on_testAir_clicked()
{
    static bool kg = false;
    kg = !kg;
    std::thread([this]
    {
        ofstream ofs;
        ofs.open("jz/uv.log");
        int count = 0;
        while(kg)
        {
            cout<<count;
            _mb_ptr->openAirValve();
            this_thread::sleep_for(std::chrono::seconds(14));
            cout<<"\t气阀打开";
            count++;
            if(count % 10 == 0)
            {
                _mb_ptr->openLaser();
                _camera_ptr->setExposure(500);
                this_thread::sleep_for(std::chrono::seconds(1));
                Mat m1 = _camera_ptr->takePicture();
                cv::imwrite("jz/openAirValve/"+to_string(count)+".png",m1);
                _mb_ptr->closeLaser();
            }
            _mb_ptr->closeAirValve();
            this_thread::sleep_for(std::chrono::seconds(14));
            if(count % 10 == 0)
            {
                _mb_ptr->openLaser();
                _camera_ptr->setExposure(500);
                this_thread::sleep_for(std::chrono::seconds(1));
                Mat m1 = _camera_ptr->takePicture();
                cv::imwrite("jz/closeAirValve/"+to_string(count)+".png",m1);
                _mb_ptr->closeLaser();
            }
            cout<<"\t气阀关闭"<<endl;
            this_thread::sleep_for(std::chrono::seconds(2));
        }
        ofs.close();
    }).detach() ;
}

void PathBuild(RobotPos pos, vector<RobotPos> &Path)
{
    RobotPos p = pos;
    double r1 = 50;
    double r2 = 100;
    double dx = 1;
    double N1 = 2*r1 / dx;
    double N2 = 2*r2 / dx;
    for(int i=0; i<N1; i++){
        double x = i*dx;
        double y = -sqrt((4.0/3)*r1*r1-(r1-x)*(r1-x))+(r1*sqrt(3)/3);
        p.x = pos.x + x;
        p.y = pos.y + y;
        Path.push_back(p);
    }
    for(int i=0; i<N2; i++){
        double x = i*dx;
        double y = sqrt((4.0/3)*r2*r2-(r2-x)*(r2-x))-(r2*sqrt(3)/3);
        p.x = pos.x + 2*r1 + x;
        p.y = pos.y + y;
        Path.push_back(p);
    }
    for(int i=N2; i>0; i--){
        double x = i*dx;
        double y = -sqrt((4.0/3)*r2*r2-(r2-x)*(r2-x))+(r2*sqrt(3)/3);
        p.x = pos.x + 2*r1 + x;
        p.y = pos.y + y;
        Path.push_back(p);
    }
    for(int i=N1; i>0; i--){
        double x = i*dx;
        double y = sqrt((4.0/3)*r1*r1-(r1-x)*(r1-x))-(r1*sqrt(3)/3);
        p.x = pos.x + x;
        p.y = pos.y + y;
        Path.push_back(p);
    }
}

void GridDev::on_pushButton_8_clicked()
{
    vector<RobotPos> Path;
    auto r = _robot->getCurrPos(RobotTCP::UPLOAD);
    PathBuild(r,Path);
    ofstream ofs;
    ofs.open("autospeed.txt");
    for(auto & p : Path)
    {
        ofs<<p.x<<","<<p.y<<","<<p.z<<","<<p.a<<","<<p.b<<","<<p.c<<endl;
    }
    ofs.close();
}

void GridDev::on_mathBtn_clicked()
{
    QString str1 = ui->temp1->text();
    QString str2 = ui->temp2->text();
    QString str3 = ui->temp3->text();
    QString str4 = ui->temp4->text();
    QString str5 = ui->temp5->text();
    RobotPos r1 = RobotPos::instance(str1.toStdString());
    RobotPos r2 = RobotPos::instance(str2.toStdString());
    RobotPos r3 = RobotPos::instance(str3.toStdString());
    RobotPos r4 =  RobotPos::instance(str4.toStdString());
    if(ui->cbx_zyx4->isChecked())
    {
        Matrix4d mr4;
        pos2Matrix(r4,mr4,zyx);
        Matrix2pos(mr4,r4,zyz);
    }
    RobotPos r5 = RobotPos::instance(str5.toStdString());
    if(str5 == "")
    {
        r5 = r1<<r2<<r3<<r4;
        ui->temp5->setText(pos2Str(r5));
        return;
    }
    if(str4 == "")
    {
        r4 = (!r3)<<(!r2)<<(!r1)<<r5;
        ui->temp4->setText(pos2Str(r4));
        return;
    }
    if(str3 == "")
    {
        r3 = (!r2)<<(!r1)<<r5<<(!r4);
        ui->temp3->setText(pos2Str(r3));
        return;
    }
    if(str2 == "")
    {
        r2 = (!r1)<<r5<<(!r4)<<(!r3);
        ui->temp2->setText(pos2Str(r2));
        return;
    }
    if(str1 == "")
    {
        r1 = r5<<(!r4)<<(!r3)<<(!r2);
        ui->temp1->setText(pos2Str(r1));
        return;
    }
}

/// 检查传感器有无偏移
void GridDev::on_btnLocation_clicked()
{
    RobotPos r = RobotPos::instance("284.417,126.072,402.029,166.484,127.555,-2.370");
    double sumlen;
    calSum(r,sumlen);
}
