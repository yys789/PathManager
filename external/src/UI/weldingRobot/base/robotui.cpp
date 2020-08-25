#include "base/robotui.h"
#include "ui_robotui.h"

#include <iostream>
#include <thread>
#include <QMessageBox>

#include <eigen3/Eigen/Dense>
#include <QFileDialog>
#include "tools.h"

using namespace std ;
using namespace Eigen;

#include <boost/scope_exit.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

RobotUI::RobotUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotUI)
//    ,_expCheck(Singleton<expCheck>::get())
{
    _robot.selectRobot(RobotType::KAWASAKI) ;
    ui->setupUi(this);
    //_robot_ptr.reset(new RobotKUKA);
    QObject::connect(&_timerPos, SIGNAL(timeout()),
                     this, SLOT(updateRobotPosAndAxle())) ;
    _timerPos.start(100) ;
    logFlag = false;
    ui->currentTCP->setCurrentIndex(_robot->getInTCP());
    //_expCheck->startRobot();
}

RobotUI::~RobotUI()
{
    cout << "关闭\n" << flush ;
    //_robot->stop() ;
    delete ui;
}
void RobotUI::on_currentTCP_currentIndexChanged(const QString &arg1)
{
    if ( "焊枪" == arg1 ) {
        _robot->setInTCP(RobotTCP::UPLOAD) ;
    }
    else if ( "寻位中心点" == arg1 ) {
        _robot->setInTCP(RobotTCP::CAMERA) ;
    }
    else if ( "法兰" == arg1 ) {
        _robot->setInTCP(RobotTCP::FLANGE) ;
    }
    else if ( "跟踪中心点" == arg1 ) {
        _robot->setInTCP(RobotTCP::CAMERA1) ;
    }
}
void RobotUI::on_relativeMove_clicked()
{
    std::thread([this]{
        try {
            RobotPos relativePos ;
            relativePos.x = ui->dPosX->text().toDouble() ;
            relativePos.y = ui->dPosY->text().toDouble() ;
            relativePos.z = ui->dPosZ->text().toDouble() ;
            relativePos.a = ui->dPosA->text().toDouble() ;
            relativePos.b = ui->dPosB->text().toDouble() ;
            relativePos.c = ui->dPosC->text().toDouble() ;
            auto oldSpeed = _sys_config.get<double>("robot.speed") ;
            clock_t t1 = clock();
            BOOST_SCOPE_EXIT(oldSpeed, &_sys_config) {
                _sys_config.put("robot.speed", oldSpeed) ;
            }BOOST_SCOPE_EXIT_END

                    if ( _bWeldingOn ) {
                _sys_config.put("robot.speed", _sys_config.get<double>("robot.weldingSpeed")) ;
            }

            if ( !_sys_config.get<bool>("robot.bEular")) {
                _robot->relativeMove_axis( {relativePos},
                                           true,
                                           _bWeldingOn) ;
            }
            else {
                _robot->relativeMove( relativePos, true, _bWeldingOn ) ;
            }
            clock_t t2 = clock();
            int t = t2-t1;
        }
        catch( const boost::exception &e ) {
            DUMPERROR(e) ;
        }
    }).detach() ;
}

void RobotUI::on_getCurrPos_clicked()
{
    auto pos = _robot->getCurrPos();
    QString posStr = QString::number(pos.x,'f',3).append(",");
    posStr.append(QString::number(pos.y,'f',3)).append(",");
    posStr.append(QString::number(pos.z,'f',3)).append(",");
    posStr.append(QString::number(pos.a,'f',3)).append(",");
    posStr.append(QString::number(pos.b,'f',3)).append(",");
    posStr.append(QString::number(pos.c,'f',3));
    ui->coordinateString->setText(posStr);
}

void RobotUI::on_stopMove_clicked()
{
    // _robot->stopMove(RobotError::FORCESTOPMOVING) ;
}

void RobotUI::on_relativeMoveAxle_clicked()
{
    try {
        RobotAxle axle ;
        axle.a1 = ui->dA1->text().toDouble() ;
        axle.a2 = ui->dA2->text().toDouble() ;
        axle.a3 = ui->dA3->text().toDouble() ;
        axle.a4 = ui->dA4->text().toDouble() ;
        axle.a5 = ui->dA5->text().toDouble() ;
        axle.a6 = ui->dA6->text().toDouble() ;
        _robot->relativeMove_axle(axle, false) ;
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

// 更新界面机器人位置
void RobotUI::updateRobotPosAndAxle()
{
    auto pos = _robot->getCurrPos() ;
    auto axle = _robot->getCurrAxle() ;
    ui->currPosX->setText(QString::number(pos.x));
    ui->currPosY->setText(QString::number(pos.y));
    ui->currPosZ->setText(QString::number(pos.z));
    ui->currPosA->setText(QString::number(pos.a));
    ui->currPosB->setText(QString::number(pos.b));
    ui->currPosC->setText(QString::number(pos.c));
    ui->currA1->setText(QString::number(axle.a1));
    ui->currA2->setText(QString::number(axle.a2));
    ui->currA3->setText(QString::number(axle.a3));
    ui->currA4->setText(QString::number(axle.a4));
    ui->currA5->setText(QString::number(axle.a5));
    ui->currA6->setText(QString::number(axle.a6));
}

void RobotUI::on_robotMoveStarted()
{
}

void RobotUI::on_robotMoveStoped()
{
}

void RobotUI::on_clear_clicked()
{
    ui->dPosX->setText("");
    ui->dPosY->setText("");
    ui->dPosZ->setText("");
    ui->dPosA->setText("");
    ui->dPosB->setText("");
    ui->dPosC->setText("");
}

void RobotUI::on_abMove_clicked()
{
    QString str = ui->coordinateString->text();
    QStringList sl = str.split(",");
    if(sl.size() < 6)
        return;
    RobotPos f;
    f.x = sl[0].toDouble();
    f.y = sl[1].toDouble();
    f.z = sl[2].toDouble();
    f.a = sl[3].toDouble();
    f.b = sl[4].toDouble();
    f.c = sl[5].toDouble();
    f.tcp = UPLOAD;
//    int sudu = ui->suEdit->text().toInt();
//    if(sudu < 1)
//        sudu = 10;
//    _sys_config.put("robot.speed", sudu);
    double sudu = ui->suEdit->text().toDouble();
    if(sudu < 0.001)
        sudu = 0.001;
    _sys_config.put("robot.speed", sudu);
    _sys_config.sync();
    _robot->absoluteMove(f);
}

void RobotUI::on_RxyzMove_clicked()
{

    double sudu = ui->suEdit->text().toDouble();
    if(sudu < 10)
        sudu = 10;
    _sys_config.put("robot.speed", sudu);
    _sys_config.sync();
    RobotPos b2b = RobotPos::instance();
    b2b.x = ui->dx->text().toDouble();
    b2b.y = ui->dy->text().toDouble();
    b2b.z = ui->dz->text().toDouble();
    b2b.c = ui->drx->text().toDouble();
    b2b.b = ui->dry->text().toDouble();
    b2b.a = ui->drz->text().toDouble();
    Matrix4d mb2b;
    pos2Matrix(b2b,mb2b,zyx);
    Matrix2pos(mb2b,b2b,zyz);
    _robot->absoluteMove(_robot->getCurrPos()<<b2b);
}

void RobotUI::on_absoluteMoveAxle_clicked()
{
    try {
        QStringList js = ui->jmove->text().split(",");
        if(js.size() < 6)
            return;
        RobotAxle axle ;
        axle.a1 = js[0].toDouble()-ui->currA1->text().toDouble();
        axle.a2 = js[1].toDouble()-ui->currA2->text().toDouble();
        axle.a3 = js[2].toDouble()-ui->currA3->text().toDouble();
        axle.a4 = js[3].toDouble()-ui->currA4->text().toDouble();
        axle.a5 = js[4].toDouble()-ui->currA5->text().toDouble();
        axle.a6 = js[5].toDouble()-ui->currA6->text().toDouble();
        _robot->relativeMove_axle(axle, false) ;
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void RobotUI::on_pushButton_clicked()
{
    QString msg;
    //_expCheck->getError(msg);
    QMessageBox::about(NULL,"notice",msg);
}

void RobotUI::on_pushButton_2_clicked()
{
    cout<<"跳过重连!"<<endl;
    //_expCheck->reconnect();
}

void RobotUI::on_btndiscon_clicked()
{
    cout<<"断开连接!"<<endl;
    //_expCheck->disconnect();
}

//void RobotUI::on_btnSockflag_clicked()
//{
//    cout<<"sockflag!"<<endl;
//    _expCheck->resetSockFlag();
//}

void RobotUI::on_cur_axis_clicked()
{
    auto axle = _robot->getCurrAxle() ;
    QString axlestr = QString::number(axle.a1);
    axlestr.append(",");
    axlestr.append(QString::number(axle.a2));
    axlestr.append(",");
    axlestr.append(QString::number(axle.a3));
    axlestr.append(",");
    axlestr.append(QString::number(axle.a4));
    axlestr.append(",");
    axlestr.append(QString::number(axle.a5));
    axlestr.append(",");
    axlestr.append(QString::number(axle.a6));
    ui->jmove->setText(axlestr);
}

void RobotUI::on_btnOpenFile_clicked()
{
    QString imgPath = QFileDialog::getOpenFileName(NULL,tr("open file"),"",tr("path File(*.txt)"));
    ui->label_path->setText(imgPath);
}

void RobotUI::on_btnMove_clicked()
{
    ifstream ifs;
    ifs.open(ui->label_path->text().toStdString());
    string s;
    vector<RobotPos> path;
    while(getline(ifs,s))
    {
        RobotPos p;
        if(str2Robot(s,p))
            path.push_back(p);
    }
    ifs.close();
    _robot->pathLog = true;
    RobotPos offset = RobotPos::instance();
    RobotPos base_offset = RobotPos::instance();
    double dr=6;
    int autoFit=0;
    _robot->absoluteMove(path,offset,base_offset,dr,autoFit);
}
