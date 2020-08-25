#include "settingui.h"
#include "ui_settingui.h"

#include "base/wzerror.h"
#include <string>
#include <iostream>

using namespace std ;

SettingUI::SettingUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingUI),
    _camera_ptr(Singleton<Camera>::get())
{
    ui->setupUi(this);
    initUI() ;
}

SettingUI::~SettingUI()
{
    delete ui;
}

void SettingUI::on_saveSetting_clicked()
{
    // 机器人
    _sys_config.put("robot.IPAddr", ui->IPAddr->text().toStdString()) ;
    _sys_config.put("robot.port", ui->port->text().toInt()) ;
    _sys_config.put("robot.moveSpeed", ui->speed->text().toDouble()) ;
    _sys_config.put("robot.scanSpeed", ui->scanSpeed->text().toDouble()) ;
    _sys_config.put("robot.simulateSpeed", ui->simulateSpeed->text().toDouble()) ;
    // 变位机
    _sys_config.put("motor.serialPort", ui->motorSerialPort->currentText().toStdString()) ;
    _sys_config.put("motor.speed", ui->motorSpeed->text().toInt()) ;
    _sys_config.put("motor.totalSteps", ui->totalSteps->text().toInt()) ;

    // 激光
    _sys_config.put("laser.serialPort", ui->laserSerialPort->currentText().toStdString()) ;
    _sys_config.put("laser.brightness", ui->laserBrightness->text().toInt()) ;
    _sys_config.put("laser.control", ui->laserControl->currentText().toStdString()) ;
    // 相机
    float gain = ui->cameraGain->text().toFloat() ;
    int exposure = ui->cameraExposure->text().toInt() ;
    auto roi_offset_x = ui->roi_offset_x->text().toFloat();
    auto roi_offset_y = ui->roi_offset_y->text().toFloat();
    int roi_width = ui->roi_width->text().toInt();
    int roi_height = ui->roi_height->text().toInt();

    _sys_config.put("camera.exposure", exposure) ;
    _sys_config.put("camera.gain", gain) ;
    _sys_config.put("camera.roi_offset_x", roi_offset_x);
    _sys_config.put("camera.roi_offset_y", roi_offset_y);
    _sys_config.put("camera.roi_width", roi_width);
    _sys_config.put("camera.roi_height", roi_height);


    _sys_config.put("bWelding", ui->weldControl->currentIndex());

    _camera_ptr->setGain(gain) ;
    int exp_ret = _camera_ptr->setExposure(exposure) ;
    _camera_ptr->setRoi(roi_offset_x,roi_offset_y,roi_width,roi_height);
    // 纠正速度
    _sys_config.sync();
}

void SettingUI::initUI()
{
    try {
        ui->IPAddr->setText(QString::fromStdString(_sys_config.get<string>("robot.IPAddr", "0.0.0.0"))) ;
        ui->port->setText(QString::number(_sys_config.get<int>("robot.port"))) ;
        ui->speed->setText(QString::number(_sys_config.get<double>("robot.moveSpeed" ))) ;
        ui->scanSpeed->setText(QString::number(_sys_config.get<double>("robot.scanSpeed")));
        ui->simulateSpeed->setText(QString::number(_sys_config.get<double>("robot.simulateSpeed")));

        // 变位机
        ui->motorSerialPort->setCurrentText(QString::fromStdString(_sys_config.get<string>("motor.serialPort")));
        ui->motorSpeed->setText(QString::number(_sys_config.get<int>("motor.speed")));
        ui->totalSteps->setText(QString::number(_sys_config.get<int>("motor.totalSteps")));

        // 激光
        ui->laserSerialPort->setCurrentText(QString::fromStdString(_sys_config.get<string>("laser.serialPort" ))) ;
        ui->laserBrightness->setText(QString::number(_sys_config.get<int>("laser.brightness" ))) ;
        ui->laserControl->setCurrentText(QString::fromStdString(_sys_config.get<string>("laser.control" ))) ;
        // 相机
        ui->cameraExposure->setText(QString::number(_sys_config.get<int>("camera.exposure" ))) ;
        ui->cameraGain->setText(QString::number(_sys_config.get<float>("camera.gain"))) ;
        ui->roi_offset_x->setText(QString::number(_sys_config.get<float>("camera.roi_offset_x"))) ;
        ui->roi_offset_y->setText(QString::number(_sys_config.get<float>("camera.roi_offset_y"))) ;
        ui->roi_width->setText(QString::number(_sys_config.get<float>("camera.roi_width"))) ;
        ui->roi_height->setText(QString::number(_sys_config.get<float>("camera.roi_height"))) ;

        ui->weldControl->setCurrentIndex(_sys_config.get<int>("bWelding")) ;
    }
    catch( boost::exception const &e ) {
        DUMPERROR(e);
    }
}
