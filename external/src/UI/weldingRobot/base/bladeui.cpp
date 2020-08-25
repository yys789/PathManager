#include "bladeui.h"
#include "ui_bladeui.h"
#include "base/errorcode.h"
#include "base/errorcode.h"
#include "tools.h"


bladeui::bladeui(QWidget *parent) :
    QWidget(parent)
  ,_mb_ptr(Singleton<mbControl>::get())
  ,_sqlite_ptr(Singleton<mySqliteApi>::get())
  //,_slave_ptr(Singleton<slave>::get())
  ,m_flag{false}
  ,s_flag{false}
  ,bRun{false}
  ,
    ui(new Ui::bladeui)
{
    ui->setupUi(this);
    ui->startMaster->setDisabled(true);
    connect(this,SIGNAL(sig_operate(int)),this,SLOT(func(int)),Qt::QueuedConnection);
    m_flag = true;
    run_master();
    ui->groundAngle->setText("50");
    ui->groundSpeed->setText("30");
    ui->rotateAngle->setText("100");
    ui->rotateSpeed->setText("3");
    ui->flipAngle->setText("50");
    ui->flipSpeed->setText("3");

    _sqlite_ptr->selectSQL({"CAD"});
    if(_sqlite_ptr->m_iRows < 1)
        return;
    auto vec = _sqlite_ptr->m_vResult;
    for(auto & v:vec)
    {
        auto it = v.begin();
        while(it != v.end())
        {
            if(it->first == "CADID")
                ui->cadList->addItem(QString::fromStdString(it->second));
            it++;
        }
    }
    auto cad = ui->cadList->currentText().toStdString();
    if(cad.compare("")) return;
    _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMNAME"},{"CADID","'"+cad+"'"});
    if(_sqlite_ptr->m_iRows < 1)
        return;
    auto vec_seam = _sqlite_ptr->m_vResult;
    for(auto & v:vec_seam)
    {
        auto it = v.begin();
        while(it != v.end())
        {
            if(it->first == "SEAMNAME")
                ui->seamList->addItem(QString::fromStdString(it->second));
            it++;
        }
    }
}

bladeui::~bladeui()
{
    delete ui;
}

void bladeui::run_master()
{
    std::thread([this]()mutable{
        try
        {
            if(!_mb_ptr)
                throw ERR_DEVICE_MODBUS_DISCONNECTED;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            int sig = 0;
            cout<<"start recv message : "<<endl;
            cs.send_cad_list();
            while(m_flag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                _mb_ptr->recv_msg();
                sig = _mb_ptr->deal_msg();
                if (sig)
                    cout<<"sig = "<<sig<<endl;
                if(sig == -1)
                    continue;
                if(sig > 0 && sig < 99)
                    emit sig_operate(sig);
                _mb_ptr->send_msg();
            }
            cout<<"quit loop to send and recv msg"<<endl;
        }catch(ReturnStatus error_code){
            cout<<getErrString(error_code)<<endl;
        }catch(...){
            cout<<getErrString(ERR_FUNC_SERVICE_PROCESS_RUN_ERROR)<<endl;
        }
         cout<<"quit loop , master stop "<<endl;
    }).detach();
}

void bladeui::func(int sig)
{
    std::thread([this,sig]()mutable{
        try
        {
            cout<<"[SIGNAL] : "<<sig<<endl;
            _mb_ptr->ReceiveReply(sig);
            switch(sig)
            {
            case 1:
                //                cs.setActionType(1);
                //                cs.autoScanWelding();
            case 2:// 0 空走 1 焊接 2 空走后焊接
                cs.setActionType(0);
                cs.autoScanWelding();
                break;
            case 3:
                cs.setActionType(1);
                cs.autoScanWelding();
            case 4:
                cs.setActionType(1);
                cs.autoScanWelding();
                break;
            case 5://同步焊缝信息
                cs.syncSeamInfo();
                break;
            case 6:
                cs.cleanTorch();
                break;
            case 7://传感器位置校验
                cs.checkSensor();
                break;
            case 10:
                cs.NewSyncMotorAngle();
                break;
            case 11:
                cs.back_home();
                break;
                //            case 14:
                //                cs.send_cad_list();
                //                break;
            case 15:
                cs.transferDBData();
                break;
            case 16:
                cs.unloading_cad();
                break;
            case 17:
                cs.loading_cad();
                break;
            default:
                break;
            }

            cout<<"[func][OVER]"<<endl;
        }catch(ReturnStatus error_code){
            cout<<getErrString(error_code)<<endl;
            _mb_ptr->feedbackErrCode(error_code);
        }catch(...){
            cout<<"func execute failed ."<<endl;
            _mb_ptr->feedbackErrCode(ERR_FUNC_EXCEPTION);
        }
    }).detach();
}


void bladeui::on_openValve_clicked()
{
    std::thread([this]()mutable{
        _mb_ptr->openAirValve();
    }).detach();
}

void bladeui::on_closeValve_clicked()
{
    std::thread([this]()mutable{
        _mb_ptr->closeAirValve();
    }).detach();
}

void bladeui::on_openLaser_clicked()
{
    std::thread([this]()mutable{
        _mb_ptr->openLaser();
    }).detach();
}

void bladeui::on_closeLaser_clicked()
{
    std::thread([this]()mutable{
        _mb_ptr->closeLaser();
    }).detach();
}


void bladeui::on_send_clicked()
{
    int count= 60;
    uint16_t iSendData[count] = {0};
    for(int i = 0; i<count;i++)
    {
        iSendData[i] = 0;
    }
    _mb_ptr->write_registers(0,count,iSendData);
}

void bladeui::on_transferData_clicked()
{
    cs.transferDBData();
}

void bladeui::on_rotate_clicked()
{
    auto groudpos = ui->groundAngle->text().toDouble();
    auto flipangle = ui->flipAngle->text().toDouble();
    auto rotateAngle = ui->rotateAngle->text().toDouble();

    //    ui->groundAngle->setText(QString::number(groudpos+2));
    //    //ui->flipAngle->setText(QString::number(groudpos+2));
    //    ui->rotateAngle->setText(QString::number(rotateAngle+2));
    std::thread([this]()mutable{
        int mode = 1;
        auto str = ui->cb_rotateMode->currentText();
        if(str == "立即转")
        {
            mode = 1;
        }else{
            mode = 2;
        }
        auto groudpos = ui->groundAngle->text().toDouble();
        auto flipangle = ui->flipAngle->text().toDouble();
        auto rotateAngle = ui->rotateAngle->text().toDouble();

        auto groudspeed = ui->groundSpeed->text().toDouble();
        auto flipspeed = ui->flipSpeed->text().toDouble();
        auto rotatespeed = ui->rotateSpeed->text().toDouble();

        _mb_ptr->rotateNew(groudpos,groudspeed,flipangle,flipspeed,rotateAngle,rotatespeed,mode);
    }).detach();

}

void bladeui::on_openIO_clicked()
{
    _robot->setRotateTrue();
}

void bladeui::on_closeIO_clicked()
{
    _robot->setRotateFalse();
}

void bladeui::on_testUpdateProgress_clicked()
{
    std::thread([this]()mutable{
        int seamId = 23;
        int result = 2;
        _mb_ptr->updateProgress(seamId,result);
    }).detach();
}

void bladeui::on_feedbackErrCode_clicked()
{
    std::thread([this]()mutable{
        int resultCode = 51;
        _mb_ptr->feedbackErrCode(resultCode);
    }).detach();
}

void bladeui::on_rotate1_clicked()
{
    std::thread([this]()mutable{
        int mode = 1;
        auto str = ui->cb_rotateMode->currentText();
        if(str == "立即转")
        {
            mode = 1;
        }else{
            mode = 2;
        }
        auto groudpos = ui->groundAngle->text().toDouble();
        auto flipangle = ui->flipAngle->text().toDouble();
        auto rotateAngle = ui->rotateAngle->text().toDouble();

        auto groudspeed = ui->groundSpeed->text().toDouble();
        auto flipspeed = ui->flipSpeed->text().toDouble();
        auto rotatespeed = ui->rotateSpeed->text().toDouble();

        _mb_ptr->rotateNew(groudpos,groudspeed,flipangle,flipspeed,rotateAngle,rotatespeed,mode);
    }).detach();
}

void bladeui::on_startMaster_clicked()
{
    m_flag = true;
    run_master();
}

void bladeui::on_stopMaster_clicked()
{
    m_flag = false;
    _mb_ptr->init();
}

void bladeui::on_send_single_clicked()
{
    func(2);
}

void bladeui::on_cadList_currentTextChanged(const QString &arg1)
{
    ui->seamList->clear();
    auto cad = arg1.toStdString();
    _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMNAME"},{"CADID","'"+cad+"'"});
    if(_sqlite_ptr->m_iRows < 1)
        return ;
    auto vec_seam = _sqlite_ptr->m_vResult;
    for(auto & v:vec_seam)
    {
        auto it = v.begin();
        while(it != v.end())
        {
            if(it->first == "SEAMNAME")
                ui->seamList->addItem(QString::fromStdString(it->second));
            it++;
        }
    }
}

void bladeui::on_read_clicked()
{
    _mb_ptr->test();
}

void bladeui::on_test_clicked()
{
    std::thread([this]()mutable{
        try
        {
            cs.send_cad_list();
            cout<<"[func][OVER]"<<endl;
        }catch(ReturnStatus error_code){
            //            _mb_ptr->feedback(error_code);
        }catch(...){

        }
    }).detach();
}

void bladeui::on_test_seamlist_clicked()
{
    std::thread([this]()mutable{
        try
        {
            cs.transferDBData();
            cout<<"[func][OVER]"<<endl;
        }catch(ReturnStatus error_code){
            //            _mb_ptr->feedback(error_code);
        }catch(...){

        }
    }).detach();
}

void bladeui::on_write_test_clicked()
{
    _mb_ptr->test();
}
