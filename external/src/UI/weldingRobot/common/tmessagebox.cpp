#include "tmessagebox.h"

#include <array>

#include "base/data_types.h"
#include "device/robotmanager.h"
#include "tools/singleton.h"
#include "tools.h"
#include "base/wzerror.h"

/// 记录在主线程中的实例地址
TMessageBox *TMessageBox::_pGuiBox = nullptr ;
/// 线程同步锁
std::recursive_mutex TMessageBox::_mtx ;
/// 条件变量使用的锁
std::condition_variable TMessageBox::_condReturn ;
/// 条件变量使用的锁
std::mutex TMessageBox::_mtxReturn ;
/// 对话框状态
std::atomic<TMessageBox::BoxStatus> TMessageBox::_status{TMessageBox::idle} ;
/// 对话框返回值
std::atomic<QMessageBox::StandardButton> TMessageBox::_ret ;
/// 纠正调整对话框
Ui::AdjustDialog *TMessageBox::_adjDlg_ptr ;
/// 对话框模板
QDialog *TMessageBox::_dlg_ptr ;
/// 焊缝对话框模板

/// 焊缝对话框
QDialog *TMessageBox::_dlgSeam_ptr ;
/// 检测出来的UV
UV TMessageBox::_uv ;

TMessageBox::TMessageBox(QObject *parent) : QObject(parent)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( !_pGuiBox ) {
        _pGuiBox = this;
        // 调整对话框
        _adjDlg_ptr = new Ui::AdjustDialog ;
        _dlg_ptr = new QDialog ;
        _adjDlg_ptr->setupUi(_dlg_ptr);
        _dlg_ptr->setAttribute(Qt::WA_AlwaysStackOnTop);
        _dlg_ptr->setWindowFlag(Qt::WindowStaysOnTopHint);
        updateOffsetLabel();
        // 焊缝对话框
        _dlgSeam_ptr = new QDialog ;

        //        QObject::connect(_adjDlg_ptr->adjustRelative, SIGNAL(clicked()),
        //                         this, SLOT(on_relative_clicked())) ;
        QObject::connect(_adjDlg_ptr->adjustOK, SIGNAL(clicked(bool)),
                         this, SLOT(on_adjustOK_clicked())) ;//
        QObject::connect(_adjDlg_ptr->addX, SIGNAL(clicked()),
                         this, SLOT(on_relative_addX())) ;
        QObject::connect(_adjDlg_ptr->addY, SIGNAL(clicked()),
                         this, SLOT(on_relative_addY())) ;
        QObject::connect(_adjDlg_ptr->addZ, SIGNAL(clicked()),
                         this, SLOT(on_relative_addZ())) ;
        QObject::connect(_adjDlg_ptr->addA, SIGNAL(clicked()),
                         this, SLOT(on_relative_addA())) ;
        QObject::connect(_adjDlg_ptr->addB, SIGNAL(clicked()),
                         this, SLOT(on_relative_addB())) ;
        QObject::connect(_adjDlg_ptr->addC, SIGNAL(clicked()),
                         this, SLOT(on_relative_addC())) ;
        QObject::connect(_adjDlg_ptr->minusX, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusX())) ;
        QObject::connect(_adjDlg_ptr->minusY, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusY())) ;
        QObject::connect(_adjDlg_ptr->minusZ, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusZ())) ;
        QObject::connect(_adjDlg_ptr->minusA, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusA())) ;
        QObject::connect(_adjDlg_ptr->minusB, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusB())) ;
        QObject::connect(_adjDlg_ptr->minusC, SIGNAL(clicked()),
                         this, SLOT(on_relative_minusC())) ;

    }
    QObject::connect(this, SIGNAL(information_sig(QWidget *, const QString &,
                                                  const QString& ,
                                                  QMessageBox::StandardButton , QMessageBox::StandardButton )),
                     _pGuiBox, SLOT(on_information(QWidget *, const QString &,
                                                   const QString& ,
                                                   QMessageBox::StandardButton , QMessageBox::StandardButton ))) ;
    QObject::connect(this, SIGNAL(information_post_sig(QWidget *, const QString &,
                                                       const QString& ,
                                                       QMessageBox::StandardButton , QMessageBox::StandardButton )),
                     _pGuiBox, SLOT(on_information_post(QWidget *, const QString &,
                                                        const QString& ,
                                                        QMessageBox::StandardButton , QMessageBox::StandardButton ))) ;
    QObject::connect(this, SIGNAL(adjust_sig()),
                     _pGuiBox, SLOT(on_adjust()) ) ;

}

TMessageBox::~TMessageBox()
{

}

void TMessageBox::updateOffsetLabel()
{
    _adjDlg_ptr->offsetX->setText(QString::number(0));
    _adjDlg_ptr->offsetY->setText(QString::number(0));
    _adjDlg_ptr->offsetZ->setText(QString::number(0));
    _adjDlg_ptr->offsetA->setText(QString::number(0));
    _adjDlg_ptr->offsetB->setText(QString::number(0));
    _adjDlg_ptr->offsetC->setText(QString::number(0));
}

QMessageBox::StandardButton TMessageBox::information(QWidget *parent, const QString &title,
                                                     const QString& text,
                                                     QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 )
{
    std::unique_lock<std::mutex> lck(_mtxReturn) ;
    _condReturn.wait(lck, [](){return _status == idle; });
    _status = busy ;
    emit information_sig(parent, title, text, button0, button1) ;
    _condReturn.wait(lck, [](){ return _status == back; }) ;
    auto ret = _ret.load() ;
    _status = idle ;
    _condReturn.notify_all();
    return ret ;
}

void TMessageBox::adjust()
{
    std::unique_lock<std::mutex> lck(_mtxReturn) ;
    _condReturn.wait(lck, [](){return _status == idle; });
    _status = busy ;
    emit adjust_sig();
    updateOffsetLabel();
    _condReturn.wait(lck, [](){ return _status == back; }) ;
    _status = idle ;
    _condReturn.notify_all();
}

void TMessageBox::information_post(QWidget *parent, const QString &title, const QString &text, QMessageBox::StandardButton button0, QMessageBox::StandardButton button1)
{
    emit information_post_sig(parent, title, text, button0, button1) ;
}

void TMessageBox::seamDlg()
{
    emit seamDlg_sig() ;
}

void TMessageBox::on_information(QWidget *parent, const QString &title,
                                 const QString& text,
                                 QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 )
{
    _ret = QMessageBox::information(parent, title, text, button0, button1) ;
    _status = back ;
    _condReturn.notify_one();
}

void TMessageBox::on_information_post(QWidget *parent, const QString &title,
                                      const QString& text,
                                      QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 )
{
    QMessageBox::information(parent, title, text, button0, button1) ;
}

void TMessageBox::on_adjust()
{
    // _dlg_ptr->show();
    _dlg_ptr->exec();
    _status = back ;
    _condReturn.notify_one();
}

//void TMessageBox::on_relative_clicked()
//{
//    try {
//        RobotPos pos{} ;
//        pos.x = _adjDlg_ptr->adjustX->text().toDouble();
//        pos.y = _adjDlg_ptr->adjustY->text().toDouble();
//        pos.z = _adjDlg_ptr->adjustZ->text().toDouble();
//        pos.a = _adjDlg_ptr->adjustA->text().toDouble();
//        pos.b = _adjDlg_ptr->adjustB->text().toDouble();
//        pos.c = _adjDlg_ptr->adjustC->text().toDouble();
//        RobotManager()->relativeMove_axis({pos});
//    }
//    catch( const boost::exception &e ) {
//        DUMPERROR(e) ;
//    }
//}

void TMessageBox::on_relative_addX()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.x = _adjDlg_ptr->comboX->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetX = _adjDlg_ptr->offsetX->text().toDouble();
        _adjDlg_ptr->offsetX->setText(QString::number(currOffsetX + pos.x));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_addY()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.y = _adjDlg_ptr->comboY->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetY = _adjDlg_ptr->offsetY->text().toDouble();
        _adjDlg_ptr->offsetY->setText(QString::number(currOffsetY + pos.y));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_addZ()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.z = _adjDlg_ptr->comboZ->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetZ = _adjDlg_ptr->offsetZ->text().toDouble();
        _adjDlg_ptr->offsetZ->setText(QString::number(currOffsetZ + pos.z));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_addA()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.a = _adjDlg_ptr->comboA->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetA = _adjDlg_ptr->offsetA->text().toDouble();
        _adjDlg_ptr->offsetA->setText(QString::number(currOffsetA + pos.a));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_addB()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.b = _adjDlg_ptr->comboB->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetB = _adjDlg_ptr->offsetB->text().toDouble();
        _adjDlg_ptr->offsetB->setText(QString::number(currOffsetB + pos.b));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_addC()
{
    try {
        RobotPos pos = RobotPos::instance() ;
        pos.c = _adjDlg_ptr->comboC->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetC = _adjDlg_ptr->offsetC->text().toDouble();
        _adjDlg_ptr->offsetC->setText(QString::number(currOffsetC + pos.c));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_minusX()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.x = -_adjDlg_ptr->comboX->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetX = _adjDlg_ptr->offsetX->text().toDouble();
        _adjDlg_ptr->offsetX->setText(QString::number(currOffsetX + pos.x));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_minusY()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.y = -_adjDlg_ptr->comboY->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetY = _adjDlg_ptr->offsetY->text().toDouble();
        _adjDlg_ptr->offsetY->setText(QString::number(currOffsetY +pos.y));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}
void TMessageBox::on_relative_minusZ()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.z = -_adjDlg_ptr->comboZ->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetZ = _adjDlg_ptr->offsetZ->text().toDouble();
        _adjDlg_ptr->offsetZ->setText(QString::number(currOffsetZ + pos.z));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_minusA()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.a = -_adjDlg_ptr->comboA->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetA = _adjDlg_ptr->offsetA->text().toDouble();
        _adjDlg_ptr->offsetA->setText(QString::number(currOffsetA + pos.a));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_minusB()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.b = -_adjDlg_ptr->comboB->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetB = _adjDlg_ptr->offsetB->text().toDouble();
        _adjDlg_ptr->offsetB->setText(QString::number(currOffsetB + pos.b));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_relative_minusC()
{
    try {
        RobotPos pos = RobotPos::instance();
        pos.c = -_adjDlg_ptr->comboC->currentText().toDouble();
        RobotManager()->relativeMove_axis({pos});
        auto currOffsetC = _adjDlg_ptr->offsetC->text().toDouble();
        _adjDlg_ptr->offsetC->setText(QString::number(currOffsetC + pos.c));
    }
    catch( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void TMessageBox::on_adjustOK_clicked()
{
    // RobotManager()->waitForMoveStoped();
    _dlg_ptr->reject();
}
