#ifndef TMESSAGEBOX_H
#define TMESSAGEBOX_H

#include <QObject>
#include <QMessageBox>
#include <QWidget>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "ui_adjust.h"
#include "base/data_types.h"

/// 支持多线程的对话框[QT的界面只能在UI线程中显示]
class TMessageBox : public QObject
{
    Q_OBJECT
public:
    /// 对话框状态
    enum BoxStatus{ busy, back, idle } ;

    explicit TMessageBox(QObject *parent = 0);
    ~TMessageBox();
    /// 模态对话框
    QMessageBox::StandardButton information(QWidget *parent, const QString &title,
                           const QString& text,
                           QMessageBox::StandardButton button0 = QMessageBox::Yes, QMessageBox::StandardButton button1 = QMessageBox::NoButton);

    /// 非模态纠正对话框
    void adjust() ;

    /// 非模态对话框
    void information_post(QWidget *parent, const QString &title,
                           const QString& text,
                           QMessageBox::StandardButton button0 = QMessageBox::Yes, QMessageBox::StandardButton button1 = QMessageBox::NoButton);

    /// 选择相机中心点纠正时从哪一步开始
    int selectStep() ;

    /// 收集焊缝信息
    void seamDlg() ;

private:
signals:
    void information_sig(QWidget *parent, const QString &title,
                         const QString& text,
                         QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 = QMessageBox::NoButton) ;

    void information_post_sig(QWidget *parent, const QString &title,
                         const QString& text,
                         QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 = QMessageBox::NoButton) ;

    void adjust_sig() ;

    void seamDlg_sig() ;

private slots:
    void on_information(QWidget *parent, const QString &title,
                        const QString& text,
                        QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 = QMessageBox::NoButton) ;

    void on_information_post(QWidget *parent, const QString &title,
                        const QString& text,
                        QMessageBox::StandardButton button0, QMessageBox::StandardButton button1 = QMessageBox::NoButton) ;

    void on_adjust() ;

//    void on_relative_clicked() ;

    void on_adjustOK_clicked();

    void on_relative_addX();

    void on_relative_addY();

    void on_relative_addZ();

    void on_relative_addA();

    void on_relative_addB();

    void on_relative_addC();

    void on_relative_minusX();

    void on_relative_minusY();

    void on_relative_minusZ();

    void on_relative_minusA();

    void on_relative_minusB();

    void on_relative_minusC();

    void updateOffsetLabel();

private:
    /// 记录在主线程中的实例地址
    static TMessageBox *_pGuiBox ;
    /// 线程同步锁
    static std::recursive_mutex _mtx ;
    /// 条件变量使用的锁
    static std::condition_variable _condReturn ;
    /// 条件变量使用的锁
    static std::mutex _mtxReturn ;
    /// 对话框状态
    static std::atomic<BoxStatus> _status ;
    /// 对话框返回值
    static std::atomic<QMessageBox::StandardButton> _ret ;
    /// 纠正调整对话框
    static Ui::AdjustDialog *_adjDlg_ptr ;
    /// 对话框模板
    static QDialog *_dlg_ptr ;
    /// 焊缝对话框
    static QDialog *_dlgSeam_ptr ;
    /// 图片检测的UV
    static UV _uv ;
};

#endif // TMESSAGEBOX_H
