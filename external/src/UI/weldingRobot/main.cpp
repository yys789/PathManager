#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include "base/errorcode.h"

int main(int argc, char *argv[])
{
    qInstallMessageHandler(myMessageOutPut);
    QApplication a(argc, argv);
    QDesktopWidget * desktopWidget = QApplication::desktop();
    QRect clientRect = desktopWidget->screenGeometry();
    MainWindow * w;
    try
    {
        w = new MainWindow;
    }catch(ReturnStatus error_code)
    {
        QString qecode(getErrString(error_code));
        QMessageBox::about(NULL,"notice",qecode);
        return 0;
    }
    w->setGeometry(clientRect);
    w->show();
    return a.exec();
}



