/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QTabWidget *weldingRobot;
    QWidget *welcome;
    QLabel *label;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1148, 784);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(0);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        weldingRobot = new QTabWidget(centralWidget);
        weldingRobot->setObjectName(QStringLiteral("weldingRobot"));
        QFont font;
        font.setPointSize(20);
        weldingRobot->setFont(font);
        weldingRobot->setTabPosition(QTabWidget::North);
        welcome = new QWidget();
        welcome->setObjectName(QStringLiteral("welcome"));
        label = new QLabel(welcome);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(180, -40, 821, 281));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        QFont font1;
        font1.setPointSize(50);
        label->setFont(font1);
        weldingRobot->addTab(welcome, QString());

        gridLayout->addWidget(weldingRobot, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        weldingRobot->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\345\250\201\345\215\223\346\231\272\350\203\275\347\204\212\346\216\245\347\263\273\347\273\237", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p>\346\254\242\350\277\216\344\275\277\347\224\250\345\250\201\345\215\223\346\231\272\350\203\275\347\204\212\346\216\245\347\263\273\347\273\237</p></body></html>", Q_NULLPTR));
        weldingRobot->setTabText(weldingRobot->indexOf(welcome), QApplication::translate("MainWindow", "\347\263\273\347\273\237\351\246\226\351\241\265", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
