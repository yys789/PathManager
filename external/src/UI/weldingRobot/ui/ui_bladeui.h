/********************************************************************************
** Form generated from reading UI file 'bladeui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BLADEUI_H
#define UI_BLADEUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bladeui
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QPushButton *feedbackErrCode;
    QPushButton *transferData;
    QPushButton *testUpdateProgress;
    QPushButton *send;
    QPushButton *read;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *openValve;
    QPushButton *closeValve;
    QPushButton *openLaser;
    QPushButton *closeLaser;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *label_3;
    QComboBox *cb_rotateMode;
    QLabel *label_8;
    QLineEdit *flipSpeed;
    QLineEdit *groundAngle;
    QLabel *label_2;
    QLineEdit *flipAngle;
    QPushButton *rotate1;
    QLineEdit *groundSpeed;
    QLabel *label_4;
    QLabel *label_6;
    QLabel *label;
    QLineEdit *rotateAngle;
    QLabel *label_7;
    QLineEdit *rotateSpeed;
    QPushButton *rotate;
    QPushButton *openIO;
    QPushButton *closeIO;
    QPushButton *startMaster;
    QPushButton *stopMaster;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QComboBox *cadList;
    QComboBox *seamList;
    QPushButton *send_single;
    QPushButton *test;
    QPushButton *test_seamlist;
    QPushButton *write_test;
    QWidget *tab_2;
    QWidget *tab_3;
    QPushButton *pushButton;

    void setupUi(QWidget *bladeui)
    {
        if (bladeui->objectName().isEmpty())
            bladeui->setObjectName(QStringLiteral("bladeui"));
        bladeui->resize(869, 912);
        tabWidget = new QTabWidget(bladeui);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(20, 10, 831, 871));
        QFont font;
        font.setPointSize(15);
        tabWidget->setFont(font);
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayoutWidget_4 = new QWidget(tab);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(230, 490, 201, 190));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        feedbackErrCode = new QPushButton(verticalLayoutWidget_4);
        feedbackErrCode->setObjectName(QStringLiteral("feedbackErrCode"));
        QFont font1;
        font1.setPointSize(12);
        feedbackErrCode->setFont(font1);

        verticalLayout_4->addWidget(feedbackErrCode);

        transferData = new QPushButton(verticalLayoutWidget_4);
        transferData->setObjectName(QStringLiteral("transferData"));
        transferData->setFont(font1);

        verticalLayout_4->addWidget(transferData);

        testUpdateProgress = new QPushButton(verticalLayoutWidget_4);
        testUpdateProgress->setObjectName(QStringLiteral("testUpdateProgress"));
        testUpdateProgress->setFont(font1);

        verticalLayout_4->addWidget(testUpdateProgress);

        send = new QPushButton(verticalLayoutWidget_4);
        send->setObjectName(QStringLiteral("send"));
        send->setFont(font1);

        verticalLayout_4->addWidget(send);

        read = new QPushButton(verticalLayoutWidget_4);
        read->setObjectName(QStringLiteral("read"));
        read->setFont(font1);

        verticalLayout_4->addWidget(read);

        verticalLayoutWidget = new QWidget(tab);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(50, 490, 161, 181));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        openValve = new QPushButton(verticalLayoutWidget);
        openValve->setObjectName(QStringLiteral("openValve"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(openValve->sizePolicy().hasHeightForWidth());
        openValve->setSizePolicy(sizePolicy);
        openValve->setFont(font1);

        verticalLayout->addWidget(openValve);

        closeValve = new QPushButton(verticalLayoutWidget);
        closeValve->setObjectName(QStringLiteral("closeValve"));
        sizePolicy.setHeightForWidth(closeValve->sizePolicy().hasHeightForWidth());
        closeValve->setSizePolicy(sizePolicy);
        closeValve->setFont(font1);

        verticalLayout->addWidget(closeValve);

        openLaser = new QPushButton(verticalLayoutWidget);
        openLaser->setObjectName(QStringLiteral("openLaser"));
        sizePolicy.setHeightForWidth(openLaser->sizePolicy().hasHeightForWidth());
        openLaser->setSizePolicy(sizePolicy);
        openLaser->setFont(font1);

        verticalLayout->addWidget(openLaser);

        closeLaser = new QPushButton(verticalLayoutWidget);
        closeLaser->setObjectName(QStringLiteral("closeLaser"));
        sizePolicy.setHeightForWidth(closeLaser->sizePolicy().hasHeightForWidth());
        closeLaser->setSizePolicy(sizePolicy);
        closeLaser->setFont(font1);

        verticalLayout->addWidget(closeLaser);

        gridLayoutWidget = new QWidget(tab);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(50, 290, 561, 173));
        sizePolicy.setHeightForWidth(gridLayoutWidget->sizePolicy().hasHeightForWidth());
        gridLayoutWidget->setSizePolicy(sizePolicy);
        gridLayoutWidget->setFont(font);
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setFont(font);

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        cb_rotateMode = new QComboBox(gridLayoutWidget);
        cb_rotateMode->setObjectName(QStringLiteral("cb_rotateMode"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cb_rotateMode->sizePolicy().hasHeightForWidth());
        cb_rotateMode->setSizePolicy(sizePolicy1);
        cb_rotateMode->setFont(font);

        gridLayout->addWidget(cb_rotateMode, 1, 4, 1, 1);

        label_8 = new QLabel(gridLayoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);
        label_8->setFont(font);

        gridLayout->addWidget(label_8, 0, 4, 1, 1);

        flipSpeed = new QLineEdit(gridLayoutWidget);
        flipSpeed->setObjectName(QStringLiteral("flipSpeed"));
        sizePolicy1.setHeightForWidth(flipSpeed->sizePolicy().hasHeightForWidth());
        flipSpeed->setSizePolicy(sizePolicy1);
        flipSpeed->setFont(font);

        gridLayout->addWidget(flipSpeed, 2, 3, 1, 1);

        groundAngle = new QLineEdit(gridLayoutWidget);
        groundAngle->setObjectName(QStringLiteral("groundAngle"));
        sizePolicy1.setHeightForWidth(groundAngle->sizePolicy().hasHeightForWidth());
        groundAngle->setSizePolicy(sizePolicy1);
        groundAngle->setFont(font);

        gridLayout->addWidget(groundAngle, 1, 2, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setFont(font);

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        flipAngle = new QLineEdit(gridLayoutWidget);
        flipAngle->setObjectName(QStringLiteral("flipAngle"));
        sizePolicy1.setHeightForWidth(flipAngle->sizePolicy().hasHeightForWidth());
        flipAngle->setSizePolicy(sizePolicy1);
        flipAngle->setFont(font);

        gridLayout->addWidget(flipAngle, 2, 2, 1, 1);

        rotate1 = new QPushButton(gridLayoutWidget);
        rotate1->setObjectName(QStringLiteral("rotate1"));
        rotate1->setFont(font);

        gridLayout->addWidget(rotate1, 3, 4, 1, 1);

        groundSpeed = new QLineEdit(gridLayoutWidget);
        groundSpeed->setObjectName(QStringLiteral("groundSpeed"));
        sizePolicy1.setHeightForWidth(groundSpeed->sizePolicy().hasHeightForWidth());
        groundSpeed->setSizePolicy(sizePolicy1);
        groundSpeed->setFont(font);

        gridLayout->addWidget(groundSpeed, 1, 3, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setFont(font);

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        label_6 = new QLabel(gridLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setFont(font);

        gridLayout->addWidget(label_6, 0, 2, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setFont(font);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        rotateAngle = new QLineEdit(gridLayoutWidget);
        rotateAngle->setObjectName(QStringLiteral("rotateAngle"));
        sizePolicy1.setHeightForWidth(rotateAngle->sizePolicy().hasHeightForWidth());
        rotateAngle->setSizePolicy(sizePolicy1);
        rotateAngle->setFont(font);

        gridLayout->addWidget(rotateAngle, 3, 2, 1, 1);

        label_7 = new QLabel(gridLayoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setFont(font);

        gridLayout->addWidget(label_7, 0, 3, 1, 1);

        rotateSpeed = new QLineEdit(gridLayoutWidget);
        rotateSpeed->setObjectName(QStringLiteral("rotateSpeed"));
        sizePolicy1.setHeightForWidth(rotateSpeed->sizePolicy().hasHeightForWidth());
        rotateSpeed->setSizePolicy(sizePolicy1);
        rotateSpeed->setFont(font);

        gridLayout->addWidget(rotateSpeed, 3, 3, 1, 1);

        rotate = new QPushButton(gridLayoutWidget);
        rotate->setObjectName(QStringLiteral("rotate"));
        rotate->setFont(font);

        gridLayout->addWidget(rotate, 2, 4, 1, 1);

        openIO = new QPushButton(gridLayoutWidget);
        openIO->setObjectName(QStringLiteral("openIO"));
        openIO->setFont(font);

        gridLayout->addWidget(openIO, 2, 5, 1, 1);

        closeIO = new QPushButton(gridLayoutWidget);
        closeIO->setObjectName(QStringLiteral("closeIO"));
        closeIO->setFont(font);

        gridLayout->addWidget(closeIO, 3, 5, 1, 1);

        startMaster = new QPushButton(tab);
        startMaster->setObjectName(QStringLiteral("startMaster"));
        startMaster->setGeometry(QRect(50, 50, 171, 61));
        startMaster->setFont(font1);
        stopMaster = new QPushButton(tab);
        stopMaster->setObjectName(QStringLiteral("stopMaster"));
        stopMaster->setGeometry(QRect(50, 130, 171, 61));
        stopMaster->setFont(font1);
        horizontalLayoutWidget = new QWidget(tab);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(50, 220, 561, 48));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        cadList = new QComboBox(horizontalLayoutWidget);
        cadList->setObjectName(QStringLiteral("cadList"));
        cadList->setFont(font);

        horizontalLayout->addWidget(cadList);

        seamList = new QComboBox(horizontalLayoutWidget);
        seamList->setObjectName(QStringLiteral("seamList"));
        seamList->setFont(font);

        horizontalLayout->addWidget(seamList);

        send_single = new QPushButton(horizontalLayoutWidget);
        send_single->setObjectName(QStringLiteral("send_single"));
        send_single->setFont(font);

        horizontalLayout->addWidget(send_single);

        test = new QPushButton(tab);
        test->setObjectName(QStringLiteral("test"));
        test->setGeometry(QRect(620, 230, 106, 30));
        test_seamlist = new QPushButton(tab);
        test_seamlist->setObjectName(QStringLiteral("test_seamlist"));
        test_seamlist->setGeometry(QRect(620, 280, 106, 30));
        write_test = new QPushButton(tab);
        write_test->setObjectName(QStringLiteral("write_test"));
        write_test->setGeometry(QRect(620, 330, 106, 30));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        pushButton = new QPushButton(tab_3);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(170, 70, 106, 30));
        tabWidget->addTab(tab_3, QString());

        retranslateUi(bladeui);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(bladeui);
    } // setupUi

    void retranslateUi(QWidget *bladeui)
    {
        bladeui->setWindowTitle(QApplication::translate("bladeui", "Form", Q_NULLPTR));
        feedbackErrCode->setText(QApplication::translate("bladeui", "error code", Q_NULLPTR));
        transferData->setText(QApplication::translate("bladeui", "transfer Data", Q_NULLPTR));
        testUpdateProgress->setText(QApplication::translate("bladeui", "Update Progress", Q_NULLPTR));
        send->setText(QApplication::translate("bladeui", "send", Q_NULLPTR));
        read->setText(QApplication::translate("bladeui", "read", Q_NULLPTR));
        openValve->setText(QApplication::translate("bladeui", "\346\211\223\345\274\200\346\260\224\351\230\200", Q_NULLPTR));
        closeValve->setText(QApplication::translate("bladeui", "\345\205\263\351\227\255\346\260\224\351\230\200", Q_NULLPTR));
        openLaser->setText(QApplication::translate("bladeui", "\346\211\223\345\274\200\346\277\200\345\205\211", Q_NULLPTR));
        closeLaser->setText(QApplication::translate("bladeui", "\345\205\263\351\227\255\346\277\200\345\205\211", Q_NULLPTR));
        label_3->setText(QApplication::translate("bladeui", "\347\277\273\350\275\254", Q_NULLPTR));
        cb_rotateMode->clear();
        cb_rotateMode->insertItems(0, QStringList()
         << QApplication::translate("bladeui", "\347\253\213\345\215\263\350\275\254", Q_NULLPTR)
         << QApplication::translate("bladeui", "\345\273\266\346\227\266\350\275\254", Q_NULLPTR)
        );
        label_8->setText(QString());
        label_2->setText(QApplication::translate("bladeui", "\345\234\260\350\275\250", Q_NULLPTR));
        rotate1->setText(QApplication::translate("bladeui", "rotate1", Q_NULLPTR));
        label_4->setText(QApplication::translate("bladeui", "\346\227\213\350\275\254", Q_NULLPTR));
        label_6->setText(QApplication::translate("bladeui", "\350\247\222\345\272\246", Q_NULLPTR));
        label->setText(QApplication::translate("bladeui", "\347\224\265\346\234\272\347\261\273\345\236\213", Q_NULLPTR));
        label_7->setText(QApplication::translate("bladeui", "\351\200\237\345\272\246", Q_NULLPTR));
        rotate->setText(QApplication::translate("bladeui", "rotate", Q_NULLPTR));
        openIO->setText(QApplication::translate("bladeui", "open IO", Q_NULLPTR));
        closeIO->setText(QApplication::translate("bladeui", "close IO", Q_NULLPTR));
        startMaster->setText(QApplication::translate("bladeui", "\345\274\200\345\220\257\344\270\273\347\253\231\345\276\252\347\216\257\346\250\241\345\274\217", Q_NULLPTR));
        stopMaster->setText(QApplication::translate("bladeui", "\345\201\234\346\255\242\345\276\252\347\216\257", Q_NULLPTR));
        send_single->setText(QApplication::translate("bladeui", "\346\211\253\346\217\217\347\251\272\350\265\260", Q_NULLPTR));
        test->setText(QApplication::translate("bladeui", "test cad", Q_NULLPTR));
        test_seamlist->setText(QApplication::translate("bladeui", "test seam", Q_NULLPTR));
        write_test->setText(QApplication::translate("bladeui", "write test", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("bladeui", "PC\344\270\273\347\253\231", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("bladeui", "PLC\344\273\216\347\253\231", Q_NULLPTR));
        pushButton->setText(QApplication::translate("bladeui", "write char", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("bladeui", "\346\265\213\350\257\225", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class bladeui: public Ui_bladeui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BLADEUI_H
