/********************************************************************************
** Form generated from reading UI file 'settingui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGUI_H
#define UI_SETTINGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SettingUI
{
public:
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_21;
    QLabel *label_57;
    QLineEdit *IPAddr;
    QHBoxLayout *horizontalLayout;
    QLabel *label_7;
    QLineEdit *port;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_14;
    QLineEdit *speed;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_42;
    QLineEdit *scanSpeed;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *simulateSpeed;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_4;
    QLabel *label_2;
    QLineEdit *laserBrightness;
    QLabel *label_5;
    QComboBox *laserSerialPort;
    QComboBox *laserControl;
    QLabel *label_6;
    QGroupBox *groupBox_13;
    QGridLayout *gridLayout_3;
    QGridLayout *gridLayout;
    QLineEdit *motorSpeed;
    QComboBox *motorSerialPort;
    QLabel *label_4;
    QLabel *label_58;
    QLabel *label_11;
    QLineEdit *totalSteps;
    QPushButton *saveSetting;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_2;
    QLabel *label_18;
    QLineEdit *cameraExposure;
    QLabel *label_17;
    QLineEdit *cameraGain;
    QLabel *label;
    QLineEdit *roi_offset_x;
    QLabel *label_8;
    QLineEdit *roi_offset_y;
    QLabel *label_9;
    QLineEdit *roi_width;
    QLabel *label_10;
    QLineEdit *roi_height;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_5;
    QComboBox *weldControl;
    QLabel *label_15;

    void setupUi(QWidget *SettingUI)
    {
        if (SettingUI->objectName().isEmpty())
            SettingUI->setObjectName(QStringLiteral("SettingUI"));
        SettingUI->resize(1144, 807);
        groupBox_3 = new QGroupBox(SettingUI);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 20, 333, 239));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(15);
        groupBox_3->setFont(font);
        verticalLayout = new QVBoxLayout(groupBox_3);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setObjectName(QStringLiteral("horizontalLayout_21"));
        label_57 = new QLabel(groupBox_3);
        label_57->setObjectName(QStringLiteral("label_57"));
        sizePolicy.setHeightForWidth(label_57->sizePolicy().hasHeightForWidth());
        label_57->setSizePolicy(sizePolicy);
        label_57->setFont(font);

        horizontalLayout_21->addWidget(label_57);

        IPAddr = new QLineEdit(groupBox_3);
        IPAddr->setObjectName(QStringLiteral("IPAddr"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(IPAddr->sizePolicy().hasHeightForWidth());
        IPAddr->setSizePolicy(sizePolicy1);
        IPAddr->setFont(font);

        horizontalLayout_21->addWidget(IPAddr);


        verticalLayout->addLayout(horizontalLayout_21);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setFont(font);

        horizontalLayout->addWidget(label_7);

        port = new QLineEdit(groupBox_3);
        port->setObjectName(QStringLiteral("port"));
        sizePolicy1.setHeightForWidth(port->sizePolicy().hasHeightForWidth());
        port->setSizePolicy(sizePolicy1);
        port->setFont(font);
        port->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout->addWidget(port);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QStringLiteral("label_14"));
        sizePolicy.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy);
        label_14->setFont(font);

        horizontalLayout_2->addWidget(label_14);

        speed = new QLineEdit(groupBox_3);
        speed->setObjectName(QStringLiteral("speed"));
        sizePolicy1.setHeightForWidth(speed->sizePolicy().hasHeightForWidth());
        speed->setSizePolicy(sizePolicy1);
        speed->setFont(font);

        horizontalLayout_2->addWidget(speed);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName(QStringLiteral("horizontalLayout_17"));
        label_42 = new QLabel(groupBox_3);
        label_42->setObjectName(QStringLiteral("label_42"));
        sizePolicy.setHeightForWidth(label_42->sizePolicy().hasHeightForWidth());
        label_42->setSizePolicy(sizePolicy);
        label_42->setFont(font);

        horizontalLayout_17->addWidget(label_42);

        scanSpeed = new QLineEdit(groupBox_3);
        scanSpeed->setObjectName(QStringLiteral("scanSpeed"));
        sizePolicy1.setHeightForWidth(scanSpeed->sizePolicy().hasHeightForWidth());
        scanSpeed->setSizePolicy(sizePolicy1);
        scanSpeed->setFont(font);

        horizontalLayout_17->addWidget(scanSpeed);


        verticalLayout->addLayout(horizontalLayout_17);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_3 = new QLabel(groupBox_3);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setFont(font);

        horizontalLayout_4->addWidget(label_3);

        simulateSpeed = new QLineEdit(groupBox_3);
        simulateSpeed->setObjectName(QStringLiteral("simulateSpeed"));
        simulateSpeed->setFont(font);

        horizontalLayout_4->addWidget(simulateSpeed);


        verticalLayout->addLayout(horizontalLayout_4);

        groupBox_4 = new QGroupBox(SettingUI);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(20, 260, 325, 191));
        sizePolicy.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy);
        groupBox_4->setFont(font);
        gridLayout_4 = new QGridLayout(groupBox_4);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_2 = new QLabel(groupBox_4);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setFont(font);

        gridLayout_4->addWidget(label_2, 0, 0, 1, 1);

        laserBrightness = new QLineEdit(groupBox_4);
        laserBrightness->setObjectName(QStringLiteral("laserBrightness"));
        sizePolicy1.setHeightForWidth(laserBrightness->sizePolicy().hasHeightForWidth());
        laserBrightness->setSizePolicy(sizePolicy1);
        laserBrightness->setFont(font);

        gridLayout_4->addWidget(laserBrightness, 1, 1, 1, 1);

        label_5 = new QLabel(groupBox_4);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setFont(font);

        gridLayout_4->addWidget(label_5, 1, 0, 1, 1);

        laserSerialPort = new QComboBox(groupBox_4);
        laserSerialPort->setObjectName(QStringLiteral("laserSerialPort"));
        sizePolicy.setHeightForWidth(laserSerialPort->sizePolicy().hasHeightForWidth());
        laserSerialPort->setSizePolicy(sizePolicy);
        laserSerialPort->setFont(font);
        laserSerialPort->setEditable(true);

        gridLayout_4->addWidget(laserSerialPort, 0, 1, 1, 1);

        laserControl = new QComboBox(groupBox_4);
        laserControl->setObjectName(QStringLiteral("laserControl"));

        gridLayout_4->addWidget(laserControl, 2, 1, 1, 1);

        label_6 = new QLabel(groupBox_4);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setFont(font);

        gridLayout_4->addWidget(label_6, 2, 0, 1, 1);

        groupBox_13 = new QGroupBox(SettingUI);
        groupBox_13->setObjectName(QStringLiteral("groupBox_13"));
        groupBox_13->setGeometry(QRect(20, 450, 321, 181));
        sizePolicy1.setHeightForWidth(groupBox_13->sizePolicy().hasHeightForWidth());
        groupBox_13->setSizePolicy(sizePolicy1);
        groupBox_13->setFont(font);
        gridLayout_3 = new QGridLayout(groupBox_13);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        motorSpeed = new QLineEdit(groupBox_13);
        motorSpeed->setObjectName(QStringLiteral("motorSpeed"));

        gridLayout->addWidget(motorSpeed, 1, 1, 1, 1);

        motorSerialPort = new QComboBox(groupBox_13);
        motorSerialPort->setObjectName(QStringLiteral("motorSerialPort"));
        sizePolicy1.setHeightForWidth(motorSerialPort->sizePolicy().hasHeightForWidth());
        motorSerialPort->setSizePolicy(sizePolicy1);
        motorSerialPort->setFont(font);
        motorSerialPort->setEditable(true);

        gridLayout->addWidget(motorSerialPort, 0, 1, 1, 1);

        label_4 = new QLabel(groupBox_13);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        label_58 = new QLabel(groupBox_13);
        label_58->setObjectName(QStringLiteral("label_58"));
        sizePolicy1.setHeightForWidth(label_58->sizePolicy().hasHeightForWidth());
        label_58->setSizePolicy(sizePolicy1);
        label_58->setFont(font);

        gridLayout->addWidget(label_58, 0, 0, 1, 1);

        label_11 = new QLabel(groupBox_13);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout->addWidget(label_11, 2, 0, 1, 1);

        totalSteps = new QLineEdit(groupBox_13);
        totalSteps->setObjectName(QStringLiteral("totalSteps"));

        gridLayout->addWidget(totalSteps, 2, 1, 1, 1);


        gridLayout_3->addLayout(gridLayout, 0, 0, 1, 1);

        saveSetting = new QPushButton(SettingUI);
        saveSetting->setObjectName(QStringLiteral("saveSetting"));
        saveSetting->setGeometry(QRect(470, 530, 191, 91));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(saveSetting->sizePolicy().hasHeightForWidth());
        saveSetting->setSizePolicy(sizePolicy2);
        QFont font1;
        font1.setPointSize(20);
        saveSetting->setFont(font1);
        groupBox_6 = new QGroupBox(SettingUI);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(430, 20, 309, 241));
        sizePolicy.setHeightForWidth(groupBox_6->sizePolicy().hasHeightForWidth());
        groupBox_6->setSizePolicy(sizePolicy);
        groupBox_6->setFont(font);
        gridLayout_2 = new QGridLayout(groupBox_6);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_18 = new QLabel(groupBox_6);
        label_18->setObjectName(QStringLiteral("label_18"));
        sizePolicy.setHeightForWidth(label_18->sizePolicy().hasHeightForWidth());
        label_18->setSizePolicy(sizePolicy);
        label_18->setFont(font);

        gridLayout_2->addWidget(label_18, 0, 0, 1, 1);

        cameraExposure = new QLineEdit(groupBox_6);
        cameraExposure->setObjectName(QStringLiteral("cameraExposure"));
        sizePolicy1.setHeightForWidth(cameraExposure->sizePolicy().hasHeightForWidth());
        cameraExposure->setSizePolicy(sizePolicy1);
        cameraExposure->setFont(font);

        gridLayout_2->addWidget(cameraExposure, 0, 1, 1, 1);

        label_17 = new QLabel(groupBox_6);
        label_17->setObjectName(QStringLiteral("label_17"));
        sizePolicy.setHeightForWidth(label_17->sizePolicy().hasHeightForWidth());
        label_17->setSizePolicy(sizePolicy);
        label_17->setFont(font);

        gridLayout_2->addWidget(label_17, 1, 0, 1, 1);

        cameraGain = new QLineEdit(groupBox_6);
        cameraGain->setObjectName(QStringLiteral("cameraGain"));
        sizePolicy1.setHeightForWidth(cameraGain->sizePolicy().hasHeightForWidth());
        cameraGain->setSizePolicy(sizePolicy1);
        cameraGain->setFont(font);

        gridLayout_2->addWidget(cameraGain, 1, 1, 1, 1);

        label = new QLabel(groupBox_6);
        label->setObjectName(QStringLiteral("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setFont(font);

        gridLayout_2->addWidget(label, 2, 0, 1, 1);

        roi_offset_x = new QLineEdit(groupBox_6);
        roi_offset_x->setObjectName(QStringLiteral("roi_offset_x"));
        sizePolicy1.setHeightForWidth(roi_offset_x->sizePolicy().hasHeightForWidth());
        roi_offset_x->setSizePolicy(sizePolicy1);
        roi_offset_x->setFont(font);

        gridLayout_2->addWidget(roi_offset_x, 2, 1, 1, 1);

        label_8 = new QLabel(groupBox_6);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);
        label_8->setFont(font);

        gridLayout_2->addWidget(label_8, 3, 0, 1, 1);

        roi_offset_y = new QLineEdit(groupBox_6);
        roi_offset_y->setObjectName(QStringLiteral("roi_offset_y"));
        sizePolicy1.setHeightForWidth(roi_offset_y->sizePolicy().hasHeightForWidth());
        roi_offset_y->setSizePolicy(sizePolicy1);
        roi_offset_y->setFont(font);

        gridLayout_2->addWidget(roi_offset_y, 3, 1, 1, 1);

        label_9 = new QLabel(groupBox_6);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);
        label_9->setFont(font);

        gridLayout_2->addWidget(label_9, 4, 0, 1, 1);

        roi_width = new QLineEdit(groupBox_6);
        roi_width->setObjectName(QStringLiteral("roi_width"));
        sizePolicy1.setHeightForWidth(roi_width->sizePolicy().hasHeightForWidth());
        roi_width->setSizePolicy(sizePolicy1);
        roi_width->setFont(font);

        gridLayout_2->addWidget(roi_width, 4, 1, 1, 1);

        label_10 = new QLabel(groupBox_6);
        label_10->setObjectName(QStringLiteral("label_10"));
        sizePolicy.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy);
        label_10->setFont(font);

        gridLayout_2->addWidget(label_10, 5, 0, 1, 1);

        roi_height = new QLineEdit(groupBox_6);
        roi_height->setObjectName(QStringLiteral("roi_height"));
        sizePolicy1.setHeightForWidth(roi_height->sizePolicy().hasHeightForWidth());
        roi_height->setSizePolicy(sizePolicy1);
        roi_height->setFont(font);

        gridLayout_2->addWidget(roi_height, 5, 1, 1, 1);

        groupBox_5 = new QGroupBox(SettingUI);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(420, 270, 325, 191));
        sizePolicy.setHeightForWidth(groupBox_5->sizePolicy().hasHeightForWidth());
        groupBox_5->setSizePolicy(sizePolicy);
        groupBox_5->setFont(font);
        gridLayout_5 = new QGridLayout(groupBox_5);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        weldControl = new QComboBox(groupBox_5);
        weldControl->setObjectName(QStringLiteral("weldControl"));

        gridLayout_5->addWidget(weldControl, 0, 1, 1, 1);

        label_15 = new QLabel(groupBox_5);
        label_15->setObjectName(QStringLiteral("label_15"));
        sizePolicy.setHeightForWidth(label_15->sizePolicy().hasHeightForWidth());
        label_15->setSizePolicy(sizePolicy);
        label_15->setFont(font);

        gridLayout_5->addWidget(label_15, 0, 0, 1, 1);


        retranslateUi(SettingUI);

        QMetaObject::connectSlotsByName(SettingUI);
    } // setupUi

    void retranslateUi(QWidget *SettingUI)
    {
        SettingUI->setWindowTitle(QApplication::translate("SettingUI", "Form", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("SettingUI", "\346\234\272\345\231\250\344\272\272\350\256\276\347\275\256", Q_NULLPTR));
        label_57->setText(QApplication::translate("SettingUI", "IP\345\234\260\345\235\200\357\274\232", Q_NULLPTR));
        IPAddr->setText(QApplication::translate("SettingUI", "192.168.1.88", Q_NULLPTR));
        label_7->setText(QApplication::translate("SettingUI", "\351\200\232\344\277\241\347\253\257\345\217\243\357\274\232", Q_NULLPTR));
        port->setText(QApplication::translate("SettingUI", "60001", Q_NULLPTR));
        label_14->setText(QApplication::translate("SettingUI", "\350\277\220\345\212\250\351\200\237\345\272\246\357\274\232", Q_NULLPTR));
        speed->setText(QApplication::translate("SettingUI", "0.06", Q_NULLPTR));
        label_42->setText(QApplication::translate("SettingUI", "\346\211\253\346\217\217\351\200\237\345\272\246\357\274\232", Q_NULLPTR));
        scanSpeed->setText(QApplication::translate("SettingUI", "0.01", Q_NULLPTR));
        label_3->setText(QApplication::translate("SettingUI", "\346\250\241\346\213\237\347\204\212\346\216\245\351\200\237\345\272\246:", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("SettingUI", "\346\277\200\345\205\211\350\256\276\347\275\256", Q_NULLPTR));
        label_2->setText(QApplication::translate("SettingUI", "\344\270\262\345\217\243\345\217\267\357\274\232", Q_NULLPTR));
        laserBrightness->setText(QApplication::translate("SettingUI", "100", Q_NULLPTR));
        label_5->setText(QApplication::translate("SettingUI", "\346\277\200\345\205\211\344\272\256\345\272\246\357\274\232", Q_NULLPTR));
        laserSerialPort->clear();
        laserSerialPort->insertItems(0, QStringList()
         << QApplication::translate("SettingUI", "/dev/ttyUSB0", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyUSB1", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyUSB2", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS0", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS1", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS2", Q_NULLPTR)
        );
        laserControl->clear();
        laserControl->insertItems(0, QStringList()
         << QApplication::translate("SettingUI", "\344\270\262\345\217\243", Q_NULLPTR)
         << QApplication::translate("SettingUI", "PLC", Q_NULLPTR)
        );
        label_6->setText(QApplication::translate("SettingUI", "\346\277\200\345\205\211\346\216\247\345\210\266\357\274\232", Q_NULLPTR));
        groupBox_13->setTitle(QApplication::translate("SettingUI", "\345\217\230\344\275\215\346\234\272\350\256\276\347\275\256", Q_NULLPTR));
        motorSerialPort->clear();
        motorSerialPort->insertItems(0, QStringList()
         << QApplication::translate("SettingUI", "/dev/ttyUSB0", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyUSB1", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyUSB2", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS0", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS1", Q_NULLPTR)
         << QApplication::translate("SettingUI", "/dev/ttyS2", Q_NULLPTR)
        );
        label_4->setText(QApplication::translate("SettingUI", "\351\200\237\345\272\246:", Q_NULLPTR));
        label_58->setText(QApplication::translate("SettingUI", "\344\270\262\345\217\243\345\217\267\357\274\232", Q_NULLPTR));
        label_11->setText(QApplication::translate("SettingUI", "\346\200\273\346\255\245\346\225\260:", Q_NULLPTR));
        saveSetting->setText(QApplication::translate("SettingUI", "\344\277\235\345\255\230", Q_NULLPTR));
        groupBox_6->setTitle(QApplication::translate("SettingUI", "\347\233\270\346\234\272\350\256\276\347\275\256", Q_NULLPTR));
        label_18->setText(QApplication::translate("SettingUI", "\346\233\235\345\205\211\357\274\232", Q_NULLPTR));
        cameraExposure->setText(QApplication::translate("SettingUI", "2000", Q_NULLPTR));
        label_17->setText(QApplication::translate("SettingUI", "\345\242\236\347\233\212\357\274\232", Q_NULLPTR));
        cameraGain->setText(QApplication::translate("SettingUI", "0", Q_NULLPTR));
        label->setText(QApplication::translate("SettingUI", "roi_offx :", Q_NULLPTR));
        label_8->setText(QApplication::translate("SettingUI", "roi_offy :", Q_NULLPTR));
        label_9->setText(QApplication::translate("SettingUI", "roi_width :", Q_NULLPTR));
        label_10->setText(QApplication::translate("SettingUI", "roi_height :", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("SettingUI", "\347\204\212\346\216\245\350\256\276\347\275\256", Q_NULLPTR));
        weldControl->clear();
        weldControl->insertItems(0, QStringList()
         << QApplication::translate("SettingUI", "\347\251\272\350\265\260", Q_NULLPTR)
         << QApplication::translate("SettingUI", "\347\204\212\346\216\245", Q_NULLPTR)
         << QApplication::translate("SettingUI", "\347\251\272\350\265\260\345\220\216\347\204\212\346\216\245", Q_NULLPTR)
         << QApplication::translate("SettingUI", "\346\211\253\346\217\217", Q_NULLPTR)
        );
        label_15->setText(QApplication::translate("SettingUI", "\347\204\212\346\216\245\346\216\247\345\210\266\357\274\232", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SettingUI: public Ui_SettingUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGUI_H
