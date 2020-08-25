/********************************************************************************
** Form generated from reading UI file 'sensorui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSORUI_H
#define UI_SENSORUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <base/imagelabel.h>

QT_BEGIN_NAMESPACE

class Ui_SensorUI
{
public:
    QGridLayout *gridLayout;
    QWidget *widget_3;
    ImageLabel *picture;
    QWidget *widget_4;
    QVBoxLayout *verticalLayout;
    QLineEdit *centerUV;
    QPushButton *laserClose;
    QPushButton *alwaysLight;
    QPushButton *reconnectLaser;
    QPushButton *takePicture;
    QPushButton *openPic;
    QPushButton *openCamera;
    QLabel *UVL;
    QLineEdit *currUVtoPos;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QComboBox *btnObject;
    QComboBox *seamList;
    QComboBox *combme;
    QPushButton *btnMoveTo;
    QPushButton *btnSave;
    QPushButton *btnHome;
    QWidget *widget_2;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *autoSaveImg;
    QCheckBox *checkMove;
    QCheckBox *tracingMode;
    QCheckBox *checkGrid;
    QLabel *cameraStatus;

    void setupUi(QWidget *SensorUI)
    {
        if (SensorUI->objectName().isEmpty())
            SensorUI->setObjectName(QStringLiteral("SensorUI"));
        SensorUI->resize(830, 673);
        SensorUI->setMaximumSize(QSize(852, 673));
        gridLayout = new QGridLayout(SensorUI);
        gridLayout->setSpacing(2);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        widget_3 = new QWidget(SensorUI);
        widget_3->setObjectName(QStringLiteral("widget_3"));
        widget_3->setMinimumSize(QSize(792, 540));
        picture = new ImageLabel(widget_3);
        picture->setObjectName(QStringLiteral("picture"));
        picture->setEnabled(true);
        picture->setGeometry(QRect(0, 0, 720, 540));
        picture->setMinimumSize(QSize(720, 540));
        picture->setMaximumSize(QSize(720, 540));
        QFont font;
        font.setPointSize(12);
        picture->setFont(font);
        picture->setAutoFillBackground(false);
        picture->setFrameShape(QFrame::Box);
        picture->setScaledContents(true);
        widget_4 = new QWidget(widget_3);
        widget_4->setObjectName(QStringLiteral("widget_4"));
        widget_4->setGeometry(QRect(720, 240, 91, 291));
        widget_4->setMaximumSize(QSize(9999, 9999));
        widget_4->setFont(font);
        verticalLayout = new QVBoxLayout(widget_4);
        verticalLayout->setSpacing(3);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        centerUV = new QLineEdit(widget_4);
        centerUV->setObjectName(QStringLiteral("centerUV"));
        centerUV->setMinimumSize(QSize(100, 30));
        centerUV->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(centerUV);

        laserClose = new QPushButton(widget_4);
        laserClose->setObjectName(QStringLiteral("laserClose"));
        laserClose->setMinimumSize(QSize(0, 30));
        laserClose->setFont(font);

        verticalLayout->addWidget(laserClose);

        alwaysLight = new QPushButton(widget_4);
        alwaysLight->setObjectName(QStringLiteral("alwaysLight"));
        alwaysLight->setMinimumSize(QSize(0, 30));
        alwaysLight->setFont(font);

        verticalLayout->addWidget(alwaysLight);

        reconnectLaser = new QPushButton(widget_4);
        reconnectLaser->setObjectName(QStringLiteral("reconnectLaser"));
        reconnectLaser->setMinimumSize(QSize(0, 30));
        reconnectLaser->setFont(font);

        verticalLayout->addWidget(reconnectLaser);

        takePicture = new QPushButton(widget_4);
        takePicture->setObjectName(QStringLiteral("takePicture"));
        takePicture->setMinimumSize(QSize(0, 30));
        takePicture->setFont(font);

        verticalLayout->addWidget(takePicture);

        openPic = new QPushButton(widget_4);
        openPic->setObjectName(QStringLiteral("openPic"));
        openPic->setMinimumSize(QSize(0, 30));

        verticalLayout->addWidget(openPic);

        openCamera = new QPushButton(widget_4);
        openCamera->setObjectName(QStringLiteral("openCamera"));
        openCamera->setMinimumSize(QSize(0, 30));
        openCamera->setFont(font);

        verticalLayout->addWidget(openCamera);

        UVL = new QLabel(widget_4);
        UVL->setObjectName(QStringLiteral("UVL"));
        UVL->setMinimumSize(QSize(80, 30));
        UVL->setMaximumSize(QSize(16777215, 30));
        UVL->setFont(font);

        verticalLayout->addWidget(UVL);


        gridLayout->addWidget(widget_3, 0, 0, 1, 1);

        currUVtoPos = new QLineEdit(SensorUI);
        currUVtoPos->setObjectName(QStringLiteral("currUVtoPos"));
        currUVtoPos->setMinimumSize(QSize(480, 30));
        currUVtoPos->setMaximumSize(QSize(9999, 40));
        currUVtoPos->setFont(font);

        gridLayout->addWidget(currUVtoPos, 1, 0, 1, 1);

        widget = new QWidget(SensorUI);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setMaximumSize(QSize(16777215, 40));
        widget->setFont(font);
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(10);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(2, 0, 0, 0);
        btnObject = new QComboBox(widget);
        btnObject->setObjectName(QStringLiteral("btnObject"));
        btnObject->setMinimumSize(QSize(0, 40));
        btnObject->setFont(font);

        horizontalLayout->addWidget(btnObject);

        seamList = new QComboBox(widget);
        seamList->setObjectName(QStringLiteral("seamList"));
        seamList->setMinimumSize(QSize(50, 40));
        seamList->setFont(font);

        horizontalLayout->addWidget(seamList);

        combme = new QComboBox(widget);
        combme->setObjectName(QStringLiteral("combme"));
        combme->setMinimumSize(QSize(0, 40));
        combme->setFont(font);

        horizontalLayout->addWidget(combme);

        btnMoveTo = new QPushButton(widget);
        btnMoveTo->setObjectName(QStringLiteral("btnMoveTo"));
        btnMoveTo->setMinimumSize(QSize(0, 40));
        btnMoveTo->setFont(font);

        horizontalLayout->addWidget(btnMoveTo);

        btnSave = new QPushButton(widget);
        btnSave->setObjectName(QStringLiteral("btnSave"));
        btnSave->setMinimumSize(QSize(0, 40));
        btnSave->setFont(font);

        horizontalLayout->addWidget(btnSave);

        btnHome = new QPushButton(widget);
        btnHome->setObjectName(QStringLiteral("btnHome"));
        btnHome->setMinimumSize(QSize(0, 40));
        btnHome->setFont(font);

        horizontalLayout->addWidget(btnHome);


        gridLayout->addWidget(widget, 2, 0, 1, 1);

        widget_2 = new QWidget(SensorUI);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        widget_2->setMaximumSize(QSize(16777215, 40));
        widget_2->setFont(font);
        horizontalLayout_2 = new QHBoxLayout(widget_2);
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        autoSaveImg = new QCheckBox(widget_2);
        autoSaveImg->setObjectName(QStringLiteral("autoSaveImg"));
        autoSaveImg->setFont(font);

        horizontalLayout_2->addWidget(autoSaveImg);

        checkMove = new QCheckBox(widget_2);
        checkMove->setObjectName(QStringLiteral("checkMove"));
        checkMove->setFont(font);

        horizontalLayout_2->addWidget(checkMove);

        tracingMode = new QCheckBox(widget_2);
        tracingMode->setObjectName(QStringLiteral("tracingMode"));
        tracingMode->setFont(font);

        horizontalLayout_2->addWidget(tracingMode);

        checkGrid = new QCheckBox(widget_2);
        checkGrid->setObjectName(QStringLiteral("checkGrid"));
        checkGrid->setMinimumSize(QSize(0, 0));
        checkGrid->setFont(font);

        horizontalLayout_2->addWidget(checkGrid);

        cameraStatus = new QLabel(widget_2);
        cameraStatus->setObjectName(QStringLiteral("cameraStatus"));
        cameraStatus->setFont(font);

        horizontalLayout_2->addWidget(cameraStatus);


        gridLayout->addWidget(widget_2, 3, 0, 1, 1);


        retranslateUi(SensorUI);

        QMetaObject::connectSlotsByName(SensorUI);
    } // setupUi

    void retranslateUi(QWidget *SensorUI)
    {
        SensorUI->setWindowTitle(QApplication::translate("SensorUI", "Form", Q_NULLPTR));
        picture->setText(QString());
        centerUV->setText(QApplication::translate("SensorUI", "615,535", Q_NULLPTR));
        laserClose->setText(QApplication::translate("SensorUI", "\345\270\270\351\227\255", Q_NULLPTR));
        alwaysLight->setText(QApplication::translate("SensorUI", "\345\270\270\344\272\256", Q_NULLPTR));
        reconnectLaser->setText(QApplication::translate("SensorUI", "\351\207\215\350\277\236\346\277\200\345\205\211", Q_NULLPTR));
        takePicture->setText(QApplication::translate("SensorUI", "\346\213\215\347\205\247", Q_NULLPTR));
        openPic->setText(QApplication::translate("SensorUI", "\346\211\223\345\274\200\345\233\276\347\211\207", Q_NULLPTR));
        openCamera->setText(QApplication::translate("SensorUI", "\346\211\223\345\274\200\347\233\270\346\234\272", Q_NULLPTR));
        UVL->setText(QString());
        seamList->clear();
        seamList->insertItems(0, QStringList()
         << QString()
        );
        combme->clear();
        combme->insertItems(0, QStringList()
         << QApplication::translate("SensorUI", "scanPos0", Q_NULLPTR)
         << QApplication::translate("SensorUI", "scanPos1", Q_NULLPTR)
         << QApplication::translate("SensorUI", "scanPos2", Q_NULLPTR)
         << QApplication::translate("SensorUI", "scanPos3", Q_NULLPTR)
         << QApplication::translate("SensorUI", "scanPos4", Q_NULLPTR)
        );
        btnMoveTo->setText(QApplication::translate("SensorUI", "move to", Q_NULLPTR));
        btnSave->setText(QApplication::translate("SensorUI", "save", Q_NULLPTR));
        btnHome->setText(QApplication::translate("SensorUI", "backHome", Q_NULLPTR));
        autoSaveImg->setText(QApplication::translate("SensorUI", "\350\207\252\345\212\250\344\277\235\345\255\230\345\233\276\347\211\207", Q_NULLPTR));
        checkMove->setText(QApplication::translate("SensorUI", "\347\247\273\345\212\250\347\204\212\346\236\252", Q_NULLPTR));
        tracingMode->setText(QApplication::translate("SensorUI", "\350\267\237\350\270\252\346\250\241\345\274\217", Q_NULLPTR));
        checkGrid->setText(QApplication::translate("SensorUI", "\346\240\274\347\202\271", Q_NULLPTR));
        cameraStatus->setText(QApplication::translate("SensorUI", "\347\233\270\346\234\272\347\212\266\346\200\201:\345\205\263\351\227\255", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SensorUI: public Ui_SensorUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSORUI_H
