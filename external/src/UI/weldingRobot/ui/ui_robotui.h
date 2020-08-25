/********************************************************************************
** Form generated from reading UI file 'robotui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOTUI_H
#define UI_ROBOTUI_H

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

class Ui_RobotUI
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_11;
    QGridLayout *gridLayout_3;
    QLabel *label_8;
    QLineEdit *dPosX;
    QLabel *label_9;
    QLabel *currPosY;
    QLineEdit *dPosY;
    QLabel *label_10;
    QLabel *currPosZ;
    QLineEdit *dPosZ;
    QLabel *label_11;
    QLabel *currPosA;
    QLineEdit *dPosA;
    QLabel *label_12;
    QLabel *currPosB;
    QLineEdit *dPosB;
    QLabel *label_13;
    QLabel *currPosC;
    QLineEdit *dPosC;
    QPushButton *stopMove;
    QComboBox *currentTCP;
    QPushButton *clear;
    QLabel *label_14;
    QPushButton *abMove;
    QLineEdit *coordinateString;
    QLineEdit *suEdit;
    QLabel *currPosX;
    QPushButton *relativeMove;
    QPushButton *getCurrPos;
    QGroupBox *groupBox_12;
    QGridLayout *gridLayout_2;
    QLabel *label_53;
    QLabel *currA1;
    QLineEdit *dA1;
    QPushButton *cur_axis;
    QLabel *label_48;
    QLabel *currA2;
    QLineEdit *dA2;
    QLabel *label_52;
    QLabel *currA3;
    QLineEdit *dA3;
    QPushButton *relativeMoveAxle;
    QLabel *label_50;
    QLabel *currA4;
    QLineEdit *dA4;
    QLabel *label_51;
    QLabel *currA5;
    QLineEdit *dA5;
    QPushButton *absoluteMoveAxle;
    QLabel *label_49;
    QLabel *currA6;
    QLineEdit *dA6;
    QLineEdit *jmove;
    QGroupBox *groupBox_13;
    QGridLayout *gridLayout;
    QLabel *label_57;
    QLineEdit *dx;
    QLabel *label_58;
    QLineEdit *dy;
    QLabel *label_59;
    QLineEdit *dz;
    QLabel *label_55;
    QLineEdit *dry;
    QLabel *label_56;
    QLineEdit *drz;
    QPushButton *RxyzMove;
    QLineEdit *drx;
    QLabel *label_54;
    QGroupBox *groupBox_14;
    QGridLayout *gridLayout_4;
    QLabel *label_path;
    QPushButton *btnOpenFile;
    QPushButton *btnMove;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *btndiscon;

    void setupUi(QWidget *RobotUI)
    {
        if (RobotUI->objectName().isEmpty())
            RobotUI->setObjectName(QStringLiteral("RobotUI"));
        RobotUI->resize(335, 717);
        RobotUI->setMaximumSize(QSize(550, 717));
        verticalLayout = new QVBoxLayout(RobotUI);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        groupBox_11 = new QGroupBox(RobotUI);
        groupBox_11->setObjectName(QStringLiteral("groupBox_11"));
        groupBox_11->setMaximumSize(QSize(16777215, 999));
        QFont font;
        font.setPointSize(12);
        groupBox_11->setFont(font);
        gridLayout_3 = new QGridLayout(groupBox_11);
        gridLayout_3->setSpacing(0);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(groupBox_11);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setMaximumSize(QSize(40, 16777215));
        label_8->setFont(font);

        gridLayout_3->addWidget(label_8, 0, 0, 1, 1);

        dPosX = new QLineEdit(groupBox_11);
        dPosX->setObjectName(QStringLiteral("dPosX"));
        dPosX->setMaximumSize(QSize(999, 16777215));
        dPosX->setFont(font);

        gridLayout_3->addWidget(dPosX, 0, 3, 1, 1);

        label_9 = new QLabel(groupBox_11);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setMaximumSize(QSize(40, 16777215));
        label_9->setFont(font);

        gridLayout_3->addWidget(label_9, 1, 0, 1, 1);

        currPosY = new QLabel(groupBox_11);
        currPosY->setObjectName(QStringLiteral("currPosY"));
        currPosY->setMinimumSize(QSize(0, 0));
        currPosY->setFont(font);

        gridLayout_3->addWidget(currPosY, 1, 2, 1, 1);

        dPosY = new QLineEdit(groupBox_11);
        dPosY->setObjectName(QStringLiteral("dPosY"));
        dPosY->setMaximumSize(QSize(999, 16777215));
        dPosY->setFont(font);

        gridLayout_3->addWidget(dPosY, 1, 3, 1, 1);

        label_10 = new QLabel(groupBox_11);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setMaximumSize(QSize(40, 16777215));
        label_10->setFont(font);

        gridLayout_3->addWidget(label_10, 2, 0, 1, 1);

        currPosZ = new QLabel(groupBox_11);
        currPosZ->setObjectName(QStringLiteral("currPosZ"));
        currPosZ->setMinimumSize(QSize(0, 0));
        currPosZ->setFont(font);

        gridLayout_3->addWidget(currPosZ, 2, 2, 1, 1);

        dPosZ = new QLineEdit(groupBox_11);
        dPosZ->setObjectName(QStringLiteral("dPosZ"));
        dPosZ->setMaximumSize(QSize(999, 16777215));
        dPosZ->setFont(font);

        gridLayout_3->addWidget(dPosZ, 2, 3, 1, 1);

        label_11 = new QLabel(groupBox_11);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setMaximumSize(QSize(40, 16777215));
        label_11->setFont(font);

        gridLayout_3->addWidget(label_11, 3, 0, 1, 1);

        currPosA = new QLabel(groupBox_11);
        currPosA->setObjectName(QStringLiteral("currPosA"));
        currPosA->setMinimumSize(QSize(0, 0));
        currPosA->setFont(font);

        gridLayout_3->addWidget(currPosA, 3, 2, 1, 1);

        dPosA = new QLineEdit(groupBox_11);
        dPosA->setObjectName(QStringLiteral("dPosA"));
        dPosA->setMaximumSize(QSize(999, 16777215));
        dPosA->setFont(font);

        gridLayout_3->addWidget(dPosA, 3, 3, 1, 1);

        label_12 = new QLabel(groupBox_11);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setMaximumSize(QSize(40, 16777215));
        label_12->setFont(font);

        gridLayout_3->addWidget(label_12, 4, 0, 1, 1);

        currPosB = new QLabel(groupBox_11);
        currPosB->setObjectName(QStringLiteral("currPosB"));
        currPosB->setMinimumSize(QSize(0, 0));
        currPosB->setFont(font);

        gridLayout_3->addWidget(currPosB, 4, 2, 1, 1);

        dPosB = new QLineEdit(groupBox_11);
        dPosB->setObjectName(QStringLiteral("dPosB"));
        dPosB->setMaximumSize(QSize(999, 16777215));
        dPosB->setFont(font);

        gridLayout_3->addWidget(dPosB, 4, 3, 1, 1);

        label_13 = new QLabel(groupBox_11);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setMaximumSize(QSize(40, 16777215));
        label_13->setFont(font);

        gridLayout_3->addWidget(label_13, 5, 0, 1, 1);

        currPosC = new QLabel(groupBox_11);
        currPosC->setObjectName(QStringLiteral("currPosC"));
        currPosC->setMinimumSize(QSize(0, 0));
        currPosC->setFont(font);

        gridLayout_3->addWidget(currPosC, 5, 2, 1, 1);

        dPosC = new QLineEdit(groupBox_11);
        dPosC->setObjectName(QStringLiteral("dPosC"));
        dPosC->setMaximumSize(QSize(999, 16777215));
        dPosC->setFont(font);

        gridLayout_3->addWidget(dPosC, 5, 3, 1, 1);

        stopMove = new QPushButton(groupBox_11);
        stopMove->setObjectName(QStringLiteral("stopMove"));
        stopMove->setMaximumSize(QSize(999, 26));
        stopMove->setFont(font);

        gridLayout_3->addWidget(stopMove, 6, 0, 1, 2);

        currentTCP = new QComboBox(groupBox_11);
        currentTCP->setObjectName(QStringLiteral("currentTCP"));
        currentTCP->setMinimumSize(QSize(0, 0));
        currentTCP->setMaximumSize(QSize(9999, 26));
        currentTCP->setFont(font);

        gridLayout_3->addWidget(currentTCP, 6, 2, 1, 1);

        clear = new QPushButton(groupBox_11);
        clear->setObjectName(QStringLiteral("clear"));
        clear->setMaximumSize(QSize(999, 26));
        clear->setFont(font);

        gridLayout_3->addWidget(clear, 6, 3, 1, 1);

        label_14 = new QLabel(groupBox_11);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setFont(font);

        gridLayout_3->addWidget(label_14, 7, 2, 1, 1);

        abMove = new QPushButton(groupBox_11);
        abMove->setObjectName(QStringLiteral("abMove"));
        abMove->setMaximumSize(QSize(999, 26));
        abMove->setFont(font);

        gridLayout_3->addWidget(abMove, 8, 2, 1, 1);

        coordinateString = new QLineEdit(groupBox_11);
        coordinateString->setObjectName(QStringLiteral("coordinateString"));
        coordinateString->setFont(font);

        gridLayout_3->addWidget(coordinateString, 9, 0, 1, 4);

        suEdit = new QLineEdit(groupBox_11);
        suEdit->setObjectName(QStringLiteral("suEdit"));
        suEdit->setMaximumSize(QSize(999, 16777215));
        suEdit->setFont(font);

        gridLayout_3->addWidget(suEdit, 7, 3, 1, 1);

        currPosX = new QLabel(groupBox_11);
        currPosX->setObjectName(QStringLiteral("currPosX"));
        currPosX->setMinimumSize(QSize(0, 0));
        currPosX->setFont(font);

        gridLayout_3->addWidget(currPosX, 0, 2, 1, 1);

        relativeMove = new QPushButton(groupBox_11);
        relativeMove->setObjectName(QStringLiteral("relativeMove"));
        relativeMove->setMaximumSize(QSize(999, 26));
        relativeMove->setFont(font);

        gridLayout_3->addWidget(relativeMove, 8, 3, 1, 1);

        getCurrPos = new QPushButton(groupBox_11);
        getCurrPos->setObjectName(QStringLiteral("getCurrPos"));
        getCurrPos->setMaximumSize(QSize(999, 26));
        getCurrPos->setFont(font);

        gridLayout_3->addWidget(getCurrPos, 8, 0, 1, 1);


        verticalLayout->addWidget(groupBox_11);

        groupBox_12 = new QGroupBox(RobotUI);
        groupBox_12->setObjectName(QStringLiteral("groupBox_12"));
        groupBox_12->setMaximumSize(QSize(16777215, 280));
        groupBox_12->setFont(font);
        gridLayout_2 = new QGridLayout(groupBox_12);
        gridLayout_2->setSpacing(0);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_53 = new QLabel(groupBox_12);
        label_53->setObjectName(QStringLiteral("label_53"));
        label_53->setMaximumSize(QSize(25, 16777215));
        label_53->setFont(font);

        gridLayout_2->addWidget(label_53, 0, 0, 1, 1);

        currA1 = new QLabel(groupBox_12);
        currA1->setObjectName(QStringLiteral("currA1"));
        currA1->setMinimumSize(QSize(40, 0));
        currA1->setMaximumSize(QSize(80, 16777215));
        currA1->setFont(font);

        gridLayout_2->addWidget(currA1, 0, 1, 1, 1);

        dA1 = new QLineEdit(groupBox_12);
        dA1->setObjectName(QStringLiteral("dA1"));
        dA1->setMaximumSize(QSize(80, 26));
        dA1->setFont(font);

        gridLayout_2->addWidget(dA1, 0, 2, 1, 1);

        cur_axis = new QPushButton(groupBox_12);
        cur_axis->setObjectName(QStringLiteral("cur_axis"));
        cur_axis->setMinimumSize(QSize(100, 0));
        cur_axis->setMaximumSize(QSize(999, 26));
        cur_axis->setFont(font);

        gridLayout_2->addWidget(cur_axis, 0, 3, 1, 1);

        label_48 = new QLabel(groupBox_12);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setMaximumSize(QSize(25, 16777215));
        label_48->setFont(font);

        gridLayout_2->addWidget(label_48, 1, 0, 1, 1);

        currA2 = new QLabel(groupBox_12);
        currA2->setObjectName(QStringLiteral("currA2"));
        currA2->setMinimumSize(QSize(40, 0));
        currA2->setMaximumSize(QSize(80, 16777215));
        currA2->setFont(font);

        gridLayout_2->addWidget(currA2, 1, 1, 1, 1);

        dA2 = new QLineEdit(groupBox_12);
        dA2->setObjectName(QStringLiteral("dA2"));
        dA2->setMaximumSize(QSize(80, 26));
        dA2->setFont(font);

        gridLayout_2->addWidget(dA2, 1, 2, 1, 1);

        label_52 = new QLabel(groupBox_12);
        label_52->setObjectName(QStringLiteral("label_52"));
        label_52->setMaximumSize(QSize(25, 16777215));
        label_52->setFont(font);

        gridLayout_2->addWidget(label_52, 2, 0, 1, 1);

        currA3 = new QLabel(groupBox_12);
        currA3->setObjectName(QStringLiteral("currA3"));
        currA3->setMinimumSize(QSize(40, 0));
        currA3->setMaximumSize(QSize(80, 16777215));
        currA3->setFont(font);

        gridLayout_2->addWidget(currA3, 2, 1, 1, 1);

        dA3 = new QLineEdit(groupBox_12);
        dA3->setObjectName(QStringLiteral("dA3"));
        dA3->setMaximumSize(QSize(80, 26));
        dA3->setFont(font);

        gridLayout_2->addWidget(dA3, 2, 2, 1, 1);

        relativeMoveAxle = new QPushButton(groupBox_12);
        relativeMoveAxle->setObjectName(QStringLiteral("relativeMoveAxle"));
        relativeMoveAxle->setMinimumSize(QSize(100, 0));
        relativeMoveAxle->setMaximumSize(QSize(999, 26));
        relativeMoveAxle->setFont(font);

        gridLayout_2->addWidget(relativeMoveAxle, 2, 3, 1, 1);

        label_50 = new QLabel(groupBox_12);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setMaximumSize(QSize(25, 16777215));
        label_50->setFont(font);

        gridLayout_2->addWidget(label_50, 3, 0, 1, 1);

        currA4 = new QLabel(groupBox_12);
        currA4->setObjectName(QStringLiteral("currA4"));
        currA4->setMinimumSize(QSize(40, 0));
        currA4->setMaximumSize(QSize(80, 16777215));
        currA4->setFont(font);

        gridLayout_2->addWidget(currA4, 3, 1, 1, 1);

        dA4 = new QLineEdit(groupBox_12);
        dA4->setObjectName(QStringLiteral("dA4"));
        dA4->setMaximumSize(QSize(80, 26));
        dA4->setFont(font);

        gridLayout_2->addWidget(dA4, 3, 2, 1, 1);

        label_51 = new QLabel(groupBox_12);
        label_51->setObjectName(QStringLiteral("label_51"));
        label_51->setMaximumSize(QSize(25, 16777215));
        label_51->setFont(font);

        gridLayout_2->addWidget(label_51, 4, 0, 1, 1);

        currA5 = new QLabel(groupBox_12);
        currA5->setObjectName(QStringLiteral("currA5"));
        currA5->setMinimumSize(QSize(40, 0));
        currA5->setMaximumSize(QSize(80, 16777215));
        currA5->setFont(font);

        gridLayout_2->addWidget(currA5, 4, 1, 1, 1);

        dA5 = new QLineEdit(groupBox_12);
        dA5->setObjectName(QStringLiteral("dA5"));
        dA5->setMaximumSize(QSize(80, 26));
        dA5->setFont(font);

        gridLayout_2->addWidget(dA5, 4, 2, 1, 1);

        absoluteMoveAxle = new QPushButton(groupBox_12);
        absoluteMoveAxle->setObjectName(QStringLiteral("absoluteMoveAxle"));
        absoluteMoveAxle->setMinimumSize(QSize(100, 0));
        absoluteMoveAxle->setMaximumSize(QSize(100, 26));
        absoluteMoveAxle->setFont(font);

        gridLayout_2->addWidget(absoluteMoveAxle, 4, 3, 1, 1);

        label_49 = new QLabel(groupBox_12);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setMaximumSize(QSize(25, 16777215));
        label_49->setFont(font);

        gridLayout_2->addWidget(label_49, 5, 0, 1, 1);

        currA6 = new QLabel(groupBox_12);
        currA6->setObjectName(QStringLiteral("currA6"));
        currA6->setMinimumSize(QSize(40, 0));
        currA6->setMaximumSize(QSize(80, 16777215));
        currA6->setFont(font);

        gridLayout_2->addWidget(currA6, 5, 1, 1, 1);

        dA6 = new QLineEdit(groupBox_12);
        dA6->setObjectName(QStringLiteral("dA6"));
        dA6->setMaximumSize(QSize(80, 26));
        dA6->setFont(font);

        gridLayout_2->addWidget(dA6, 5, 2, 1, 1);

        jmove = new QLineEdit(groupBox_12);
        jmove->setObjectName(QStringLiteral("jmove"));
        jmove->setMaximumSize(QSize(500, 26));
        jmove->setFont(font);

        gridLayout_2->addWidget(jmove, 6, 0, 1, 4);


        verticalLayout->addWidget(groupBox_12);

        groupBox_13 = new QGroupBox(RobotUI);
        groupBox_13->setObjectName(QStringLiteral("groupBox_13"));
        groupBox_13->setMaximumSize(QSize(16777215, 120));
        groupBox_13->setFont(font);
        gridLayout = new QGridLayout(groupBox_13);
        gridLayout->setSpacing(0);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_57 = new QLabel(groupBox_13);
        label_57->setObjectName(QStringLiteral("label_57"));
        label_57->setMaximumSize(QSize(16777215, 26));
        label_57->setFont(font);

        gridLayout->addWidget(label_57, 0, 0, 1, 1);

        dx = new QLineEdit(groupBox_13);
        dx->setObjectName(QStringLiteral("dx"));
        dx->setMaximumSize(QSize(999, 26));
        dx->setFont(font);

        gridLayout->addWidget(dx, 0, 1, 1, 1);

        label_58 = new QLabel(groupBox_13);
        label_58->setObjectName(QStringLiteral("label_58"));
        label_58->setMaximumSize(QSize(16777215, 26));
        label_58->setFont(font);

        gridLayout->addWidget(label_58, 0, 2, 1, 1);

        dy = new QLineEdit(groupBox_13);
        dy->setObjectName(QStringLiteral("dy"));
        dy->setMaximumSize(QSize(999, 26));
        dy->setFont(font);

        gridLayout->addWidget(dy, 0, 3, 1, 1);

        label_59 = new QLabel(groupBox_13);
        label_59->setObjectName(QStringLiteral("label_59"));
        label_59->setMaximumSize(QSize(16777215, 26));
        label_59->setFont(font);

        gridLayout->addWidget(label_59, 0, 4, 1, 1);

        dz = new QLineEdit(groupBox_13);
        dz->setObjectName(QStringLiteral("dz"));
        dz->setMaximumSize(QSize(999, 26));
        dz->setFont(font);

        gridLayout->addWidget(dz, 0, 5, 1, 1);

        label_55 = new QLabel(groupBox_13);
        label_55->setObjectName(QStringLiteral("label_55"));
        label_55->setMaximumSize(QSize(16777215, 26));
        label_55->setFont(font);

        gridLayout->addWidget(label_55, 1, 2, 1, 1);

        dry = new QLineEdit(groupBox_13);
        dry->setObjectName(QStringLiteral("dry"));
        dry->setMaximumSize(QSize(999, 26));
        dry->setFont(font);

        gridLayout->addWidget(dry, 1, 3, 1, 1);

        label_56 = new QLabel(groupBox_13);
        label_56->setObjectName(QStringLiteral("label_56"));
        label_56->setMaximumSize(QSize(16777215, 26));
        label_56->setFont(font);

        gridLayout->addWidget(label_56, 1, 4, 1, 1);

        drz = new QLineEdit(groupBox_13);
        drz->setObjectName(QStringLiteral("drz"));
        drz->setMaximumSize(QSize(999, 26));
        drz->setFont(font);

        gridLayout->addWidget(drz, 1, 5, 1, 1);

        RxyzMove = new QPushButton(groupBox_13);
        RxyzMove->setObjectName(QStringLiteral("RxyzMove"));
        RxyzMove->setMinimumSize(QSize(60, 0));
        RxyzMove->setMaximumSize(QSize(999, 26));
        RxyzMove->setFont(font);

        gridLayout->addWidget(RxyzMove, 2, 4, 1, 2);

        drx = new QLineEdit(groupBox_13);
        drx->setObjectName(QStringLiteral("drx"));
        drx->setMaximumSize(QSize(999, 26));
        drx->setFont(font);

        gridLayout->addWidget(drx, 1, 1, 1, 1);

        label_54 = new QLabel(groupBox_13);
        label_54->setObjectName(QStringLiteral("label_54"));
        label_54->setMaximumSize(QSize(16777215, 26));
        label_54->setFont(font);

        gridLayout->addWidget(label_54, 1, 0, 1, 1);


        verticalLayout->addWidget(groupBox_13);

        groupBox_14 = new QGroupBox(RobotUI);
        groupBox_14->setObjectName(QStringLiteral("groupBox_14"));
        groupBox_14->setMaximumSize(QSize(16777215, 120));
        groupBox_14->setFont(font);
        gridLayout_4 = new QGridLayout(groupBox_14);
        gridLayout_4->setSpacing(0);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_path = new QLabel(groupBox_14);
        label_path->setObjectName(QStringLiteral("label_path"));
        label_path->setMaximumSize(QSize(16777215, 26));
        label_path->setFont(font);

        gridLayout_4->addWidget(label_path, 0, 0, 1, 1);

        btnOpenFile = new QPushButton(groupBox_14);
        btnOpenFile->setObjectName(QStringLiteral("btnOpenFile"));
        btnOpenFile->setMaximumSize(QSize(16777215, 26));

        gridLayout_4->addWidget(btnOpenFile, 1, 0, 1, 1);

        btnMove = new QPushButton(groupBox_14);
        btnMove->setObjectName(QStringLiteral("btnMove"));
        btnMove->setMinimumSize(QSize(60, 0));
        btnMove->setMaximumSize(QSize(999, 26));
        btnMove->setFont(font);

        gridLayout_4->addWidget(btnMove, 1, 1, 1, 1);


        verticalLayout->addWidget(groupBox_14);

        widget = new QWidget(RobotUI);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setMaximumSize(QSize(16777215, 30));
        widget->setFont(font);
        horizontalLayout_2 = new QHBoxLayout(widget);
        horizontalLayout_2->setSpacing(3);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        pushButton = new QPushButton(widget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setMaximumSize(QSize(16777215, 26));
        pushButton->setFont(font);

        horizontalLayout_2->addWidget(pushButton);

        pushButton_2 = new QPushButton(widget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setMaximumSize(QSize(16777215, 26));
        pushButton_2->setFont(font);

        horizontalLayout_2->addWidget(pushButton_2);

        btndiscon = new QPushButton(widget);
        btndiscon->setObjectName(QStringLiteral("btndiscon"));
        btndiscon->setMaximumSize(QSize(16777215, 26));
        btndiscon->setFont(font);

        horizontalLayout_2->addWidget(btndiscon);


        verticalLayout->addWidget(widget);


        retranslateUi(RobotUI);

        QMetaObject::connectSlotsByName(RobotUI);
    } // setupUi

    void retranslateUi(QWidget *RobotUI)
    {
        RobotUI->setWindowTitle(QApplication::translate("RobotUI", "Form", Q_NULLPTR));
        groupBox_11->setTitle(QApplication::translate("RobotUI", "\345\235\220\346\240\207\347\263\273\350\277\220\345\212\250", Q_NULLPTR));
        label_8->setText(QApplication::translate("RobotUI", "X \357\274\232", Q_NULLPTR));
        dPosX->setText(QString());
        label_9->setText(QApplication::translate("RobotUI", "Y \357\274\232", Q_NULLPTR));
        currPosY->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_10->setText(QApplication::translate("RobotUI", "Z \357\274\232", Q_NULLPTR));
        currPosZ->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_11->setText(QApplication::translate("RobotUI", "A\357\274\232", Q_NULLPTR));
        currPosA->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_12->setText(QApplication::translate("RobotUI", "B\357\274\232", Q_NULLPTR));
        currPosB->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_13->setText(QApplication::translate("RobotUI", "C\357\274\232", Q_NULLPTR));
        currPosC->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        stopMove->setText(QApplication::translate("RobotUI", "\345\201\234\346\255\242", Q_NULLPTR));
        currentTCP->clear();
        currentTCP->insertItems(0, QStringList()
         << QApplication::translate("RobotUI", "\347\204\212\346\236\252", Q_NULLPTR)
         << QApplication::translate("RobotUI", "\345\257\273\344\275\215\344\270\255\345\277\203\347\202\271", Q_NULLPTR)
         << QApplication::translate("RobotUI", "\346\263\225\345\205\260", Q_NULLPTR)
         << QApplication::translate("RobotUI", "\350\267\237\350\270\252\344\270\255\345\277\203\347\202\271", Q_NULLPTR)
        );
        clear->setText(QApplication::translate("RobotUI", "\346\270\205\347\251\272", Q_NULLPTR));
        label_14->setText(QApplication::translate("RobotUI", "\351\200\237\345\272\246\357\274\232", Q_NULLPTR));
        abMove->setText(QApplication::translate("RobotUI", "\347\273\235\345\257\271\350\277\220\345\212\250", Q_NULLPTR));
        suEdit->setText(QApplication::translate("RobotUI", "100", Q_NULLPTR));
        currPosX->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        relativeMove->setText(QApplication::translate("RobotUI", "\347\233\270\345\257\271\350\277\220\345\212\250", Q_NULLPTR));
        getCurrPos->setText(QApplication::translate("RobotUI", "\344\275\215\347\275\256", Q_NULLPTR));
        groupBox_12->setTitle(QApplication::translate("RobotUI", "\346\234\272\345\231\250\350\275\264\346\226\271\345\274\217\350\277\220\345\212\250", Q_NULLPTR));
        label_53->setText(QApplication::translate("RobotUI", "A1", Q_NULLPTR));
        currA1->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        cur_axis->setText(QApplication::translate("RobotUI", "\345\275\223\345\211\215\345\220\204\350\275\264", Q_NULLPTR));
        label_48->setText(QApplication::translate("RobotUI", "A2", Q_NULLPTR));
        currA2->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_52->setText(QApplication::translate("RobotUI", "A3", Q_NULLPTR));
        currA3->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        relativeMoveAxle->setText(QApplication::translate("RobotUI", "\347\233\270\345\257\271\350\277\220\345\212\250", Q_NULLPTR));
        label_50->setText(QApplication::translate("RobotUI", "A4", Q_NULLPTR));
        currA4->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        label_51->setText(QApplication::translate("RobotUI", "A5", Q_NULLPTR));
        currA5->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        absoluteMoveAxle->setText(QApplication::translate("RobotUI", "\347\273\235\345\257\271\350\277\220\345\212\250", Q_NULLPTR));
        label_49->setText(QApplication::translate("RobotUI", "A6", Q_NULLPTR));
        currA6->setText(QApplication::translate("RobotUI", "0", Q_NULLPTR));
        groupBox_13->setTitle(QApplication::translate("RobotUI", "\347\204\212\346\236\252", Q_NULLPTR));
        label_57->setText(QApplication::translate("RobotUI", "X", Q_NULLPTR));
        label_58->setText(QApplication::translate("RobotUI", "Y", Q_NULLPTR));
        label_59->setText(QApplication::translate("RobotUI", "Z", Q_NULLPTR));
        label_55->setText(QApplication::translate("RobotUI", "Ry", Q_NULLPTR));
        label_56->setText(QApplication::translate("RobotUI", "Rz", Q_NULLPTR));
        RxyzMove->setText(QApplication::translate("RobotUI", "\350\277\220\345\212\250", Q_NULLPTR));
        label_54->setText(QApplication::translate("RobotUI", "Rx", Q_NULLPTR));
        groupBox_14->setTitle(QApplication::translate("RobotUI", "\345\256\232\350\275\250\350\277\271\350\277\220\345\212\250", Q_NULLPTR));
        label_path->setText(QApplication::translate("RobotUI", "\350\275\250\350\277\271\346\226\207\344\273\266", Q_NULLPTR));
        btnOpenFile->setText(QApplication::translate("RobotUI", "\346\211\223\345\274\200\346\226\207\344\273\266", Q_NULLPTR));
        btnMove->setText(QApplication::translate("RobotUI", "\350\277\220\345\212\250", Q_NULLPTR));
        pushButton->setText(QApplication::translate("RobotUI", "\350\216\267\345\217\226\345\274\202\345\270\270", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("RobotUI", "\351\207\215\346\226\260\350\277\236\346\216\245", Q_NULLPTR));
        btndiscon->setText(QApplication::translate("RobotUI", "\346\226\255\345\274\200\350\277\236\346\216\245", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RobotUI: public Ui_RobotUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTUI_H
