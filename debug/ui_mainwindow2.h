/********************************************************************************
** Form generated from reading UI file 'mainwindow2.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW2_H
#define UI_MAINWINDOW2_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow2
{
public:
    QWidget *centralwidget;
    QListView *listView;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QLineEdit *edtCad;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pushButton_GetWorkIfo;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *cbxMode;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QComboBox *cbxState;
    QSpacerItem *verticalSpacer_8;
    QSpacerItem *verticalSpacer_7;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_9;
    QComboBox *cbxRelate;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_goAll;
    QPushButton *pushButton_goMode;
    QPushButton *pushButton_goState;
    QPushButton *pushButton_goRelation;
    QPushButton *pushButton_goto;
    QCheckBox *checkBox_posture;
    QCheckBox *checkBox_Multiple;
    QCheckBox *checkBox_track;
    QRadioButton *radioButton;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_3;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow2)
    {
        if (MainWindow2->objectName().isEmpty())
            MainWindow2->setObjectName(QStringLiteral("MainWindow2"));
        MainWindow2->resize(782, 567);
        centralwidget = new QWidget(MainWindow2);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        listView = new QListView(centralwidget);
        listView->setObjectName(QStringLiteral("listView"));
        listView->setGeometry(QRect(50, 280, 241, 211));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(50, 40, 241, 211));
        verticalLayout_6 = new QVBoxLayout(layoutWidget);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        edtCad = new QLineEdit(layoutWidget);
        edtCad->setObjectName(QStringLiteral("edtCad"));

        horizontalLayout->addWidget(edtCad);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        pushButton_GetWorkIfo = new QPushButton(layoutWidget);
        pushButton_GetWorkIfo->setObjectName(QStringLiteral("pushButton_GetWorkIfo"));

        horizontalLayout->addWidget(pushButton_GetWorkIfo);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        cbxMode = new QComboBox(layoutWidget);
        cbxMode->setObjectName(QStringLiteral("cbxMode"));

        horizontalLayout_2->addWidget(cbxMode);


        verticalLayout->addLayout(horizontalLayout_2);


        verticalLayout_3->addLayout(verticalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_5->addWidget(label_5);

        cbxState = new QComboBox(layoutWidget);
        cbxState->setObjectName(QStringLiteral("cbxState"));

        horizontalLayout_5->addWidget(cbxState);


        verticalLayout_3->addLayout(horizontalLayout_5);


        verticalLayout_6->addLayout(verticalLayout_3);

        verticalSpacer_8 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_8);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_7);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        horizontalLayout_9->addWidget(label_9);

        cbxRelate = new QComboBox(layoutWidget);
        cbxRelate->setObjectName(QStringLiteral("cbxRelate"));

        horizontalLayout_9->addWidget(cbxRelate);


        verticalLayout_6->addLayout(horizontalLayout_9);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(310, 20, 81, 251));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        pushButton_goAll = new QPushButton(layoutWidget1);
        pushButton_goAll->setObjectName(QStringLiteral("pushButton_goAll"));
        pushButton_goAll->setMinimumSize(QSize(0, 40));

        verticalLayout_2->addWidget(pushButton_goAll);

        pushButton_goMode = new QPushButton(layoutWidget1);
        pushButton_goMode->setObjectName(QStringLiteral("pushButton_goMode"));
        pushButton_goMode->setMinimumSize(QSize(0, 40));

        verticalLayout_2->addWidget(pushButton_goMode);

        pushButton_goState = new QPushButton(layoutWidget1);
        pushButton_goState->setObjectName(QStringLiteral("pushButton_goState"));
        pushButton_goState->setMinimumSize(QSize(0, 40));

        verticalLayout_2->addWidget(pushButton_goState);

        pushButton_goRelation = new QPushButton(layoutWidget1);
        pushButton_goRelation->setObjectName(QStringLiteral("pushButton_goRelation"));
        pushButton_goRelation->setMinimumSize(QSize(0, 40));

        verticalLayout_2->addWidget(pushButton_goRelation);

        pushButton_goto = new QPushButton(centralwidget);
        pushButton_goto->setObjectName(QStringLiteral("pushButton_goto"));
        pushButton_goto->setGeometry(QRect(310, 390, 79, 40));
        pushButton_goto->setMinimumSize(QSize(0, 40));
        checkBox_posture = new QCheckBox(centralwidget);
        checkBox_posture->setObjectName(QStringLiteral("checkBox_posture"));
        checkBox_posture->setGeometry(QRect(310, 300, 71, 16));
        checkBox_Multiple = new QCheckBox(centralwidget);
        checkBox_Multiple->setObjectName(QStringLiteral("checkBox_Multiple"));
        checkBox_Multiple->setGeometry(QRect(310, 330, 71, 16));
        checkBox_track = new QCheckBox(centralwidget);
        checkBox_track->setObjectName(QStringLiteral("checkBox_track"));
        checkBox_track->setGeometry(QRect(310, 360, 71, 16));
        radioButton = new QRadioButton(centralwidget);
        radioButton->setObjectName(QStringLiteral("radioButton"));
        radioButton->setGeometry(QRect(560, 190, 89, 16));
        radioButton_2 = new QRadioButton(centralwidget);
        radioButton_2->setObjectName(QStringLiteral("radioButton_2"));
        radioButton_2->setGeometry(QRect(560, 210, 89, 41));
        radioButton_3 = new QRadioButton(centralwidget);
        radioButton_3->setObjectName(QStringLiteral("radioButton_3"));
        radioButton_3->setGeometry(QRect(560, 240, 89, 41));
        MainWindow2->setCentralWidget(centralwidget);
        layoutWidget->raise();
        layoutWidget->raise();
        listView->raise();
        pushButton_goto->raise();
        checkBox_posture->raise();
        checkBox_Multiple->raise();
        checkBox_track->raise();
        radioButton->raise();
        radioButton_2->raise();
        radioButton_3->raise();
        menubar = new QMenuBar(MainWindow2);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 782, 23));
        MainWindow2->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow2);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow2->setStatusBar(statusbar);

        retranslateUi(MainWindow2);

        QMetaObject::connectSlotsByName(MainWindow2);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow2)
    {
        MainWindow2->setWindowTitle(QApplication::translate("MainWindow2", "MainWindow", 0));
        label->setText(QApplication::translate("MainWindow2", "\345\267\245\344\273\266\345\220\215\357\274\232", 0));
        pushButton_GetWorkIfo->setText(QApplication::translate("MainWindow2", "\350\216\267\345\217\226", 0));
        label_2->setText(QApplication::translate("MainWindow2", "\350\267\237\350\270\252\346\226\271\345\274\217\357\274\232", 0));
        cbxMode->clear();
        cbxMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow2", "\345\257\273\344\275\215", 0)
         << QApplication::translate("MainWindow2", "\350\267\237\350\270\252", 0)
        );
        label_5->setText(QApplication::translate("MainWindow2", "\347\212\266\346\200\201\351\200\211\346\213\251\357\274\232", 0));
        label_9->setText(QApplication::translate("MainWindow2", "\347\204\212\347\274\235\351\200\211\346\213\251\357\274\232", 0));
        pushButton_goAll->setText(QApplication::translate("MainWindow2", "\345\205\250\351\203\250", 0));
        pushButton_goMode->setText(QApplication::translate("MainWindow2", "\346\250\241\345\274\217\347\273\204", 0));
        pushButton_goState->setText(QApplication::translate("MainWindow2", "\347\212\266\346\200\201\347\273\204", 0));
        pushButton_goRelation->setText(QApplication::translate("MainWindow2", "\345\205\263\350\201\224\347\273\204", 0));
        pushButton_goto->setText(QApplication::translate("MainWindow2", "\345\274\200\345\247\213", 0));
        checkBox_posture->setText(QApplication::translate("MainWindow2", "\347\244\272\346\225\231\345\247\277\346\200\201", 0));
        checkBox_Multiple->setText(QApplication::translate("MainWindow2", "\345\257\273\344\275\215\350\275\250\350\277\271", 0));
        checkBox_track->setText(QApplication::translate("MainWindow2", "\350\267\237\350\270\252\350\275\250\350\277\271", 0));
        radioButton->setText(QApplication::translate("MainWindow2", "RadioButton", 0));
        radioButton_2->setText(QApplication::translate("MainWindow2", "RadioButton", 0));
        radioButton_3->setText(QApplication::translate("MainWindow2", "RadioButton", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow2: public Ui_MainWindow2 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW2_H
