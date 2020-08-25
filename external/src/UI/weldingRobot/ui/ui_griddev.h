/********************************************************************************
** Form generated from reading UI file 'griddev.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRIDDEV_H
#define UI_GRIDDEV_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GridForm
{
public:
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QWidget *tab1;
    QGridLayout *gridLayout_3;
    QWidget *widget;
    QGridLayout *gridLayout_2;
    QLineEdit *editdis5;
    QLabel *label_17;
    QLineEdit *editface2;
    QLineEdit *editface4;
    QLineEdit *editdis1;
    QLabel *label_5;
    QLabel *label_7;
    QLineEdit *editdis4;
    QLabel *label_4;
    QLabel *label_20;
    QLabel *label_19;
    QLineEdit *editdis3;
    QLineEdit *editc2f;
    QLineEdit *editface1;
    QLineEdit *editt2c;
    QLineEdit *editface0;
    QLineEdit *editface;
    QLabel *label_6;
    QLineEdit *editdis2;
    QLineEdit *editface3;
    QLineEdit *editdis0;
    QLabel *label_3;
    QTableWidget *twtgrid;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QLabel *label_12;
    QLabel *label_16;
    QLabel *label_10;
    QLabel *label_8;
    QLabel *label_9;
    QPushButton *btnsave;
    QComboBox *comboBox;
    QLineEdit *editline;
    QLineEdit *editcicle;
    QLineEdit *editRoi;
    QLineEdit *editcenter;
    QLabel *label;
    QLineEdit *edithq2fl;
    QLineEdit *editctpos;
    QLabel *label_2;
    QPushButton *autoWork;
    QTableWidget *twtball;
    QWidget *widget_2;
    QGridLayout *gridLayout_5;
    QPushButton *btnnihe_2;
    QPushButton *btnBasePos;
    QPushButton *btn4p2abc;
    QPushButton *initGrid;
    QPushButton *checkGrid;
    QPushButton *btnface;
    QPushButton *btn4p2abc_2;
    QPushButton *btnGrid;
    QPushButton *btn4p;
    QPushButton *btnxz;
    QPushButton *btnnihe;
    QWidget *tab_2;
    QLabel *label_11;
    QLineEdit *temp2;
    QLabel *label_13;
    QLabel *label_14;
    QPushButton *mathBtn;
    QLabel *label_15;
    QLineEdit *temp3;
    QLineEdit *temp4;
    QLineEdit *temp1;
    QLineEdit *temp5;
    QPushButton *pushButton_8;
    QPushButton *testAir;
    QPushButton *btnLocation;
    QCheckBox *cbx_zyx4;

    void setupUi(QWidget *GridForm)
    {
        if (GridForm->objectName().isEmpty())
            GridForm->setObjectName(QStringLiteral("GridForm"));
        GridForm->resize(1055, 595);
        horizontalLayout = new QHBoxLayout(GridForm);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        tabWidget = new QTabWidget(GridForm);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        QFont font;
        font.setPointSize(11);
        tabWidget->setFont(font);
        tab1 = new QWidget();
        tab1->setObjectName(QStringLiteral("tab1"));
        gridLayout_3 = new QGridLayout(tab1);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(tab1);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setMaximumSize(QSize(650, 400));
        gridLayout_2 = new QGridLayout(widget);
        gridLayout_2->setSpacing(2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        editdis5 = new QLineEdit(widget);
        editdis5->setObjectName(QStringLiteral("editdis5"));
        editdis5->setMaximumSize(QSize(100, 16777215));
        editdis5->setFont(font);
        editdis5->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis5, 5, 2, 1, 1);

        label_17 = new QLabel(widget);
        label_17->setObjectName(QStringLiteral("label_17"));
        QFont font1;
        font1.setPointSize(11);
        font1.setBold(true);
        font1.setWeight(75);
        label_17->setFont(font1);
        label_17->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_17, 7, 0, 1, 1);

        editface2 = new QLineEdit(widget);
        editface2->setObjectName(QStringLiteral("editface2"));
        editface2->setMinimumSize(QSize(380, 0));
        editface2->setMaximumSize(QSize(380, 16777215));
        editface2->setFont(font);
        editface2->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface2, 2, 1, 1, 1);

        editface4 = new QLineEdit(widget);
        editface4->setObjectName(QStringLiteral("editface4"));
        editface4->setMinimumSize(QSize(380, 0));
        editface4->setMaximumSize(QSize(380, 16777215));
        editface4->setFont(font);
        editface4->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface4, 4, 1, 1, 1);

        editdis1 = new QLineEdit(widget);
        editdis1->setObjectName(QStringLiteral("editdis1"));
        editdis1->setMaximumSize(QSize(100, 16777215));
        editdis1->setFont(font);
        editdis1->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis1, 1, 2, 1, 1);

        label_5 = new QLabel(widget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_5, 3, 0, 1, 1);

        label_7 = new QLabel(widget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setFont(font1);
        label_7->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_7, 6, 0, 1, 1);

        editdis4 = new QLineEdit(widget);
        editdis4->setObjectName(QStringLiteral("editdis4"));
        editdis4->setMaximumSize(QSize(100, 16777215));
        editdis4->setFont(font);
        editdis4->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis4, 4, 2, 1, 1);

        label_4 = new QLabel(widget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_4, 2, 0, 1, 1);

        label_20 = new QLabel(widget);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setFont(font);
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_20, 5, 0, 1, 1);

        label_19 = new QLabel(widget);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setFont(font);
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_19, 0, 0, 1, 1);

        editdis3 = new QLineEdit(widget);
        editdis3->setObjectName(QStringLiteral("editdis3"));
        editdis3->setMaximumSize(QSize(100, 16777215));
        editdis3->setFont(font);
        editdis3->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis3, 3, 2, 1, 1);

        editc2f = new QLineEdit(widget);
        editc2f->setObjectName(QStringLiteral("editc2f"));
        editc2f->setMinimumSize(QSize(380, 0));
        editc2f->setMaximumSize(QSize(380, 16777215));
        editc2f->setFont(font);
        editc2f->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editc2f, 6, 1, 1, 1);

        editface1 = new QLineEdit(widget);
        editface1->setObjectName(QStringLiteral("editface1"));
        editface1->setMinimumSize(QSize(380, 0));
        editface1->setMaximumSize(QSize(380, 16777215));
        editface1->setFont(font);
        editface1->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface1, 1, 1, 1, 1);

        editt2c = new QLineEdit(widget);
        editt2c->setObjectName(QStringLiteral("editt2c"));
        editt2c->setMinimumSize(QSize(380, 0));
        editt2c->setMaximumSize(QSize(380, 16777215));
        editt2c->setFont(font);
        editt2c->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editt2c, 7, 1, 1, 1);

        editface0 = new QLineEdit(widget);
        editface0->setObjectName(QStringLiteral("editface0"));
        editface0->setMinimumSize(QSize(380, 0));
        editface0->setMaximumSize(QSize(380, 16777215));
        editface0->setFont(font);
        editface0->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface0, 0, 1, 1, 1);

        editface = new QLineEdit(widget);
        editface->setObjectName(QStringLiteral("editface"));
        editface->setMinimumSize(QSize(380, 0));
        editface->setMaximumSize(QSize(380, 16777215));
        editface->setFont(font);
        editface->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface, 5, 1, 1, 1);

        label_6 = new QLabel(widget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_6, 4, 0, 1, 1);

        editdis2 = new QLineEdit(widget);
        editdis2->setObjectName(QStringLiteral("editdis2"));
        editdis2->setMaximumSize(QSize(100, 16777215));
        editdis2->setFont(font);
        editdis2->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis2, 2, 2, 1, 1);

        editface3 = new QLineEdit(widget);
        editface3->setObjectName(QStringLiteral("editface3"));
        editface3->setMinimumSize(QSize(380, 0));
        editface3->setMaximumSize(QSize(380, 16777215));
        editface3->setFont(font);
        editface3->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editface3, 3, 1, 1, 1);

        editdis0 = new QLineEdit(widget);
        editdis0->setObjectName(QStringLiteral("editdis0"));
        editdis0->setMaximumSize(QSize(100, 16777215));
        editdis0->setFont(font);
        editdis0->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(editdis0, 0, 2, 1, 1);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);


        gridLayout_3->addWidget(widget, 0, 0, 1, 1);

        twtgrid = new QTableWidget(tab1);
        if (twtgrid->columnCount() < 6)
            twtgrid->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        twtgrid->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        twtgrid->setObjectName(QStringLiteral("twtgrid"));
        twtgrid->setMaximumSize(QSize(16777215, 400));
        twtgrid->setFont(font);

        gridLayout_3->addWidget(twtgrid, 0, 1, 1, 1);

        groupBox = new QGroupBox(tab1);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMinimumSize(QSize(0, 150));
        groupBox->setMaximumSize(QSize(650, 120));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setHorizontalSpacing(10);
        gridLayout->setVerticalSpacing(0);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setFont(font);
        label_12->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_12, 0, 0, 1, 1);

        label_16 = new QLabel(groupBox);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setFont(font);
        label_16->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_16, 0, 1, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setFont(font);
        label_10->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_10, 0, 2, 1, 1);

        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setFont(font);
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_8, 0, 3, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setFont(font);
        label_9->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_9, 0, 4, 1, 1);

        btnsave = new QPushButton(groupBox);
        btnsave->setObjectName(QStringLiteral("btnsave"));
        btnsave->setFont(font);

        gridLayout->addWidget(btnsave, 0, 5, 1, 1);

        comboBox = new QComboBox(groupBox);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        gridLayout->addWidget(comboBox, 1, 0, 1, 1);

        editline = new QLineEdit(groupBox);
        editline->setObjectName(QStringLiteral("editline"));
        editline->setFont(font);
        editline->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(editline, 1, 1, 1, 1);

        editcicle = new QLineEdit(groupBox);
        editcicle->setObjectName(QStringLiteral("editcicle"));
        editcicle->setFont(font);
        editcicle->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(editcicle, 1, 2, 1, 1);

        editRoi = new QLineEdit(groupBox);
        editRoi->setObjectName(QStringLiteral("editRoi"));
        editRoi->setFont(font);
        editRoi->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(editRoi, 1, 3, 1, 1);

        editcenter = new QLineEdit(groupBox);
        editcenter->setObjectName(QStringLiteral("editcenter"));
        editcenter->setFont(font);
        editcenter->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(editcenter, 1, 4, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 2, 0, 1, 1);

        edithq2fl = new QLineEdit(groupBox);
        edithq2fl->setObjectName(QStringLiteral("edithq2fl"));
        edithq2fl->setMinimumSize(QSize(380, 0));
        edithq2fl->setMaximumSize(QSize(888, 16777215));
        edithq2fl->setFont(font);
        edithq2fl->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(edithq2fl, 2, 1, 1, 4);

        editctpos = new QLineEdit(groupBox);
        editctpos->setObjectName(QStringLiteral("editctpos"));
        editctpos->setMinimumSize(QSize(380, 0));
        editctpos->setMaximumSize(QSize(888, 16777215));
        editctpos->setFont(font);
        editctpos->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(editctpos, 3, 1, 1, 4);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 3, 0, 1, 1);

        autoWork = new QPushButton(groupBox);
        autoWork->setObjectName(QStringLiteral("autoWork"));
        autoWork->setFont(font);

        gridLayout->addWidget(autoWork, 1, 5, 3, 1);


        gridLayout_3->addWidget(groupBox, 1, 0, 1, 1);

        twtball = new QTableWidget(tab1);
        if (twtball->columnCount() < 4)
            twtball->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        twtball->setHorizontalHeaderItem(0, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        twtball->setHorizontalHeaderItem(1, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        twtball->setHorizontalHeaderItem(2, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        twtball->setHorizontalHeaderItem(3, __qtablewidgetitem9);
        if (twtball->rowCount() < 9)
            twtball->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(0, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(1, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(2, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(3, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(4, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(5, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(6, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(7, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        twtball->setVerticalHeaderItem(8, __qtablewidgetitem18);
        twtball->setObjectName(QStringLiteral("twtball"));
        twtball->setMinimumSize(QSize(0, 310));
        twtball->setMaximumSize(QSize(16777215, 310));
        twtball->setFont(font);

        gridLayout_3->addWidget(twtball, 1, 1, 2, 1);

        widget_2 = new QWidget(tab1);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        widget_2->setMinimumSize(QSize(0, 120));
        widget_2->setMaximumSize(QSize(650, 120));
        gridLayout_5 = new QGridLayout(widget_2);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setHorizontalSpacing(5);
        gridLayout_5->setVerticalSpacing(0);
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        btnnihe_2 = new QPushButton(widget_2);
        btnnihe_2->setObjectName(QStringLiteral("btnnihe_2"));
        btnnihe_2->setEnabled(false);
        btnnihe_2->setMinimumSize(QSize(100, 0));
        btnnihe_2->setMaximumSize(QSize(100, 16777215));
        btnnihe_2->setFont(font);

        gridLayout_5->addWidget(btnnihe_2, 2, 4, 1, 1);

        btnBasePos = new QPushButton(widget_2);
        btnBasePos->setObjectName(QStringLiteral("btnBasePos"));
        btnBasePos->setMinimumSize(QSize(100, 0));
        btnBasePos->setMaximumSize(QSize(100, 16777215));
        btnBasePos->setFont(font);

        gridLayout_5->addWidget(btnBasePos, 0, 0, 1, 1);

        btn4p2abc = new QPushButton(widget_2);
        btn4p2abc->setObjectName(QStringLiteral("btn4p2abc"));
        btn4p2abc->setMinimumSize(QSize(100, 0));
        btn4p2abc->setMaximumSize(QSize(100, 16777215));
        btn4p2abc->setFont(font);

        gridLayout_5->addWidget(btn4p2abc, 0, 1, 1, 1);

        initGrid = new QPushButton(widget_2);
        initGrid->setObjectName(QStringLiteral("initGrid"));
        initGrid->setMinimumSize(QSize(100, 0));
        initGrid->setMaximumSize(QSize(100, 16777215));
        initGrid->setFont(font);

        gridLayout_5->addWidget(initGrid, 0, 2, 1, 1);

        checkGrid = new QPushButton(widget_2);
        checkGrid->setObjectName(QStringLiteral("checkGrid"));
        checkGrid->setMinimumSize(QSize(100, 0));
        checkGrid->setMaximumSize(QSize(100, 16777215));
        checkGrid->setFont(font);

        gridLayout_5->addWidget(checkGrid, 0, 3, 1, 1);

        btnface = new QPushButton(widget_2);
        btnface->setObjectName(QStringLiteral("btnface"));
        btnface->setMinimumSize(QSize(100, 0));
        btnface->setMaximumSize(QSize(100, 16777215));
        btnface->setFont(font);

        gridLayout_5->addWidget(btnface, 1, 0, 1, 1);

        btn4p2abc_2 = new QPushButton(widget_2);
        btn4p2abc_2->setObjectName(QStringLiteral("btn4p2abc_2"));
        btn4p2abc_2->setMinimumSize(QSize(100, 0));
        btn4p2abc_2->setMaximumSize(QSize(100, 16777215));
        btn4p2abc_2->setFont(font);

        gridLayout_5->addWidget(btn4p2abc_2, 1, 1, 1, 1);

        btnGrid = new QPushButton(widget_2);
        btnGrid->setObjectName(QStringLiteral("btnGrid"));
        btnGrid->setMinimumSize(QSize(100, 0));
        btnGrid->setMaximumSize(QSize(100, 16777215));
        btnGrid->setFont(font);

        gridLayout_5->addWidget(btnGrid, 1, 2, 1, 1);

        btn4p = new QPushButton(widget_2);
        btn4p->setObjectName(QStringLiteral("btn4p"));
        btn4p->setMinimumSize(QSize(100, 0));
        btn4p->setMaximumSize(QSize(100, 16777215));
        btn4p->setFont(font);

        gridLayout_5->addWidget(btn4p, 2, 0, 1, 1);

        btnxz = new QPushButton(widget_2);
        btnxz->setObjectName(QStringLiteral("btnxz"));
        btnxz->setMinimumSize(QSize(100, 0));
        btnxz->setMaximumSize(QSize(100, 16777215));
        btnxz->setFont(font);

        gridLayout_5->addWidget(btnxz, 1, 3, 1, 1);

        btnnihe = new QPushButton(widget_2);
        btnnihe->setObjectName(QStringLiteral("btnnihe"));
        btnnihe->setMinimumSize(QSize(100, 0));
        btnnihe->setMaximumSize(QSize(100, 16777215));
        btnnihe->setFont(font);

        gridLayout_5->addWidget(btnnihe, 2, 3, 1, 1);


        gridLayout_3->addWidget(widget_2, 2, 0, 1, 1);

        tabWidget->addTab(tab1, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(6, 50, 80, 29));
        label_11->setAlignment(Qt::AlignCenter);
        temp2 = new QLineEdit(tab_2);
        temp2->setObjectName(QStringLiteral("temp2"));
        temp2->setGeometry(QRect(100, 50, 471, 29));
        label_13 = new QLabel(tab_2);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(16, 130, 61, 29));
        label_13->setAlignment(Qt::AlignCenter);
        label_14 = new QLabel(tab_2);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(10, 90, 71, 29));
        label_14->setAlignment(Qt::AlignCenter);
        mathBtn = new QPushButton(tab_2);
        mathBtn->setObjectName(QStringLiteral("mathBtn"));
        mathBtn->setGeometry(QRect(6, 167, 80, 36));
        label_15 = new QLabel(tab_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(6, 13, 80, 29));
        label_15->setAlignment(Qt::AlignCenter);
        temp3 = new QLineEdit(tab_2);
        temp3->setObjectName(QStringLiteral("temp3"));
        temp3->setGeometry(QRect(100, 90, 471, 29));
        temp4 = new QLineEdit(tab_2);
        temp4->setObjectName(QStringLiteral("temp4"));
        temp4->setGeometry(QRect(100, 130, 471, 29));
        temp1 = new QLineEdit(tab_2);
        temp1->setObjectName(QStringLiteral("temp1"));
        temp1->setGeometry(QRect(100, 13, 471, 29));
        temp5 = new QLineEdit(tab_2);
        temp5->setObjectName(QStringLiteral("temp5"));
        temp5->setGeometry(QRect(100, 170, 471, 29));
        pushButton_8 = new QPushButton(tab_2);
        pushButton_8->setObjectName(QStringLiteral("pushButton_8"));
        pushButton_8->setGeometry(QRect(30, 310, 106, 30));
        testAir = new QPushButton(tab_2);
        testAir->setObjectName(QStringLiteral("testAir"));
        testAir->setGeometry(QRect(20, 270, 106, 30));
        btnLocation = new QPushButton(tab_2);
        btnLocation->setObjectName(QStringLiteral("btnLocation"));
        btnLocation->setGeometry(QRect(30, 350, 101, 31));
        btnLocation->setFont(font);
        cbx_zyx4 = new QCheckBox(tab_2);
        cbx_zyx4->setObjectName(QStringLiteral("cbx_zyx4"));
        cbx_zyx4->setGeometry(QRect(580, 130, 71, 23));
        tabWidget->addTab(tab_2, QString());

        horizontalLayout->addWidget(tabWidget);


        retranslateUi(GridForm);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(GridForm);
    } // setupUi

    void retranslateUi(QWidget *GridForm)
    {
        GridForm->setWindowTitle(QApplication::translate("GridForm", "Form", Q_NULLPTR));
        editdis5->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
        label_17->setText(QApplication::translate("GridForm", "TCP/\346\277\200\345\205\211\351\235\242", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface2->setToolTip(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={16,-16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface2->setText(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={16,-16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface4->setToolTip(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={-16,-16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface4->setText(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={-16,-16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
        editdis1->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
        label_5->setText(QApplication::translate("GridForm", "\345\217\263\344\270\212:Y -16,Z+16,\350\257\257\345\267\256pix", Q_NULLPTR));
        label_7->setText(QApplication::translate("GridForm", "\346\277\200\345\205\211\351\235\242/\346\263\225\345\205\260", Q_NULLPTR));
        editdis4->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
        label_4->setText(QApplication::translate("GridForm", "\345\267\246\344\270\213:Y+16,Z -16,\350\257\257\345\267\256pix", Q_NULLPTR));
        label_20->setText(QApplication::translate("GridForm", "\344\277\256\346\255\243\345\220\216\344\275\215\345\247\277,\350\257\257\345\267\256mm", Q_NULLPTR));
        label_19->setText(QApplication::translate("GridForm", "\345\210\235\345\247\213\344\277\256\346\255\243\344\275\215\345\247\277,\350\257\257\345\267\256mm", Q_NULLPTR));
        editdis3->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editc2f->setToolTip(QApplication::translate("GridForm", "\346\277\200\345\205\211\351\235\242\345\235\220\346\240\207\347\263\273\347\233\270\345\257\271\346\263\225\345\205\260\347\232\204RobotPos", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editc2f->setText(QApplication::translate("GridForm", "RobotPos:camera to flange", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface1->setToolTip(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={16,16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface1->setText(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={16,16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editt2c->setToolTip(QApplication::translate("GridForm", "TCP\347\202\271\347\233\270\345\257\271\346\277\200\345\205\211\351\235\242\345\235\220\346\240\207\347\263\273\347\232\204RobotPos", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editt2c->setText(QApplication::translate("GridForm", "RobotPos:tcp to laser", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface0->setToolTip(QApplication::translate("GridForm", "\345\234\206\345\277\203,\344\270\255\345\277\203\347\202\271,\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210,\346\277\200\345\205\211\351\235\242\345\244\247\350\207\264\345\236\202\347\233\264", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface0->setText(QApplication::translate("GridForm", "\345\234\206\345\277\203,\344\270\255\345\277\203\347\202\271,\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210,\346\277\200\345\205\211\351\235\242\345\244\247\350\207\264\345\236\202\347\233\264", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface->setToolTip(QApplication::translate("GridForm", "\345\234\206\345\277\203,\344\270\255\345\277\203\347\202\271,\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210,\346\277\200\345\205\211\351\235\242\347\273\235\345\257\271\345\236\202\347\233\264", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface->setText(QApplication::translate("GridForm", "\345\234\206\345\277\203,\344\270\255\345\277\203\347\202\271,\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210,\346\277\200\345\205\211\351\235\242\347\273\235\345\257\271\345\236\202\347\233\264", Q_NULLPTR));
        label_6->setText(QApplication::translate("GridForm", "\345\217\263\344\270\213:Y -16,Z -16,\350\257\257\345\267\256pix", Q_NULLPTR));
        editdis2->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editface3->setToolTip(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={-16,16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editface3->setText(QApplication::translate("GridForm", "\344\270\215\345\217\230\345\214\226\345\247\277\346\200\201,\345\234\206\345\277\203,yz={-16,16},\346\277\200\345\205\211\347\272\277\351\207\215\345\220\210", Q_NULLPTR));
        editdis0->setText(QApplication::translate("GridForm", "\350\257\257\345\267\256", Q_NULLPTR));
        label_3->setText(QApplication::translate("GridForm", "\345\267\246\344\270\212:Y+16,Z+16,\350\257\257\345\267\256pix", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem = twtgrid->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("GridForm", "\346\240\274\347\202\271", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = twtgrid->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("GridForm", "\344\275\215\345\247\277", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem2 = twtgrid->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("GridForm", "\345\234\206\345\277\203U", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem3 = twtgrid->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("GridForm", "\345\234\206\345\277\203V", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem4 = twtgrid->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("GridForm", "y\345\201\217\345\267\256", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem5 = twtgrid->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("GridForm", "z\345\201\217\345\267\256", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("GridForm", "\350\256\276\347\275\256", Q_NULLPTR));
        label_12->setText(QApplication::translate("GridForm", "\347\272\240\346\255\243\345\257\271\350\261\241", Q_NULLPTR));
        label_16->setText(QApplication::translate("GridForm", "\347\272\277\346\233\235\345\205\211", Q_NULLPTR));
        label_10->setText(QApplication::translate("GridForm", "\345\234\206\346\233\235\345\205\211", Q_NULLPTR));
        label_8->setText(QApplication::translate("GridForm", "ROI", Q_NULLPTR));
        label_9->setText(QApplication::translate("GridForm", "\344\270\255\345\277\203\347\202\271", Q_NULLPTR));
        btnsave->setText(QApplication::translate("GridForm", "\344\277\235\345\255\230", Q_NULLPTR));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("GridForm", "\345\257\273\344\275\215\345\217\202\346\225\260", Q_NULLPTR)
         << QApplication::translate("GridForm", "\350\267\237\350\270\252\345\217\202\346\225\260", Q_NULLPTR)
        );
        editline->setText(QApplication::translate("GridForm", "400", Q_NULLPTR));
        editcicle->setText(QApplication::translate("GridForm", "40000", Q_NULLPTR));
        editRoi->setText(QApplication::translate("GridForm", "150", Q_NULLPTR));
        editcenter->setText(QApplication::translate("GridForm", "360,270", Q_NULLPTR));
        label->setText(QApplication::translate("GridForm", " \347\204\212\346\236\252/\346\263\225\345\205\260", Q_NULLPTR));
        edithq2fl->setText(QApplication::translate("GridForm", "-66,45,470,147,31.5,-147.4", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        editctpos->setToolTip(QApplication::translate("GridForm", "\344\270\215\346\224\271\345\217\230\346\234\272\345\231\250\344\272\272\345\247\277\346\200\201,\347\204\212\346\236\252\345\244\264\345\257\271\345\207\206\345\234\206\345\277\203", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        editctpos->setText(QApplication::translate("GridForm", "925.28,216.613,818.332,15.962,159.955,-150.12", Q_NULLPTR));
        label_2->setText(QApplication::translate("GridForm", "\345\234\206\345\277\203\344\275\215\347\275\256", Q_NULLPTR));
        autoWork->setText(QApplication::translate("GridForm", "\350\207\252\345\212\250\347\272\240\346\255\243", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem6 = twtball->horizontalHeaderItem(0);
        ___qtablewidgetitem6->setText(QApplication::translate("GridForm", "\345\216\237\345\247\213POS", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem7 = twtball->horizontalHeaderItem(1);
        ___qtablewidgetitem7->setText(QApplication::translate("GridForm", "\344\277\256\346\255\243POS", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem8 = twtball->horizontalHeaderItem(2);
        ___qtablewidgetitem8->setText(QApplication::translate("GridForm", "\344\277\256\346\255\243\350\257\257\345\267\256", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem9 = twtball->horizontalHeaderItem(3);
        ___qtablewidgetitem9->setText(QApplication::translate("GridForm", "\346\213\237\345\220\210\350\257\257\345\267\256", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem10 = twtball->verticalHeaderItem(0);
        ___qtablewidgetitem10->setText(QApplication::translate("GridForm", "1", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem11 = twtball->verticalHeaderItem(1);
        ___qtablewidgetitem11->setText(QApplication::translate("GridForm", "2", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem12 = twtball->verticalHeaderItem(2);
        ___qtablewidgetitem12->setText(QApplication::translate("GridForm", "3", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem13 = twtball->verticalHeaderItem(3);
        ___qtablewidgetitem13->setText(QApplication::translate("GridForm", "4", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem14 = twtball->verticalHeaderItem(4);
        ___qtablewidgetitem14->setText(QApplication::translate("GridForm", "5", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem15 = twtball->verticalHeaderItem(5);
        ___qtablewidgetitem15->setText(QApplication::translate("GridForm", "6", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem16 = twtball->verticalHeaderItem(6);
        ___qtablewidgetitem16->setText(QApplication::translate("GridForm", "7", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem17 = twtball->verticalHeaderItem(7);
        ___qtablewidgetitem17->setText(QApplication::translate("GridForm", "8", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem18 = twtball->verticalHeaderItem(8);
        ___qtablewidgetitem18->setText(QApplication::translate("GridForm", "9", Q_NULLPTR));
        btnnihe_2->setText(QApplication::translate("GridForm", "\345\244\232\347\202\271\346\213\237\345\220\210", Q_NULLPTR));
        btnBasePos->setText(QApplication::translate("GridForm", "1-\345\210\235\345\247\213\344\275\215\347\275\256", Q_NULLPTR));
        btn4p2abc->setText(QApplication::translate("GridForm", "1-\346\213\237\345\220\210ABC", Q_NULLPTR));
        initGrid->setText(QApplication::translate("GridForm", "1-\345\207\206\345\244\207\346\240\274\347\202\271", Q_NULLPTR));
        checkGrid->setText(QApplication::translate("GridForm", "1-\351\252\214\350\257\201\346\240\274\347\202\271", Q_NULLPTR));
        btnface->setText(QApplication::translate("GridForm", "2-\344\270\255\345\277\203\344\277\256\346\255\243", Q_NULLPTR));
        btn4p2abc_2->setText(QApplication::translate("GridForm", "2-\344\277\256\346\255\243", Q_NULLPTR));
        btnGrid->setText(QApplication::translate("GridForm", "2-\346\265\213\351\207\217\345\274\200\345\247\213", Q_NULLPTR));
        btn4p->setText(QApplication::translate("GridForm", "3-\345\233\233\350\247\222\347\237\253\346\255\243", Q_NULLPTR));
        btnxz->setText(QApplication::translate("GridForm", "2-\344\277\256\346\255\243", Q_NULLPTR));
        btnnihe->setText(QApplication::translate("GridForm", "3-\346\213\237\345\220\210XYZ", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab1), QApplication::translate("GridForm", "ABC\344\270\216\346\240\274\347\202\271", Q_NULLPTR));
        label_11->setText(QApplication::translate("GridForm", "X", Q_NULLPTR));
        label_13->setText(QApplication::translate("GridForm", "X", Q_NULLPTR));
        label_14->setText(QApplication::translate("GridForm", "X", Q_NULLPTR));
        mathBtn->setText(QApplication::translate("GridForm", "\347\256\227", Q_NULLPTR));
        label_15->setText(QApplication::translate("GridForm", "P0", Q_NULLPTR));
        pushButton_8->setText(QApplication::translate("GridForm", "autoSpeed", Q_NULLPTR));
        testAir->setText(QApplication::translate("GridForm", "\346\265\213\350\257\225\346\260\224\351\230\200", Q_NULLPTR));
        btnLocation->setText(QApplication::translate("GridForm", "\344\274\240\346\204\237\345\231\250\346\243\200\346\265\213", Q_NULLPTR));
        cbx_zyx4->setText(QApplication::translate("GridForm", "zyx", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("GridForm", "\345\212\251\346\211\213", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class GridForm: public Ui_GridForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRIDDEV_H
