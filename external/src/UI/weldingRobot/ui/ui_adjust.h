/********************************************************************************
** Form generated from reading UI file 'adjust.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ADJUST_H
#define UI_ADJUST_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AdjustDialog
{
public:
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QPushButton *adjustOK;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *offsetX;
    QLabel *offsetY;
    QLabel *offsetZ;
    QLabel *offsetA;
    QLabel *offsetB;
    QLabel *offsetC;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *addY;
    QComboBox *comboY;
    QPushButton *minusY;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *addZ;
    QComboBox *comboZ;
    QPushButton *minusZ;
    QWidget *horizontalLayoutWidget_4;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *addA;
    QComboBox *comboA;
    QPushButton *minusA;
    QWidget *horizontalLayoutWidget_5;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *addB;
    QComboBox *comboB;
    QPushButton *minusB;
    QWidget *horizontalLayoutWidget_6;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *addC;
    QComboBox *comboC;
    QPushButton *minusC;
    QWidget *horizontalLayoutWidget_7;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *addX;
    QComboBox *comboX;
    QPushButton *minusX;

    void setupUi(QDialog *AdjustDialog)
    {
        if (AdjustDialog->objectName().isEmpty())
            AdjustDialog->setObjectName(QStringLiteral("AdjustDialog"));
        AdjustDialog->resize(701, 595);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AdjustDialog->sizePolicy().hasHeightForWidth());
        AdjustDialog->setSizePolicy(sizePolicy);
        label = new QLabel(AdjustDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(60, 30, 31, 61));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(25);
        label->setFont(font);
        label_2 = new QLabel(AdjustDialog);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(60, 110, 31, 61));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setFont(font);
        label_3 = new QLabel(AdjustDialog);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(60, 190, 31, 61));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setFont(font);
        adjustOK = new QPushButton(AdjustDialog);
        adjustOK->setObjectName(QStringLiteral("adjustOK"));
        adjustOK->setGeometry(QRect(500, 510, 131, 71));
        sizePolicy.setHeightForWidth(adjustOK->sizePolicy().hasHeightForWidth());
        adjustOK->setSizePolicy(sizePolicy);
        adjustOK->setFont(font);
        label_4 = new QLabel(AdjustDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(60, 270, 31, 61));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);
        label_4->setFont(font);
        label_5 = new QLabel(AdjustDialog);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(60, 350, 31, 61));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setFont(font);
        label_6 = new QLabel(AdjustDialog);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(60, 430, 31, 61));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setFont(font);
        offsetX = new QLabel(AdjustDialog);
        offsetX->setObjectName(QStringLiteral("offsetX"));
        offsetX->setGeometry(QRect(111, 30, 91, 61));
        offsetX->setFont(font);
        offsetY = new QLabel(AdjustDialog);
        offsetY->setObjectName(QStringLiteral("offsetY"));
        offsetY->setGeometry(QRect(110, 110, 91, 61));
        offsetY->setFont(font);
        offsetZ = new QLabel(AdjustDialog);
        offsetZ->setObjectName(QStringLiteral("offsetZ"));
        offsetZ->setGeometry(QRect(110, 190, 91, 61));
        offsetZ->setFont(font);
        offsetA = new QLabel(AdjustDialog);
        offsetA->setObjectName(QStringLiteral("offsetA"));
        offsetA->setGeometry(QRect(110, 270, 91, 61));
        offsetA->setFont(font);
        offsetB = new QLabel(AdjustDialog);
        offsetB->setObjectName(QStringLiteral("offsetB"));
        offsetB->setGeometry(QRect(110, 350, 91, 61));
        offsetB->setFont(font);
        offsetC = new QLabel(AdjustDialog);
        offsetC->setObjectName(QStringLiteral("offsetC"));
        offsetC->setGeometry(QRect(110, 430, 91, 61));
        offsetC->setFont(font);
        horizontalLayoutWidget_2 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(240, 110, 391, 61));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        addY = new QPushButton(horizontalLayoutWidget_2);
        addY->setObjectName(QStringLiteral("addY"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(addY->sizePolicy().hasHeightForWidth());
        addY->setSizePolicy(sizePolicy1);
        QFont font1;
        font1.setPointSize(20);
        addY->setFont(font1);

        horizontalLayout_2->addWidget(addY);

        comboY = new QComboBox(horizontalLayoutWidget_2);
        comboY->setObjectName(QStringLiteral("comboY"));
        sizePolicy1.setHeightForWidth(comboY->sizePolicy().hasHeightForWidth());
        comboY->setSizePolicy(sizePolicy1);
        comboY->setFont(font1);

        horizontalLayout_2->addWidget(comboY);

        minusY = new QPushButton(horizontalLayoutWidget_2);
        minusY->setObjectName(QStringLiteral("minusY"));
        sizePolicy1.setHeightForWidth(minusY->sizePolicy().hasHeightForWidth());
        minusY->setSizePolicy(sizePolicy1);
        minusY->setFont(font1);

        horizontalLayout_2->addWidget(minusY);

        horizontalLayoutWidget_3 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_3->setObjectName(QStringLiteral("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(240, 190, 391, 61));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        addZ = new QPushButton(horizontalLayoutWidget_3);
        addZ->setObjectName(QStringLiteral("addZ"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(addZ->sizePolicy().hasHeightForWidth());
        addZ->setSizePolicy(sizePolicy2);
        addZ->setFont(font1);

        horizontalLayout_3->addWidget(addZ);

        comboZ = new QComboBox(horizontalLayoutWidget_3);
        comboZ->setObjectName(QStringLiteral("comboZ"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(comboZ->sizePolicy().hasHeightForWidth());
        comboZ->setSizePolicy(sizePolicy3);
        comboZ->setFont(font1);

        horizontalLayout_3->addWidget(comboZ);

        minusZ = new QPushButton(horizontalLayoutWidget_3);
        minusZ->setObjectName(QStringLiteral("minusZ"));
        sizePolicy2.setHeightForWidth(minusZ->sizePolicy().hasHeightForWidth());
        minusZ->setSizePolicy(sizePolicy2);
        minusZ->setFont(font1);

        horizontalLayout_3->addWidget(minusZ);

        horizontalLayoutWidget_4 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_4->setObjectName(QStringLiteral("horizontalLayoutWidget_4"));
        horizontalLayoutWidget_4->setGeometry(QRect(240, 270, 391, 61));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget_4);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        addA = new QPushButton(horizontalLayoutWidget_4);
        addA->setObjectName(QStringLiteral("addA"));
        sizePolicy2.setHeightForWidth(addA->sizePolicy().hasHeightForWidth());
        addA->setSizePolicy(sizePolicy2);
        addA->setFont(font1);

        horizontalLayout_4->addWidget(addA);

        comboA = new QComboBox(horizontalLayoutWidget_4);
        comboA->setObjectName(QStringLiteral("comboA"));
        sizePolicy3.setHeightForWidth(comboA->sizePolicy().hasHeightForWidth());
        comboA->setSizePolicy(sizePolicy3);
        comboA->setFont(font1);

        horizontalLayout_4->addWidget(comboA);

        minusA = new QPushButton(horizontalLayoutWidget_4);
        minusA->setObjectName(QStringLiteral("minusA"));
        sizePolicy2.setHeightForWidth(minusA->sizePolicy().hasHeightForWidth());
        minusA->setSizePolicy(sizePolicy2);
        minusA->setFont(font1);

        horizontalLayout_4->addWidget(minusA);

        horizontalLayoutWidget_5 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_5->setObjectName(QStringLiteral("horizontalLayoutWidget_5"));
        horizontalLayoutWidget_5->setGeometry(QRect(240, 350, 391, 61));
        horizontalLayout_5 = new QHBoxLayout(horizontalLayoutWidget_5);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        addB = new QPushButton(horizontalLayoutWidget_5);
        addB->setObjectName(QStringLiteral("addB"));
        sizePolicy2.setHeightForWidth(addB->sizePolicy().hasHeightForWidth());
        addB->setSizePolicy(sizePolicy2);
        addB->setFont(font1);

        horizontalLayout_5->addWidget(addB);

        comboB = new QComboBox(horizontalLayoutWidget_5);
        comboB->setObjectName(QStringLiteral("comboB"));
        sizePolicy3.setHeightForWidth(comboB->sizePolicy().hasHeightForWidth());
        comboB->setSizePolicy(sizePolicy3);
        comboB->setFont(font1);

        horizontalLayout_5->addWidget(comboB);

        minusB = new QPushButton(horizontalLayoutWidget_5);
        minusB->setObjectName(QStringLiteral("minusB"));
        sizePolicy2.setHeightForWidth(minusB->sizePolicy().hasHeightForWidth());
        minusB->setSizePolicy(sizePolicy2);
        minusB->setFont(font1);

        horizontalLayout_5->addWidget(minusB);

        horizontalLayoutWidget_6 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_6->setObjectName(QStringLiteral("horizontalLayoutWidget_6"));
        horizontalLayoutWidget_6->setGeometry(QRect(240, 430, 391, 61));
        horizontalLayout_6 = new QHBoxLayout(horizontalLayoutWidget_6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        addC = new QPushButton(horizontalLayoutWidget_6);
        addC->setObjectName(QStringLiteral("addC"));
        sizePolicy2.setHeightForWidth(addC->sizePolicy().hasHeightForWidth());
        addC->setSizePolicy(sizePolicy2);
        addC->setFont(font1);

        horizontalLayout_6->addWidget(addC);

        comboC = new QComboBox(horizontalLayoutWidget_6);
        comboC->setObjectName(QStringLiteral("comboC"));
        sizePolicy3.setHeightForWidth(comboC->sizePolicy().hasHeightForWidth());
        comboC->setSizePolicy(sizePolicy3);
        comboC->setFont(font1);

        horizontalLayout_6->addWidget(comboC);

        minusC = new QPushButton(horizontalLayoutWidget_6);
        minusC->setObjectName(QStringLiteral("minusC"));
        sizePolicy2.setHeightForWidth(minusC->sizePolicy().hasHeightForWidth());
        minusC->setSizePolicy(sizePolicy2);
        minusC->setFont(font1);

        horizontalLayout_6->addWidget(minusC);

        horizontalLayoutWidget_7 = new QWidget(AdjustDialog);
        horizontalLayoutWidget_7->setObjectName(QStringLiteral("horizontalLayoutWidget_7"));
        horizontalLayoutWidget_7->setGeometry(QRect(240, 30, 391, 61));
        horizontalLayout_7 = new QHBoxLayout(horizontalLayoutWidget_7);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        addX = new QPushButton(horizontalLayoutWidget_7);
        addX->setObjectName(QStringLiteral("addX"));
        sizePolicy1.setHeightForWidth(addX->sizePolicy().hasHeightForWidth());
        addX->setSizePolicy(sizePolicy1);
        addX->setFont(font1);

        horizontalLayout_7->addWidget(addX);

        comboX = new QComboBox(horizontalLayoutWidget_7);
        comboX->setObjectName(QStringLiteral("comboX"));
        sizePolicy1.setHeightForWidth(comboX->sizePolicy().hasHeightForWidth());
        comboX->setSizePolicy(sizePolicy1);
        comboX->setFont(font1);

        horizontalLayout_7->addWidget(comboX);

        minusX = new QPushButton(horizontalLayoutWidget_7);
        minusX->setObjectName(QStringLiteral("minusX"));
        sizePolicy1.setHeightForWidth(minusX->sizePolicy().hasHeightForWidth());
        minusX->setSizePolicy(sizePolicy1);
        minusX->setFont(font1);

        horizontalLayout_7->addWidget(minusX);


        retranslateUi(AdjustDialog);

        QMetaObject::connectSlotsByName(AdjustDialog);
    } // setupUi

    void retranslateUi(QDialog *AdjustDialog)
    {
        AdjustDialog->setWindowTitle(QApplication::translate("AdjustDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("AdjustDialog", "X:", Q_NULLPTR));
        label_2->setText(QApplication::translate("AdjustDialog", "Y:", Q_NULLPTR));
        label_3->setText(QApplication::translate("AdjustDialog", "Z:", Q_NULLPTR));
        adjustOK->setText(QApplication::translate("AdjustDialog", "\344\270\213\344\270\200\346\255\245", Q_NULLPTR));
        label_4->setText(QApplication::translate("AdjustDialog", "A:", Q_NULLPTR));
        label_5->setText(QApplication::translate("AdjustDialog", "B:", Q_NULLPTR));
        label_6->setText(QApplication::translate("AdjustDialog", "C:", Q_NULLPTR));
        offsetX->setText(QString());
        offsetY->setText(QString());
        offsetZ->setText(QString());
        offsetA->setText(QString());
        offsetB->setText(QString());
        offsetC->setText(QString());
        addY->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboY->clear();
        comboY->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusY->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
        addZ->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboZ->clear();
        comboZ->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusZ->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
        addA->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboA->clear();
        comboA->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusA->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
        addB->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboB->clear();
        comboB->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusB->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
        addC->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboC->clear();
        comboC->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusC->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
        addX->setText(QApplication::translate("AdjustDialog", "+", Q_NULLPTR));
        comboX->clear();
        comboX->insertItems(0, QStringList()
         << QApplication::translate("AdjustDialog", "1", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "0.5", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "2", Q_NULLPTR)
         << QApplication::translate("AdjustDialog", "5", Q_NULLPTR)
        );
        minusX->setText(QApplication::translate("AdjustDialog", "-", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AdjustDialog: public Ui_AdjustDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ADJUST_H
