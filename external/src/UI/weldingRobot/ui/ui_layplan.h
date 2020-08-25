/********************************************************************************
** Form generated from reading UI file 'layplan.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LAYPLAN_H
#define UI_LAYPLAN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "base/curveCAD.h"

QT_BEGIN_NAMESPACE

class Ui_layPlan
{
public:
    QLineEdit *edt_pix;
    QLabel *label;
    QLabel *label_2;
    curveCAD *res;
    QPushButton *btnPlan;
    QComboBox *comboBox;

    void setupUi(QWidget *layPlan)
    {
        if (layPlan->objectName().isEmpty())
            layPlan->setObjectName(QStringLiteral("layPlan"));
        layPlan->resize(916, 550);
        edt_pix = new QLineEdit(layPlan);
        edt_pix->setObjectName(QStringLiteral("edt_pix"));
        edt_pix->setGeometry(QRect(80, 10, 41, 25));
        QFont font;
        font.setPointSize(11);
        edt_pix->setFont(font);
        label = new QLabel(layPlan);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 10, 71, 31));
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        label_2 = new QLabel(layPlan);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(130, 10, 71, 31));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignCenter);
        res = new curveCAD(layPlan);
        res->setObjectName(QStringLiteral("res"));
        res->setGeometry(QRect(10, 90, 500, 400));
        res->setMinimumSize(QSize(500, 400));
        res->setMaximumSize(QSize(500, 400));
        res->setFont(font);
        res->setStyleSheet(QStringLiteral("border: 2px solid black"));
        btnPlan = new QPushButton(layPlan);
        btnPlan->setObjectName(QStringLiteral("btnPlan"));
        btnPlan->setGeometry(QRect(310, 10, 89, 25));
        btnPlan->setFont(font);
        comboBox = new QComboBox(layPlan);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(200, 10, 86, 25));
        comboBox->setFont(font);

        retranslateUi(layPlan);

        QMetaObject::connectSlotsByName(layPlan);
    } // setupUi

    void retranslateUi(QWidget *layPlan)
    {
        layPlan->setWindowTitle(QApplication::translate("layPlan", "Form", Q_NULLPTR));
        edt_pix->setText(QApplication::translate("layPlan", "2", Q_NULLPTR));
        label->setText(QApplication::translate("layPlan", "mm-pix", Q_NULLPTR));
        label_2->setText(QApplication::translate("layPlan", "base", Q_NULLPTR));
        res->setText(QString());
        btnPlan->setText(QApplication::translate("layPlan", "\350\247\204\345\210\222", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class layPlan: public Ui_layPlan {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAYPLAN_H
