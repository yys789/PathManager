/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *edtAglS;
    QWidget *layoutWidget_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLineEdit *edtAglE;
    QPushButton *pbn_set;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *edtAglO;
    QWidget *layoutWidget_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLineEdit *edtAglT;
    QWidget *layoutWidget_4;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLineEdit *edtAglA;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QStringLiteral("Dialog"));
        Dialog->resize(484, 357);
        layoutWidget = new QWidget(Dialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(30, 140, 225, 22));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        edtAglS = new QLineEdit(layoutWidget);
        edtAglS->setObjectName(QStringLiteral("edtAglS"));

        horizontalLayout_2->addWidget(edtAglS);

        layoutWidget_2 = new QWidget(Dialog);
        layoutWidget_2->setObjectName(QStringLiteral("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(30, 180, 225, 22));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget_2);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget_2);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        edtAglE = new QLineEdit(layoutWidget_2);
        edtAglE->setObjectName(QStringLiteral("edtAglE"));

        horizontalLayout_3->addWidget(edtAglE);

        pbn_set = new QPushButton(Dialog);
        pbn_set->setObjectName(QStringLiteral("pbn_set"));
        pbn_set->setGeometry(QRect(30, 220, 75, 23));
        layoutWidget1 = new QWidget(Dialog);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(30, 20, 225, 22));
        horizontalLayout = new QHBoxLayout(layoutWidget1);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget1);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        edtAglO = new QLineEdit(layoutWidget1);
        edtAglO->setObjectName(QStringLiteral("edtAglO"));

        horizontalLayout->addWidget(edtAglO);

        layoutWidget_3 = new QWidget(Dialog);
        layoutWidget_3->setObjectName(QStringLiteral("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(30, 100, 225, 22));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget_3);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget_3);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_4->addWidget(label_4);

        edtAglT = new QLineEdit(layoutWidget_3);
        edtAglT->setObjectName(QStringLiteral("edtAglT"));

        horizontalLayout_4->addWidget(edtAglT);

        layoutWidget_4 = new QWidget(Dialog);
        layoutWidget_4->setObjectName(QStringLiteral("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(30, 60, 225, 22));
        horizontalLayout_5 = new QHBoxLayout(layoutWidget_4);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(layoutWidget_4);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_5->addWidget(label_5);

        edtAglA = new QLineEdit(layoutWidget_4);
        edtAglA->setObjectName(QStringLiteral("edtAglA"));

        horizontalLayout_5->addWidget(edtAglA);

        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget_2->raise();
        pbn_set->raise();
        layoutWidget_3->raise();
        layoutWidget_4->raise();

        retranslateUi(Dialog);

        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0));
        label_2->setText(QApplication::translate("Dialog", "\351\246\226\347\253\257\350\247\222\345\272\246\350\256\276\347\275\256\357\274\232", 0));
        label_3->setText(QApplication::translate("Dialog", "\346\234\253\347\253\257\350\247\222\345\272\246\350\256\276\347\275\256\357\274\232", 0));
        pbn_set->setText(QApplication::translate("Dialog", "\350\256\276\347\275\256", 0));
        label->setText(QApplication::translate("Dialog", "\347\204\212\346\216\245\350\247\222\345\272\246\350\256\276\347\275\256(O)\357\274\232", 0));
        label_4->setText(QApplication::translate("Dialog", "\347\204\212\346\216\245\350\247\222\345\272\246\350\256\276\347\275\256(T)\357\274\232", 0));
        label_5->setText(QApplication::translate("Dialog", "\347\204\212\346\216\245\350\247\222\345\272\246\350\256\276\347\275\256(A)\357\274\232", 0));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
