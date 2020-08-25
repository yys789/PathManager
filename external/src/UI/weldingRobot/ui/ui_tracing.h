/********************************************************************************
** Form generated from reading UI file 'tracing.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRACING_H
#define UI_TRACING_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Tracing
{
public:
    QLabel *label;
    QComboBox *cbxSEAM;
    QLabel *label_2;
    QComboBox *cbxCAD;
    QCheckBox *ckxWeld;
    QPushButton *btnPosition3;
    QPushButton *home;
    QLineEdit *dig;
    QPushButton *rotateTrack;
    QPushButton *btnPosition2;
    QPushButton *btnPosition1;
    QPushButton *home_2;
    QPushButton *home_3;
    QPushButton *signal51;
    QPushButton *verifiKeel;
    QPushButton *fitBase;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *testRotate;
    QPushButton *fitHGBase;
    QPushButton *testFit;
    QPushButton *simuFit;
    QPushButton *testRotate_2;

    void setupUi(QWidget *Tracing)
    {
        if (Tracing->objectName().isEmpty())
            Tracing->setObjectName(QStringLiteral("Tracing"));
        Tracing->resize(508, 538);
        label = new QLabel(Tracing);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(150, 10, 71, 21));
        QFont font;
        font.setPointSize(11);
        label->setFont(font);
        cbxSEAM = new QComboBox(Tracing);
        cbxSEAM->setObjectName(QStringLiteral("cbxSEAM"));
        cbxSEAM->setGeometry(QRect(150, 30, 120, 25));
        cbxSEAM->setFont(font);
        label_2 = new QLabel(Tracing);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 10, 51, 21));
        label_2->setFont(font);
        cbxCAD = new QComboBox(Tracing);
        cbxCAD->setObjectName(QStringLiteral("cbxCAD"));
        cbxCAD->setGeometry(QRect(10, 30, 120, 25));
        cbxCAD->setFont(font);
        ckxWeld = new QCheckBox(Tracing);
        ckxWeld->setObjectName(QStringLiteral("ckxWeld"));
        ckxWeld->setGeometry(QRect(20, 120, 61, 23));
        ckxWeld->setFont(font);
        btnPosition3 = new QPushButton(Tracing);
        btnPosition3->setObjectName(QStringLiteral("btnPosition3"));
        btnPosition3->setGeometry(QRect(10, 210, 120, 25));
        btnPosition3->setFont(font);
        home = new QPushButton(Tracing);
        home->setObjectName(QStringLiteral("home"));
        home->setGeometry(QRect(10, 330, 120, 25));
        home->setFont(font);
        dig = new QLineEdit(Tracing);
        dig->setObjectName(QStringLiteral("dig"));
        dig->setGeometry(QRect(10, 300, 120, 25));
        dig->setFont(font);
        rotateTrack = new QPushButton(Tracing);
        rotateTrack->setObjectName(QStringLiteral("rotateTrack"));
        rotateTrack->setGeometry(QRect(10, 490, 120, 25));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setWeight(75);
        rotateTrack->setFont(font1);
        btnPosition2 = new QPushButton(Tracing);
        btnPosition2->setObjectName(QStringLiteral("btnPosition2"));
        btnPosition2->setGeometry(QRect(10, 180, 120, 25));
        btnPosition2->setFont(font);
        btnPosition1 = new QPushButton(Tracing);
        btnPosition1->setObjectName(QStringLiteral("btnPosition1"));
        btnPosition1->setGeometry(QRect(10, 150, 120, 25));
        btnPosition1->setFont(font);
        home_2 = new QPushButton(Tracing);
        home_2->setObjectName(QStringLiteral("home_2"));
        home_2->setGeometry(QRect(10, 460, 120, 25));
        home_2->setFont(font);
        home_3 = new QPushButton(Tracing);
        home_3->setObjectName(QStringLiteral("home_3"));
        home_3->setGeometry(QRect(10, 400, 120, 25));
        home_3->setFont(font);
        signal51 = new QPushButton(Tracing);
        signal51->setObjectName(QStringLiteral("signal51"));
        signal51->setGeometry(QRect(10, 430, 120, 25));
        signal51->setFont(font);
        verifiKeel = new QPushButton(Tracing);
        verifiKeel->setObjectName(QStringLiteral("verifiKeel"));
        verifiKeel->setGeometry(QRect(10, 270, 120, 25));
        verifiKeel->setFont(font);
        fitBase = new QPushButton(Tracing);
        fitBase->setObjectName(QStringLiteral("fitBase"));
        fitBase->setGeometry(QRect(10, 240, 120, 25));
        fitBase->setFont(font);
        pushButton_2 = new QPushButton(Tracing);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(150, 460, 89, 25));
        pushButton_2->setFont(font);
        pushButton_3 = new QPushButton(Tracing);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(150, 490, 89, 25));
        pushButton_3->setFont(font);
        testRotate = new QPushButton(Tracing);
        testRotate->setObjectName(QStringLiteral("testRotate"));
        testRotate->setGeometry(QRect(10, 370, 121, 25));
        testRotate->setFont(font);
        fitHGBase = new QPushButton(Tracing);
        fitHGBase->setObjectName(QStringLiteral("fitHGBase"));
        fitHGBase->setGeometry(QRect(140, 240, 101, 25));
        fitHGBase->setFont(font);
        testFit = new QPushButton(Tracing);
        testFit->setObjectName(QStringLiteral("testFit"));
        testFit->setGeometry(QRect(140, 370, 101, 25));
        testFit->setFont(font);
        simuFit = new QPushButton(Tracing);
        simuFit->setObjectName(QStringLiteral("simuFit"));
        simuFit->setGeometry(QRect(140, 400, 101, 25));
        simuFit->setFont(font);
        testRotate_2 = new QPushButton(Tracing);
        testRotate_2->setObjectName(QStringLiteral("testRotate_2"));
        testRotate_2->setGeometry(QRect(140, 270, 121, 25));
        testRotate_2->setFont(font);

        retranslateUi(Tracing);

        QMetaObject::connectSlotsByName(Tracing);
    } // setupUi

    void retranslateUi(QWidget *Tracing)
    {
        Tracing->setWindowTitle(QApplication::translate("Tracing", "Form", Q_NULLPTR));
        label->setText(QApplication::translate("Tracing", "\351\200\211\346\213\251\347\204\212\347\274\235:", Q_NULLPTR));
        label_2->setText(QApplication::translate("Tracing", "CAD:", Q_NULLPTR));
        ckxWeld->setText(QApplication::translate("Tracing", "\347\204\212\346\216\245", Q_NULLPTR));
        btnPosition3->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\347\253\226\350\275\264", Q_NULLPTR));
        home->setText(QApplication::translate("Tracing", "\350\275\254\345\212\250\350\207\263", Q_NULLPTR));
        dig->setText(QApplication::translate("Tracing", "0", Q_NULLPTR));
        rotateTrack->setText(QApplication::translate("Tracing", "\350\267\237\350\270\252", Q_NULLPTR));
        btnPosition2->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\346\250\252\350\275\264", Q_NULLPTR));
        btnPosition1->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\345\234\260\350\275\250", Q_NULLPTR));
        home_2->setText(QApplication::translate("Tracing", "\345\205\263\351\227\255\344\277\241\345\217\267", Q_NULLPTR));
        home_3->setText(QApplication::translate("Tracing", "\346\265\213\350\257\225IO\344\277\241\345\217\267", Q_NULLPTR));
        signal51->setText(QApplication::translate("Tracing", "\346\211\223\345\274\200\344\277\241\345\217\267", Q_NULLPTR));
        verifiKeel->setText(QApplication::translate("Tracing", "\351\252\214\350\257\201\351\276\231\351\252\250", Q_NULLPTR));
        fitBase->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\345\237\272\346\234\254\347\202\271", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\351\276\231\351\252\250", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("Tracing", "\345\210\207\345\211\262\346\225\260\346\215\256", Q_NULLPTR));
        testRotate->setText(QApplication::translate("Tracing", "\346\265\213\350\257\225\350\275\254\345\212\250", Q_NULLPTR));
        fitHGBase->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\345\215\216\345\205\211", Q_NULLPTR));
        testFit->setText(QApplication::translate("Tracing", "\346\265\213\350\257\225fit", Q_NULLPTR));
        simuFit->setText(QApplication::translate("Tracing", "\346\213\237\345\220\210\344\273\277\347\234\237", Q_NULLPTR));
        testRotate_2->setText(QApplication::translate("Tracing", "\346\265\213\350\257\225\350\256\241\347\256\227", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Tracing: public Ui_Tracing {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRACING_H
