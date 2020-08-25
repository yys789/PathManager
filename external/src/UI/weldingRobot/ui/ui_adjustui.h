/********************************************************************************
** Form generated from reading UI file 'adjustui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ADJUSTUI_H
#define UI_ADJUSTUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AdjustUI
{
public:

    void setupUi(QWidget *AdjustUI)
    {
        if (AdjustUI->objectName().isEmpty())
            AdjustUI->setObjectName(QStringLiteral("AdjustUI"));
        AdjustUI->resize(670, 594);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AdjustUI->sizePolicy().hasHeightForWidth());
        AdjustUI->setSizePolicy(sizePolicy);

        retranslateUi(AdjustUI);

        QMetaObject::connectSlotsByName(AdjustUI);
    } // setupUi

    void retranslateUi(QWidget *AdjustUI)
    {
        AdjustUI->setWindowTitle(QApplication::translate("AdjustUI", "Form", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AdjustUI: public Ui_AdjustUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ADJUSTUI_H
