#ifndef ADJUSTUI_H
#define ADJUSTUI_H

#include <QWidget>

namespace Ui {
class AdjustUI;
}

class AdjustUI : public QWidget
{
    Q_OBJECT

public:
    explicit AdjustUI(QWidget *parent = 0);
    ~AdjustUI();

private:
    Ui::AdjustUI *ui;
};

#endif // ADJUSTUI_H
