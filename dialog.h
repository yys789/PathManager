#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include<string>

using namespace std;
using std::vector;

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

    double aglWeld, aglA, aglT, aglInclude, aglHead, aglEnd;
    string strSelect;
    Ui::Dialog *ui;

private slots:
    void on_pbn_set_clicked();

private:

};

#endif // DIALOG_H
