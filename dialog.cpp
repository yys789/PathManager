#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_pbn_set_clicked()
{
    aglWeld = (ui->edtAglO->text()).toDouble();
    aglA = (ui->edtAglA->text()).toDouble();
    aglT = (ui->edtAglT->text()).toDouble();
    aglInclude = 7.5;
    aglHead =(ui->edtAglS->text()).toDouble();
    aglEnd =(ui->edtAglE->text()).toDouble();
}
