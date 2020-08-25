#ifndef SETTINGUI_H
#define SETTINGUI_H

#include <QWidget>
#include "tools/singleton.h"
#include "device/camera.h"
#include "base/config.h"

namespace Ui {
class SettingUI;
}

class SettingUI : public QWidget
{
    Q_OBJECT

public:
    explicit SettingUI(QWidget *parent = 0);
    ~SettingUI();

private slots:
    void on_saveSetting_clicked();

private:
    Ui::SettingUI *ui;
    Config _sys_config ;
    std::shared_ptr<Camera> _camera_ptr ;
private:
    void initUI() ;
};

#endif // SETTINGUI_H
