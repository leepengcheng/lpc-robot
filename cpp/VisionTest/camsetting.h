#ifndef CAMSETTING_H
#define CAMSETTING_H

#include <QDialog>

namespace Ui {
class CamSetting;
}
namespace HalconCpp {
class HTuple;
}
class QSettings;
class CamSetting : public QDialog
{
    Q_OBJECT

public:
    explicit CamSetting(HalconCpp::HTuple& fghandle);
    ~CamSetting();

private:
    void InitControlData();
    Ui::CamSetting *ui;
    QSettings* config;
};

#endif // CAMSETTING_H
