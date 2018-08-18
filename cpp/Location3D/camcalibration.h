#ifndef CAMCALIBRATION_H
#define CAMCALIBRATION_H

#include <QDialog>

namespace Ui {
class CamCalibration;
}
namespace HalconCpp {
class HTuple;
}
class CamCalibration : public QDialog
{
    Q_OBJECT

public:
    explicit CamCalibration(HalconCpp::HTuple& fghandle);
    ~CamCalibration();

private:
    Ui::CamCalibration *ui;
};

#endif // CAMCALIBRATION_H
