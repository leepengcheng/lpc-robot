#include "camcalibration.h"
#include "ui_camcalibration.h"
#include <HalconCpp.h>

CamCalibration::CamCalibration(HalconCpp::HTuple& fghandle) :
    QDialog(0),
    ui(new Ui::CamCalibration)
{
    ui->setupUi(this);

    connect(ui->btn_ok,&QPushButton::clicked,[&]{this->accept();});
    connect(ui->btn_cancel,&QPushButton::clicked,[&]{this->reject();});
}

CamCalibration::~CamCalibration()
{
    delete ui;
}
