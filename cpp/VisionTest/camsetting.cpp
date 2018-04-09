#include "camsetting.h"
#include "ui_camsetting.h"
#include <HalconCpp.h>
#include <QSettings>

CamSetting::CamSetting(HalconCpp::HTuple& fghandle) :QDialog(0),ui(new Ui::CamSetting)
{
    ui->setupUi(this);
    //配置文件
    config=new QSettings("appconf.ini",QSettings::IniFormat);

    connect(ui->scroll_brightness,&QSlider::valueChanged,
            [&](int val)
    {
        ui->edit_brightness->setText(QString::number(val));
        HalconCpp::SetFramegrabberParam(fghandle,"brightness",val);
    });
    connect(ui->scroll_exposure,&QSlider::valueChanged,
            [&](int val)
    {
        ui->edit_exposure->setText(QString::number(val));
        HalconCpp::SetFramegrabberParam(fghandle,"exposure",val);
    });
    connect(ui->scroll_gain,&QSlider::valueChanged,
            [&](int val)
    {
        ui->edit_gain->setText(QString::number(val));
        HalconCpp::SetFramegrabberParam(fghandle,"video_gain",val);
    });

    connect(ui->button_save, &QPushButton::clicked,[&]{
        config->setValue("brightness",ui->scroll_brightness->value());
        config->setValue("exposure",ui->scroll_exposure->value());
        config->setValue("gain",ui->scroll_gain->value());
        this->accept();
    });

    connect(ui->button_exit,&QPushButton::clicked,[&]{this->reject();});

    //初始化控件参数
    this->InitControlData();
}

CamSetting::~CamSetting()
{
    delete config;
    delete ui;
}

void CamSetting::InitControlData()
{
    int brightness_val=config->value("brightness",50).value<int>();
    ui->scroll_brightness->setValue(brightness_val);
    emit ui->scroll_brightness->valueChanged(brightness_val);
}
