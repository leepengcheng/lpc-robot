#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <HalconCpp.h>
#include <queue>
#include <memory>




struct ResultContainer
{
    HalconCpp::HImage   result_img;
    //  HalconCpp::HXLDCont symbol_data;
    //  HalconCpp::HTuple   time_needed;
    //  HalconCpp::HTuple   result_handle;
    //  HalconCpp::HTuple   decoded_data;
};

namespace Ui {
class MainWindow;
}
class CamSetting;
class CamCalibration;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    HalconCpp::HTuple   fgHandle;
    HalconCpp::HWindow*   hwindow;

private:
    Ui::MainWindow *ui;
    void initialize();
    bool openDevice(const char* deviceType);
    void grabImage();
    void processImage();
    void singleShot();
    void stopGrabImage();
    void displayImage();


    bool isDeviceOpen=false;
    const int MAX_IMGBUFFER=3;
    int processCout=0;

    //共享数据
    struct   ResultContainer resultData;
    std::queue<HalconCpp::HImage> imgQueue;

    //相机设置界面
    std::shared_ptr<CamSetting> camSetting;
    std::shared_ptr<CamCalibration> camCalibration;

};

#endif // MAINWINDOW_H
