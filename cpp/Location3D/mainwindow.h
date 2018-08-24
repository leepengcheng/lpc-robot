#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <HalconCpp.h>
#include <memory>



using namespace HalconCpp;
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
    HTuple   fgHandle;
    HWindow*   hwindow;
    HObject Image;

private:
    Ui::MainWindow *ui;
    void initialize();
    void action();
    bool isDeviceOpen=false;

    // Timer
    long Timer;

    //相机设置界面
    std::shared_ptr<CamSetting> camSetting;
    std::shared_ptr<CamCalibration> camCalibration;

    HTuple  Width, Height;
    HTuple  modelID;

    //显示
//    void display_match_pose (HTuple, HTuple, HTuple);
    void disp_3d_coord_system (HTuple& , HTuple& , HTuple);
    void gen_arrow_contour_xld (HObject *, HTuple , HTuple , HTuple , HTuple , HTuple , HTuple );

private slots:
    void openDevice();
    void startGrab();
    void processImage();
    void singleShot();
    void stopGrab();



    // QObject interface
protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MAINWINDOW_H
