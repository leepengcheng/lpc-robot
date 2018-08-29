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
namespace b0 {
class Node;
class Subscriber;
class Publisher;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    void initUI();
    void initBZERO();
    void action();
    bool isDeviceOpen=false;

    // Timer
    long timer=0;


    std::shared_ptr<CamSetting> camSetting;
    std::shared_ptr<CamCalibration> camCalibration;

    //Halcon Variable
    HTuple   fgHandle;
    HWindow*   hwindow;
    HObject Image;
    HTuple  Width, Height;
    HTuple  modelID;

    //Display function
//    void display_match_pose (HTuple, HTuple, HTuple);
    void disp_3d_coord_system (HTuple& , HTuple& , HTuple);
    void gen_arrow_contour_xld (HObject *, HTuple , HTuple , HTuple , HTuple , HTuple , HTuple );

    //BlueZero Node
    b0::Node* node=NULL;
    b0::Subscriber* sub_node=NULL;
    b0::Publisher*  pub_node=NULL;

private slots:
    void openDevice();
    void startGrab();
    void processImage();
    void singleShot();
    void stopGrab();

protected:
    void timerEvent(QTimerEvent *event);
    void resizeEvent(QResizeEvent *event);


};

#endif // MAINWINDOW_H
