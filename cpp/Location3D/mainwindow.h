#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <HalconCpp.h>
// for parse json data
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

//share ptr
#include <memory>
//#include <chrono>
//#include <thread>


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

namespace rpc {
class client;
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
    void initRPC();
    void initTemplate();

    // Timer
    long timer=0;
    bool isDeviceOpen=false;


    std::shared_ptr<CamSetting> camSetting;
    std::shared_ptr<CamCalibration> camCalibration;

    //Halcon Variable
    HObject Image;
    HTuple   fgHandle;
    HWindow*   hwindow;
    HTuple  Width, Height;
    HTuple  modelID;
    HTuple  camParam;




    //Display function
    //    void display_match_pose (HTuple, HTuple, HTuple);
    void disp_3d_coord_system (HTuple& , HTuple& , HTuple);
    void gen_arrow_contour_xld (HObject *, HTuple , HTuple , HTuple , HTuple , HTuple , HTuple );

//BlueZero Node
#ifdef WITH_BZERO
    std::shared_ptr<b0::Node> node;
    std::shared_ptr<b0::Subscriber> sub_node;
    std::shared_ptr<b0::Publisher>  pub_node;
#endif

    //rpc libs
    //    rpc::client* rpc_cli;
    std::shared_ptr<rpc::client> rpc_cli;
    byte* r;
    byte* g;
    byte* b;

    QJsonParseError jsonerr;
    QJsonDocument jsondoc;
    QJsonObject jsonobj;

private slots:
    void openDevice();
    void startGrab();
    void processImage();
    void callYoloService();
    void singleShot();
    void stopGrab();

protected:
    void timerEvent(QTimerEvent *event);
    void resizeEvent(QResizeEvent *event);


};

#endif // MAINWINDOW_H
