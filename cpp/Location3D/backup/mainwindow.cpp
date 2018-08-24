#define NO_EXPORT_APP_MAIN
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "imgprocess.hpp"
#include <list>
#include <future>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <condition_variable>
#include "camsetting.h"
#include "camcalibration.h"

//realsense
//#include <iostream>
//#include <librealsense2/rs.hpp>
//using namespace rs2;

//原子锁
std::atomic<bool> isgrab(false);
std::atomic<bool> isprocess(false);
std::atomic<bool> isdisplay(false);

//条件变量
std::condition_variable cv_process;
std::condition_variable cv_display;

//图片队列/处理后的图片的互斥体
std::mutex  imgQueueMutex;
std::mutex  resultDataMutex;



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->initialize();

    connect(ui->btn_opendevice,&QPushButton::clicked,[=](){
        isDeviceOpen=this->openDevice("DirectShow");
        ui->btn_opendevice->setEnabled(!isDeviceOpen);
        ui->btn_startgrab->setEnabled(isDeviceOpen);
    });

    connect(ui->btn_startgrab,&QPushButton::clicked,[=](){

        //使能按钮
        ui->btn_startgrab->setEnabled(false);
        ui->btn_stopgrab->setEnabled(true);

        //清空队列
        imgQueue.swap(std::queue<HImage>());

        //图像采集线程
        this->startGrab();

        //图像处理线程
        this->processImage();

        //图像显示线程
        this->displayImage();

    });

    connect(ui->btn_stopgrab,&QPushButton::clicked,[=](){
        this->stopGrab();
    });

    //单张图拍摄
    connect(ui->btn_snapshot,&QPushButton::clicked,[=](){
        this->singleShot();
    });
}


MainWindow::~MainWindow()
{

    isgrab.store(false);
    isprocess.store(false);
    isdisplay.store(false);
    CloseAllFramegrabbers();
    delete hwindow;
    delete ui;
}


//初始化
void MainWindow::initialize()
{
    SetWindowAttr("border_width", 0);
    SetWindowAttr("background_color", "white");
    SetCheck("~father");
    hwindow=new HWindow(0,0,ui->hwindow->width(),ui->hwindow->height(),(Hlong)ui->hwindow->winId(), "visible", "");
    hwindow->SetColor("green");
    hwindow->SetLineWidth(20);
    SetCheck("father");

    //处理的图片数目
    ui->lcdSuccessNum->display(processCout);


    //打开相机处理界面
    ui->menu_camera->addAction(style()->standardIcon(QStyle::SP_ArrowBack),tr("参数设置"),
                               [&](){
        if(isDeviceOpen)
        {
            camSetting.reset(new CamSetting(fgHandle));
            (*camSetting).show();
        } else
        {
            this->statusBar()->showMessage("Please Open Device First!",3000);
        }

    });

    //打开图片标定界面
    ui->menu_camera->addAction(style()->standardIcon(QStyle::SP_ArrowBack),tr("相机标定"),
                               [&](){
        if(isDeviceOpen)
        {
            camCalibration.reset(new CamCalibration(fgHandle));
            (*camCalibration).show();
        } else
        {
            this->statusBar()->showMessage("Please Open Device First!",3000);
        }

    });


}



//打开摄像头
bool MainWindow::openDevice(const char* deviceType)
{
    try
    {
        //close all framegrabbers
//        CloseAllFramegrabbers();
        //get the device info and open
//        HalconCpp::HTuple device_info,device_list;
//        InfoFramegrabber(deviceType, "device", &device_info, &device_list);
//        OpenFramegrabber(deviceType, 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
//                         "default", HTuple(device_list[0]), 0, -1, &fgHandle);
        OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", -1, "default", -1,
            "false", "default", "default", -1, -1, &fgHandle);
//        InfoFramegrabber(deviceType, "parameters", &device_info, &device_list);
//        for(int i=0;i<device_list.Length();++i)
//        {
//            std::cout<<"device:"<<device_list[i].S().Text()<<std::endl;
//        }
        GrabImageStart(fgHandle, -1);

    }
    catch (HException &except)
    {
        //        this->statusBar()->showMessage(except.ErrorMessage().Text(),3000);
        this->statusBar()->showMessage(except.ProcName().Text(),3000);

        return false;
    }

    //notify message
    this->statusBar()->showMessage("device is opening",3000);
    return true;
}



void MainWindow::startGrab()
{
    //if grab is running,return
    if(isgrab.load()){return;}
    isgrab.store(true);
    //start grab image thread
    std::async(std::launch::async,[&]{
        HImage image;
        while (isgrab.load())
        {
            GrabImageAsync(&image, fgHandle, -1);
            {

                std::lock_guard<std::mutex> lg(imgQueueMutex);
                if(imgQueue.size()<MAX_IMGBUFFER)
                {
                    imgQueue.push(image);
                    std::cout<<"grabbing"<<std::endl;
                }
            }
            cv_process.notify_one();
        }
    });
}


//处理图片
void MainWindow::processImage()
{

    if(isprocess.load()){return;}
    isprocess.store(true);
    //start image process thread
    std::async(std::launch::async,[&]{
        HImage image;
        while(isprocess.load())
        {
            //从队列中取出图片并进行处理
            {
                std::unique_lock<std::mutex> ul(imgQueueMutex);
                cv_process.wait(ul,[&]{return !imgQueue.empty();});
                std::cout<<"procesing"<<std::endl;
                image=imgQueue.front();
                imgQueue.pop();
            }
            //将图片赋值给全局结构体
            {
                std::lock_guard<std::mutex> lg(resultDataMutex);
                resultData.result_img=image;
            }
            //通知给结果显示线程
            cv_display.notify_one();
        }

    });
}

void MainWindow::singleShot()
{

    std::async(std::launch::async,[&]{
        HImage image;
        GrabImageAsync(&image, fgHandle, -1);
        hwindow->DispObj(image);
    });
}

void MainWindow::stopGrab()
{
    isgrab.store(false);
    isprocess.store(false);
    isdisplay.store(false);
    ui->btn_startgrab->setEnabled(true);
    ui->btn_stopgrab->setEnabled(false);
}

void MainWindow::displayImage()
{
    if(isdisplay.load()){return;}
    isdisplay.store(true);
    //start display image thread
    std::async(std::launch::async,[&]{
        Hlong Second,Minute, Hour, Day, YDay, Month, Year;
        auto t1=std::chrono::steady_clock::now();
        auto t2=t1;
        while(isdisplay.load())
        {
            {
                std::unique_lock<std::mutex> ul(resultDataMutex);
                cv_display.wait(ul); //添加[]{return true;}会报错?
                resultData.result_img.GetImageTime(&Second,&Minute, &Hour, &Day, &YDay, &Month, &Year);
                //                std::cout<<"Hour:"<<Hour<<" Minute:"<<Minute<<" Second: "<<Second<<std::endl;
                hwindow->DispObj(resultData.result_img);
                ui->lcdSuccessNum->display(++processCout);
                if(processCout%20==0)
                {
                    t2=std::chrono::steady_clock::now();
                    auto duration=std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
                    t1=t2;
                    std::cout<<QString(">>>Framte Rage:%1").arg(20000/duration.count()).toStdString().c_str()<<std::endl;
                    //                hwindow->DispCross(50,50,100,15);
                }

            }
        }
    });

}
