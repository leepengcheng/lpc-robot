#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camsetting.h"
#include "camcalibration.h"

//BZERO
#include <b0/node.h>
#include <b0/publisher.h>
#include <b0/subscriber.h>
#include <boost/lexical_cast.hpp>
#include <Windows.h>

//thread_sleep
//#include <chrono>
//#include <thread>

#define SLEEP_MS(x) Sleep(x)



void sensorCallback(const std::string &sensTrigger_packedInt)
{
    //    sensorTrigger=((int*)sensTrigger_packedInt.c_str())[0];
    printf("%s\n",sensTrigger_packedInt.c_str());
}

//void MainWindow::display_match_pose (HTuple hv_ShapeModel3DID, HTuple hv_Pose, HTuple hv_WindowHandle)
//{
//    // Local control variables
//    HTuple  hv_DegreeSign, hv_ReferencePoint, hv_CamParam;
//    HTuple  hv_HomMat3D, hv_X, hv_Y, hv_Z, hv_Row, hv_Column;

//    //Construct the degree sign
//    TupleChr(0xb0, &hv_DegreeSign);
//    GetShapeModel3dParams(hv_ShapeModel3DID, "reference_point", &hv_ReferencePoint);
//    GetShapeModel3dParams(hv_ShapeModel3DID, "cam_param", &hv_CamParam);
//    //
//    //Project the reference point
//    PoseToHomMat3d(hv_Pose, &hv_HomMat3D);
//    AffineTransPoint3d(hv_HomMat3D, HTuple(hv_ReferencePoint[0]), HTuple(hv_ReferencePoint[1]),
//            HTuple(hv_ReferencePoint[2]), &hv_X, &hv_Y, &hv_Z);
//    Project3dPoint(hv_X, hv_Y, hv_Z, hv_CamParam, &hv_Row, &hv_Column);
//    //
//    //Display the pose at the projected reference point
//    SetTposition(hv_WindowHandle, hv_Row, hv_Column-10);
//    WriteString(hv_WindowHandle, "Pose:");
//    SetTposition(hv_WindowHandle, hv_Row+15, hv_Column);
//    WriteString(hv_WindowHandle, ("X: "+((1000*HTuple(hv_Pose[0])).TupleString("4.1f")))+" mm");
//    SetTposition(hv_WindowHandle, hv_Row+30, hv_Column);
//    WriteString(hv_WindowHandle, ("Y: "+((1000*HTuple(hv_Pose[1])).TupleString("4.1f")))+" mm");
//    SetTposition(hv_WindowHandle, hv_Row+45, hv_Column);
//    WriteString(hv_WindowHandle, ("Z: "+((1000*HTuple(hv_Pose[2])).TupleString("4.1f")))+" mm");
//    SetTposition(hv_WindowHandle, hv_Row+60, hv_Column);
//    WriteString(hv_WindowHandle, ("Alpha: "+(HTuple(hv_Pose[3]).TupleString("4.1f")))+hv_DegreeSign);
//    SetTposition(hv_WindowHandle, hv_Row+75, hv_Column);
//    WriteString(hv_WindowHandle, ("Beta: "+(HTuple(hv_Pose[4]).TupleString("4.1f")))+hv_DegreeSign);
//    SetTposition(hv_WindowHandle, hv_Row+90, hv_Column);
//    WriteString(hv_WindowHandle, ("Gamma: "+(HTuple(hv_Pose[5]).TupleString("4.1f")))+hv_DegreeSign);
//    return;
//}

void MainWindow::disp_3d_coord_system ( HTuple &camParam, HTuple &pose, HTuple axesLength)
{

    // Local iconic variables
    HObject  ho_Arrows;

    // Local control variables
    HTuple  hv_TransWorld2Cam, hv_OrigCamX, hv_OrigCamY;
    HTuple  hv_OrigCamZ, hv_Row0, hv_Column0, hv_X, hv_Y, hv_Z;
    HTuple  hv_RowAxX, hv_ColumnAxX, hv_RowAxY, hv_ColumnAxY;
    HTuple  hv_RowAxZ, hv_ColumnAxZ, hv_Distance, hv_HeadLength;

    if (0 != ((pose.TupleLength())!=7))
    {
        return;
    }
    if (0 != (HTuple(HTuple(pose[2])==0.0).TupleAnd(HTuple(camParam[0])!=0)))
    {
        return;
    }
    //Convert to pose to a transformation matrix
    PoseToHomMat3d(pose, &hv_TransWorld2Cam);
    //Project the world origin into the image
    AffineTransPoint3d(hv_TransWorld2Cam, 0, 0, 0, &hv_OrigCamX, &hv_OrigCamY, &hv_OrigCamZ);
    Project3dPoint(hv_OrigCamX, hv_OrigCamY, hv_OrigCamZ, camParam, &hv_Row0, &hv_Column0);
    //Project the coordinate axes into the image
    AffineTransPoint3d(hv_TransWorld2Cam, axesLength, 0, 0, &hv_X, &hv_Y, &hv_Z);
    Project3dPoint(hv_X, hv_Y, hv_Z, camParam, &hv_RowAxX, &hv_ColumnAxX);
    AffineTransPoint3d(hv_TransWorld2Cam, 0, axesLength, 0, &hv_X, &hv_Y, &hv_Z);
    Project3dPoint(hv_X, hv_Y, hv_Z, camParam, &hv_RowAxY, &hv_ColumnAxY);
    AffineTransPoint3d(hv_TransWorld2Cam, 0, 0, axesLength, &hv_X, &hv_Y, &hv_Z);
    Project3dPoint(hv_X, hv_Y, hv_Z, camParam, &hv_RowAxZ, &hv_ColumnAxZ);
    //
    //Generate an XLD contour for each axis
    DistancePp((hv_Row0.TupleConcat(hv_Row0)).TupleConcat(hv_Row0), (hv_Column0.TupleConcat(hv_Column0)).TupleConcat(hv_Column0),
               (hv_RowAxX.TupleConcat(hv_RowAxY)).TupleConcat(hv_RowAxZ), (hv_ColumnAxX.TupleConcat(hv_ColumnAxY)).TupleConcat(hv_ColumnAxZ),
               &hv_Distance);
    hv_HeadLength = ((((hv_Distance.TupleMax())/12.0).TupleConcat(5.0)).TupleMax()).TupleInt();
    gen_arrow_contour_xld(&ho_Arrows, (hv_Row0.TupleConcat(hv_Row0)).TupleConcat(hv_Row0),
                          (hv_Column0.TupleConcat(hv_Column0)).TupleConcat(hv_Column0), (hv_RowAxX.TupleConcat(hv_RowAxY)).TupleConcat(hv_RowAxZ),
                          (hv_ColumnAxX.TupleConcat(hv_ColumnAxY)).TupleConcat(hv_ColumnAxZ), hv_HeadLength,
                          hv_HeadLength);
    //
    //Display coordinate system
    hwindow->DispXld(ho_Arrows);
}

void MainWindow::gen_arrow_contour_xld (HObject *ho_Arrow, HTuple row1, HTuple column1,
                                        HTuple row2, HTuple column2, HTuple headLength, HTuple headWidth)
{

    // Local iconic variables
    HObject  tempArrow;

    // Local control variables
    HTuple  hv_Length, zeroLengthIndex, hv_DR;
    HTuple  hv_DC, halfHeadWidth, rowP1, colP1, rowP2;
    HTuple  colP2, index;


    GenEmptyObj(&(*ho_Arrow));
    //
    //Calculate the arrow length
    DistancePp(row1, column1, row2, column2, &hv_Length);

    zeroLengthIndex = hv_Length.TupleFind(0);
    if (0 != (zeroLengthIndex!=-1))
    {
        hv_Length[zeroLengthIndex] = -1;
    }
    //
    //Calculate auxiliary variables.
    hv_DR = (1.0*(row2-row1))/hv_Length;
    hv_DC = (1.0*(column2-column1))/hv_Length;
    halfHeadWidth = headWidth/2.0;
    //
    //Calculate end points of the arrow head.
    rowP1 = (row1+((hv_Length-headLength)*hv_DR))+(halfHeadWidth*hv_DC);
    colP1 = (column1+((hv_Length-headLength)*hv_DC))-(halfHeadWidth*hv_DR);
    rowP2 = (row1+((hv_Length-headLength)*hv_DR))-(halfHeadWidth*hv_DC);
    colP2 = (column1+((hv_Length-headLength)*hv_DC))+(halfHeadWidth*hv_DR);
    //
    //Finally create output XLD contour for each input point pair
    {
        HTuple end_val45 = (hv_Length.TupleLength())-1;
        HTuple step_val45 = 1;
        for (index=0; index.Continue(end_val45, step_val45); index += step_val45)
        {
            if (0 != (HTuple(hv_Length[index])==-1))
            {
                //Create_ single points for arrows with identical start and end point
                GenContourPolygonXld(&tempArrow, HTuple(row1[index]), HTuple(column1[index]));
            }
            else
            {
                //Create arrow contour
                GenContourPolygonXld(&tempArrow, ((((HTuple(row1[index]).TupleConcat(HTuple(row2[index]))).TupleConcat(HTuple(rowP1[index]))).TupleConcat(HTuple(row2[index]))).TupleConcat(HTuple(rowP2[index]))).TupleConcat(HTuple(row2[index])),
                                     ((((HTuple(column1[index]).TupleConcat(HTuple(column2[index]))).TupleConcat(HTuple(colP1[index]))).TupleConcat(HTuple(column2[index]))).TupleConcat(HTuple(colP2[index]))).TupleConcat(HTuple(column2[index])));
            }
            ConcatObj((*ho_Arrow), tempArrow, &(*ho_Arrow));
        }
    }
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->initUI();
    this->initBZERO();
    //    connect(ui->btn_opendevice,SIGNAL(clicked()),SLOT(openDevice()));
    //    connect(ui->btn_startgrab,SIGNAL(clicked()),SLOT(startGrab()));
    //    connect(ui->btn_stopgrab,SIGNAL(clicked()),SLOT(stopGrab()));
    //    connect(ui->btn_snapshot,SIGNAL(clicked()),SLOT(singleShot()));
}



MainWindow::~MainWindow()
{


    //clear Node
    delete pub_node;
    delete sub_node;
    node->cleanup();

    CloseAllFramegrabbers();
    //    if(isDeviceOpen)
    //    {
    //        CloseFramegrabber(fgHandle);
    //    }
    if (modelID != -1)
    {
        ClearShapeModel3d(modelID);
    }
    if(timer)
    {
        killTimer(timer);
    }
    delete hwindow;
    delete ui;

}



void MainWindow::initUI()
{
    SetWindowAttr("border_width", 0);
    SetWindowAttr("background_color", "white");
    SetCheck("~father");
    hwindow=new HWindow(0,0,this->width(),this->height(),(Hlong)ui->hwindow->winId(), "visible", "");
    hwindow->SetColor("green");
    hwindow->SetLineWidth(2);
    SetCheck("father");

    ui->menu_camera->addAction(style()->standardIcon(QStyle::SP_ArrowBack),tr("Cam Setting"),
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


    ui->menu_camera->addAction(style()->standardIcon(QStyle::SP_ArrowBack),tr("Cam Calibration"),[&]()
    {
        if(isDeviceOpen)
        {
            camCalibration.reset(new CamCalibration(fgHandle));
            (*camCalibration).show();
        } else
        {
            this->statusBar()->showMessage("Please Open Device First!",3000);
        }

    });


    //    ui->menu_system->addAction(style()->standardIcon(QStyle::SP_CommandLink),tr("OpenDevice"),[&](){this->openDevice();});

    ui->menu_system->addAction(style()->standardIcon(QStyle::SP_DialogOpenButton),tr("Start"),[&](){this->startGrab();});

    ui->menu_system->addAction(style()->standardIcon(QStyle::SP_DialogOpenButton),tr("Snap"),[&](){this->singleShot();});

    ui->menu_system->addAction(style()->standardIcon(QStyle::SP_DialogOpenButton),tr("Stop"),[&](){this->stopGrab();});

}

void MainWindow::initBZERO()
{
    node=new b0::Node("subNode");
    sub_node=new b0::Subscriber(node,"topic1_string",&sensorCallback);
    pub_node=new b0::Publisher(node,"topic1_string");
    node->init();
}




void MainWindow::openDevice()
{
    isDeviceOpen=false;
    try
    {
        CloseAllFramegrabbers();
        OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
                         "[0] yuv (960x540)", "[0] Intel(R) RealSense(TM) 430 with RGB Module RGB",
                         0, -1, &fgHandle);
        GrabImageStart(fgHandle, -1);
    }
    catch (HException &except)
    {
        this->statusBar()->showMessage(except.ProcName().Text(),3000);
        return;
    }

    isDeviceOpen=true;
    GrabImage(&Image,fgHandle);
    GetImageSize(Image,&Width,&Height);
    ui->hwindow->resize(Width[0].L(),Height[0].L());
    hwindow->SetPart(0,0,Height-1,Width-1);
    this->resize(Width[0].L(),Height[0].L());
    this->statusBar()->showMessage("Device is Opening",3000);
}



void MainWindow::startGrab()
{
    if(!isDeviceOpen)
    {
        this->openDevice();
    }
    if(isDeviceOpen)
    {
        timer = startTimer(30);
    }
    else
    {
        this->statusBar()->showMessage("Can Not Open Device,Please Check Camera",3000);
    }

}



void MainWindow::processImage()
{
    GrabImage(&Image,fgHandle);
    hwindow->DispObj(Image);
}

void MainWindow::singleShot()
{
    this->processImage();
    //    this->action();
}

void MainWindow::stopGrab()
{
    if(timer)
    {
        killTimer(timer);
        timer=0;
    }
}


void MainWindow::timerEvent(QTimerEvent *event)
{
    this->processImage();

    //publish messages
    pub_node->publish("hello world123");

    //     handle B0 messages:
    node->spinOnce();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{

    ui->hwindow->setGeometry(0,0,this->width(),this->height());
    hwindow->SetWindowExtents(0,0,this->width(),this->height());
}

void MainWindow:: action()
{


    HObject  inImage, modelContours;

    // Local control variables
    HTuple  camParam, imgWidth, imgHeight, hv_WindowHandle;
    HTuple  objModelID;
    HTuple  dxfStatus, hv_S1, hv_S2, hv_T, hv_Times, numImages;
    HTuple  hv_I, seconds1, pose, covPose, score;
    HTuple  seconds2, hv_Time, hv_J, poseTmp;



    camParam.Clear();
    camParam[0] = 0.0269462;
    camParam[1] = -354.842;
    camParam[2] = 1.27964e-005;
    camParam[3] = 1.28e-005;
    camParam[4] = 254.24;
    camParam[5] = 201.977;
    camParam[6] = 512;
    camParam[7] = 384;
    //
    imgWidth = ((const HTuple&)camParam)[6];
    imgHeight = ((const HTuple&)camParam)[7];

    ReadImage(&inImage, "C:/Users/Public/Documents/MVTec/HALCON-12.0/examples/images/tile_spacers/tile_spacers_color_01.png");


    try
    {
        ReadShapeModel3d("tile_spacer.sm3", &modelID);
    }
    catch (HalconCpp::HException)
    {
        //Load dxf Template
        ReadObjectModel3d("C:/Users/Public/Documents/MVTec/HALCON-12.0/examples/3d_models/tile_spacer.dxf", 0.0001, HTuple(), HTuple(), &objModelID,
                          &dxfStatus);
        PrepareObjectModel3d(objModelID, "shape_based_matching_3d", "true", HTuple(),HTuple());

        CountSeconds(&hv_S1);

        CreateShapeModel3d(objModelID, camParam, 0, 0, 0, "gba", -(HTuple(60).TupleRad()),
                           HTuple(60).TupleRad(), -(HTuple(60).TupleRad()), HTuple(60).TupleRad(), 0,
                           HTuple(360).TupleRad(), 0.26, 0.27, 10, "lowest_model_level", 3, &modelID);
        CountSeconds(&hv_S2);
        hv_T = hv_S2-hv_S1;

        ClearObjectModel3d(objModelID);

        try
        {
            WriteShapeModel3d(modelID, "tile_spacer.sm3");
        }

        catch (HalconCpp::HException)
        {
            //HDevExpDefaultException.ToHTuple(&hException);
            this->statusBar()->showMessage("Writing model to disk ... failed!");
        }
    }


    hv_Times = HTuple();
    numImages = 12;
    {
        HTuple end = numImages;
        HTuple step = 1;
        for (hv_I=1; hv_I.Continue(end, step); hv_I += step)
        {
            ReadImage(&inImage, "tile_spacers/tile_spacers_color_"+(hv_I.TupleString("02")));
            hwindow->DispImage(inImage);

            CountSeconds(&seconds1);
            FindShapeModel3d(inImage, modelID, 0.7, 0.85, 0, ((HTuple("num_matches").Append("max_overlap")).Append("border_model")),
                             ((HTuple(3).Append(0.75)).Append("true")), &pose, &covPose, &score);
            CountSeconds(&seconds2);
            hv_Time = seconds2-seconds1;
            hv_Times = hv_Times.TupleConcat(hv_Time);

            {
                HTuple end_val93 = (score.TupleLength())-1;
                HTuple step_val93 = 1;
                for (hv_J=0; hv_J.Continue(end_val93, step_val93); hv_J += step_val93)
                {

                    //Display contour
                    poseTmp = pose.TupleSelectRange(hv_J*7,(hv_J*7)+6);
                    ProjectShapeModel3d(&modelContours, modelID, camParam, poseTmp,"true", HTuple(30).TupleRad());
                    hwindow->SetColor("white");
                    hwindow->DispObj(modelContours);

                    //Display Axis
                    hwindow->SetColored(3);
                    disp_3d_coord_system(camParam, poseTmp, 0.015);

                }
            }
//            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        }
    }

}


