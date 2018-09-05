#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camsetting.h"
#include "camcalibration.h"

#ifdef WITH_BZERO
#include <b0/node.h>
#include <b0/publisher.h>
#include <b0/subscriber.h>
#include <boost/lexical_cast.hpp>
#include <Windows.h>
#define SLEEP_MS(x) Sleep(x)
#endif



//for msgpack-rpc
#include "rpc/client.h"
#include "rpc/rpc_error.h"
#include "image.h"



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
    this->initRPC();
    this->initTemplate();
    //    connect(ui->btn_opendevice,SIGNAL(clicked()),SLOT(openDevice()));
    //    connect(ui->btn_startgrab,SIGNAL(clicked()),SLOT(startGrab()));
    //    connect(ui->btn_stopgrab,SIGNAL(clicked()),SLOT(stopGrab()));
    //    connect(ui->btn_snapshot,SIGNAL(clicked()),SLOT(singleShot()));
}



MainWindow::~MainWindow()
{

    //clear Node
#ifdef WITH_BZERO
    //delete pub_node;
    //delete sub_node;
    node->cleanup();
#endif
    
    //delete rpc
    //    delete rpc_cli;
    delete r;
    delete b;
    delete g;
    
    CloseAllFramegrabbers();
    //    if(isDeviceOpen)CloseFramegrabber(fgHandle);
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
    isDeviceOpen=false;
    SetSystem("use_window_thread", "true");
    SetWindowAttr("border_width", 0);
    SetWindowAttr("background_color", "white");
    SetCheck("~father");
    hwindow=new HWindow(0,0,this->width(),this->height(),(Hlong)ui->hwindow->winId(), "visible", "");
    hwindow->SetColor("green");
    hwindow->SetDraw("margin");
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
#ifdef WITH_BZERO
    //    node=new b0::Node("subNode");
    //    sub_node=new b0::Subscriber(node,"topic1_string",&sensorCallback);
    //    pub_node=new b0::Publisher(node,"topic1_string");

    node.reset(new b0::Node("subNode"));
    sub_node.reset(new b0::Subscriber(node.get(),"topic1_string",&sensorCallback));
    pub_node.reset(new b0::Publisher(node.get(),"topic1_string"));
    node->init();
#endif
}

void MainWindow::initRPC()
{
    //    rpc_cli=new  rpc::client("192.168.6.85", 8800);
    rpc_cli.reset(new  rpc::client("127.0.0.1", 8800));
}

void MainWindow::initTemplate()
{
    //init Template
    camParam.Clear();
    camParam[0] = 0.00521;
    camParam[1] = 4899.91;
    camParam[2] = 8.3e-006;
    camParam[3] = 8.3e-006;
    camParam[4] = 325.976;
    camParam[5] = 240.643;
    camParam[6] = 640;
    camParam[7] = 480;


    HTuple  dxfStatus;
    HTuple  dxfModelID;
    try
    {
        ReadShapeModel3d("template.sm3", &modelID);
    }
    catch (HalconCpp::HException)
    {
        //Load dxf Template
        ReadObjectModel3d("../lib/template.dxf", "mm", HTuple(), HTuple(), &dxfModelID,
                          &dxfStatus);
        PrepareObjectModel3d(dxfModelID, "shape_based_matching_3d", "true", HTuple(),HTuple());


        CreateShapeModel3d(dxfModelID, camParam, 0, 0, 0, "gba", -(HTuple(60).TupleRad()),
                           HTuple(60).TupleRad(), -(HTuple(60).TupleRad()), HTuple(60).TupleRad(), 0,
                           HTuple(360).TupleRad(), 0.2, 0.4, 10, "lowest_model_level", 3, &modelID);

        ClearObjectModel3d(dxfModelID);

        try
        {
            WriteShapeModel3d(modelID, "template.sm3");
        }

        catch (HalconCpp::HException)
        {
            //HDevExpDefaultException.ToHTuple(&hException);
            std::cout<<"Writing model to disk ... failed!"<<std::endl;
        }
    }
}

void MainWindow::openDevice()
{
    isDeviceOpen=false;
    try
    {
        CloseAllFramegrabbers();
        OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
                         "[0] yuv (640x480)", "[0] Intel(R) RealSense(TM) 430 with RGB Module RGB",
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
        timer = startTimer(20);
    }
    else
    {
        this->statusBar()->showMessage("Can Not Open Device,Please Check Camera",3000);
    }
    
}




void MainWindow::callYoloService()
{
    try{

        HTuple cType,RedPtr,GreenPtr,BluePtr;
        GetImagePointer3(Image,&RedPtr,&GreenPtr,&BluePtr,&cType,&Width,&Height);
        HalconImageMsg himg_msg;
        himg_msg.width = Width.I();
        himg_msg.height = Height.I();
        r=((byte*)RedPtr.L());
        g=((byte*)GreenPtr.L());
        b=((byte*)BluePtr.L());

        int  buff_size=himg_msg.width*himg_msg.height;
        std::vector<uchar>  redrbuff(r, r + buff_size);
        std::vector<uchar>  greenbuff(g, g + buff_size);
        std::vector<uchar>  bluebuff(b, b + buff_size);
        himg_msg.r = redrbuff;
        himg_msg.g = greenbuff;
        himg_msg.b = bluebuff;

        auto jsonData = rpc_cli->call("inference",himg_msg).as<std::string>();
        jsondoc=QJsonDocument::fromJson(jsonData.c_str(),&jsonerr);
        if(jsonerr.error==QJsonParseError::NoError)
        {
            jsonobj=jsondoc.object();
            std::cout<<jsonobj["model"].toString().toStdString()<<"\n";
            auto objs=jsonobj["objects"].toArray();
            for (auto obj:objs)
            {
                double c1=obj.toObject()["xmin"].toDouble();
                double r1=obj.toObject()["ymin"].toDouble();
                double c2=obj.toObject()["xmax"].toDouble();
                double r2=obj.toObject()["ymax"].toDouble();
                hwindow->DispRectangle1(r1,c1,r2,c2);
                hwindow->SetTposition(r1,c1);
                hwindow->WriteString(obj.toObject()["class"].toString().toStdString().c_str());
            }
        }
        else
        {
            std::cout<<"json parse error\n";
        }
    }
    catch (rpc::rpc_error &e) {
        std::cout << std::endl
                  << e.what() << std::endl;
        std::cout << "in function '" << e.get_function_name() << "': ";

        using err_t = std::tuple<int, std::string>;
        auto err = e.get_error().as<err_t>();
        std::cout << "[error " << std::get<0>(err) << "]: " << std::get<1>(err)
                  << std::endl;
    }
}

void MainWindow::singleShot()
{
    if(!isDeviceOpen)
    {
        this->statusBar()->showMessage("Please Open Device First");
        return;
    }
    this->processImage();


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
    GrabImage(&Image,fgHandle);
    hwindow->DispObj(Image);
    this->callYoloService();

    
    ////  ##publish messages
    //    pub_node->publish("hello world123");
    
    //    //handle B0 messages:
    //    node->spinOnce();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    
    ui->hwindow->setGeometry(0,0,this->width(),this->height());
    hwindow->SetWindowExtents(0,0,this->width(),this->height());
}

void MainWindow:: processImage()
{
    HObject  objContours;
    HTuple  pose,poseTmp, covPose, score;
    double t1 = HSystem::CountSeconds();
    //    FindShapeModel3d(Image, modelID, 0.7, 0.9, 0, ((HTuple("num_matches").Append("max_overlap")).Append("border_model")),
    //                     ((HTuple(1).Append(0.75)).Append("true")), &pose, &covPose, &score);

    FindShapeModel3d(Image, modelID, 0.7, 0.9, (HTuple(4).Append(2)),
                     ((HTuple("num_matches").Append("max_overlap")).Append("border_model")),
                     ((HTuple(1).Append(0.5)).Append("true")), &pose, &covPose,
                     &score);

    HTuple obj_num = (score.TupleLength())-1;
    for (HTuple i=0; i.Continue(obj_num, 1); i += 1)
    {

        //    Display contour
        poseTmp = pose.TupleSelectRange(i*7,(i*7)+6);
        ProjectShapeModel3d(&objContours, modelID, camParam, poseTmp,"true", HTuple(30).TupleRad());
        hwindow->SetColor("white");
        hwindow->DispObj(objContours);

        //Display Axis
        hwindow->SetColored(3);
        disp_3d_coord_system(camParam, poseTmp, 0.015);
    }
    double t2 = HSystem::CountSeconds();
    this->statusBar()->showMessage(QString("Time Cost %1").arg(t2-t1));
}


