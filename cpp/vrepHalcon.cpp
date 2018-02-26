#include <opencv/cv.hpp>
#include <HalconCpp.h>
#include <vector>

extern "C" {
#include "extApi.h"
}

using namespace std;
void getObjectPoints(vector<cv::Point3d>& objPoints,const cv::Size& board_size)
{
    cv::Size square_size(20,20);//单位mm
    for (int i = 0; i < board_size.height; i++)
    {
        for (int j = 0; j < board_size.width; j++)
        {
            cv::Point3d realPoint;
            // 假设标定板放在世界坐标系中z=0的平面上
            realPoint.x = i * square_size.width;
            realPoint.y = j * square_size.height;
            realPoint.z = 0;
            objPoints.push_back(realPoint);
        }
    }
}

int main(int argc,char* argv[])
{
    simxFinish(-1); //关闭所有连接
    int clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5); //打开连接
    if (clientID!=-1)
    {
        printf("Successfully Connected to remote API server\n");
        //获得相机句柄
        int visionHandle=0;
        simxGetObjectHandle(clientID,"Vision_sensor#",&visionHandle,simx_opmode_oneshot_wait);

        //获得物体的句柄
        int objectHandle=0;
        simxGetObjectHandle(clientID,"Sphere",&objectHandle,simx_opmode_oneshot_wait);


        //获得深度相机的近景和远景距离
        float nearClip=0;
        simxGetObjectFloatParameter(clientID,visionHandle,sim_visionfloatparam_near_clipping,&nearClip,simx_opmode_oneshot_wait);
        float farClip=0;
        simxGetObjectFloatParameter(clientID,visionHandle,sim_visionfloatparam_far_clipping,&farClip,simx_opmode_oneshot_wait);
        printf("nearclip:%f   farclip:%f\n",nearClip,farClip);
        int resolution[2];//获取传感器的分辨率
        simxGetObjectIntParameter(clientID,visionHandle,sim_visionintparam_resolution_x,&resolution[0],simx_opmode_oneshot_wait);
        simxGetObjectIntParameter(clientID,visionHandle,sim_visionintparam_resolution_y,&resolution[1],simx_opmode_oneshot_wait);
        printf("reslution x:%d  reslution x:%d\n",resolution[0],resolution[1]);

        //获得相机的位置
        float visionPos=0;
        simxGetObjectPosition(clientID,visionHandle,-1,&visionPos,simx_opmode_oneshot_wait);


        simxUChar* vimage;

        //流传输开始,此时无法获得图片的尺寸
        simxGetVisionSensorImage(clientID, visionHandle,resolution,&vimage,0, simx_opmode_streaming);
        extApi_sleepMs(500);//暂停500ms
        //        cv::namedWindow("remote depth image");


        //halcon 窗口设置
        //HalconCpp::SetWindowAttr("border_width", 0);
        //HalconCpp::SetWindowAttr("background_color", "white");
        HalconCpp::HWindow  hwindow(0,0,resolution[0],resolution[1]);
        hwindow.SetPart(0,0,resolution[1],resolution[0]);//设置范围，非常重要
        hwindow.SetWindowParam("window_title","Halcon Image");
        hwindow.SetWindowParam("background_color", "white");
        hwindow.SetColor("green");
        hwindow.SetLineWidth(20);

        HalconCpp::HObject      himage;//图片
        cv::Size boardsize(9,6);

        int num=0;
        while (simxGetConnectionId(clientID)!=-1)
        {
            if(simxGetVisionSensorImage(clientID, visionHandle,resolution,&vimage,0, simx_opmode_buffer)==simx_return_ok)
            {
                cv::Mat oimage(resolution[1],resolution[0],CV_8UC3,vimage);//转换为MAT
                cv::flip(oimage,oimage,0);//水平翻转
                char buff[256];
                if(num%50==0)
                {
                    sprintf(buff,"chessboard-%d.png",num);
                    cv::imwrite(buff,oimage);
                }
                num++;
                cv::cvtColor(oimage,oimage,cv::COLOR_BGR2RGB);//转换为RGB
                std::vector<cv::Point2f> pointbuf;
                cv::findChessboardCorners(oimage,boardsize,pointbuf);
                cv::drawChessboardCorners(oimage, boardsize, pointbuf, true); //用于在图片中标记角点


                cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, cv::Scalar::all(0)); // 摄像机内参数矩阵
                cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, cv::Scalar::all(0));   // 摄像机的5个畸变系数：k1,k2,p1,p2,k3

                cv::Vec3d rMat(0,0,-2.094);
                cv::Vec3d tMat(0,0,0);

                cameraMatrix.at<double>(0,0)=1;//fx
                cameraMatrix.at<double>(1,1)=1;//fy
                cameraMatrix.at<double>(0,2)=resolution[1]/2;//sx
                cameraMatrix.at<double>(1,2)=resolution[0]/2;//sy

                std::vector<cv::Point3d> objectPts;//相机坐标系下的目标空间点
                getObjectPoints(objectPts,boardsize);


                vector<cv::Point2d> image_points; // 保存重新计算得到的投影点
                cv::projectPoints(objectPts,rMat,tMat,cameraMatrix,distCoeffs,image_points);
                auto pt=image_points[0];
//                std::cout<<"("<<pt.x<<","<<pt.y<<")"<<" ";
                cv::imshow("Opencv image",oimage); //显示图片
                std::vector<cv::Mat> rgb(3);
                cv::split(oimage,rgb);//通道分离
                if(cv::waitKey(1)==27) //如果1ms内按下esc则推出
                {
                    return 0;
                }
                HalconCpp::GenImage3Extern(&himage,"byte",resolution[0],resolution[1],(Hlong)rgb[2].ptr(),(Hlong)rgb[1].ptr(),(Hlong)rgb[0].ptr(),(Hlong)0);
                //HalconCpp::GenImage1Extern(&himage,"byte",resolution[0],resolution[1],(Hlong)vimage,(Hlong)0);
                //HalconCpp::MirrorImage(himage,&himage,"row");//水平镜像图片
                hwindow.DispObj(himage);
                //HalconCpp::WriteImage(himage,"png",0,"data.png");//写出图片
                //HalconCpp::WaitSeconds(1);
            }
        }
        simxFinish(clientID);
    }
    return(0);
}


