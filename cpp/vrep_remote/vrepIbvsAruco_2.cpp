#include <iostream>
//Opencv
#include <opencv2/aruco.hpp>
#include <opencv/cv.hpp>
//Visp
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpConvert.h>
#include <visp3/robot/vpSimulatorPioneer.h>
//#include <math.h>

using namespace std;
extern "C" {
#include "extApi.h"
}



#define DISP_IMAGE  1  //是否显示图片
#define WHEEL_D 0.054 //轮子的直径

//将线速度转换为轮子的角速度
constexpr double wheelSpeed(double lineSpeed)
{
    return lineSpeed/WHEEL_D;
}

//计算图像的位姿
bool getMarkerPoseImage(cv::InputArray img,const cv::Ptr<cv::aruco::Dictionary> &dict,cv::InputArray camMat,
                        cv::InputArray distMat,cv::OutputArray rvecs,cv::OutputArray tvecs)
{
    vector<int> ids;
    vector<vector<cv::Point2f>> corners,rejected;
    //cv::Ptr<cv::aruco::DetectorParameters> params=new cv::aruco::DetectorParameters();
    cv::aruco::detectMarkers(img,dict,corners,ids,cv::aruco::DetectorParameters::create(),rejected);
    if(ids.size()>0)
    {
        //        cv::aruco::drawDetectedMarkers(img,corners,ids);
        cv::aruco::estimatePoseSingleMarkers(corners,0.1,camMat,distMat,rvecs,tvecs);
        return 1;
    }

    return 0;
}

//根据相机速度和采样时间计算位姿的变化量矩阵,等价于v*timeStep
vpHomogeneousMatrix getStepDelta(const vpColVector &v, const float &timeStep)
{
    //单个仿真步后的位姿变化量
    vpColVector v_dt = v * timeStep;
    vpHomogeneousMatrix Delta;
    Delta.buildFrom(v_dt[0],v_dt[1],v_dt[2],v_dt[3],v_dt[4],v_dt[5]);
    return Delta;
}


int main()
{
    simxFinish(-1); //关闭所有连接
    int clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5); //打开连接
    if (clientID==-1)
    {
        printf("Can not Connected to remote API server\n");
        return 1;
    }
    printf("Successfully Connected to remote API server\n");
    //获得相机句柄
    int camHandle=0;
    simxGetObjectHandle(clientID,"camera_sensor",&camHandle,simx_opmode_oneshot_wait);

    //左轮的关节
    int leftJointHandle=0;
    simxGetObjectHandle(clientID,"DynamicLeftJoint",&leftJointHandle,simx_opmode_oneshot_wait);

    //右轮的关节
    int rightJointHandle=0;
    simxGetObjectHandle(clientID,"DynamicRightJoint",&rightJointHandle,simx_opmode_oneshot_wait);

    //相机的关节
    int cameraJointHandle=0;
    simxGetObjectHandle(clientID,"CameraJoint",&cameraJointHandle,simx_opmode_oneshot_wait);

    //获得深度相机的近景和远景距离
    float nearClip=0;
    simxGetObjectFloatParameter(clientID,camHandle,sim_visionfloatparam_near_clipping,&nearClip,simx_opmode_oneshot_wait);
    float farClip=0;
    simxGetObjectFloatParameter(clientID,camHandle,sim_visionfloatparam_far_clipping,&farClip,simx_opmode_oneshot_wait);

    int resolution[2];//获取传感器的分辨率
    simxGetObjectIntParameter(clientID,camHandle,sim_visionintparam_resolution_x,&resolution[0],simx_opmode_oneshot_wait);
    simxGetObjectIntParameter(clientID,camHandle,sim_visionintparam_resolution_y,&resolution[1],simx_opmode_oneshot_wait);

    float timeStep=0;
    simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&timeStep,simx_opmode_oneshot_wait);

    float angle=0;
    simxGetObjectFloatParameter(clientID,camHandle,sim_visionfloatparam_perspective_angle,&angle,simx_opmode_oneshot_wait);

    printf("time step:%f\n",timeStep);
    printf("nearclip:%f   farclip:%f\n",nearClip,farClip);
    printf("reslution x:%d  reslution x:%d\n",resolution[0],resolution[1]);
    printf("perspective Angle:%f",angle);

    //获得相机的初始位置和姿态
    float camPos[3];
    simxGetObjectPosition(clientID,camHandle,-1,camPos,simx_opmode_oneshot_wait);
    float camEuler[3];
    simxGetObjectOrientation(clientID,camHandle,-1,camEuler,simx_opmode_oneshot_wait);



    //#####################初始化伺服任务########################
    vpHomogeneousMatrix cdMo(0, 0, 0.3, vpMath::rad(180),0, 0);//终止时对象在相机中的位姿
    vpHomogeneousMatrix cMo(0, 0, 0.2, 0, vpMath::rad(180), 0);//初始时对象在相机坐标系中的位姿


    vpRxyzVector rxyz(camEuler[0],camEuler[1],camEuler[2]);
    vpThetaUVector camRvecs(rxyz);//转换为旋转向量
    //相机在世界坐标系下的位姿
    vpHomogeneousMatrix wMc(camPos[0],camPos[1],camPos[2],camRvecs[0],camRvecs[1],camRvecs[2]);
    vpHomogeneousMatrix wMo= wMc * cMo;//目标对象在世界坐标系下的位姿










    /////***********伺服任务************////

    vpSimulatorPioneer robot;//仿真器,此处只用于获得雅克比矩阵/相机与ee的相对关系
    vpServo task;
    //设置视觉伺服的类型和参数
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    //图像雅克比的类型和求逆的方法
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(1.0);
    //设置ee和cam的相对位置
    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe(cVe);
    //设置雅克比矩阵(本例中不变)
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    ////虚拟特征点
    vpPoint point(0, 0, 0);
    point.track(cMo);

    vector<vpPoint> points;
    points.push_back(vpPoint(-0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, 0.1, 0));
    points.push_back(vpPoint(-0.1, 0.1, 0));

    //初始化特征点
    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
        points[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], points[i]);
        points[i].track(cMo);
        vpFeatureBuilder::create(p[i], points[i]);
        task.addFeature(p[i], pd[i]);
    }

    //###################################################################
    //流传输开始,此时无法获得图片的尺寸
    simxUChar* vimage;
    simxGetVisionSensorImage(clientID, camHandle,resolution,&vimage,0, simx_opmode_streaming);
    extApi_sleepMs(500);//暂停500ms


    //##################创建marker 图像
    //    cv::Mat markersImage;
    cv::Ptr<cv::aruco::Dictionary> markersDict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); //创建marker字典
    //    绘制marker:marker字典,marker的序号(0~249),marker大小,output marker image,marker border with
    //    cv::aruco::drawMarker(markersDict,49,200,markersImage,1);
    //    cv::imshow("markers",markersImage);
    //    cv::imwrite("../Marker_6X6_250_49.png",markersImage);


    //##############相机内参和畸变系数
    double cxy=resolution[0]*0.5;
    double fxy=cxy/tan(angle / 2);
    cv::Mat K=(cv::Mat_<double>(3,3)<<fxy,0,cxy,0,fxy,cxy,0,0,1);
    cv::Mat D=(cv::Mat_<double>(5,1)<<0,0,0,0,0);



    vpTranslationVector T;
    vpRxyzVector R;
    while (simxGetConnectionId(clientID)!=-1)
    {
        if(simxGetVisionSensorImage(clientID, camHandle,resolution,&vimage,0, simx_opmode_buffer)==simx_return_ok)
        {
            cv::Mat oimage(resolution[1],resolution[0],CV_8UC3,vimage);//转换为MAT
            cv::flip(oimage,oimage,0);//水平翻转
            cv::cvtColor(oimage,oimage,cv::COLOR_BGR2RGB);//转换为RGB
            //检测marker及姿态
            vector<cv::Vec3d> rvecs,tvecs;
            bool hasMarker=getMarkerPoseImage(oimage,markersDict,K,D,rvecs,tvecs);

            //绘制轴
            if(hasMarker)
            {
                /*多个Marker*/
                //for(size_t i=0;i<rvecs.size();++i)
                //{
                //    cv::aruco::drawAxis(oimage,K,D,rvecs[i],tvecs[i],0.1);
                //}
                /*单个marker*/
                cv::aruco::drawAxis(oimage,K,D,rvecs[0],tvecs[0],0.1);
                cMo.buildFrom(tvecs[0][0],tvecs[0][1],tvecs[0][2],rvecs[0][0],rvecs[0][1],rvecs[0][2]);
                //更新特征点
                for (unsigned int i = 0; i < 4; i++) {
                    points[i].track(cMo);
                    vpFeatureBuilder::create(p[i], points[i]);
                }


                //更新雅克比矩阵
                robot.get_cVe(cVe);
                task.set_cVe(cVe);
                robot.get_eJe(eJe);
                task.set_eJe(eJe);

                //计算出速度(v和相机的角度
                vpColVector v = task.computeControlLaw();
//                wMc=wMc*getStepDelta(v,timeStep);//更新wMc

                double v_line=v[0];
                double w_camera=v[1];
                printf("v:%f w:%f\n",v_line,w_camera);
                simxSetJointTargetVelocity(clientID,leftJointHandle,wheelSpeed(v_line),simx_opmode_oneshot);
                simxSetJointTargetVelocity(clientID,rightJointHandle,wheelSpeed(v_line),simx_opmode_oneshot);
                simxSetJointTargetVelocity(clientID,cameraJointHandle,w_camera,simx_opmode_oneshot);


                //执行动作
//                T=wMc.getTranslationVector();//平移部分
//                R.buildFrom(wMc.getThetaUVector());//旋转部分
//                for(size_t i=0;i<3;++i)
//                {
//                    camPos[i]=static_cast<float>(T[i]);
//                    camEuler[i]=static_cast<float>(R[i]);
//                }
//                simxSetObjectPosition(clientID,camHandle,-1,camPos,simx_opmode_oneshot);
//                simxSetObjectOrientation(clientID,camHandle,-1,camEuler,simx_opmode_oneshot);
                //if ((task.getError()).sumSquare() < 0.001)
                //break;
            }
//            else
//            {
//                printf("No Marker Was Found!\n");
//            }
            if(DISP_IMAGE)
            {
                cv::imshow("Opencv image",oimage); //显示图片
            }
            extApi_sleepMs(10);//暂停10ms
            if(cv::waitKey(1)==27) //如果1ms内按下esc则推出
            {
                cv::destroyAllWindows();
                simxFinish(clientID);
                return 0;
            }

        }
    }
    cv::destroyAllWindows();
    simxFinish(clientID);
    return 0;
}
