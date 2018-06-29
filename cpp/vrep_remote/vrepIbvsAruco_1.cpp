#include <iostream>
//Opencv
#include <opencv2/aruco.hpp>
#include <opencv/cv.hpp>
//Visp
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpConvert.h>


using namespace std;
extern "C" {
#include "extApi.h"
}


#define DISP_IMAGE  0  //是否显示图片
const char* BASE_NAME="UR5";//参考坐标系
const char* VREP_CAMERA="camera";//visp_rgb相机的名称
const char* VREP_OBJECT="aruco_box";//visp_box追踪的物体名称
const char* VREP_COMMANDER_SERVER="remoteApiCommandServer";//远程服务调用脚本
const char* TIP_NAME="tip"; //tip
const char* TARGET_NAME="target";//target
const float MIN_THREHOLD=0.0001; //收敛的最小阈值
int clientID=-1; //连接ID
int baseHandle=-1;//基座句柄
int camHandle=-1;//相机句柄
int tipHandle=-1;//末端的句柄
int targetHandle=-1;//目标的句柄

vpRotationMatrix target_rot;
vpTranslationVector target_xyz;

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

    vpHomogeneousMatrix Delta;
    //单个仿真步后的位姿变化量
    vpColVector v_dt = v * timeStep;
    Delta.buildFrom(v_dt[0],v_dt[1],v_dt[2],v_dt[3],v_dt[4],v_dt[5]);
    return Delta;
}

//远程调用:设置机器人的速度
void setRobotjointSpeed(vpColVector& v)
{


    simxInt inFloatCnt=6;
    simxFloat inFloats[6];
    for(int i=0;i<6;++i)
    {
        inFloats[i]=static_cast<float>(v[i]);
    }
    simxCallScriptFunction(clientID,VREP_COMMANDER_SERVER,sim_scripttype_childscript,"setRobotJointPosition",0,NULL,inFloatCnt,inFloats,0,NULL,0,NULL,NULL,NULL,
                           NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_oneshot_wait);
}


//远程调用:获取机器人的雅克比矩阵
vpMatrix getRobotJacobianMatrix()
{
    simxInt retFloatCnt;
    simxFloat* retFloats;
    int result=simxCallScriptFunction(clientID,VREP_COMMANDER_SERVER,sim_scripttype_childscript,"getJacobianfunction",0,NULL,0,NULL,0,NULL,0,NULL,NULL,NULL,
                                      &retFloatCnt,&retFloats,NULL,NULL,NULL,NULL,simx_opmode_blocking);
    vpMatrix mat(6,6);

    if (result==simx_return_ok){
        //for(unsigned int j=0;j<retFloatCnt;++j)
        //{
        //    std::cout<<retFloats[j]<<std::endl;
        //}
        for(unsigned int i=0;i<6;++i)
        {
            for (unsigned int j=0; j<6;++j)
            {
                mat[i][j]=retFloats[i+j*6];
            }
        }
    }
    return mat;
}

//获取当前末端和期望末端的差值
vpColVector getTipTargetDistance()
{

    float tipPos[3],tipEuler[3];
    simxGetObjectPosition(clientID,tipHandle,baseHandle,tipPos,simx_opmode_oneshot_wait);
    simxGetObjectOrientation(clientID,tipHandle,baseHandle,tipEuler ,simx_opmode_oneshot_wait);
    vpRxyzVector tipRxyz(tipEuler[0],tipEuler[1],tipEuler[2]);
    vpRotationMatrix f_M_tip(tipRxyz);//末端姿态:旋转向量
    vpTranslationVector t(tipPos[0],tipPos[1],tipPos[2]);//末端位置

    //dis_rot*f_M_tip=target_rot >>dis_rot=target_rot*f_M_tip.inverse()
    vpRotationMatrix dis_rot=target_rot*f_M_tip.inverse();//旋转差值:旋转矩阵

    //    dis_rot[0][0]=dis_rot[0][0]-1;
    //    dis_rot[1][1]=dis_rot[1][1]-1;
    //    dis_rot[2][2]=dis_rot[2][2]-1;

    //    std::cout<<"*****"<<"f_M_tip"<<"*****"<<std::endl;
    //    std::cout<<f_M_tip<<std::endl;

    //    std::cout<<"*****"<<"target_rot"<<"*****"<<std::endl;
    //    std::cout<<target_xyz.t()<<std::endl;



    vpTranslationVector dis_xyz=target_xyz-t;//位移差值

    vpThetaUVector wxyz(dis_rot);//旋转差值:旋转向量
    //    wxyz[0]=dis_rot[2][1]-dis_rot[1][2];
    //    wxyz[1]=dis_rot[0][2]-dis_rot[2][0];
    //    wxyz[2]=dis_rot[1][0]-dis_rot[0][1];
    //    wxyz=wxyz*0.5;

    vpColVector dis(6);
    dis[0]=dis_xyz[0];
    dis[1]=dis_xyz[1];
    dis[2]=dis_xyz[2];
    //    dis[3]=wxyz[0];
    //    dis[4]=wxyz[1];
    //    dis[5]=wxyz[2];
    return dis;

}

//更新目标的位置和姿态
void updateTargetMatrix()
{
    float tarPos[3],tarEuler[3];
    simxGetObjectPosition(clientID,targetHandle,baseHandle,tarPos,simx_opmode_oneshot_wait);
    simxGetObjectOrientation(clientID,targetHandle,baseHandle,tarEuler ,simx_opmode_oneshot_wait);
    vpRxyzVector TarRxyz(tarEuler[0],tarEuler[1],tarEuler[2]);
    target_rot.buildFrom(TarRxyz);
    target_xyz.buildFrom(tarPos[0],tarPos[1],tarPos[2]);
}



int main()
{

    simxFinish(-1); //关闭所有连接
    clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5); //打开连接
    if (clientID==-1)
    {
        printf("Can not Connected to remote API server\n");
        return 1;
    }
    printf("Successfully Connected to remote API server\n");


    simxGetObjectHandle(clientID,BASE_NAME,&baseHandle,simx_opmode_oneshot_wait);
    //获得相机句柄
    simxGetObjectHandle(clientID,VREP_CAMERA,&camHandle,simx_opmode_oneshot_wait);

    //获得tip句柄
    simxGetObjectHandle(clientID,TIP_NAME,&tipHandle,simx_opmode_oneshot_wait);

    //获得target句柄
    simxGetObjectHandle(clientID,TARGET_NAME,&targetHandle,simx_opmode_oneshot_wait);

    //获得物体的句柄
    int boxHandle=0;
    simxGetObjectHandle(clientID,VREP_OBJECT,&boxHandle,simx_opmode_oneshot_wait);

    std::cout<<baseHandle<<" "<<tipHandle<<" "<<targetHandle<<" "<<targetHandle;

    //获取关节的句柄
    char buffer[64];
    int jointHandles[6];
    for(int i=0;i<6;++i)
    {
        sprintf(buffer,"UR5_joint%d",i+1);

        simxGetObjectHandle(clientID,buffer,&jointHandles[i],simx_opmode_oneshot_wait);
    }

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
    printf("perspective Angle:%f\n",angle);

    //获得相机的初始位置和姿态
    float camPos[3];
    simxGetObjectPosition(clientID,camHandle,-1,camPos,simx_opmode_oneshot_wait);
    float camEuler[3];
    simxGetObjectOrientation(clientID,camHandle,-1,camEuler,simx_opmode_oneshot_wait);



    //vpMatrix m=getRobotJacobianMatrix();

    //#####################初始化伺服任务########################
    vpServo task;
    vpRxyzVector rxyz(camEuler[0],camEuler[1],camEuler[2]);
    vpThetaUVector camRvecs(rxyz);//转换为旋转向量
    vpHomogeneousMatrix wMc(camPos[0],camPos[1],camPos[2],camRvecs[0],camRvecs[1],camRvecs[2]);


    vpHomogeneousMatrix cdMo(0, 0, 0.5, vpMath::rad(180),0, 0);//终止时对象在相机中的位姿
    vpHomogeneousMatrix cMo(0, 0, 0.2, 0, vpMath::rad(180), 0);//初始时对象在相机坐标系中的位姿(初始化),下面重新赋值

    //虚拟特征点
    vector<vpPoint> points;
    points.push_back(vpPoint(-0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, 0.1, 0));
    points.push_back(vpPoint(-0.1, 0.1, 0));


    //伺服任务
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(1);
    //初始化特征点
    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
        points[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], points[i]);
        points[i].track(cMo);
        vpFeatureBuilder::create(p[i], points[i]);
        task.addFeature(p[i], pd[i]);
    }
#ifndef FAKE
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
#endif


    vpTranslationVector T;
    vpRxyzVector R;

    while (simxGetConnectionId(clientID)!=-1)
    {

#ifdef FAKE
        //获得box在相机坐标系下的位姿,并转换为vpMatrix
        float boxPos[3],boxEuler[3];
        simxGetObjectPosition(clientID,boxHandle,camHandle,boxPos,simx_opmode_oneshot_wait);
        simxGetObjectOrientation(clientID,boxHandle,camHandle,boxEuler ,simx_opmode_oneshot_wait);
        vpRxyzVector boxRxyz(boxEuler[0],boxEuler[1],boxEuler[2]);
        vpThetaUVector boxRvecs(boxRxyz);//转换为旋转向量
        cMo.buildFrom(boxPos[0],boxPos[1],boxPos[2],boxRvecs[0],boxRvecs[1],boxRvecs[2]);

        //获得tip在baseHandle坐标系下的位姿,并转换为vpMatrix
        float tipPos[3],tipEuler[3];
        simxGetObjectPosition(clientID,tipHandle,baseHandle,tipPos,simx_opmode_oneshot_wait);
        simxGetObjectOrientation(clientID,tipHandle,baseHandle,tipEuler ,simx_opmode_oneshot_wait);
        vpRxyzVector tipRxyz(tipEuler[0],tipEuler[1],tipEuler[2]);
        vpRotationMatrix f_M_e(tipRxyz);//末端的姿态:旋转矩阵
        //        vpVelocityTwistMatrix f_V_e(f_M_e);


        //更新目标的位置和姿态
        updateTargetMatrix();

#else
        if(simxGetVisionSensorImage(clientID, camHandle,resolution,&vimage,0, simx_opmode_buffer)==simx_return_ok)
        {
            cv::Mat oimage(resolution[1],resolution[0],CV_8UC3,vimage);//转换为MAT
            cv::flip(oimage,oimage,0);//水平翻转
            cv::cvtColor(oimage,oimage,cv::COLOR_BGR2RGB);//转换为RGB
            //检测marker及姿态
            vector<cv::Vec3d> rvecs,tvecs;
            bool hasMarker=getMarkerPoseImage(oimage,markersDict,K,D,rvecs,tvecs);
            //绘制轴
            if(!hasMarker)
            {
                printf("No Marker Was Found!\n");
                continue;
            }
            /*多个Marker*/
            //for(size_t i=0;i<rvecs.size();++i)
            //{
            //    cv::aruco::drawAxis(oimage,K,D,rvecs[i],tvecs[i],0.1);
            //}
            /*单个marker*/
            cv::aruco::drawAxis(oimage,K,D,rvecs[0],tvecs[0],0.1);
            cMo.buildFrom(tvecs[0][0],tvecs[0][1],tvecs[0][2],rvecs[0][0],rvecs[0][1],rvecs[0][2]);
            if(DISP_IMAGE)
            {
                cv::imshow("Opencv image",oimage); //显示图片
            }

            if(cv::waitKey(1)==27) //如果1ms内按下esc则推出
            {
                cv::destroyAllWindows();
                simxFinish(clientID);
                return 0;
            }
        }
#endif
        try{
            //更新特征点
            //            for (unsigned int i = 0; i < 4; i++) {
            //                points[i].track(cMo);
            //                vpFeatureBuilder::create(p[i], points[i]);
            //            }
            //            vpColVector v = task.computeControlLaw();
            //            wMc=wMc*getStepDelta(v,timeStep);//更新wMc
            //            T=wMc.getTranslationVector();//平移部分
            //            R.buildFrom(wMc.getThetaUVector());//旋转部分
            //            for(size_t i=0;i<3;++i)
            //            {
            //                camPos[i]=static_cast<float>(T[i]);
            //                camEuler[i]=static_cast<float>(R[i]);
            //            }


            //            std::vector<float> pos;
            //            float val=0;
            //            for(int i=0;i<6;++i)
            //            {
            //                simxGetJointPosition(clientID,jointHandles[i],&val,simx_opmode_oneshot_wait);
            //                pos.push_back(val);
            //                //                std::cout<<val/3.14159*180<<std::endl;
            //            }
            //            vpColVector pos_(pos);

            //f_J_e=f_V_e*e_J_e
            //e_J_e=f_V_e_p*f_J_e
            //e_J_e_p=f_J_e_p*f_V_e
            //theta=e_j_e_p*v=f_J_e_p*f_V_e*v
            vpMatrix fJe=getRobotJacobianMatrix();//获取机械臂的雅克比矩阵
            vpColVector dis_tip=getTipTargetDistance();

            //            std::cout<<"Error:"<<dis_tip.t()<<std::endl;
            //            if(dis_tip.sumSquare()<0.01)
            //            {
            //                std::cout<<"sumSquare:"<<dis_tip.sumSquare()<<std::endl;
            //                continue;
            //            }
            vpMatrix imJ1t, imJ1,fJe_p;
            vpColVector sv;
            unsigned int rankJ1 = fJe.pseudoInverse(fJe_p, sv, 1e-6, imJ1, imJ1t);
            vpColVector theta=fJe_p*dis_tip;
            //            std::cout<<fJe*pos_<<std::endl;
            //            vpMatrix fJe_p=fJe.pseudoInverse();
            //            vpColVector theta=fJe_p*f_V_e*v*0.05;


            std::cout<<fJe<<std::endl;
            std::cout<<"--------------------"<<std::endl;
            std::cout<<fJe.pseudoInverse()<<std::endl;
            std::cout<<"###################"<<std::endl;
            std::cout<<theta.t()<<std::endl;
            setRobotjointSpeed(theta);

        }
        catch(exception &e)
        {
            std::cout<<e.what()<<std::endl;
        }
        //        simxSetObjectPosition(clientID,camHandle,-1,camPos,simx_opmode_oneshot);
        //        simxSetObjectOrientation(clientID,camHandle,-1,camEuler,simx_opmode_oneshot);
        //        if ((task.getError()).sumSquare() < MIN_THREHOLD)
        //        {
        //            std::cout<<"Task Finished"<<std::endl;
        //            continue;
        //        }
        //        extApi_sleepMs(100);//暂停10ms
    }
#ifndef FAKE
    cv::destroyAllWindows();
#endif
    simxFinish(clientID);
    return 0;
}
