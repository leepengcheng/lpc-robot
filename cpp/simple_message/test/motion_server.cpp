#include "simple_message/simple_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/shared_types.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/udp_client.h"
#include "simple_message/socket/udp_server.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/ping_message.h"
#include "simple_message/ping_handler.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_manager.h"
#include "simple_message/simple_comms_fault_handler.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/joint_traj.h"

#include <thread> //多线程
#include <atomic>
#include <chrono>
using namespace industrial::simple_message;
using namespace industrial::byte_array;
using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::udp_socket;
using namespace industrial::udp_client;
using namespace industrial::udp_server;
using namespace industrial::tcp_socket;
using namespace industrial::tcp_client;
using namespace industrial::tcp_server;
using namespace industrial::ping_message;
using namespace industrial::ping_handler;
using namespace industrial::joint_data;
//using namespace industrial::joint_message;
using namespace industrial::message_manager;
using namespace industrial::simple_comms_fault_handler;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;

#ifdef LOG
#include <iostream>
#define INFO_LOG(msg) std::cout  << "[INFO ]  " << (msg) << std::endl
#define ERROR_LOG(msg) std::cout << "[ERROR]  " << (msg) << std::endl
#define WARN_LOG(msg) std::cout  << "[WARN]   " << (msg) << std::endl
#else
#define INFO_LOG(msg)
#define ERROR_LOG(msg)
#endif


#define TEST_PORT_BASE 8080 //端口号
#define MAX_TRAJ_LENGTH 200 //最多200个轨迹点

JointTrajPt recTraj[MAX_TRAJ_LENGTH];//定义的轨迹点数组
JointTrajPt exeTraj[MAX_TRAJ_LENGTH];//定义的轨迹点数组
int         trajectory_size=0;//定义轨迹点的个数

std::atomic<bool>  excuteFlag(true);//执行轨迹
std::atomic<bool>  addPointFlag(true);//添加点



//清空轨迹
void clearTrajectory()
{

}

//将接收到的轨迹点存入轨迹数组
void addTrajectoryPoint(JointTrajPtMessage &jptmsg)
{
    if(trajectory_size>=MAX_TRAJ_LENGTH)
    {
        WARN_LOG("Too Many Trajectory Points,Trajectory has already reached its maximum size");
    }
    else
    {
        recTraj[trajectory_size]=jptmsg.point_;
        ++trajectory_size;
    }
}

void stopMotion()
{

}

void initMotionTrajectory()
{

}


//执行轨迹
void excuteMotionTrajectory()
{
    while(true)
    {
        if(excuteFlag.load())
        {
            INFO_LOG("Excute Motion...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));//进行操作
        }
    }
}



int main(int argc, char **argv)
{
    const int tcpPort = TEST_PORT_BASE;
    char ipAddr[] = "127.0.0.1";
    // 构建server
    INFO_LOG("Motion Server Start Initialize");
    TcpServer tcpServer;
    if (!tcpServer.init(tcpPort))
    {
        ERROR_LOG("Motion Server Initialize Failed!");
        return 1;
    }
    else
    {
        INFO_LOG("Motion Server Initialize Successfully");
    }

    while (!tcpServer.makeConnect())
    {
        INFO_LOG("Motion Server No Client Connect");
    }
    INFO_LOG("Motion Server Client Connected");

    SimpleMessage msgRecv,msgSend;        //接收到的二进制数据
    JointTrajPtMessage trajPtMsg; //解析为轨迹点消息
    JointTrajPt trajPt;           //解析为轨迹点结构体
    JointData jData;              //关节位置/速度数据


    while (tcpServer.receiveMsg(msgRecv))
    {
        trajPtMsg.init(msgRecv);
        switch (trajPtMsg.point_.getSequence())
        {
        case SpecialSeqValues::START_TRAJECTORY_DOWNLOAD:
            INFO_LOG("TRAJECTORY START");
            break;
        case SpecialSeqValues::END_TRAJECTORY:
            INFO_LOG("TRAJECTORY END");
            break;
        case SpecialSeqValues::STOP_TRAJECTORY:
            trajectory_size=0;
            INFO_LOG("TRAJECTORY STOP");
            break;
        }
        if(trajPtMsg.getCommType()==CommTypes::SERVICE_REQUEST)
        {
            //发送接收成功的消息
            msgSend.init(StandardMsgTypes::JOINT_TRAJ_PT,CommTypes::SERVICE_REPLY,ReplyTypes::SUCCESS);
        }
        INFO_LOG("*************************");
    }
}
