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
#include <iostream>

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
using namespace industrial::joint_message;
using namespace industrial::message_manager;
using namespace industrial::simple_comms_fault_handler;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;


#define TEST_PORT_BASE 8080

int main(int argc, char **argv)
{
    std::cout<<"*************************"<<std::endl;
    const int tcpPort = TEST_PORT_BASE;
    char ipAddr[] = "127.0.0.1";
    SimpleMessage msgRecv;

    // Construct server
    TcpServer tcpServer;
    tcpServer.init(tcpPort);
    while (!tcpServer.makeConnect()) {
        std::cout<<"No client";
//        sleep(2);
    }
    std::cout<<"Client Connected..."<<std::endl;

    //虚拟数据
    JointTrajPtMessage jointRecv;
    JointTrajPt  posRecv;
    JointData data_recv;

    while(tcpServer.receiveMsg(msgRecv))
    {
        jointRecv.init(msgRecv);
        posRecv.copyFrom(jointRecv.point_);
        posRecv.getJointPosition(data_recv);
        int JOINT_NUM=data_recv.getMaxNumJoints();
        for(int i=0;i<JOINT_NUM;++i)
        {
            std::cout<<data_recv.getJoint(i)<<std::endl;
        }
        std::cout<<"*************************"<<std::endl;
    }
}

