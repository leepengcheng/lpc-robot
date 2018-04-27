-- function include(absolutePathAndFile)
--     if not __notFirst__ then
--         __notFirst__=true
--         __scriptCodeToRun__=assert(loadfile(absolutePathAndFile))
--     end
--     if __scriptCodeToRun__ then
--         __scriptCodeToRun__()
--     end
-- end
-- include('/media/zen/workspace/GitWorkSpace/vrep-control/lua/vrep_robot.lua')

--收到路径规划的轨迹
-- function trajectory_callback(msg)
--     local jpos=msg['position']
--     --设置机械臂关节位置
--     for i=1,6,1 do
--         simSetJointTargetPosition(robotHandles[i],jpos[i])
--         -- simSetJointPosition(robotHandles[i],jpos[i])
--     end

--     --设置手指关节位置
--     for j=1,8,1 do
--         simSetJointTargetPosition(bhandHandles[j],jpos[j+6])
--     end
        
-- end

function robot_state_callback(msg)
    local jpos=msg['position']
    --设置机械臂关节位置
    for i=1,6,1 do
        simSetJointTargetPosition(robotHandles[i],jpos[i])
        -- simSetJointPosition(robotHandles[i],jpos[i])
    end

    --设置手指关节位置
    for j=1,8,1 do
        simSetJointTargetPosition(bhandHandles[j],jpos[j+6])
    end
        
end


if (sim_call_type==sim_childscriptcall_initialization) then
    -- 获取句柄
    --camSensor=simGetObjectHandle('Vision_sensor')
    robotHandles={-1,-1,-1,-1,-1,-1} --机械臂6关节句柄
    for i=1,6,1 do
        robotHandles[i]=simGetObjectHandle("UR5_joint"..i)
    end
    -- Enable an image publisher and subscriber:
    --pub=simExtRosInterface_advertise('/rgbd_camera/depth/image_raw', 'sensor_msgs/Image')
    --simExtRosInterface_publisherTreatUInt8ArrayAsString(pub) -- lua中发送 strings 比table快很多

    --ros 关节数组名称
    -- ros_joint_names={'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
    -- 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint', 'bh_j32_joint', 
    -- 'bh_j33_joint', 'bh_j11_joint', 'bh_j12_joint', 'bh_j13_joint',
    -- 'bh_j21_joint', 'bh_j22_joint', 'bh_j23_joint'}

    -- --robot state message 初始化
    -- state={}
    -- state['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id=" "}
    -- state['name']=ros_joint_names
    -- state['position']={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    -- state['velocity']={}
    -- state['effort']={}

    -- --
    -- feedback={}
    -- traj_pt={positions={},velocities={},accelerations={},effort={},duration=0.0}
    -- feedback['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id=" "}
    -- feedback['joint_names']=ros_joint_names
    -- feedback['desired']=traj_pt
    -- feedback['error']=traj_pt
    -- feedback['actual']=traj_pt

    --ros joint名称和vrep关节名称对应的关系
    --{bh_j32_joint, bh_j33_joint},{bh_j11_joint, bh_j12_joint, bh_j13_joint},{bh_j21_joint, bh_j22_joint, bh_j23_joint}
    --1号手指：B_1 ,C_1                         2号手指:A_2   B_2  C_2                     0号手指：A_0  B_0  C_0
    bhandHandles={-1,-1,-1,-1,-1,-1,-1,-1} --BarrettHand爪子7关节句柄
    bhandHandles[1]=simGetObjectHandle('BarrettHand_jointB_1')
    bhandHandles[2]=simGetObjectHandle('BarrettHand_jointC_1')
    bhandHandles[3]=simGetObjectHandle('BarrettHand_jointA_2')
    bhandHandles[4]=simGetObjectHandle('BarrettHand_jointB_2')
    bhandHandles[5]=simGetObjectHandle('BarrettHand_jointC_2')
    bhandHandles[6]=simGetObjectHandle('BarrettHand_jointA_0')
    bhandHandles[7]=simGetObjectHandle('BarrettHand_jointB_0')
    bhandHandles[8]=simGetObjectHandle('BarrettHand_jointC_0')

    --------------------------消息订阅-----------------------------------
    --订阅joint_states主题，jointState_callback中执行动作
    -- traj_sub=simExtRosInterface_subscribe('/joint_states', 'trajectory_msgs/JointTrajectory', 'trajectory_callback')
    -- simExtRosInterface_subscriberTreatUInt8ArrayAsString(sub)

    robo_state_sub=simExtRosInterface_subscribe('/joint_states', 'sensor_msgs/JointState','robot_state_callback')
    simExtRosInterface_subscriberTreatUInt8ArrayAsString(robo_state_sub)

    ----------------------------消息发布------------------------------------------------------
    --消息关节位置
    -- robo_state_pub=simExtRosInterface_advertise('/joint_states', 'sensor_msgs/JointState')
    -- simExtRosInterface_publisherTreatUInt8ArrayAsString(robo_state_pub)

    -- --发送机器人的状态
    -- robo_status_pub=simExtRosInterface_advertise('/robot_status', 'industrial_msgs/RobotStatus')
    -- simExtRosInterface_publisherTreatUInt8ArrayAsString(robo_status_pub)


    -- --发送关节状态
    -- robo_feedback_pub=simExtRosInterface_advertise('/feedback_states', 'control_msgs/FollowJointTrajectoryFeedback')
    -- simExtRosInterface_publisherTreatUInt8ArrayAsString(robo_feedback_pub)

end



-- function getRobotPosition()
--     local jpos={0,0,0,0,0,0,0,0,0,0,0,0,0,0}
--     --
--     for i=1,6,1 do
--         jpos[i]=simGetJointPosition(robotHandles[i])
--     end

--     --设置手指关节位置
--     for j=1,8,1 do
--         jpos[6+j]=simGetJointPosition(bhandHandles[j])
--     end
--     return jpos
-- end



-- --发送信息
-- if (sim_call_type==sim_childscriptcall_sensing) then
    -- Publish the image of the active vision sensor:
    -- local data,w,h=simGetVisionSensorCharImage(camSensor)
    -- d={}
    -- d['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="camera"}
    -- d['height']=h
    -- d['width']=w
    -- d['encoding']='32FC1'
    -- d['is_bigendian']=0
    -- d['step']=w*4
    -- d['data']=data
    -- simExtRosInterface_publish(pub,d)
--     local t=simExtRosInterface_getTime()

--     state['position']=getRobotPosition()
--     state['header']['stamp']=t
--     simExtRosInterface_publish(robo_state_pub,state)

--     feedback['header']['stamp']=t
--     feedback['actual']['positions']=getRobotPosition()

--     simExtRosInterface_publish(robo_feedback_pub,feedback)
-- end

--关闭节点
if (sim_call_type==sim_childscriptcall_cleanup) then
    simExtRosInterface_shutdownSubscriber(robo_state_sub)
    -- simExtRosInterface_shutdownPublisher(robo_state_pub)
    -- simExtRosInterface_shutdownPublisher(robo_feedback_pub)
end