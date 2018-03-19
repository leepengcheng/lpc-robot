function include(absolutePathAndFile)
    if not __notFirst__ then
        __notFirst__=true
        __scriptCodeToRun__=assert(loadfile(absolutePathAndFile))
    end
    if __scriptCodeToRun__ then
        __scriptCodeToRun__()
    end
end
include('/media/zen/workspace/GitWorkSpace/vrep-control/lua/vrep_robot.lua')



-- simExtRosInterface_imageTransportAdvertise
-- simExtRosInterface_imageTransportPublish
-- simExtRosInterface_imageTransportShutdownPublisher
-- simExtRosInterface_imageTransportShutdownSubscriber
-- simExtRosInterface_imageTransportSubscribe


function jointState_callback(msg)
    local jpos=msg['position']
    --设置机械臂关节位置
    for i=1,6,1 do
        simSetJointTargetPosition(robotHandles[i],jpos[i])
    end

    --设置手指关节位置
    -- for j=1,8,1 do
    --     simSetJointTargetPosition(bhandHandles[j],jpos[j+6])
    -- end
        
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


    --{bh_j32_joint, bh_j33_joint},{bh_j11_joint, bh_j12_joint, bh_j13_joint},{bh_j21_joint, bh_j22_joint, bh_j23_joint}
    --1号手指：B_1 ,C_1                         2号手指:A_2   B_2  C_2                     0号手指：A_0  B_0  C_0
    -- bhandHandles={-1,-1,-1,-1,-1,-1,-1,-1} --BarrettHand爪子7关节句柄
    -- bhandHandles[1]=simGetObjectHandle('BarrettHand_jointB_1')
    -- bhandHandles[2]=simGetObjectHandle('BarrettHand_jointC_1')
    -- bhandHandles[3]=simGetObjectHandle('BarrettHand_jointA_2')
    -- bhandHandles[4]=simGetObjectHandle('BarrettHand_jointB_2')
    -- bhandHandles[5]=simGetObjectHandle('BarrettHand_jointC_2')
    -- bhandHandles[6]=simGetObjectHandle('BarrettHand_jointA_0')
    -- bhandHandles[7]=simGetObjectHandle('BarrettHand_jointB_0')
    -- bhandHandles[8]=simGetObjectHandle('BarrettHand_jointC_0')

    --订阅joint_states主题，jointState_callback中执行动作
    sub=simExtRosInterface_subscribe('/joint_states', 'sensor_msgs/JointState', 'jointState_callback')
    simExtRosInterface_subscriberTreatUInt8ArrayAsString(sub) 
end



-- --发送图片
-- if (sim_call_type==sim_childscriptcall_sensing) then
--     -- Publish the image of the active vision sensor:
--     local data,w,h=simGetVisionSensorCharImage(camSensor)
--     d={}
--     d['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="camera"}
--     d['height']=h
--     d['width']=w
--     d['encoding']='32FC1'
--     d['is_bigendian']=0
--     d['step']=w*4
--     d['data']=data
--     simExtRosInterface_publish(pub,d)
-- end

--关闭节点
if (sim_call_type==sim_childscriptcall_cleanup) then
    --simExtRosInterface_shutdownPublisher(pub)
    simExtRosInterface_shutdownSubscriber(sub)
end