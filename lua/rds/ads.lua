--@@控制脚本-ADS通讯：
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)

local const=require("const")
local tools=require("tools")







---格式化输出robotStatus
function stringFormatRobotStatus(status)
    local s1="驱动上电:%s"
    if status.driverPowered==const.COM.ROBO_STATUS_TRUE then
        s1=string.format(s1,"YES")
    else
        s1=string.format(s1,"NO")
    end

    local s2="非正常停止:%s"
    if status.eStopped==const.COM.ROBO_STATUS_TRUE then
        s2=string.format(s2,"YES")
    else
        s2=string.format(s2,"NO")
    end

    local s3="发生错误:%s"
    if status.inError==const.COM.ROBO_STATUS_TRUE then
        s3=string.format(s3,"YES")
    else
        s3=string.format(s3,"NO")
    end

    local s4="正在运动:%s"
    if status.inMotion==const.COM.ROBO_STATUS_TRUE then
        s4=string.format(s4,"YES")
    else
        s4=string.format(s4,"NO")
    end

    local s5="能否运动:%s"
    if status.motionPossible==const.COM.ROBO_STATUS_TRUE then
        s5=string.format(s5,"YES")
    else
        s5=string.format(s5,"NO")
    end

    s5=string.format("错误码:%d",status.errorCode)

    s6=string.format("关节位置:%f %f %f %f %f %f %f",status.position[1],
    status.position[2],status.position[3],
    status.position[4],status.position[5],
    status.position[6],status.position[7])
    s=string.format("%s %s %s %s %s %s\n",s1,s2,s3,s4,s5,s6)
    return s

end







--读取RobotStatus:包含关节位置/是否上电
function readRobotStatus(isprint)
    local isprint=false or isprint
    local readData=simADS.read(readAddr,9,simADS_handle_none)
    local status=nil
    if readData then
        status=tools:unpackRobotStatus(readData) 
    end
    if isprint then
        print(stringFormatRobotStatus(status))
    end
    return status
end



--接受发布轨迹的命令
function on_sub_trajcmd(packedData)
    print("正在转发ADS数据到控制器")
    local data=sim.unpackTable(packedData)
    -- simADS.write(writeAddr,data,simADS_handle_none)
end


function sysCall_threadmain()
    -- adsHasInit=initADSConnection()
        -- --------读机器人状态
    -- if readAddr and adsHasInit  then
    --     readRobotStatus(true)
    -- end
    --#####BlueZero##################
    adsNode=simB0.create("adsNode")
    --Publc_Topic发送传感器数据
    topicPubRoboStatus=simB0.createPublisher(adsNode,const.TOPICS.ROBOSTATUS)
    --sub_Topic接受轨迹命令 new|start|pause|stop,转换为ADS
    topicSubTrajCmd=simB0.createSubscriber(adsNode,const.TOPICS.TRAJCMD,'on_sub_trajcmd')
    simB0.init(adsNode)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        simB0.spinOnce(adsNode)
        sim.switchThread() -- resume in next simulation step
    end
end


--停止
function sysCall_cleanup()

        -- readAddr=READ_ADDR_BACKUP --销毁时需要回原地址

    -- if writeAddr and adsHasInit then
    --     simADS.write(writeAddr,{},simADS_handle_close)  --close write Handle
    -- end


    -- if readAddr and adsHasInit then
    --     simADS.read(readAddr,0,simADS_handle_close)     --close read Handle
    -- end
    
    -- if adsHasInit then
    --     simADS.destory()
    --     adsHasInit=false
    -- end
    if adsNode then
        simB0.cleanup(adsNode)
        if topicSubTrajCmd then
            simB0.destroySubscriber(topicSubTrajCmd)
        end
        if topicPubRoboStatus then
            simB0.destroyPublisher(topicPubRoboStatus)
        end
        simB0.destroy(adsNode)
    end

end
