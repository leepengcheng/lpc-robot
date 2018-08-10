--@@控制脚本-ADS通讯：
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)

local const=require("const")
local tools=require("tools")


function vrep_Uint32_Float32(uint32NumTable)
    local numBuffer=sim.packUInt32Table(uint32NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end

function vrep_Uint16x2_Float32(uint16NumTable)
    local numBuffer=sim.packUInt16Table(uint16NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end


--解析数据,返回ROBOT_STATUS对象
function unpackRobotStatus(readData)
    local buffer=sim.packFloatTable(readData)
    local buffer_args=string.sub(buffer,1,6)
    local buffer_errorcode=string.sub(buffer,7,8)
    local buffer_positions=string.sub(buffer,9)

    local d1=sim.unpackUInt8Table(buffer_args)
    local d2=sim.unpackUInt16Table(buffer_errorcode)
    local d3=sim.unpackFloatTable(buffer_positions)

    local robo_statu={}
    robo_statu.driverPowered=d1[1]; --//驱动是否上电
    robo_statu.eStopped=d1[2];  --//是否非正常停止(如碰撞)
    robo_statu.inError=d1[3];  --//是否发生错误
    robo_statu.inMotion=d1[4];  --//是否在运动
    robo_statu.motionPossible=d1[5]; --//是否可以运动
    robo_statu.mode=d1[6]; --//当前的模式 自动/手动/未知
    robo_statu.errorCode=d2[1];   --//Uint16(错误码：自定义)
    robo_statu.position=d3
    
    return robo_statu
end



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

--初始化ADS
function initADSConnection()
    local adsHasInit=false
    if const.COM.readAddr or const.COM.writeAddr then
        adsHasInit=simADS.create({192,168,6,90,1,1},"127.0.0.1",{192,168,6,90,2,1})
    end

    if const.COM.readAddr and adsHasInit then
        simADS.read(const.COM.readAddr,0,simADS_handle_open)    --open read Handle
    end

    if const.COM.writeAddr and adsHasInit then
        simADS.write(const.COM.writeAddr,{},simADS_handle_open) --open write handle 
    end
    return adsHasInit
end


--读取RobotStatus:包含关节位置/是否上电
function readRobotStatus(isprint)
    local isprint=false or isprint
    local readData=simADS.read(readAddr,9,simADS_handle_none)
    local status=nil
    if readData then
        status=unpackRobotStatus(readData) 
    end

    if isprint then
        print(stringFormatRobotStatus(status))
    end
    return status
end

 --写出轨迹到twincat
--@cmd：控制指令 START/STOP/PAUSE/NEW
--@pts:轨迹点数据,1维数组,最大容量7*100000 (twincat定义))
--@step：默认1ms
--@index：起始位,默认1
function writeTrajectory(cmd,pts,index,step)

    local step=step or 1
    local index=index or 1
    local size=#pts/7
    --限制最大值
    if size>=const.COM.PTS_NUMBER then
        size=const.COM.PTS_NUMBER
    end

    local data={}
    data[1]=vrep_Uint16x2_Float32({cmd,step})
    if size>0 then
        data[2]=vrep_Uint32_Float32({size})
        data[3]=vrep_Uint32_Float32({index})
    end

    
    for i=1,size do
        data[7*(i-1)+3+1]=pts[7*(i-1)+1]
        data[7*(i-1)+3+2]=pts[7*(i-1)+2]
        data[7*(i-1)+3+3]=pts[7*(i-1)+3]
        data[7*(i-1)+3+4]=pts[7*(i-1)+4]
        data[7*(i-1)+3+5]=pts[7*(i-1)+5]
        data[7*(i-1)+3+6]=pts[7*(i-1)+6]
        data[7*(i-1)+3+7]=pts[7*(i-1)+7]
    end

    simADS.write(const.COM.writeAddr,data,simADS_handle_none)
end

--新轨迹:发送新轨迹后暂停不运动
--@pts:轨迹点table,size=N*7
--@index:起始位置:默认0
--@step:时间间隔:默认1ms
function newTrajectory(pts,index,step)
    writeTrajectory(const.COM.TRAJ_CMD_NEW,pts,index,step)
end


--开始
function startTrajectory()
    writeTrajectory(const.COM.TRAJ_CMD_START,{})
end

--停止
function stopTrajectory()
    writeTrajectory(const.COM.TRAJ_CMD_STOP,{})
end

--暂停
function pauseTrajectory()
    writeTrajectory(const.COM.TRAJ_CMD_PAUSE,{})
end

function on_sub_trajcmd(packedData)
    local data=sim.unpackTable(packedData)
    print("ADS RECEIVE: "..data[1])
end

function sysCall_threadmain()
    -- adsHasInit=initADSConnection()
        -- --------读机器人状态
    -- if readAddr and adsHasInit  then
    --     readRobotStatus(true)
    -- end
    --#####BlueZero##################
    adsNode=simB0.create("adsNode")
    --发送传感器数据
    topicPubRoboStatus=simB0.createPublisher(adsNode,const.TOPICS.ROBOSTATUS)

    --接受轨迹命令 new|start|pause|stop,转换为ADS
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
