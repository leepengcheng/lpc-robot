--@@控制脚本-机器人状态采集：
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)
local const=require("const")
local tools=require("tools")

--读取RobotStatus:包含关节位置/是否上电
function readRobotStatus()
    local readData=simADS.read("MAIN.robo_status",9,simADS_handle_none)
    local status=nil
    if readData then
        status=tools:unpackRobotStatus(readData) 
    end
    return status
end


function sysCall_init()
    isReadStatus=true
    uploadNode=simB0.create("uploadNode")
    --Publc_Topic发送传感器数据
    topicPubRobostates=simB0.createPublisher(uploadNode,const.TOPICS.ROBOSTATES)
    simB0.init(uploadNode)
end

-- function sysCall_actuation()
-- end

function sysCall_sensing()
    if isReadStatus then
        local msgTable=readRobotStatus()
        if msgTable then
            local msg=sim.packTable(msgTable)
            if uploadNode then
                simB0.publish(topicPubRobostates,msg)
            end
        end
    end
end


function sysCall_cleanup()
    if uploadNode then
        simB0.cleanup(uploadNode)
        if topicPubRobostates then
            simB0.destroyPublisher(topicPubRobostates)
        end
        simB0.destroy(uploadNode)
    end
end

