--@@控制脚本-多线程：主程序
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)
local tools=require("tools")
local gripper=require("gripper")
local param=require("param")
local robot=require("robot")
local gripper=require("gripper")
local const=require("const")



function on_sub_plancmd(packedData)
    local data=sim.unpackTable(packedData)
    local status,objHandle=pcall(sim.getObjectHandle,data[1])
    local i=data[3]
    if not status then
        print("目标对象不存在")
        return
    end
    local config=RDS:getConfig() --保存初始位 
    local path1=RDS:moveObjectToRelativeTxyzRxyz(objHandle,{0,0,-0.1},nil,nil,"IK")

    local path2=RDS:moveObjectToRelativeTxyzRxyz(objHandle,{0,0,-0.03},nil,const.action.close,"IK")

    local path3=RDS:moveObjectToAbsTxyz(objHandle,{0.3-i*0.1,0.4,0.1},const.action.open,"IK")

    local path=TOOLS:tableConcat(path1,path2,path3)

    TOOLS:visualizePath(path,false)
    local msgTable={obj=data[1],data=path}
    local msg=sim.packTable(msgTable)
    simB0.publish(topicPubPlanedpath,msg)
    -- TOOLS:writeInfo(const.PATHNAME,msgTable,sim.getObjectHandle("path"))
    -- RDS:setConfig(config)  --返回初始位
    sim.wait(2.0)
end

function sysCall_threadmain()
    TOOLS=tools:new()
    RDS=robot:new(param)
    TOOLS:setRobot(RDS)

    gripper:setWaitTime(1.0) --设置gripper的等待时间

    RDS:setGripper(gripper) --设置机械臂的gripper

    --#####BlueZero##################
    nodePlan=simB0.create("planNode")
    topicSubPlanCmd=simB0.createSubscriber(nodePlan,const.TOPICS.PLANCMD,'on_sub_plancmd')
    topicPubPlanedpath=simB0.createPublisher(nodePlan,const.TOPICS.PLANEDPATH)
    simB0.init(nodePlan)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        simB0.spinOnce(nodePlan)
        sim.switchThread() -- resume in next simulation step
    end
end 


--停止
function sysCall_cleanup()

    if nodePlan then
        simB0.cleanup(nodePlan)
        if topicSubPlanCmd then
            simB0.destroySubscriber(topicSubPlanCmd)
        end
        if topicPubPlanedpath then
            simB0.destroyPublisher(topicPubPlanedpath)
        end
        simB0.destroy(nodePlan)
    end

end