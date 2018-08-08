--@@控制脚本-多线程：主程序
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)
local tools=require("tools")
local gripper=require("gripper")
local param=require("param")
local robot=require("robot")
local gripper=require("gripper")
local const=require("const")

function sysCall_threadmain()
    -- local ghandle=sim.getObjectHandle("Graph")
    
    local TOOLS=tools:new()
    local RDS=robot:new(param)
    TOOLS:setRobot(RDS)

    gripper:setWaitTime(1.0) --设置gripper的等待时间

    RDS:setGripper(gripper) --设置机械臂的gripper

    local objects = {
        sim.getObjectHandle("target1"),
        sim.getObjectHandle("target2"),
        sim.getObjectHandle("target3"),
        sim.getObjectHandle("target4")
    }

    for i=1,4 do
        local path1=RDS:moveObjectToRelativeTxyzRxyz(objects[i],{0,0,-0.1},nil,nil,"IK")

        local path2=RDS:moveObjectToRelativeTxyzRxyz(objects[i],{0,0,-0.03},nil,const.action.close,"IK")

        local path3=RDS:moveObjectToAbsTxyz(objects[i],{0.3-i*0.1,0.4,0.1},const.action.open,"IK")

        local path=TOOLS:tableConcat(path1,path2,path3)

        TOOLS:visualizePath(path,false)

        sim.wait(2.0)
    end

    -- RDS:moveObjectToRelativeTxyzRxyz(objects[4],{0,0,-0.1},nil,const.action.close,"IK")
    -- RDS:moveObjectToRelativeTxyzRxyz(objects[4],{0.1,0,0},nil,nil,"IK")
    -- RDS:moveObjectToRelativeTxyzRxyz(objects[4],{-0.06,0,0},nil,nil,"IK")
end 