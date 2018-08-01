--添加工作路径
local workdir="/media/lee/workspace/GitWorkSpace/lpc-robot/lua/rds"
package.path=string.format( "%s;%s/?.lua",package.path,workdir)
local tools=require("tools")
local gripper=require("gripper")
local param=require("param")
local robot=require("robot")
local gripper=require("gripper")

function sysCall_threadmain()
    -- local ghandle=sim.getObjectHandle("Graph")
    
    local TOOLS=tools:new()

    local RDS=robot:new(param)
    RDS:setGripper(gripper)

    local objects = {
        sim.getObjectHandle("target1"),
        sim.getObjectHandle("target2"),
        sim.getObjectHandle("target3"),
        sim.getObjectHandle("target4")
    }

    for i=1,4 do
        RDS:moveObjectToRelativeTxyzRxyz(objects[i],{0,0,-0.1},nil,"IK")

        RDS:moveObjectToRelativeTxyzRxyz(objects[i],{0,0,-0.03},nil,"IK")

        RDS.gripper:close()

        -- RDS:moveObjectToRelativeTxyzRxyz(objects[1],{-0.2,-0.2,-0.10},nil,"IK")
        RDS:moveObjectToAbsTxyz(objects[i],{0.3-i*0.1,0.4,0.1},nil,"IK")

        RDS.gripper:open()
    end

    RDS:moveObjectToRelativeTxyzRxyz(objects[4],{0,0,-0.1},nil,"IK")
    RDS.gripper:close()
    RDS:moveObjectToRelativeTxyzRxyz(objects[4],{0.1,0,0},nil,"IK")
    RDS:moveObjectToRelativeTxyzRxyz(objects[4],{-0.06,0,0},nil,"IK")
    



end 