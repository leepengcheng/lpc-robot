--添加工作路径
local workdir="/media/zen/workspace/GitWorkSpace/lpc-robot/lua/rds"
package.path=string.format( "%s;%s/?.lua",package.path,workdir)
local common=require("common")
local gripper=require("gripper")
local param=require("param")
local robot=require("robot")


function sysCall_threadmain()
    -- local ghandle=sim.getObjectHandle("Graph")
    
    local RDS=robot:new(param)
    common:clearPath(RDS.robotHandle) --清除上次生成的路径
    local pickTargets = {
        sim.getObjectHandle("target1"),
        sim.getObjectHandle("target2"),
        sim.getObjectHandle("target3"),
    }

    local targetMatrix = common:getTranslatedMatrix(pickTargets[1], {0, 0, -0.1})
    RDS:replaningAndExcuteMotion(targetMatrix)

end 