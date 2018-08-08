--@@机械爪控制

local const=require("const")
local gripper={}

--等待时间(s)
gripper.waitTime=1.0

gripper.setWaitTime=function (self,t)
    self.waitTime=t or 1.0
end


gripper.open=function (self,isWait)
    isWait=isWait or true --默认等待
    sim.setIntegerSignal('RG2_close',const.action.open)
    if isWait then
        sim.wait(self.waitTime)
    end
end


gripper.close=function (self,isWait)
    isWait=isWait or true --默认等待
    sim.setIntegerSignal('RG2_close',const.action.close)
    if isWait then
        sim.wait(self.waitTime)
    end
end

gripper.openClose=function(self,action,isWait)
    isWait=isWait or true --默认等待
    action=action or const.action.none
    if action==const.action.none then
        return
    else
        if action==const.action.open or action==const.action.close then
            sim.setIntegerSignal('RG2_close',action)
            if isWait then
                sim.wait(self.waitTime)
            end
        else
            print("Wrong Gripper Command")
        end
        
    end
end

return gripper