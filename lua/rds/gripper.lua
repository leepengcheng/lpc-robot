local gripper={}



gripper.open=function (t)
    local t=t or 1.0
    sim.setIntegerSignal('RG2_close',0)
    sim.wait(t)
end


gripper.close=function (t)
    local t=t or 1.0
    sim.setIntegerSignal('RG2_close',1)
    sim.wait(1.0)
end


return gripper