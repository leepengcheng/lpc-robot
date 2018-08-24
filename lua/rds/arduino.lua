function sysCall_threadmain()
    -- sim.setThreadSwitchTiming(3)
    serial=sim.serialOpen("/dev/ttyACM0",9600)
    sim.serialSend(serial,sim.packUInt8Table({100}))
    -- sim.wait(1.0)
    -- local ret=sim.serialCheck(serial)
    -- print("check result"..ret)
    -- sim.serialSend(serial,'D')
    -- local data=sim.serialRead(serial,100,true,"",5)
    -- if data then
    --     print("data: "..data)
    -- else
    --     print("data: nil")
    -- end
    sim.wait(1.0)
end
    
function sysCall_cleanup()
    sim.serialClose(serial)
end
