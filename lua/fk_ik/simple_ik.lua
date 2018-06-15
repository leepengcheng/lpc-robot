[[
    IK算法测试
]]
if (sim_call_type==sim_childscriptcall_initialization) then
    ui=simGetUIHandle('UI')
    J1_handle = simGetObjectHandle('j1')
    J2_handle = simGetObjectHandle('j2')
    target_handle = simGetObjectHandle('target')
    
    --link length
    L1 = 0.5
    L2 = 0.5

    gamma = 1       --step size
    stol = 1e-2     --tolerance
    nm = 100        --initial error
    count = 0       --iteration count
    ilimit = 1000   --maximum iteratio
    
    --initial joint value
    q1 = 0   
    q2 = 0
end


if (sim_call_type==sim_childscriptcall_actuation) then
    local ui=simGetUIEventButton(ui)
    local target_pos = simGetObjectPosition(target_handle, -1)

    if(nm > stol) then
        

        local x, y = L1*math.cos(q1)+L2*math.cos(q1+q2), L1*math.sin(q1)+L2*math.sin(q1+q2)
        local delta_x, delta_y = target_pos[1] - x, target_pos[2] - y
        local dq_1 = (-L1*math.sin(q1)-L2*math.sin(q1+q2))*delta_x + (L1*math.cos(q1)+L2*math.cos(q1+q2))*delta_y
        local dq_2 = (-L2*math.sin(q1+q2))*delta_x + (L2*math.cos(q1+q2))*delta_y
        q1, q2 = q1 + gamma*dq_1,  q2 + gamma*dq_2

        nm = math.sqrt(delta_x * delta_x + delta_y * delta_y)

        count = count + 1
        if count > ilimit   then
            simAddStatusbarMessage("Solution wouldn't converge\r\n")
        end
        simAddStatusbarMessage( string.format("iterations:%d     err:%.4f     q1:%.2f     q2:%.2f     x:%.2f     y:%.2f ",count,nm,q1*180/math.pi,q2*180/math.pi,x,y))
    end

    -- if the button(a is the button handle) is pressed
    if ui==1 then
        simSetJointPosition(J1_handle, q1+math.pi/2)  -- the angle between L1 and X-axis is 90 degree
        simSetJointPosition(J2_handle, q2)      
    end
end