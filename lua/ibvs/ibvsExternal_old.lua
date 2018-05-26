
if (sim_call_type==sim_childscriptcall_initialization) then
    
    D={}
    local file=io.open("/media/zen/document/visp-master/tutorial/build/visual-servo/ibvs/output.txt","r")
    for line in file:lines() do
        table.insert(D,tonumber(line))
    end
    file:close()
    
    camHandle=simGetObjectHandle("Vision_sensor")
    objHandle=simGetObjectHandle("objFrame")
    objPos={D[1],D[2],D[3]}
    objOri={D[4],D[5],D[6]}
    -- 设置对象的初始位姿态
    simSetObjectPosition(objHandle,-1,objPos)
    simSetObjectOrientation(objHandle,-1,objOri)
    -- 设置特征点的初始位姿
    markerHandles={-1,-1,-1}
    for i=1,4,1 do
        markerHandles[i]=simGetObjectHandle("marker_"..i)
        markerPos={D[i*3+4],D[i*3+5],D[i*3+6]}
        simSetObjectPosition(markerHandles[i],objHandle,markerPos)
        simSetObjectOrientation(markerHandles[i],objHandle,{0,0,0})
    end
    num=18
    print("number",#D)
end

if (sim_call_type==sim_childscriptcall_actuation) then

    if num<#D then
        position={D[num+1],D[num+2],D[num+3]}
        orientation={D[num+4],D[num+5],D[num+6]}
        simSetObjectPosition(camHandle,-1,position)
        simSetObjectOrientation(camHandle,-1,orientation)
    end
    num=num+6
end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

    -- Put some restoration code here

end