-- function include(absolutePathAndFile)
--     if not __notFirst__ then
--         __notFirst__=true
--         __scriptCodeToRun__=assert(loadfile(absolutePathAndFile))
--     end
--     if __scriptCodeToRun__ then
--         __scriptCodeToRun__()
--     end
-- end
-- include('/media/zen/workspace/GitWorkSpace/vrep-control/lua/create_boundingbox.lua')




-- 创建机械臂包围盒
function createShapeBoundingBoxAndJoints(linkHandles,jointHandles,output,color)
    color=color or {0,0,1}
    local file=io.open(output,"w") --写出对应的arm的boundingbox尺寸
    
    for i=1,#linkHandles,1 do
        linkHandle=linkHandles[i]
        pos=simGetObjectPosition(linkHandle,-1) --link的全局pos
        ori=simGetObjectOrientation(linkHandle,-1) --link的全局方位
        start=14
        --获得对象的包围盒的尺寸
        res,xmin=simGetObjectFloatParameter(linkHandle,start+1)
        res,ymin=simGetObjectFloatParameter(linkHandle,start+2 )
        res,zmin=simGetObjectFloatParameter(linkHandle,start+3 )
        res,xmax=simGetObjectFloatParameter(linkHandle,start+4 )
        res,ymax=simGetObjectFloatParameter(linkHandle,start+5 )
        res,zmax=simGetObjectFloatParameter(linkHandle,start+6 )
        xsize=xmax-xmin
        ysize=ymax-ymin
        zsize=zmax-zmin
        --创建包围盒
        boxHandle=simCreatePureShape(0,16,{xsize,ysize,zsize},0.0)
        simSetObjectPosition(boxHandle,-1,pos)
        simSetObjectOrientation(boxHandle,-1,ori)
        simSetShapeColor(boxHandle,nil,sim_colorcomponent_ambient_diffuse,color)--颜色
        simSetShapeColor(boxHandle,nil,sim_colorcomponent_transparency,{0.5})--透明度
        _pos=simGetObjectPosition(linkHandle,jointHandles[i])    --当前关节相对
        _ori=simGetObjectOrientation(linkHandle,jointHandles[i]) --目标的方位
        data=string.format("%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",_pos[1],_pos[2],_pos[3],_ori[1],_ori[2],_ori[3],xsize,ysize,zsize)
        file:write(data)
    end
    file:close()
end



--关闭动力学
simSetBooleanParameter(sim_boolparam_dynamics_handling_enabled,false)

--导入配置参数文件
configfile="/media/zen/workspace/GitWorkSpace/vrep-control/lua/config.lua"  --参数配置
pyurdffile="/media/zen/workspace/GitWorkSpace/vrep-control/lua/create_boundingbox.py" --python urdf解析文件
dofile(configfile)

roboLinkHandles={}
roboJointHandles={}

handJointHandles={}
handLinkHandles={}

--机械本体的句柄
for i=1,#roboLinkNames,1 do
    roboLinkHandles[i]=simGetObjectHandle(roboLinkNames[i])
end
for i=1,#roboJointNames,1 do
    roboJointHandles[i]=simGetObjectHandle(roboJointNames[i])
end
--机械手的句柄
for i=1,#handLinkNames,1 do
    handLinkHandles[i]=simGetObjectHandle(handLinkNames[i])
end
for i=1,#handJointNames,1 do
    handJointHandles[i]=simGetObjectHandle(handJointNames[i])
end


robot_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/robot.txt"
hand_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/hand.txt"


createShapeBoundingBoxAndJoints(roboLinkHandles,roboJointHandles,robot_out)
createShapeBoundingBoxAndJoints(handLinkHandles,handJointHandles,hand_out,{1,0,0})


result=simLaunchExecutable('/usr/bin/python',string.format("%s %s",pyurdffile,configfile),0)

if (result==-1) then
    -- The executable could not be launched!
    simDisplayDialog('Error',"Script could not be launched. &&nSimulation will not run properly",sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
end