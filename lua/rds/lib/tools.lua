--@@通用工具函数
local const=require("const")
local tools={}
tools.forbidLevel=0
tools.robot=nil
tools.colors={{1,0,0},{0,1,0},{0,0,1},{1,1,0},{0,1,1}} --可用颜色


--清除路径数据
tools.clearPath=function(self,handle)
    sim.writeCustomDataBlock(handle, "", nil)
end

tools.new=function(self)
    return setmetatable(tools,{__index={version=1.0}})
end


-- 保存自定义路径
tools.savePath =function(self,handle,filename, path, lengths)
    sim.writeCustomDataBlock(handle, filename .. ".pathData1", sim.packFloatTable(path))
    sim.writeCustomDataBlock(handle, filename .. ".pathLength1", sim.packFloatTable(lengths))
end

--加载自定义路径
tools.loadPath=function(self,handle,filename)
    local path = sim.readCustomDataBlock(handle, filename .. ".pathData1")
    if (not path) then
        return nil
    end
    path = sim.unpackFloatTable(path)

    local lengths = sim.readCustomDataBlock(handle, filename .. ".pathLength1")
    if (not lengths) then
        return nil
    end
    lengths = sim.unpackFloatTable(lengths)
    return path, lengths
end

-- 获得沿着指定矩阵的坐标轴放心进行平移后的矩阵
-- @matrix:指定的矩阵
-- @trans:xyz平移量
-- @return:平移后的矩阵
tools.getTransMatrix=function(self,matrix, trans)
    local m = {}
    for i = 1, 12, 1 do
        m[i] = matrix[i]
    end
    m[4] = m[4] + m[1] * trans[1] + m[2] * trans[2] + m[3] * trans[3]
    m[8] = m[8] + m[5] * trans[1] + m[6] * trans[2] + m[7] * trans[3]
    m[12] = m[12] + m[9] * trans[1] + m[10] * trans[2] + m[11] * trans[3]
    return m
end



tools.getTransRotMatrix=function(self,handle,pos,euler)
    local pos=pos or {0,0,0}
    local euler=euler or {0,0,0}
    local baseMatrix=sim.getObjectMatrix(handle,-1)
    local matrix=sim.buildMatrix(pos,euler)
    local targetMatrix=sim.multiplyMatrices(baseMatrix,matrix)
    return targetMatrix
end


tools.setRobot=function(self,robot)
    self.robot=robot
end

--可视化末端的轨迹
--@path:计算出的路径点(关节空间/构型空间)
--@isAppend:是否追加
tools.visualizePath=function(self,path,isAppend)
    local isAppend=isAppend or false
    if not isAppend and _lineContainer then
        sim.addDrawingObjectItem(_lineContainer, nil)
    end
    local index=math.random(1,#self.colors)
    --如果没有_lineContainer则创建
    if not _lineContainer then
        _lineContainer = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 99999, self.colors[index])
    end

    if path then
        self:forbidThreadSwitches(true)
        local initConfig = self.robot:getConfig()
        local l = self.robot.joint.num
        local pc = #path / l
        for i = 1, pc - 1, 1 do
            local config1 = {
                path[(i - 1) * l + 1],
                path[(i - 1) * l + 2],
                path[(i - 1) * l + 3],
                path[(i - 1) * l + 4],
                path[(i - 1) * l + 5],
                path[(i - 1) * l + 6],
                path[(i - 1) * l + 7]
            }
            local config2 = {
                path[i * l + 1],
                path[i * l + 2],
                path[i * l + 3],
                path[i * l + 4],
                path[i * l + 5],
                path[i * l + 6],
                path[i * l + 7]
            }
            self.robot:setConfig(config1)
            local lineDat = sim.getObjectPosition(self.robot.ik.tipHandle, -1)
            self.robot:setConfig(config2)
            local p = sim.getObjectPosition(self.robot.ik.tipHandle, -1)
            lineDat[4] = p[1]
            lineDat[5] = p[2]
            lineDat[6] = p[3]
            sim.addDrawingObjectItem(_lineContainer, lineDat)
        end
        self.robot:setConfig(initConfig)
        self:forbidThreadSwitches(false)
    end
    sim.switchThread()
end



tools.forbidThreadSwitches = function(self, forbid)
    if forbid then
        self.forbidLevel = self.forbidLevel + 1
        if self.forbidLevel == 1 then
            sim.setThreadAutomaticSwitch(false)
        end
    else
        self.forbidLevel = self.forbidLevel - 1
        if self.forbidLevel == 0 then
            sim.setThreadAutomaticSwitch(true)
        end
    end
end


--合并多个table
tools.tableConcat=function(self,...)
 local paths={...}
 local data={}
 local n=1
 for i=1,#paths do
    local p=paths[i]
    for j=1,#p do
        -- table.insert( data,p[j]) --效率较低
        data[n]=p[j]
        n=n+1
    end
 end
 return data
end


--读取保存的数据:未指定model时，默认为self.model
--返回table
tools.readInfo=function(self,TAG,model)
    model=model or self.model 
    local data=sim.readCustomDataBlock(model,TAG)
    if data then
        data=sim.unpackTable(data)
    else
        data={}
    end
    return data
end


--写入数据到对象
--写入table
tools.writeInfo=function(self,TAG,data,model)
    model=model or self.model 
    if data then
        sim.writeCustomDataBlock(model,TAG,sim.packTable(data))
    else
        sim.writeCustomDataBlock(model,TAG,'')
    end
end


--###############################################
--#--------------ui部分-------------------------
--###############################################

--界面初始化接口
tools.loadUiConfig=function(self)
    print("Function loadUiConfig() : Not Implement")
end

--界面数据保存接口
tools.saveUiConfig=function(self)
    print("Function saveUiConfig() : Not Implement")
end

tools.createUiHeader=function(self,header)
    local title,size,pos=header[1],header[2],header[3]
    title=title or "RDS智能分拣系统"
    size=size or  "400,200"
    pos=pos or "290,90"
    return string.format([[<ui title="%s"  closeable="false"  resizable="true" size="%s" position="%s">]],title,size,pos)
end


tools.createUiTab=function(self,xml,title,layout)
    title=title or ""
    if layout then
        return string.format([[<tab title="%s" layout="%s" >%s</tab>]],title,layout,xml)
    else
        return string.format([[<tab title="%s">%s</tab>]],title,xml)
    end
    
end

tools.createUiFromTabs=function(self,tabs,header)
    local headerXml=self:createUiHeader(header)
    local bodyXml=""
    for i=1,#tabs do
        bodyXml=bodyXml..tabs[i]
    end
    return string.format("%s<tabs>%s</tabs></ui>",headerXml,bodyXml)

end


--新建窗口
tools.createUi=function(self,xml,model,loadFunc,saveFunc)
    self.model=model
    self.xml=xml
    self.loadUiConfig=loadFunc or self.loadUiConfig
    self.saveUiConfig=saveFunc or self.saveUiConfig
    if not self.ui then
        self.ui=simUI.create(self.xml)
        self:loadUiConfig()
    end
    if not self.model then
        error("Function new(): No define of model")
    end
end

--显示窗口
tools.showUi=function(self)
    if not self.ui then
        self:new(self.xml,self.model)
    end
end





--销毁窗口并保存上次的位置
tools.destroyUi=function(self,isSaveConfig)
    --先保存在销毁
    if isSaveConfig then
        self:saveUiConfig()
    end
    if self.ui then
        simUI.destroy(self.ui)
        self.ui=nil
    end

end



--单选或者多选时显示本项
tools.showOrHideUi=function(self)
    local s=sim.getObjectSelection()
    if s and #s>=1 and s[#s]==self.model then
        self:showUi()
    else
        self:destroyUi()
    end
end

tools.parseADSNetID=function(self,idstr)
    strlist=string.split(idstr,"([^.]+)")
    local data={}
    for i=1,6 do
        -- table.insert( data, tonumber(strlist[i]))
        data[i]=tonumber(strlist[i])
    end
    return data
 end


---b0
tools.initResolverB0=function(self,isShow)
    local show=isShow or 1
    if not simB0.pingResolver() then
        print('启动ZeroMQ 服务器')
        sim.launchExecutable('b0_resolver','',show)
        if not simB0.pingResolver() then
            error("Can not Start Server")
        end
    end
end



tools.vrep_Uint32_Float32=function(self,uint32NumTable)
    local numBuffer=sim.packUInt32Table(uint32NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end

tools.vrep_Uint16x2_Float32=function(self,uint16NumTable)
    local numBuffer=sim.packUInt16Table(uint16NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end



tools.vrep_Uint16x2_Float32=function(self,uint16NumTable)
    local numBuffer=sim.packUInt16Table(uint16NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end

--打包需要发送的数据
tools.packTrajData=function(self,cmd,path,index,step)
    path=path or {}
    step=step or 1
    index=index or 1
    local ptsNum=#path
    local size= ptsNum/7  --size:轨迹点的数目(每个轨迹点包含7个关节的数据))
    --限制size最大值
    if size>=const.COM.MAX_PTS_NUMBER then
        size=const.COM.MAX_PTS_NUMBER
    end

    local data={}
    data[1]=self:vrep_Uint16x2_Float32({cmd,step})
    if size>0 then
        data[2]=self:vrep_Uint32_Float32({size})
        data[3]=self:vrep_Uint32_Float32({index})
        for i=1,ptsNum do
            data[i+3]=path[i]
        end
    end
    return data
end


--解析数据,返回ROBOT_STATUS对象
tools.unpackRobotStatus=function(self,readData)
    local buffer=sim.packFloatTable(readData)
    local buffer_args=string.sub(buffer,1,6)
    local buffer_errorcode=string.sub(buffer,7,8)
    local buffer_positions=string.sub(buffer,9)

    local d1=sim.unpackUInt8Table(buffer_args)
    local d2=sim.unpackUInt16Table(buffer_errorcode)
    local d3=sim.unpackFloatTable(buffer_positions)

    local robo_statu={}
    robo_statu.driverPowered=d1[1]; --//驱动是否上电
    robo_statu.eStopped=d1[2];  --//是否非正常停止(如碰撞)
    robo_statu.inError=d1[3];  --//是否发生错误
    robo_statu.inMotion=d1[4];  --//是否在运动
    robo_statu.motionPossible=d1[5]; --//是否可以运动
    robo_statu.mode=d1[6]; --//当前的模式 自动/手动/未知
    robo_statu.errorCode=d2[1];   --//Uint16(错误码：自定义)
    robo_statu.position=d3
    
    return robo_statu
end
---格式化输出robotStatus
tools.stringFormatRobotStatus=function(self,status)
    local s1="驱动上电:%s"
    if status.driverPowered==const.COM.ROBO_STATUS_TRUE then
        s1=string.format(s1,"YES")
    else
        s1=string.format(s1,"NO")
    end

    local s2="非正常停止:%s"
    if status.eStopped==const.COM.ROBO_STATUS_TRUE then
        s2=string.format(s2,"YES")
    else
        s2=string.format(s2,"NO")
    end

    local s3="发生错误:%s"
    if status.inError==const.COM.ROBO_STATUS_TRUE then
        s3=string.format(s3,"YES")
    else
        s3=string.format(s3,"NO")
    end

    local s4="正在运动:%s"
    if status.inMotion==const.COM.ROBO_STATUS_TRUE then
        s4=string.format(s4,"YES")
    else
        s4=string.format(s4,"NO")
    end

    local s5="能否运动:%s"
    if status.motionPossible==const.COM.ROBO_STATUS_TRUE then
        s5=string.format(s5,"YES")
    else
        s5=string.format(s5,"NO")
    end

    s5=string.format("错误码:%d",status.errorCode)

    s6=string.format("关节位置:%f %f %f %f %f %f %f",status.position[1],
    status.position[2],status.position[3],
    status.position[4],status.position[5],
    status.position[6],status.position[7])
    s=string.format("%s %s %s %s %s %s\n",s1,s2,s3,s4,s5,s6)
    return s
end


-- tools:initADS=function(self,remoteID,remoteIP,localID,readAddr,writeAddr)
--     local adsHasInit=simADS.create(remoteID,remoteIP,localID)
--     if adsHasInit then
--         simADS.read(readAddr,0,simADS_handle_open)    --open read Handle
--     end
--     if adsHasInit then
--         simADS.write(writeAddr,{},simADS_handle_open) --open write handle 
--     end
--     return adsHasInit
-- end

return tools