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
 for i=1,#paths do
    local p=paths[i]
    for j=1,#p do
        table.insert( data,p[j])
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

tools.createUiHeader=function(self,title,size)
    title=title or "RDS智能分拣系统"
    size=size or  "400,200"
    return string.format([[<ui title="%s"  closeable="false"  resizable="false" size="%s">]],title,size)
end


tools.createUiTab=function(self,xml,title,layout)
    title=title or ""
    layout=layout or  "grid"
    return string.format([[<tab title="%s" layout="%s" >%s</tab>]],title,layout,xml)
end

tools.createUiFromTabs=function(self,tabs,header)
    local title,size=header
    local headerXml=self:createUiHeader(title,size)
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
        table.insert( data, tonumber(strlist[i]))
    end
    return data
 end

--B0 UI发布命令的handle
-- tools.trajCmdPubHandle=nil
-- tools.trajCmdSubHandle=nil

-- --信号
-- tools.sendTrajCmdSignal=function(self,signal,handle)
--     handle=handle or self.trajCmdPubHandle
--     if not handle then
--         error("Wrong BlueZero Handle")
--     end
--     -- sim.setIntegerSignal(const.SIGNAL.TRAJ_CMD,signal)
--     simB0.publish(handle,signal)
-- end

-- tools.sendAdsCmdSignal=function(self,signal)
--     handle=handle or self.trajCmdPubHandle
--     if not handle then
--         error("Wrong BlueZero Handle")
--     end
--     -- sim.setIntegerSignal(const.SIGNAL.ADS_CMD,signal)
--     simB0.publish(self.trajCmdPubHandle,signal)
-- end


-- tools.readAdsCmdSignal=function(self,func,handle)
--     -- return sim.getIntegerSignal(const.SIGNAL.ADS_CMD)
--     local data=simB0.sub(self.trajCmdPubHandle)
--     return data
    
-- end

-- tools.readTrajCmdSignal=function(self,func,handle)
--     handle=handle or self.trajCmdPubHandle
--     -- return sim.getIntegerSignal(const.SIGNAL.TRAJ_CMD)
--     local data=simB0.sub(self.trajCmdPubHandle)
-- end



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

return tools