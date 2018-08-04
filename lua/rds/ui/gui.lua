local gui={}
gui.lastPos={800,400}



--界面初始化接口
gui.loadConfig=function(self)
    print("Function loadConfig() : Not Implement")
end

--界面数据保存接口
gui.saveConfig=function(self)
    print("Function saveConfig() : Not Implement")
end


--新建窗口
gui.new=function(self,xml,model,loadFunc,saveFunc)
    self.model=model
    self.xml=xml
    self.loadConfig=loadFunc or self.loadConfig
    self.saveConfig=saveFunc or self.saveConfig
    if not self.ui then
        self.ui=simUI.create(self.xml)
        self:loadConfig()
    end
    --读取上一次的窗口位置
    simUI.setPosition(self.ui,self.lastPos[1],self.lastPos[2])
    if not self.model then
        error("Function new(): No define of model")
    end
end

--显示窗口
gui.show=function(self)
    if not self.ui then
        self:new(self.xml,self.model)
    end
end



--销毁窗口并保存上次的位置
gui.destroy=function(self)
    if self.ui then
        local x,y=simUI.getPosition(self.ui)
        self.lastPos={x,y}
        simUI.destroy(self.ui)
        self.ui=nil
    end
end



--读取保存的数据:未指定model时，默认为self.model
--返回table
gui.readInfo=function(self,TAG,model)
    model=model or self.model 
    local data=sim.readCustomDataBlock(model,TAG)
    if data then
        data=sim.unpackTable(data)
    end
    return data
end


--写入数据到对象
--写入table
gui.writeInfo=function(self,TAG,data,model)
    model=model or self.model 
    if data then
        sim.writeCustomDataBlock(model,TAG,sim.packTable(data))
    else
        sim.writeCustomDataBlock(model,TAG,'')
    end
end

--单选或者多选时显示本项
gui.showOrHide=function(self)
    local s=sim.getObjectSelection()
    if s and #s>=1 and s[#s]==self.model then
        self:show()
    else
        self:destroy()
    end
end

return gui