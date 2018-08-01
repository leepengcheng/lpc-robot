local uitools={}
--单选或者多选时显示本项
uitools.showOrHideUiIfNeeded=function(self)
    local s=sim.getObjectSelection()
    if s and #s>=1 and s[#s]==model then
        self:showDlg()
    else
        removeDlg()
    end
end


uitools.createDlg=function(self,xml)
    if not ui then
        ui=simUI.create(xml)
        setDlgItemContent()
        updateEnabledDisabledItemsDlg()
    end
end


uitools.showDlg=function(self)
    if not ui then
    self:createDlg()
    end
end

uitools.removeDlg=function(self)
    if ui then
        local x,y=simUI.getPosition(ui)
        previousDlgPos={x,y}
        simUI.destroy(ui)
        ui=nil
    end
end

uitools.setDlgItemContent=function(self)
    print("Please Implement This Method")
end


uitools.readInfo=function(self,TAG)
    local data=sim.readCustomDataBlock(model,TAG)
    if data then
        data=sim.unpackTable(data)
    else
        data={}
    end
    return data
end


uitools.writeInfo=function(self,TAG,data)
    if data then
        sim.writeCustomDataBlock(model,TAG,sim.packTable(data))
    else
        sim.writeCustomDataBlock(model,TAG,'')
    end
end

return uitools