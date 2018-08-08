--控制脚本-多线程：UI
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)
local tools=require("tools")
local const=require("const")


---###############自定义函数###############
--加载并保存数据
function loadConfig(self) 
    --init
    simUI.insertComboboxItem(self.ui,const.UI.comboSel,0,"自主规划路径")
    simUI.insertComboboxItem(self.ui,const.UI.comboSel,1,"人工指定路径")
    --populate data
    local data=self:readInfo(const.SIGNAL.UI_PARAM)
    simUI.setEditValue(self.ui,const.UI.editPath,data.editPath or "")
    simUI.setEditValue(self.ui,const.UI.editIndex,data.editIndex or "0")
    simUI.setEditValue(self.ui,const.UI.editStep,data.editStep or "1")
    simUI.setEditValue(self.ui,const.UI.remoteID,data.remoteID or "162.168.6.90.1.1")
    simUI.setEditValue(self.ui,const.UI.remoteIP,data.remoteIP or "127.0.0.1")
    simUI.setEditValue(self.ui,const.UI.localID,data.localID or "192.168.6.85.1.1")
    simUI.setEditValue(self.ui,const.UI.editWriteAddr,data.editWriteAddr or  "MAIN.traj_path")
    simUI.setEditValue(self.ui,const.UI.editReadAddr,data.editReadAddr or "MAIN.robo_status")
    simUI.setComboboxSelectedIndex(self.ui,const.UI.comboSel,data.comboSel or 0,false)

    --UI position
    local pos=data.position or {800,400}
    simUI.setPosition(self.ui,pos[1],pos[2],false)
end

--保存数据到对象
function saveConfig(self)
    local data={}
    data.comboSel= comboSel or 0
    data.editPath=simUI.getEditValue(self.ui,const.UI.editPath)
    data.editIndex=simUI.getEditValue(self.ui,const.UI.editIndex)
    data.editStep=simUI.getEditValue(self.ui,const.UI.editStep)
    data.remoteID=simUI.getEditValue(self.ui,const.UI.remoteID)
    data.remoteIP=simUI.getEditValue(self.ui,const.UI.remoteIP)
    data.localID=simUI.getEditValue(self.ui,const.UI.localID)
    data.writeAddr=simUI.getEditValue(self.ui,const.UI.editWriteAddr)
    data.readAddr=simUI.getEditValue(self.ui,const.UI.editReadAddr)

    --ui position
    local x,y=simUI.getPosition(self.ui)
    data.position={x,y}
    tools:writeInfo(const.SIGNAL.UI_PARAM,data)
end


--验证输入的数据
function validatePathFileContent(ui)
    --自主规划路径不需要验证
    if comboSel==0 then
        return true
    end
    local pathStr=simUI.getEditValue(ui,const.UI.editPath)
    if not isFileExist(pathStr) then
        print("请输入正确的文件路径")
        return false
    else
        local n=0
        for line in io.lines(pathStr) do
            n=n+1
        end
        if n%7~=0 then
            print("错误:文件中轨迹点数据长度不等于关节数7的倍数")
            return false
        end
    end
    return true
end

--检查文件是否存在
function isFileExist(name)
    local f=io.open(name,'r')
    if f~=nil then 
         io.close(f)
         return true
    end
    return false
 end









--初始化UI的xml描述文件
function initUIXml()
    local uiXml =[[
        <ui title="RDS智能分拣系统"  closeable="false"  resizable="false" size="400,200">
            <tabs>
                <tab title="输入参数" layout="grid" >
                    <checkbox style="font:13px;color:rgba(0,0,255,255)" text="读取数据" on-change="on_check_readstatus" id="900" checked="false" />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="控制方式" />
                    <combobox id="%d" on-change="onComboselChanged" />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" id="%d" text="轨迹文件" />

                        <edit  id="%d" enabled="false" />
                        <button id="%d" text="选择文件" enabled="false" on-click="on_openfile_click"/>


                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="起始位置 " />
                    <edit  id="%d"  />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="(Int) " />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="时间间隔 " />
                    <edit  id="%d" /> 
                    <label style="font:13px;color:rgba(0,0,255,255)" text="(ms) " />
                    <br />
                    <button text="发送轨迹" on-click="on_new_click"/>
                    <button text="开始运动" on-click="on_start_click"/><br />
                    <button text="暂停运动" on-click="on_pause_click"/>
                    <button text="停止运动" on-click="on_stop_click"/>
                </tab>
                <tab title="ADS设置" layout="form">
                    <label style="font:13px;color:rgba(0,0,255,255)" text="远程NetID" />
                    <edit  id="%d" />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="远程IP " />
                    <edit  id="%d"/>
                    <label style="font:13px;color:rgba(0,0,255,255)" text="本地NetID" />
                    <edit  id="%d" />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="数据读取地址" />
                    <edit  id="%d"  />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="数据写入地址" />
                    <edit  id="%d"  />
                </tab>
            </tabs>

        </ui>
    ]]
    uiXml=string.format(uiXml,const.UI.comboSel,const.UI.labelPath,
    const.UI.editPath,const.UI.buttonPath,const.UI.editIndex,const.UI.editStep,const.UI.remoteID,
    const.UI.remoteIP,const.UI.localID,const.UI.editReadAddr,const.UI.editWriteAddr)
    return uiXml
end


---############## UI 函数 ###############-
function onComboselChanged(ui,id,index)
    comboSel=index
    simUI.setEnabled(ui,const.UI.editPath,comboSel==1)
    simUI.setEnabled(ui,const.UI.buttonPath,comboSel==1)
end





function on_check_readstatus(ui,id,newVal)
    -- READ_ADDR_BACKUP=READ_ADDR_BACKUP or ADS.readAddr
    -- if newVal==2 then
    --     ADS.readAddr=READ_ADDR_BACKUP
    -- else
    --     ADS.readAddr=nil
    -- end
end

function on_openfile_click(ui,id)
    local f=sim.fileDialog(sim.filedlg_type_load,'打开文件','','','轨迹文件','txt')
    if f then
        simUI.setEditValue(ui,const.UI.editPath,f,true)
        -- ret=sim.msgBox(sim.msgbox_type_info,sim.msgbox_buttons_ok,'File Read Error',"The specified file could not be read.")
        -- sim.msgbox_return_ok
    end
end


function on_new_click(ui,id)
    local index=math.floor(tonumber(simUI.getEditValue(ui,const.UI.editIndex))+0.5)
    local step=math.floor(tonumber(simUI.getEditValue(ui,const.UI.editStep))+0.5)
    if validatePathFileContent(ui) then
        print(string.format("发送命令:新的轨迹"))
        tools:sendTrajCmdSignal(const.COM.TRAJ_CMD_NEW)
    end

end


function on_start_click(ui,id)
    print("发送命令:执行运动")
    tools:sendTrajCmdSignal(const.COM.TRAJ_CMD_START)
end

function on_pause_click(ui,id)
    print("发送命令:暂停运动")
    tools:sendTrajCmdSignal(const.COM.TRAJ_CMD_PAUSE)
end

function on_stop_click(ui,id)
    print("发送命令:停止运动")
    tools:sendTrajCmdSignal(const.COM.TRAJ_CMD_STOP)
end
---@@@@@@@@@@@ UI 函数 @@@@@@@@@@@--



----#############运行函数##################---
function sysCall_threadmain()
    handle=sim.getObjectAssociatedWithScript(sim.handle_self)
    local xml=initUIXml()
    tools:createUi(xml,handle,loadConfig,saveConfig)
    comboSel=0
    -- sim.setThreadAutomaticSwitch(false)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        sim.switchThread() -- resume in next simulation step
    end
end


--停止
function sysCall_cleanup()
    tools:destroyUi(true)
end
















