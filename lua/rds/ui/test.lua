local workdir="/media/lee/workspace/GitWorkSpace/lpc-robot/lua/rds/ui"
package.path=string.format( "%s;%s/?.lua",package.path,workdir)
local gui=require("gui")


---################ 常量区 ######################---
local CONST={
    --轨迹命令:uint8
    TRAJ_CMD_NEW=1,
    TRAJ_CMD_START=2,
    TRAJ_CMD_PAUSE=3,
    TRAJ_CMD_STOP=0,

    --机器人模式:uint8
    ROBO_MODE_AUTO=  1, --自动模式
    ROBO_MODE_MANNUL=  2,--手动模式
    ROBO_MODE_UNKNOW=  3,--未知模式

    --机器人状态:uint8
    ROBO_STATUS_TRUE=255,
    ROBO_STATUS_FALSE=0,

    --机器人错误码 uint16
    ROBO_ERRORCODE_NONE=0,  --无错误
    ROBO_ERRORCODE_ALARM=10, --报警错误
    PTS_NUMBER=100000  --最大的轨迹点的数据
}

local UIID={
    comboSel=1000,
    labelPath=1200,
    editPath=1201,
    editIndex=1300,
    editStep=1400,
    remoteID=2000,
    remoteIP=2001,
    localID=2002
}
--@@@@@@@@@@@ 常量区(结束)) @@@@@@@@@@@--



---###############自定义函数###############

--加载并保存数据
function loadConfig(self) 
    --init
    simUI.insertComboboxItem(self.ui,UIID.comboSel,0,"自定义")
    simUI.insertComboboxItem(self.ui,UIID.comboSel,1,"文件读入")
    --populate data
    local data=self:readInfo("input")
    simUI.setEditValue(self.ui,UIID.editPath,data.editPath or "")
    simUI.setEditValue(self.ui,UIID.editIndex,data.editIndex or "")
    simUI.setEditValue(self.ui,UIID.editStep,data.editStep or "")
    simUI.setEditValue(self.ui,UIID.remoteID,data.remoteID or "")
    simUI.setEditValue(self.ui,UIID.remoteIP,data.remoteIP or "")
    simUI.setEditValue(self.ui,UIID.localID,data.localID or "")
    
    simUI.setComboboxSelectedIndex(self.ui,UIID.comboSel,data.comboSel or 0)
end

--保存数据到对象
function saveConfig(self)
    local data={}
    data.comboSel=comboIndex or 0
    data.editPath=simUI.getEditValue(self.ui,UIID.editPath)
    data.editIndex=simUI.getEditValue(self.ui,UIID.editIndex)
    data.editStep=simUI.getEditValue(self.ui,UIID.editStep)
    data.remoteID=simUI.getEditValue(self.ui,UIID.remoteID)
    data.remoteIP=simUI.getEditValue(self.ui,UIID.remoteIP)
    data.localID=simUI.getEditValue(self.ui,UIID.localID)
    gui:writeInfo("input",data)
end


function vrep_Uint32_Float32(uint32NumTable)
    local numBuffer=sim.packUInt32Table(uint32NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end

function vrep_Uint16x2_Float32(uint16NumTable)
    local numBuffer=sim.packUInt16Table(uint16NumTable)
    local float32NumTable=sim.unpackFloatTable(numBuffer)
    return float32NumTable[1]
end
--解析数据,返回ROBOT_STATUS对象
function unpackRobotStatus(readData)
    local buffer=sim.packFloatTable(readData)
    local buffer_args=string.sub(buffer,1,6)
    local buffer_errorcode=string.sub(buffer,7,8)
    local buffer_positions=string.sub(buffer,9)

    local d1=sim.unpackUInt8Table(buffer_args)
    local d2=sim.unpackUInt16Table(buffer_errorcode)
    local d3=sim.unpackFloatTable(buffer_positions)
    -- print(d1[1],d1[2],d1[3],d1[4],d1[5],d1[6],d2[1],d3[1],d3[2],d3[3],d3[4],d3[5],d3[6],d3[7])

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

---格式化robotStaus，转换为字符串
function stringFormatRobotStatus(status)
    local s1="驱动上电:%s"
    if status.driverPowered==CONST.ROBO_STATUS_TRUE then
        s1=string.format(s1,"YES")
    else
        s1=string.format(s1,"NO")
    end

    local s2="非正常停止:%s"
    if status.eStopped==CONST.ROBO_STATUS_TRUE then
        s2=string.format(s2,"YES")
    else
        s2=string.format(s2,"NO")
    end

    local s3="发生错误:%s"
    if status.inError==CONST.ROBO_STATUS_TRUE then
        s3=string.format(s3,"YES")
    else
        s3=string.format(s3,"NO")
    end

    local s4="正在运动:%s"
    if status.inMotion==CONST.ROBO_STATUS_TRUE then
        s4=string.format(s4,"YES")
    else
        s4=string.format(s4,"NO")
    end

    local s5="能否运动:%s"
    if status.motionPossible==CONST.ROBO_STATUS_TRUE then
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

--验证输入的数据
function validateInputData()

    local  data,num,pathString=getInputPathList()
    --0：自定义----------
    if comboIndex==0 then
        if num%7~=0 or num==0 then
            print("错误:输入轨迹点数据长度不等于关节数7的倍数")
            return false
        end

    ---1：文件读入---------------------------------
    else
        if not isFileExist(pathString) then
            print("请输入正确的文件路径")
            return false
        else
            local n=0
            for line in io.lines(pathString) do
                n=n+1
            end
            if n%7~=0 then
                print("错误:文件中轨迹点数据长度不等于关节数7的倍数")
                return false
            end
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



 function parseADSNetID(idstr)
    strlist=string.split(idstr,"([^.]+)")
    local data={}
    for i=1,6 do
        table.insert( data, tonumber(strlist[i]))
    end
    return data
 end


 --写出轨迹到twincat
--@cmd：控制指令 START/STOP/PAUSE/NEW
--@pts:轨迹点数据,1维数组,最大容量7*100000 (twincat定义))
--@step：默认1ms
--@index：起始位,默认1
function writeTrajectory(cmd,pts,index,step)

    local step=step or 1
    local index=index or 1
    local size=#pts/7
    --限制最大值
    if size>=CONST.PTS_NUMBER then
        size=CONST.PTS_NUMBER
    end

    local data={}
    data[1]=vrep_Uint16x2_Float32({cmd,step})
    if size>0 then
        data[2]=vrep_Uint32_Float32({size})
        data[3]=vrep_Uint32_Float32({index})
    end

    
    for i=1,size do
        data[7*(i-1)+3+1]=pts[7*(i-1)+1]
        data[7*(i-1)+3+2]=pts[7*(i-1)+2]
        data[7*(i-1)+3+3]=pts[7*(i-1)+3]
        data[7*(i-1)+3+4]=pts[7*(i-1)+4]
        data[7*(i-1)+3+5]=pts[7*(i-1)+5]
        data[7*(i-1)+3+6]=pts[7*(i-1)+6]
        data[7*(i-1)+3+7]=pts[7*(i-1)+7]
    end

    simADS.write(WRITE_ADDR,data,simADS_handle_none)
end

--读取robotStatus
--@isprint:是否打印消息，默认false
function readRobotStatus(isprint)
    local isprint=false or isprint
    local readData=simADS.read(READ_ADDR,9,simADS_handle_none)
    local status=nil
    if readData then
        status=unpackRobotStatus(readData) 
    end

    if isprint then
        print(stringFormatRobotStatus(status))
    end
    return status
end
---@@@@@@@@@@@@自定义函数@@@@@@@@@@@@@@@@@@




---############## UI 函数 ###############-
function onComboselChanged(ui,id,newIndex)
    comboIndex=newIndex
    --自定义
    if comboIndex==0 then
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(1维数组) ")
        simUI.setEditValue(ui,UIID.editPath,"1 2 3 4 5 6 7")
    else
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(文件路径) ")
        simUI.setEditValue(ui,UIID.editPath,"D:/data.txt")
    end   
end





function on_check_readstatus(ui,id,newVal)
    READ_ADDR_BACKUP=READ_ADDR_BACKUP or READ_ADDR
    if newVal==2 then
        READ_ADDR=READ_ADDR_BACKUP
    else
        READ_ADDR=nil
    end
end


function on_edit_finished(handle, id, newValue)
    if comboIndex==0 then
        local data=string.split(newValue,"[^%s]+")
        local num=#data-1
        if num%7~=0 then
            print("错误:自定义轨迹点的数组长度为7的倍数")
        end
    else
        if not isFileExist(newValue) then
            print("请检查文件路径是否正确")
        end
    end   
end

function on_new_click(ui,id)
    local index=math.floor(tonumber(simUI.getEditValue(ui,UIID.editIndex))+0.5)
    local step=math.floor(tonumber(simUI.getEditValue(ui,UIID.editStep))+0.5)
    if validateInputData() then
        local pts=parseInputPts()
        print(string.format("发送命令:新的轨迹：Size:%s  Index:%s  Step:%s",#pts/7,index,step))
        -- newTrajectory(pts,index,step)
    end

end

function on_openfile_click(ui,id)
    local f=sim.fileDialog(sim.filedlg_type_load,'打开文件','','','轨迹文件','txt')
    if f then
        simUI.setEditValue(ui,UIID.editPath,f,true)
        -- ret=sim.msgBox(sim.msgbox_type_info,sim.msgbox_buttons_ok,'File Read Error',"The specified file could not be read.")
        -- sim.msgbox_return_ok
    end
end

function on_start_click(ui,id)
    print("发送命令:执行运动")
    print(parseADSNetID(simUI.getEditValue(ui,UIID.remoteID)))
    -- startTrajectory()
end

function on_pause_click(ui,id)
    print("发送命令:暂停运动")
    -- pauseTrajectory()
end

function on_stop_click(ui,id)
    print("发送命令:停止运动")
    -- stopTrajectory()
end
---@@@@@@@@@@@ UI 函数 @@@@@@@@@@@--









----#############运行函数##################---
function sysCall_threadmain()
-- function sysCall_init() --初始化
    handle=sim.getObjectAssociatedWithScript(sim.handle_self)
    local xml =[[
        <ui title="RDS智能分拣系统"  closeable="false"  resizable="false" size="500,200">
            <tabs>
                <tab title="输入参数" layout="grid" >
                    <checkbox style="font:13px;color:rgba(0,0,255,255)" text="读取数据" on-change="on_check_readstatus" id="900" checked="false" />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="输入方式" />
                    <combobox id="%d" on-change="onComboselChanged" />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" id="%d" text="轨迹点 " />
                    <group layout="hbox" flat="true">
                        <edit  id="%d" value="1 2 3 4 5 6 7" on-editing-finished="on_edit_finished" />
                        <button text="选择文件" on-click="on_openfile_click"/>
                    </group>

                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="起始位置(整数) " />
                    <edit  id="%d" value="0" />
                    <br />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="时间间隔(ms) " />
                    <edit  id="%d" value="1" /> <br />
                    <button text="发送轨迹" on-click="on_new_click"/>
                    <button text="开始运动" on-click="on_start_click"/><br />
                    <button text="暂停运动" on-click="on_pause_click"/>
                    <button text="停止运动" on-click="on_stop_click"/>
                </tab>
                <tab title="ADS设置" layout="form">
                    <label style="font:13px;color:rgba(0,0,255,255)" text="远程NetID" />
                    <edit  id="%d" value="192.168.6.90.1.1" />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="远程IP" />
                    <edit  id="%d" value="127.0.0.1" />
                    <label style="font:13px;color:rgba(0,0,255,255)" text="本地NetID" />
                    <edit  id="%d" value="192.168.60.90.2.1" />
                </tab>
            </tabs>

        </ui>
    ]]
    xml=string.format(xml,UIID.comboSel,UIID.labelPath,
    UIID.editPath,UIID.editIndex,UIID.editStep,UIID.remoteID,
    UIID.remoteIP,UIID.localID)
    gui:new(xml,handle,loadConfig,saveConfig)
    comboIndex=0 --下拉框的默认值
    gHandle=sim.getObjectHandle("Graph")


    ----------------ADS----------------------------------------

    -- READ_ADDR="MAIN.robo_status"   --需要读数据时取消注释本行
    -- WRITE_ADDR="MAIN.traj_path" --需要写数据时取消注释本行
    adsHasInit=false
    if READ_ADDR or WRITE_ADDR then
        adsHasInit=simADS.create({192,168,6,90,1,1},"127.0.0.1",{192,168,6,90,2,1})
    end

    if READ_ADDR and adsHasInit then
        simADS.read(READ_ADDR,0,simADS_handle_open)    --open read Handle
    end

    if WRITE_ADDR and adsHasInit then
        simADS.write(WRITE_ADDR,{},simADS_handle_open) --open write handle 
    end


    -- sim.setThreadAutomaticSwitch(false)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        sim.switchThread() -- resume in next simulation step
    end
end




--没有仿真时
-- function sysCall_nonSimulation()
--     gui:showOrHide()
-- end

function sysCall_beforeSimulation()
    print("beforeSimulation")
    local c=gui:readInfo("hello")
end


function sysCall_afterSimulation()
    print("afterSimulation")
end


function sysCall_beforeInstanceSwitch()
    print("afterInstanceSwitch")
    gui:destroy()
end


--传感
function sysCall_sensing()
    -- print("sensing")
end

--暂停
function sysCall_suspend()
    -- print("suspend")
end

--恢复
function sysCall_resume()
    -- print("resume")
end

--停止
function sysCall_cleanup()
    gui:saveConfig()
    gui:destroy()
end
















