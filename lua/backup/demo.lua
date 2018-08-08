--常量
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
    labelSize=1100,
    sliderSize=1101,
    labelPath=1200,
    editPath=1201,
    editIndex=1300,
    editStep=1400
}



-- local ROBOT_STATUS={
--     driverPowered=CONST.ROBO_STATUS_TRUE; --//驱动是否上电
--     eStopped=CONST.ROBO_STATUS_FALSE;  --//estoped:是否非正常停止(如碰撞)
--     inError=CONST.ROBO_STATUS_FALSE;  --//是否发生错误
--     inMotion=CONST.ROBO_STATUS_TRUE;  --//是否在运动
--     motionPossible=CONST.ROBO_STATUS_TRUE; --//是否可以运动
--     mode=CONST.ROBO_MODE_AUTO; --//当前的模式 自动/手动/未知
--     errorCode=CONST.ROBO_ERRORCODE_NONE;   --//Uint16(错误码：自定义)
--     position={-1,-1,-1,-1,-1,-1,-1}
-- }


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


-- function twincat_Uint32_Float(floatNumTable)
--     local numBuffer=sim.packFloatTable(floatNumTable)
--     local uint32NumTable=sim.unpackUInt32Table(numBuffer)
--     return uint32NumTable
-- end


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


function isFileExist(name)
    local f=io.open(name,'r')
    if f~=nil then 
         io.close(f)
         return true
    end
    return false
 end


function on_new_click(ui,id)
    
    local index=math.floor(tonumber(simUI.getEditValue(ui,UIID.editIndex))+0.5)
    local step=math.floor(tonumber(simUI.getEditValue(ui,UIID.editStep))+0.5)
    if validateInputData() then
        local pts=parseInputPts()
        print(string.format("发送命令:新的轨迹：Size:%s  Index:%s  Step:%s",#pts/7,index,step))
        newTrajectory(pts,index,step)
    end

end

function on_start_click(ui,id)
    print("发送命令:执行运动")
    startTrajectory()
end

function on_pause_click(ui,id)
    print("发送命令:暂停运动")
    pauseTrajectory()
end

function on_stop_click(ui,id)
    print("发送命令:停止运动")
    stopTrajectory()
end

function on_slider_Change(ui,id,newVal)
    simUI.setLabelText(ui,UIID.labelSize,string.format("轨迹点个数:%6d",newVal))
end

function on_editing_finished(handle, id, newValue)
    if comboIndex==0 then
        local data=string.split(newValue,"[^%s]+")
        local num=#data-1
        if num%7~=0 then
            print("错误:自定义轨迹点的数组长度为7的倍数")
        else
            setSliderValue_trigerOnChange(num/7)
        end
    --随机值
    elseif comboIndex==1 then
        local data=string.split(newValue,"[^%s]+")
        local num=#data-1
        if num~=14 then
            print("错误:随机值需要输入7个关节*2的位置的随机上下限")
        end
    --固定值
    elseif comboIndex==2 then
        local data=string.split(newValue,"[^%s]+")
        local num=#data-1
        if num~=7 then
            print("错误:固定值需要输入7个关节的位置值")
        end
    --文件读入
    else
        if not isFileExist(newValue) then
            print("请检查文件路径是否正确")
        end
    end   
end


function on_close_ui(h)
    simUI.hide(h)
end

function setSliderValue_trigerOnChange(s)
    simUI.setSliderValue(ui,UIID.sliderSize,s) --默认1个点
    on_slider_Change(ui,UIID.sliderSize,s) 
end


function onComboselChanged(ui,id,newIndex)
    comboIndex=newIndex
    --自定义
    if comboIndex==0 then
        simUI.setEnabled(ui,UIID.sliderSize,false)
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(1维数组) ")
        simUI.setEditValue(ui,UIID.editPath,"1 2 3 4 5 6 7")
        setSliderValue_trigerOnChange(1)
    --随机值
    elseif comboIndex==1 then
        simUI.setEnabled(ui,UIID.sliderSize,true)
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(关节随机范围) ")
        simUI.setEditValue(ui,UIID.editPath,"0 1 0 1 0 1 0 1 0 1 0 1 0 1 ")
        setSliderValue_trigerOnChange(0)
    --固定值
    elseif comboIndex==2 then
        simUI.setEnabled(ui,UIID.sliderSize,true)
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(关节恒定值) ")
        simUI.setEditValue(ui,UIID.editPath,"1 2 3 4 5 6 7")
        setSliderValue_trigerOnChange(0)
    --文件读入
    else
        simUI.setEnabled(ui,UIID.sliderSize,false)
        simUI.setLabelText(ui,UIID.labelPath,"轨迹点(文件路径) ")
        simUI.setEditValue(ui,UIID.editPath,"D:/data.txt")
    end   
end


function initializeUI()
    simUI.insertComboboxItem(ui,UIID.comboSel,0,"自定义")
    simUI.insertComboboxItem(ui,UIID.comboSel,1,"随机值")
    simUI.insertComboboxItem(ui,UIID.comboSel,2,"恒定值")
    simUI.insertComboboxItem(ui,UIID.comboSel,3,"文件读入")
    simUI.setEnabled(ui,UIID.sliderSize,false)
    setSliderValue_trigerOnChange(1) --自定义默认1个点
end

function on_select_read(ui,id,newVal)
    READ_ADDR_BACKUP=READ_ADDR_BACKUP or readAddr
    if newVal==2 then
        readAddr=READ_ADDR_BACKUP
    else
        readAddr=nil
    end
end

function sysCall_init()

    local xml =[[
        <ui title="ADS测试" closeable="true" on-close="on_close_ui" resizable="true" size="512,100">
            
            <tabs>
                <tab title="输入参数" layout="grid">
                    <checkbox text="读取数据" on-change="on_select_read" id="900" checked="true" />
                    <br />
                    <label text="输入方式" />
                    <combobox id="%d" on-change="onComboselChanged" />
                    <br />
                    <label id="%d" text="轨迹点个数:0     " />
                    <hslider id="%d"  tick-interval="50" minimum="0" maximum="100000" on-change="on_slider_Change" />
                    <br />
                    <label id="%d" text="轨迹点(1维数组) " />
                    <edit  id="%d" value="1 2 3 4 5 6 7" on-editing-finished="on_editing_finished" />
                    <br />
                    <label text="起始位置(整数) " />
                    <edit  id="%d" value="0" />
                    <br />
                    <label text="时间间隔(s) " />
                    <edit  id="%d" value="1" />
                </tab>
            </tabs>
            <button text="发送轨迹" on-click="on_new_click"/>
            <button text="开始运动" on-click="on_start_click"/>
            <button text="暂停运动" on-click="on_pause_click"/>
            <button text="停止运动" on-click="on_stop_click"/>
        </ui>
    ]]
    
    ui=simUI.create(string.format(xml,UIID.comboSel,UIID.labelSize,UIID.sliderSize,UIID.labelPath,UIID.editPath,UIID.editIndex,UIID.editStep))
    initializeUI()
    comboIndex=0
    gHandle=sim.getObjectHandle("Graph")


    ----------------ADS----------------------------------------

    -- readAddr="MAIN.robo_status"   --需要读数据时取消注释本行
    -- writeAddr="MAIN.traj_path" --需要写数据时取消注释本行

    adsHasInit=false
    if readAddr or writeAddr then
        adsHasInit=simADS.create({192,168,6,90,1,1},"127.0.0.1",{192,168,6,90,2,1})
    end

    if readAddr and adsHasInit then
        simADS.read(readAddr,0,simADS_handle_open)    --open read Handle
    end

    if writeAddr and adsHasInit then
        simADS.write(writeAddr,{},simADS_handle_open) --open write handle 
    end
    num=0
end


--新轨迹:发送新轨迹后暂停不运动
--@pts:轨迹点table,size=N*7
--@index:起始位置:默认0
--@step:时间间隔:默认1ms
function newTrajectory(pts,index,step)
    writeTrajectory(CONST.TRAJ_CMD_NEW,pts,index,step)
end


--开始
function startTrajectory()
    writeTrajectory(CONST.TRAJ_CMD_START,{})
end

--停止
function stopTrajectory()
    writeTrajectory(CONST.TRAJ_CMD_STOP,{})
end

--暂停
function pauseTrajectory()
    writeTrajectory(CONST.TRAJ_CMD_PAUSE,{})
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

    simADS.write(writeAddr,data,simADS_handle_none)
end



--读取robotStatus
--@isprint:是否打印消息，默认false
function readRobotStatus(isprint)
    local isprint=false or isprint
    local readData=simADS.read(readAddr,9,simADS_handle_none)
    local status=nil
    if readData then
        status=unpackRobotStatus(readData) 
    end

    if isprint then
        print(stringFormatRobotStatus(status))
    end
    return status
end



--验证输入
function validateInputData()

    local  data,num,pathString=getInputPathList()
    local  pts_num=simUI.getSliderValue(ui,UIID.sliderSize)
    --0：自定义----------
    if comboIndex==0 then
        if num%7~=0 or num==0 then
            print("错误:输入轨迹点数据长度不等于关节数7的倍数")
            return false
        end

    --------------------------------------------
    --1：随机值----------
    elseif  comboIndex==1 then
        if pts_num==0 then
            print("错误:轨迹点的数目不能为0")
            return false
        end
        if num~=14 then
            print("错误:输入轨迹点的长度不等于关节数*2=14")
            return false
        end
    ------------------------------------------------

    --2:恒定值---------------------------------
    elseif comboIndex==2 then
        if pts_num==0 then
            print("错误:轨迹点的数目不能为0")
            return false
        end
        if num~=7 then
            print("错误:输入轨迹点数据长度不等于关节数7")
            return false
        end
    ----------------------------------------------------

    ---3：文件读入---------------------------------
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
            setSliderValue_trigerOnChange(n/7)
        end
    end

    return true
end


function getInputPathList()
    local pathString=simUI.getEditValue(ui,UIID.editPath)
    local data=string.split(pathString,"[^%s]+")
    local num=#data-1
    return data,num,pathString
end

----要发送的虚拟数据---------
function parseInputPts()

    local pts={}
    local data,num,pathString=getInputPathList()
    local pts_num=simUI.getSliderValue(ui,UIID.sliderSize) --输入的轨迹点数目

    --0：自定义----------
    if comboIndex==0 then
        for i=1,num do
            table.insert( pts,tonumber(data[i]) )
        end
    --------------------------------------------

    
    --1：随机值----------
    elseif  comboIndex==1 then
        local li={}
        for i=1,7 do
            local x=tonumber(data[i])
            local y=tonumber(data[i+1])
            li[i]={x,y-x}
        end
        for i=1,pts_num do
            pts[7*(i-1)+1]=li[1][1]+math.random()*li[1][2]
            pts[7*(i-1)+2]=li[2][1]+math.random()*li[2][2]
            pts[7*(i-1)+3]=li[3][1]+math.random()*li[3][2]
            pts[7*(i-1)+4]=li[4][1]+math.random()*li[4][2]
            pts[7*(i-1)+5]=li[5][1]+math.random()*li[5][2]
            pts[7*(i-1)+6]=li[6][1]+math.random()*li[6][2]
            pts[7*(i-1)+7]=li[7][1]+math.random()*li[7][2]
        end

    ------------------------------------------------
    --2:恒定值---------------------------------
    elseif comboIndex==2 then
        for i=1,pts_num do
            pts[7*(i-1)+1]=tonumber(data[1])
            pts[7*(i-1)+2]=tonumber(data[2])
            pts[7*(i-1)+3]=tonumber(data[3])
            pts[7*(i-1)+4]=tonumber(data[4])
            pts[7*(i-1)+5]=tonumber(data[5])
            pts[7*(i-1)+6]=tonumber(data[6])
            pts[7*(i-1)+7]=tonumber(data[7])
        end
    ----------------------------------------------------
    ---文件读入
    else
        for line in io.lines(pathString,'r') do
            table.insert( pts, tonumber(line))
        end
    end
    return pts
end

function sysCall_actuation()

    --------读机器人状态
    if readAddr and adsHasInit  then
        readRobotStatus(true)
    end
    --绘图
    --sim.setGraphUserData(gHandle,"data",r[1])
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()

    simUI.destroy(ui)

    readAddr=READ_ADDR_BACKUP --销毁时需要回原地址

    if writeAddr and adsHasInit then
        simADS.write(writeAddr,{},simADS_handle_close)  --close write Handle
    end


    if readAddr and adsHasInit then
        simADS.read(readAddr,0,simADS_handle_close)     --close read Handle
    end
    
    if adsHasInit then
        simADS.destory()
        adsHasInit=false
    end

end
