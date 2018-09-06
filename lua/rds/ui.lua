--控制脚本-多线程：UI
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)
local tools=require("tools")
local const=require("const")


---###############自定义函数###############
--加载并保存数据
function loadUiConfig(self,isDefault) 
    print("加载参数设置")
    isDefault=isDefault or false
    simUI.insertComboboxItem(self.ui,const.UI.comboSel,0,"自动")
    simUI.insertComboboxItem(self.ui,const.UI.comboSel,1,"手动")
    for i=1,7 do
        simUI.addTreeItem(self.ui,const.UI.treeJointStatus,const.UI.treeJointStatus+i,{""..i,""})
    end
    --populate data
    local data={}
    if not isDefault then
        data=self:readInfo(const.DB.UI)
    end
    simUI.setEditValue(self.ui,const.UI.editPath,data.editPath or "")
    simUI.setEditValue(self.ui,const.UI.editTarget,data.editTarget or "")
    simUI.setEditValue(self.ui,const.UI.editIndex,data.editIndex or "0")
    simUI.setEditValue(self.ui,const.UI.editStep,data.editStep or "1")
    simUI.setEditValue(self.ui,const.UI.remoteID,data.remoteID or "192.168.6.90.1.1")
    simUI.setEditValue(self.ui,const.UI.remoteIP,data.remoteIP or "127.0.0.1")
    simUI.setEditValue(self.ui,const.UI.localID,data.localID or "192.168.6.85.1.1")
    simUI.setEditValue(self.ui,const.UI.editWriteAddr,data.editWriteAddr or  "MAIN.traj_path")
    simUI.setEditValue(self.ui,const.UI.editReadAddr,data.editReadAddr or "MAIN.robo_status")

    --选择模式
    comboSel=data.comboSel or 0
    simUI.setComboboxSelectedIndex(self.ui,const.UI.comboSel,comboSel)
    onComboselChanged(self.ui,const.UI.comboSel,comboSel)

    --UI position
    -- local pos=data.position or {290,90}
    -- simUI.setPosition(self.ui,pos[1],pos[2])
end

--保存数据到对象
function saveUiConfig(self)
    print("保存参数设置")
    local data={}
    data.comboSel= comboSel or 0
    data.editTarget=simUI.getEditValue(self.ui,const.UI.editTarget)
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
    tools:writeInfo(const.DB.UI,data)
end


--验证输入的数据
function parsePathFileContent(ui)
    --自主规划路径不需要验证
    local data={}
    if comboSel==0 then
        return data,nil
    end
    local pathStr=simUI.getEditValue(ui,const.UI.editPath)
    if not isFileExist(pathStr) then
        print("请输入正确的文件路径")
        return nil,nil
    else
        f=io.open(pathStr,"r")
        local dataList=string.split(f:read(),"[^%s]+")
        f:close()
        if #dataList~=8 then
            print("错误:轨迹文件每行包含7个轴的数据并用空格分开")
            return nil,nil
        end
    end
    --读入并解析数据点
    local n=1
    local lineNum=0
    for line in io.lines(pathStr) do
        lineNum=lineNum+1
        local dataList=string.split(line,"[^%s]+")
        for i=1,7 do
            -- table.insert(data,tonumber(dataList[i]))
            data[n]=tonumber(dataList[i])
            n=n+1
        end
    end
    return data,lineNum
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


function initUIXml()
    local tab1Xml=string.format([[

        <combobox id="%d" on-change="onComboselChanged" />
        <group layout="grid" id="8000" visible="true">
            <label style="font:13px;color:rgba(0,0,255,255)" text="请选择路径文件(关节数据用空格区分)"/><br/>
            <edit  id="%d"  />
            <button id="%d" text="选择" on-click="on_openfile_click"/>
            <button  text="导入" on-click="on_importpath_click"/>
        </group>
        <group layout="grid" id="8001" visible="true">
        <label style="font:13px;color:rgba(0,0,255,255)" text="请选择需要抓取的对象名称"/><br/>
            <edit  id="%d" />
            <button text="路径规划" on-click="on_plan_click"/>
            <button text="清除路径" on-click="on_planedpath_clear"/>
        </group>
        <tree id="%d" autosize-header="true" on-selection-change="on_treepath_selchange">
            <header>
                <item>   路径ID</item>
                <item>路径点数量</item>
                <item> 对象名称</item>
                <item> 目标位姿</item>
            </header>
        </tree>

        <group layout="grid">
            <button text="发送路径" on-click="on_traj_new_click"/>
            <button text="开始运动" on-click="on_traj_start_click"/><br/>
            <button text="暂停运动" on-click="on_traj_pause_click"/>
            <button text="停止运动" on-click="on_traj_stop_click"/>
        </group>
    ]],const.UI.comboSel,const.UI.editPath,const.UI.buttonPath,const.UI.editTarget,const.UI.treePathStatus)
    local tab1=tools:createUiTab(tab1Xml,"主界面")
    ----------------------------------------------------------------------------------------------

    local tab2Xml=string.format([[
    <group layout="form">
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
    </group>
    <group layout="grid">
        <button text="打开ADS连接" on-click="on_ads_init_click"/>
        <button text="关闭ADS连接" on-click="on_ads_destroy_click"/><br/>
    </group>
    <button text="加载默认设置" on-click="on_load_config_click"/>
    <button text="保存当前设置" on-click="on_save_config_click"/>
    ]],const.UI.remoteID,const.UI.remoteIP,const.UI.localID,const.UI.editReadAddr,const.UI.editWriteAddr)
    
    local tab2=tools:createUiTab(tab2Xml,"ADS设置")
--------------------------------------------------------------------
    local tab3Xml=string.format([[
        <label style="font:13px;color:rgba(0,0,255,255)" text="起始位置 " />
        <edit  id="%d"  />
        <label style="font:13px;color:rgba(0,0,255,255)" text="时间间隔 " />
        <edit  id="%d" /> ]],const.UI.editIndex,const.UI.editStep)
    local tab3=tools:createUiTab(tab3Xml,"路径设置","form")
    ---------------------------------------------------------------

    local tab4Xml=string.format([[
        <group layout="grid">
            <checkbox style="font:13px;color:rgba(0,0,255,255)" text="关节状态" on-change="on_check_readstatus" id="%d" checked="false" />
            <checkbox style="font:13px;color:rgba(0,0,255,255)" text="关节位置" on-change="on_check_readstatus" id="%d" checked="false" />
        </group>
        <tree id="%d" autosize-header="true" on-selection-change="on_treestatus_change">
            <header>
                <item>   关节编号   </item>
                <item>关节位置(弧度)))</item>
            </header>
        </tree>
        <group layout="form" id="9100" visible="false">
            <label text="Object name" /><label id="9020" />
            <label text="Object position" /><label id="9030" />
            <label text="Object orientation" /><label id="9040" />
        </group>
        ]],902,903,const.UI.treeJointStatus)
    local tab4=tools:createUiTab(tab4Xml,"关节反馈")
    ---------------------------------------------------------------

    return tools:createUiFromTabs({tab1,tab2,tab3,tab4},{"RDS智能分拣系统","400,600","290,90"})

end



function on_treepath_selchange(ui,id,itemid)
       if itemid>const.UI.treePathStatus then
            local n=itemid-const.UI.treePathStatus
            local msgTable={"disp",pathDb[n]}
            local msg=sim.packTable(msgTable)
            simB0.publish(topicPubPlanCmd,msg)
       end
end
---############## UI 函数 ###############-
function onComboselChanged(ui,id,index)
    comboSel=index
    simUI.setWidgetVisibility(ui,const.UI.groupManul,comboSel==1)
    simUI.setWidgetVisibility(ui,const.UI.groupAuto,comboSel==0)
end


function on_save_config_click(ui,id)
    tools:saveUiConfig()
end

function on_load_config_click(ui,id)
    tools:loadUiConfig(true)
end

function on_treestatus_change(ui,id,itemid)
    
--     simUI.setWidgetVisibility(ui,9100,true)
--     local h=itemid-2
--     local p=sim.getObjectPosition(h,-1)
--     local o=sim.getObjectOrientation(h,-1)
--     simUI.setLabelText(ui,9020,sim.getObjectName(h))
--     simUI.setLabelText(ui,9030,string.format('x: %f  y: %f  z: %f', p[1], p[2], p[3]))
--     simUI.setLabelText(ui,9040,string.format('a: %f  b: %f  g: %f', o[1], o[2], o[3]))
end

function on_check_readstatus(ui,id,newVal)
    if newVal>0 then
        sim.setIntegerSignal("upload",1)
    end
end

function on_openfile_click(ui,id)
    
    local f=sim.fileDialog(sim.filedlg_type_load,'打开文件','','','路径文件','txt')
    if f then
        simUI.setEditValue(ui,const.UI.editPath,f,true)
        -- ret=sim.msgBox(sim.msgbox_type_info,sim.msgbox_buttons_ok,'File Read Error',"The specified file could not be read.")
        -- sim.msgbox_return_ok
    end
end

function on_importpath_click(ui,id)
    local path,num=parsePathFileContent(ui)
    print(path)
    if path then
        simUI.addTreeItem(ui,const.UI.treePathStatus,const.UI.treePathStatus+1,{""..1,""..num,"",""})
        print("count"..simUI.getColumnCount(ui,const.UI.treePathStatus))
    end
end



function on_plan_click(ui,id)
    if #pathDb>=20 then
        print("最多只可保存20条路径，请清楚路径后重新规划")
        return
    end
    local objName=simUI.getEditValue(ui,const.UI.editTarget)
    if objName=="" then
        local handles=sim.getObjectSelection()
        if handles then
            objName=sim.getObjectName(handles[#handles])
        else
            print("请输入要抓取的目标名称或直接选中目标")
            return
        end
    end
    local msgTable={"new",objName,4}
    local msg=sim.packTable(msgTable)
    simB0.publish(topicPubPlanCmd,msg)
end

function on_planedpath_clear(ui,id)
    -- local data=tools:readInfo(const.PATHNAME,sim.getObjectHandle("path"))
    simUI.clearTree(ui,const.UI.treePathStatus)
    pathDb={}
end

function on_traj_new_click(ui,id)
    if isInMotion then
        print("机械臂运动过程中无法发送新轨迹")
        return
    end
    local index=math.ceil(tonumber(simUI.getEditValue(ui,const.UI.editIndex)))
    local step=math.ceil(tonumber(simUI.getEditValue(ui,const.UI.editStep)))
    local path,num=parsePathFileContent(ui)
    if  path then
        print(string.format("发送命令:新的路径,路径点数目: %s",num))
        local msgTable=tools:packTrajData(const.COM.TRAJ_CMD_NEW,path,index,step)
        local msg=sim.packTable(msgTable)
        simB0.publish(topicPubTrajCmd,msg)
    end
end


function on_ads_init_click(ui,id)
        print("正在打开ADS连接")
        local remoteID=simUI.getEditValue(ui,const.UI.remoteID)
        local remoteIP=simUI.getEditValue(ui,const.UI.remoteIP)
        local localID=simUI.getEditValue(ui,const.UI.localID)
        writeAddr=simUI.getEditValue(ui,const.UI.editWriteAddr)
        readAddr=simUI.getEditValue(ui,const.UI.editReadAddr)
        remoteID=tools:parseADSNetID(remoteID) --解析为number table
        localID=tools:parseADSNetID(localID) --解析为number table
        adsHasInit=simADS.create(remoteID,remoteIP,localID)
        if adsHasInit then
            simADS.read(readAddr,0,simADS_handle_open)    --open read Handle
        end
        if adsHasInit then
            simADS.write(writeAddr,{},simADS_handle_open) --open write handle 
        end
end

function on_ads_destroy_click(ui,id)
    print("关闭ADS连接")
    if adsHasInit then
        simADS.write(writeAddr,{},simADS_handle_close)  --close write Handle
        simADS.read(readAddr,0,simADS_handle_close)     --close read Handle
        simADS.destory()
        adsHasInit=nil
    end
end



function on_traj_start_click(ui,id)
    if isInMotion then
        print("机械臂已经在运动")
        return
    end
    print("发送命令:执行运动")
    local msgTable=tools:packTrajData(const.COM.TRAJ_CMD_START)
    local msg=sim.packTable(msgTable)
    simB0.publish(topicPubTrajCmd,msg)
end

function on_traj_pause_click(ui,id)
    print("发送命令:暂停运动")
    local msgTable=tools:packTrajData(const.COM.TRAJ_CMD_PAUSE)
    local msg=sim.packTable(msgTable)
    simB0.publish(topicPubTrajCmd,msg)
end

function on_traj_stop_click(ui,id)
    print("发送命令:停止运动")
    local msgTable=tools:packTrajData(const.COM.TRAJ_CMD_STOP)
    local msg=sim.packTable(msgTable)
    simB0.publish(topicPubTrajCmd,msg)
end


function on_sub_robostates(packedData)
    local data=sim.unpackTable(packedData)
    isInMotion=data.inMotion --赋值给全局变量
    for i=1,7 do
        simUI.updateTreeItemText(tools.ui,const.UI.treeJointStatus,const.UI.treeJointStatus+i,{""..i,""..data.position[i]})
    end

end

function on_sub_planedpath(packedData)
    local msg=sim.unpackTable(packedData)
    -- tools:writeInfo(const.DB.TRAJ,msg,sim.getObjectHandle("path")) --把数据存入path dummy
    local n=#pathDb+1
    pathDb[n]=msg.path  --把数据存入pathDb
    simUI.addTreeItem(tools.ui,const.UI.treePathStatus,const.UI.treePathStatus+n,{""..n,""..#msg.path/7,msg.objname,""})
end
---@@@@@@@@@@@ UI 函数 @@@@@@@@@@@--



----#############运行函数##################---
function sysCall_threadmain()
    local handle=sim.getObjectAssociatedWithScript(sim.handle_self)
    local xml=initUIXml()
    tools:createUi(xml,handle,loadUiConfig,saveUiConfig)
    comboSel=0
    pathTreeSel=0    --path tree选中的id
    isInMotion=true  --机械臂是否在运动
    pathDb={}        --规划出的轨迹保存的字典
    --#####BlueZero##################
    tools:initResolverB0()
    nodeUI=simB0.create("uiNode")
    topicPubPlanCmd=simB0.createPublisher(nodeUI,const.TOPICS.PLANCMD)
    topicPubTrajCmd=simB0.createPublisher(nodeUI,const.TOPICS.TRAJCMD)
    topicSubRobostates=simB0.createSubscriber(nodeUI,const.TOPICS.ROBOSTATES,'on_sub_robostates')
    topicSubPlanedpath=simB0.createSubscriber(nodeUI,const.TOPICS.PLANEDPATH,'on_sub_planedpath')
    simB0.init(nodeUI)
    -- on_ads_init_click(tools.ui) --初始化ads链接
    
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        if nodeUI then
            -- simB0.spinOnce(nodeUI)
            pcall(simB0.spinOnce,nodeUI)
        end
        sim.switchThread() -- resume in next simulation step
    end
end


--停止
function sysCall_cleanup()
    -- on_ads_destroy_click(tools.ui) --关闭ADS连接
    if nodeUI then
        simB0.cleanup(nodeUI)
        if topicPubPlanCmd then
            simB0.destroyPublisher(topicPubPlanCmd)
        end
        if topicPubTrajCmd then
            simB0.destroyPublisher(topicPubTrajCmd)
        end
        if topicSubRobostates then
            simB0.destroySubscriber(topicSubRobostates)
        end        
        if topicSubPlanedpath then
            simB0.destroySubscriber(topicSubPlanedpath)
        end
        simB0.destroy(nodeUI)
    end
    tools:destroyUi(true)
end
















