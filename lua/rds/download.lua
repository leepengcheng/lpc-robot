--@@控制脚本-路径下载：
local dir=os.getenv("RDS") --添加工作路径
package.path=string.format( "%s;%s/lib/?.lua",package.path,dir)

local const=require("const")
local tools=require("tools")















--接受发布轨迹的命令
function on_sub_trajcmd(packedData)
    print("正在转发ADS数据到控制器")
    local data=sim.unpackTable(packedData)
    simADS.write("MAIN.traj_path",data,simADS_handle_none)
end


function sysCall_threadmain()
    -- adsHasInit=initADSConnection()
        -- --------读机器人状态
    -- if readAddr and adsHasInit  then
    --     readRobotStatus(true)
    -- end
    --#####BlueZero##################
    downloadNode=simB0.create("downloadNode")
    --sub_Topic接受轨迹命令 new|start|pause|stop,转换为ADS
    topicSubTrajCmd=simB0.createSubscriber(downloadNode,const.TOPICS.TRAJCMD,'on_sub_trajcmd')
    simB0.init(downloadNode)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        simB0.spinOnce(downloadNode)
        sim.switchThread() -- resume in next simulation step
    end
end


--停止
function sysCall_cleanup()

        -- readAddr=READ_ADDR_BACKUP --销毁时需要回原地址

    -- if writeAddr and adsHasInit then
    --     simADS.write(writeAddr,{},simADS_handle_close)  --close write Handle
    -- end


    -- if readAddr and adsHasInit then
    --     simADS.read(readAddr,0,simADS_handle_close)     --close read Handle
    -- end
    
    -- if adsHasInit then
    --     simADS.destory()
    --     adsHasInit=false
    -- end
    if downloadNode then
        simB0.cleanup(downloadNode)
        if topicSubTrajCmd then
            simB0.destroySubscriber(topicSubTrajCmd)
        end
        simB0.destroy(downloadNode)
    end

end
