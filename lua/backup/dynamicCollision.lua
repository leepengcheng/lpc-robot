function getConfig()
    -- 返回机械臂的当前构型
    local config={}
    for i=1,#jh,1 do
        config[i]=simGetJointPosition(jh[i])
    end
    return config
end


function setConfig(config)
    -- 设置虚拟机械臂到指定构型
    if config then
        for i=1,#jh,1 do
            simSetJointPosition(jh_[i],config[i])
        end
    end
end

function loadPath(filename)
    path=simReadCustomDataBlock(rdsHandle,filename..'.pathData1')
    if (not path) then return nil end
    path=simUnpackFloatTable(path)
    return path
end


-- 检测余下路径是否与目标是否有碰撞,返回碰撞的构型序号
--进行障碍物检查逻辑
--1、获取当前构型绝对编号index，此编号有函数executeMotion每个循环周期都发送更新
--2、从index编号开始，循环设置虚拟机械臂余下的构型，并检查虚拟构型与障碍物是否有碰撞
--3、如果有碰撞，那么返回当前的绝对位置n和相对位置n-index
function  checkCollision()
    path=loadPath('jacoPath_1') --获取所有路径的构型
    if path then
        index=simGetIntegerSignal("configNumer_1")--获取当前构型的步数
        ncfg=#path/7
        for n=index,ncfg-1 do 
            initConfig={path[n*7+1],path[n*7+2],path[n*7+3],path[n*7+4],path[n*7+5],path[n*7+6],path[n*7+7]}
            --if index==ncfg-1 then
                setConfig(initConfig)
            --end
            if simCheckCollision(rdsCHandle_,cubeCHandle)==1 then 
                simAddStatusbarMessage("Find collision..."..n)
                return n,n-index
            end 
        end
    end
    return 0
end


if (sim_call_type==sim_childscriptcall_initialization) then

    -- 初始化
    --对象获取
    C1Handle=simGetObjectHandle('Collision1')
    C2Handle=simGetObjectHandle('Collision2')
    rdsHandle=simGetObjectHandle('RDS_01')   --机械臂对象(保存路径的对象)
    rdsCHandle=simGetCollectionHandle("RDS_01")    --机械臂集合
    rdsCHandle_=simGetCollectionHandle("RDS_01_")  --虚拟机械臂
    cubeCHandle=simGetCollectionHandle("CollisionCubes")
    simWriteCustomDataBlock(rdsHandle,'',nil)
    -- 或得机械臂关节的句柄
    jh={-1,-1,-1,-1,-1,-1,-1}
    jh_={-1,-1,-1,-1,-1,-1,-1}
    arm_={-1,-1,-1,-1,-1,-1,-1}
    for i=1,7,1 do
        jh[i]=simGetObjectHandle('j'..i)
        jh_[i]=simGetObjectHandle(string.format("j%s_v",i))
        arm_[i]=simGetObjectHandle(string.format("link%s_v",i))
    end
    
    ----控制参数
    THRESHOLD_DISTANCE=0.5                   --距离检测的阈值,触发报警的障碍物距离
    THRESHOLD_INTERVAL=1.0                   --检查路径碰撞的时间间隔s
    STOP_RATIO=0.2                          --目标上有障碍物时停止的阈值
    CONFIG_NUM=200                           --生成的构型数目
    REPLAN_INTERVAL=2                        --重新路径规划的时间间隔
    
end





if (sim_call_type==sim_childscriptcall_actuation) then
    --逻辑如下
    --1，先判断实际机械臂与障碍距离是否小于设定阈值
    --2、如果小于，那么进行虚拟机械臂与障碍物检查，否则依旧发送命令command=start
    --3、如果有碰撞，那么返回发生碰撞的构型绝对位置nindex和相对位置nconfigs
    --4、如果相对位置小于设定阈值，那么接收命令command
    --5、如果命令command=start，那么记录当前时间，并发送停止命令command=sotp，开始记录时间
    --6、如果时间唱过阈值，那么重新规划，发送命令command=replan
    
    --大于距离阈值时res=0,小于阈值时res=1,其他情况res=-1
    -- disData: disData[1]-disData[6] 代表距离分段
    -- disData[7]是实体的距离. disData is nil if result is different from 1

    res,distance=sim.checkDistance(rdsCHandle,cubeCHandle,THRESHOLD_DISTANCE)--计算障碍物到机械臂的最短距离
    --当res=1时表示最短距离小于THRESHOLD_DISTANCE
    if res==1 then
        minDist=distance[7] --获得最短距离
        --当障碍物离机械臂的最短距离小于阈值时
        if minDist<THRESHOLD_DISTANCE then
            nindex,nconfigs=checkCollision() --检测是否会碰撞上,返回碰撞的步数
            if nconfigs and nconfigs>0  then
                simAddStatusbarMessage(string.format("检测到规划路径上有干涉,绝对位置:%d 步,相对位置:%d 步",nindex,nconfigs))
                if nconfigs<=STOP_RATIO*CONFIG_NUM then
                    command=simGetStringSignal("command")
                    if command=="start" then
                        lastTime=simGetSimulationTime()
                        simSetStringSignal("command","stop")
                    end
                    thisTime=simGetSimulationTime()
                    --如果暂停的时间大于REPLAN_INTERVAL
                    if thisTime-lastTime>=REPLAN_INTERVAL then
                        simSetStringSignal("command","replan")
                    end
                end
            else
                simSetStringSignal("command","start")     --发送启动的信号
            end

        end
        -- print(string.format( "%d %f  %f  %f  %f  %f  %f  %f",res,disData[1],disData[2],disData[3],disData[4],disData[5],disData[6],disData[7])) 
    else
        simSetStringSignal("command","start")             --发送启动信号
    end

    --每隔 THRESHOLD_INTERVAL时间就进行检测
    -- thisTime=simGetSimulationTime()
    
end
