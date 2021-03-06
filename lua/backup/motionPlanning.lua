visualizePath=function(path)
    if not _lineContainer then
        _lineContainer=simAddDrawingObject(sim_drawing_lines,3,0,-1,99999,{1,1,0})
    end
    simAddDrawingObjectItem(_lineContainer,nil)
    if path then
        forbidThreadSwitches(true)
        local initConfig=getConfig()
        local l=#jh
        local pc=#path/l
        for i=1,pc-1,1 do
            local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
            local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
            setConfig(config1)
            local lineDat=simGetObjectPosition(ikTip,-1)
            setConfig(config2)
            local p=simGetObjectPosition(ikTip,-1)
            lineDat[4]=p[1]
            lineDat[5]=p[2]
            lineDat[6]=p[3]
            simAddDrawingObjectItem(_lineContainer,lineDat)
        end
        setConfig(initConfig)
        forbidThreadSwitches(false)
    end
    simSwitchThread()
end

displayInfo=function(txt)
    if dlgHandle then
        simEndDialog(dlgHandle)
    end
    dlgHandle=nil
    if txt and #txt>0 then
        dlgHandle=simDisplayDialog('Jaco info',txt,sim_dlgstyle_message,false)
        simSwitchThread()
    end
end

_getJointPosDifference=function(startValue,goalValue,isRevolute)
    local dx=goalValue-startValue
    if (isRevolute) then
        if (dx>=0) then
            dx=math.mod(dx+math.pi,2*math.pi)-math.pi
        else
            dx=math.mod(dx-math.pi,2*math.pi)+math.pi
        end
    end
    return(dx)
end

_applyJoints=function(jointHandles,joints)
    for i=1,#jointHandles,1 do
        -- simSetJointTargetPosition(jointHandles[i],joints[i])
        simSetJointPosition(jointHandles[i],joints[i])
    end
end

generatePathLengths=function(path)
    -- 返回每个构型距离递增table,用于RML速度映射,大小等于规划的目标构型的数目，此处为200
    local d=0
    local l=#jh
    local pc=#path/l
    local retLengths={0}
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
        retLengths[i+1]=d
    end
    return retLengths
end

getShiftedMatrix=function(matrix,localShift,dir)
    -- Returns a pose or matrix shifted by vector localShift
    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    m[4]=m[4]+dir*(m[1]*localShift[1]+m[2]*localShift[2]+m[3]*localShift[3])
    m[8]=m[8]+dir*(m[5]*localShift[1]+m[6]*localShift[2]+m[7]*localShift[3])
    m[12]=m[12]+dir*(m[9]*localShift[1]+m[10]*localShift[2]+m[11]*localShift[3])
    return m
end

forbidThreadSwitches=function(forbid)
    --开启或者禁止线程自动切换，多次开启/禁止时只会调用一次simSetThreadAutomaticSwitch
    --用于占用系统资源，例如想临时设置目标位置，但是不想立即显示，如果不禁用线程切换则会
    if forbid then
        forbidLevel=forbidLevel+1
        if forbidLevel==1 then
            simSetThreadAutomaticSwitch(false)
        end
    else
        forbidLevel=forbidLevel-1
        if forbidLevel==0 then
            simSetThreadAutomaticSwitch(true)
        end
    end
end

findCollisionFreeConfig=function(matrix)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 寻找无碰撞的目标构型
    simSetObjectMatrix(ikTarget,-1,matrix)

    -- This robot has 4 joints that have a huge range (i.e. -10'000 - +10'000 degrees)
    -- And since we do not want to search that huge space, we limit the range around the current configuration
    -- We actually do the same during path search
    local cc=getConfig()
    local jointLimitsL={}
    local jointRanges={}
    for i=1,#jh,1 do
        jointLimitsL[i]=-math.pi
        jointRanges[i]=2*math.pi
    end
    jointLimitsL[2]=-0.67*math.pi
    jointRanges[2]=0.67*math.pi*2
    jointLimitsL[4]=-0.67*math.pi
    jointRanges[4]=0.67*math.pi*2
    jointLimitsL[6]=-0.67*math.pi
    jointRanges[6]=0.67*math.pi*2
    --jh:关节句柄 
    -- 0.65：当distance小于该值时，进行IK解算至目标位置，过大会导致IK计算量过大，过小会导致小步的迭代
    -- 10:maxTimeInMs，超过10ms无结果则停止搜索
    -- nil:metric default:{1.0,1.0,1.0,0.1}. 
    -- distance=sqrt((dx*metric[0])^2+(dy*metric[1])^2+(dz*metric[2])^2+(angle*metric[3])^2)
    local c=simGetConfigForTipPose(ikGroup,jh,0.65,10,nil,collisionPairs,nil,jointLimitsL,jointRanges)
    return c
end

findSeveralCollisionFreeConfigs=function(matrix,trialCnt,maxConfigs)
    -- 搜索可用的目标构型，同时进行静态避障
    -- matrix:目标姿态
    -- trialCnt:尝试寻找目标构型的次数
    -- maxConfigs:返回成功构型的最大个数
    -- cs:返回计算出的目标构型的table，大小为N*6的二维数组,N<=maxConfigs
    -- simSetObjectMatrix(ikTarget,-1,matrix) --将目标设置为target
    local cc=getConfig()
    local cs={}
    local l={}
    simAddStatusbarMessage("开始寻找可用的目标构型:")
    for i=1,trialCnt,1 do
        --寻找最多maxConfigs个目标构型
        local c=findCollisionFreeConfig(matrix)
        -----------------打印消息----------------
        if c then
            simAddStatusbarMessage("第 "..i.." 次搜索目标构型: Success")
        else
            simAddStatusbarMessage("第 "..i.." 次搜索目标构型: Falied")
        end
        ---------------------------------------
        if c then
            --计算构型的能量距离(构型空间)
            local dist=getConfigConfigDistance(cc,c)
            local p=0
            local same=false
            -- 有可能获得的构型相同，为避免返回多个相同的目标构型
            -- 先检查构型能量，然后检查每个关节运动角度的偏差是否大于阈值
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    same=true
                    for k=1,#jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=maxConfigs then
            break
        end
    end
    --如果无可以的构型，则返回空
    if #cs==0 then
        cs=nil
    end
    return cs
end

getConfig=function()
    -- 返回机械臂的当前构型
    local config={}
    for i=1,#jh,1 do
        config[i]=simGetJointPosition(jh[i])
    end
    return config
end

setConfig=function(config)
    -- 执行机械臂到指定构型
    if config then
        for i=1,#jh,1 do
            simSetJointPosition(jh[i],config[i])
        end
    end
end

getConfigConfigDistance=function(config1,config2)
    --计算2个目标构型之间的能量距离(构型空间)
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

getPathLength=function(path)
    -- 计算整条路径的能量距离(构型空间)
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

findPath=function(startConfig,goalConfigs,cnt)
    -- 计算起始构型到目标构型的路径，每个目标构型计算cnt次
    --返回最短的路径以及长度(构型空间的能量距离)
    --部分关节的运动范围过大,例如+-10'000,将会导致搜索空间过大/速度过慢/效率降低,所以限制关节的运动范围
    local jointLimitsL={}
    local jointLimitsH={}
    
    for i=1,#jh,1 do
        jointLimitsL[i]=startConfig[i]-2*math.pi
        -- if jointLimitsL[i]<-10000 then jointLimitsL[i]=-10000 end
        jointLimitsH[i]=startConfig[i]+2*math.pi
        -- if jointLimitsH[i]>10000 then jointLimitsH[i]=10000 end
    end
    -- --设置2/3关节的运动范围(这2个关节的能量消耗最大)
    -- jointLimitsL[2]=47*math.pi/180
    -- jointLimitsH[2]=313*math.pi/180
    -- jointLimitsL[3]=19*math.pi/180
    -- jointLimitsH[3]=341*math.pi/180

    

    local task=simExtOMPL_createTask('task') --创建任务
    simExtOMPL_setAlgorithm(task,algoOMPL)   --设置算法
    simExtOMPL_setVerboseLevel(task,0)       --设置消息级别
    
    local jSpaces={}
    for i=1,#jh,1 do
        local proj=i
        if i>3 then proj=0 end
        --设置关节状态空间：名称，类型，关节句柄table,关节下限table,关节上限table,是否用于计算关节映射table(为true时)：1,2,3关节映射，其他几个关节不映射
        --weight:默认为1.0，用于计算不同构型之间的距离(在后面自定义)。返回值为该关节空间的句柄
        jSpaces[#jSpaces+1]=simExtOMPL_createStateSpace('j_space'..i,sim_ompl_statespacetype_joint_position,jh[i],{jointLimitsL[i]},{jointLimitsH[i]},proj)
    end
    simExtOMPL_setStateSpace(task,jSpaces)
    simExtOMPL_setCollisionPairs(task,collisionPairs) --设置碰撞对
    simExtOMPL_setStartState(task,startConfig)        --设置初始构型
    simExtOMPL_setGoalState(task,goalConfigs[1])      --设置目标构型
    for i=2,#goalConfigs,1 do
        simExtOMPL_addGoalState(task,goalConfigs[i])  --添加其他的目标构型
    end
    local path=nil
    local l=999999999999
--    forbidThreadSwitches(true)
    --计算CNT次目标构型
    for i=1,cnt,1 do
        -- 等价于:
        -- simExtOMPL_setup(task)
        -- if simExtOMPL_solve(task, maxTime) then
        --     simExtOMPL_simplifyPath(task, maxSimplificationTime)
        --     simExtOMPL_interpolatePath(task, stateCnt)
        --     result,path = simExtOMPL_getPath(task)
        -- end
        --参数：maxSimplificationTime,用于简化路径的时间，-1表示默认；stateCnt：返回的差值路径点(构型)数量
        local res,_path=simExtOMPL_compute(task,singlePlanTime,-1,configCount)
        if res and _path then
            local _l=getPathLength(_path)
            if _l<l then
                l=_l
                path=_path
            end
        end
        if path then
            visualizePath(path)
        end
    end
--    forbidThreadSwitches(false)
    simExtOMPL_destroyTask(task)--清除任务
    return path,l
end

findShortestPath=function(startConfig,goalConfigs,searchCntPerGoalConfig)
    -- 计算起始构型到目标构型的路径，每个目标构型计算cnt次
    --返回最短的路径以及长度(构型空间的能量距离)
    --其中onepath 为N*6=1200,即路径上200个构型*6个关节的角度
    local onePath,onePathLength=findPath(startConfig,goalConfigs,searchCntPerGoalConfig)
    if onePath then
        return onePath,generatePathLengths(onePath)
    else
        return nil,nil
    end
end

generateIkPath=function(startConfig,goalPose,steps,ignoreCollisions)
    --生成从当前构型到目标位姿的、线性的、无碰撞的构型
    forbidThreadSwitches(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    simSetObjectMatrix(ikTarget,-1,goalPose)
    local coll=collisionPairs
    if ignoreCollisions then
        coll=nil
    end
    local c=simGenerateIkPath(ikGroup,jh,steps,coll)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    if c then
        return c, generatePathLengths(c)
    end
end

executeMotion=function(path,lengths,maxVel,maxAccel,maxJerk,pathName)
    dt=simGetSimulationTimeStep()

    -- 1.折算出每个关节最大的速度
    jointsUpperVelocityLimits={}
    for j=1,7,1 do
        res,jointsUpperVelocityLimits[j]=simGetObjectFloatParameter(jh[j],sim_jointfloatparam_upper_limit)
    end
    velCorrection=1

    simSetThreadSwitchTiming(200)
    while true do
        posVelAccel={0,0,0}
        targetPosVel={lengths[#lengths],0}
        pos=0
        res=0
        previousQ={path[1],path[2],path[3],path[4],path[5],path[6],path[7]}
        local rMax=0
        rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
        while res==0 do
            res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
            if (res>=0) then
                l=posVelAccel[1]
                for i=1,#lengths-1,1 do
                    l1=lengths[i]
                    l2=lengths[i+1]
                    if (l>=l1)and(l<=l2) then
                        t=(l-l1)/(l2-l1)
                        for j=1,7,1 do
                            q=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim_joint_revolute_subtype)*t
                            dq=_getJointPosDifference(previousQ[j],q,jt[j]==sim_joint_revolute_subtype)
                            previousQ[j]=q
                            r=math.abs(dq/dt)/jointsUpperVelocityLimits[j]
                            if (r>rMax) then
                                rMax=r
                            end
                        end
                        break
                    end
                end
            end
        end
        simRMLRemove(rmlHandle)
        if rMax>1.001 then
            velCorrection=velCorrection/rMax
        else
            break
        end
    end
    simSetThreadSwitchTiming(2)

    -- 2. 执行动作
    posVelAccel={0,0,0}
    targetPosVel={lengths[#lengths],0}
    pos=0
    res=0
    jointPos={}
    rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
    while res==0 do
        dt=simGetSimulationTimeStep()
        res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
        if (res>=0) then
            l=posVelAccel[1]
            for i=1,#lengths-1,1 do
                ---------------设置当前的构型的步数-----------
                --当command!="start"时会卡在这里
                repeat
                    command=simGetStringSignal("command")
                    --当命令为replan时,直接退出并返回标识
                    if command=="replan" then
                        simRMLRemove(rmlHandle)
                        return 1
                    end
                until(command=="start")
                simSetIntegerSignal(pathName,i)
                -----------------------------------------
                l1=lengths[i]
                l2=lengths[i+1]
                if (l>=l1)and(l<=l2) then
                    t=(l-l1)/(l2-l1)
                    for j=1,7,1 do
                        jointPos[j]=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim_joint_revolute_subtype)*t
                    end
                    _applyJoints(jh,jointPos)
                    break
                end
            end
        end
        simSwitchThread()
    end
    simRMLRemove(rmlHandle)
end


savePath=function(filename,path,lengths)
    -- 保存自定义路径
    simWriteCustomDataBlock(rdsHandle,filename..'.pathData1',simPackFloatTable(path))
    simWriteCustomDataBlock(rdsHandle,filename..'.pathLength1',simPackFloatTable(lengths))
end

loadPath=function(filename)
    path=simReadCustomDataBlock(rdsHandle,filename..'.pathData1')
    if (not path) then return nil end
    path=simUnpackFloatTable(path)

    lengths=simReadCustomDataBlock(rdsHandle,filename..'.pathLength1')
    if (not lengths) then return nil end
    lengths=simUnpackFloatTable(lengths)
    return path,lengths
end

function pathPlaning()
    --加载保存的路径
    -- path,lengths=loadPath('jacoPath_1')
    --进行路径规划
    simAddStatusbarMessage("路径规划开始")
    local m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{-0.05,0,0.1},-1)
    simAddStatusbarMessage('开始搜索可用目标构型...')

    --开始搜索目标构型，搜索300次，找到最多5个可用的目标构型
    local configs=findSeveralCollisionFreeConfigs(m,300,5)

    --如果没有找到可用的目标构型则返回
    if configs==nil then
        simAddStatusbarMessage('未搜索到可用的目标构型，无法进行路径规划')
        return nil,nil
    end
    simAddStatusbarMessage('搜索到 '..#configs..' 个可用的目标构型')

    --计算路径，返回200个构型*6个关节的角度值，200个构型对应的关节距离累加值
    simAddStatusbarMessage('开始进行路径规划')
    path,lengths=findShortestPath(getConfig(),configs,configPlanAttempts)
    return path,lengths
end

-- START HERE:
jh={-1,-1,-1,-1,-1,-1,-1}
jt={-1,-1,-1,-1,-1,-1,-1}
for i=1,7,1 do
    jh[i]=simGetObjectHandle('j'..i)
    jt[i]=simGetJointType(jh[i])
end
rdsHandle=simGetObjectHandle('RDS_01')    
ikTarget=simGetObjectHandle('RDS_01_target')
ikTip=simGetObjectHandle('RDS_01_tip')
ikGroup=simGetIkGroupHandle('RDS_IK_PINV')
target1=simGetObjectHandle('jacoTarget1')
target2=simGetObjectHandle('jacoTarget2')
------------------------------------------------------
collisionHandle=simGetCollectionHandle("CollisionObjects") --机械臂的碰撞对象集合的句柄
configCount=200
simSetStringSignal("command","start")       --初始化启动信号
simSetIntegerSignal("configNumer_1",1)        --初始当前路径点标识(1~configCount)
--获得目标

--@@@@@@@@@@@@@@@@@@@@@@@@参数表@@@@@@@@@@@@@@@@@@@@@@@@
--碰撞对：1-2:机械臂本身不发生碰撞，3-4：机械臂和其他的对象不发生碰撞
-- collisionPairs={simGetCollectionHandle('RDS_01'),simGetCollectionHandle('RDS_01'),simGetCollectionHandle('RDS_01'),collisionHandle}
collisionPairs={simGetCollectionHandle('RDS_01'),collisionHandle}
maxVel=1    --最大速度
maxAccel=1  --最大加速度
maxJerk=8000 --最大加加速度
forbidLevel=0 
metric={0.2,1,0.8,0.1,0.1,0.1,0.1} --关节能量权重
ikSteps=20                     --Ik
singlePlanTime=4 -- 单次路径规划的最长允许时间，单位s
-- sim_ompl_algorithm_BKPIECE1
algoOMPL=sim_ompl_algorithm_RRTConnect -- OMPL路径规划所用的算法
configPlanAttempts=4 -- 单个目标构型的路径规划最大次数
-- @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

--清除上次生成的路径
simWriteCustomDataBlock(rdsHandle,'',nil)

repeat
    
    path,lengths=pathPlaning()
    --如果找到最短路径则保存路径和能量累加值
    if path then
        simAddStatusbarMessage("路径规划成功，保存该路径")
        savePath('jacoPath_1',path,lengths)
        visualizePath(path)--可视化路径
        --路径参数化，然后执行
        simAddStatusbarMessage("执行动作1: 达到目标姿态点")
        replan=executeMotion(path,lengths,maxVel,maxAccel,maxJerk,"configNumer_1")
    else
        simAddStatusbarMessage('未搜索到可用路径,路径规划失败')
        return
    end
until(replan~=1)


-- --靠近杯子(IK)
simAddStatusbarMessage("执行动作2：靠近杯子(逆解算)")
local m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,0.01,0},-1)
path,lengths=generateIkPath(getConfig(),m,ikSteps,true) --不考虑避障
if path then
    executeMotion(path,lengths,maxVel,maxAccel,maxJerk,"configNumer_2")
end



-- -- -- 合并机械手,抓住杯子
simAddStatusbarMessage("执行动作3：合并机械手")
sim.setIntegerSignal('RG2_close',1)
simWait(1.0)



-- -- -- 举起杯子(IK)
simAddStatusbarMessage("执行动作4：举起杯子(逆解算)")
local m=simGetObjectMatrix(target2,-1)
path,lengths=generateIkPath(getConfig(),m,ikSteps,true)
if path then
    executeMotion(path,lengths,maxVel,maxAccel,maxJerk,"configNumer_3")
end



