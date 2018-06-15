--可视化末端的轨迹
--@path:计算出的路径点(关节空间/构型空间)
visualizePath = function(path)
    --如果没有_lineContainer则创建
    if not _lineContainer then
        _lineContainer = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 99999, {1, 1, 0})
    end
    sim.addDrawingObjectItem(_lineContainer, nil)
    if path then
        forbidThreadSwitches(true)
        local initConfig = getConfig()
        local l = #jhandles
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
            setConfig(config1)
            local lineDat = sim.getObjectPosition(ikTip, -1)
            setConfig(config2)
            local p = sim.getObjectPosition(ikTip, -1)
            lineDat[4] = p[1]
            lineDat[5] = p[2]
            lineDat[6] = p[3]
            sim.addDrawingObjectItem(_lineContainer, lineDat)
        end
        setConfig(initConfig)
        forbidThreadSwitches(false)
    end
    sim.switchThread()
end

--计算关节的差值,限制在(-pi,pi)区间
--@startValue:起始关节角度
--@goalValue:终止关节角度
--@isRevolute:是否为旋转副
getJointPosDifference = function(startValue, goalValue, isRevolute)
    local dx = goalValue - startValue
    if (isRevolute) then
        if (dx >= 0) then
            dx = math.mod(dx + math.pi, 2 * math.pi) - math.pi
        else
            dx = math.mod(dx - math.pi, 2 * math.pi) + math.pi
        end
    end
    return (dx)
end

--执行关节运动
--@jointHandles:关节句柄
--@joints:关节值
applyJoints = function(jointHandles, joints)
    for i = 1, #jointHandles, 1 do
        -- sim.setJointTargetPosition(jointHandles[i],joints[i])
        sim.setJointPosition(jointHandles[i], joints[i])
    end
end

--返回每个构型距离依次递增table,用于RML速度映射
--小等于规划的目标构型的数目，此处为200
--@path:规划出的关节控制的路径点,size=joint_size*config_size
generatePathLengths = function(path)
    local d = 0
    local l = #jhandles
    local pc = #path / l
    local retLengths = {0}
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
        d = d + getConfigConfigDistance(config1, config2)
        retLengths[i + 1] = d
    end
    return retLengths
end

-- 获得沿着指定矩阵的坐标轴放心进行平移后的矩阵
-- @matrix:指定的矩阵
-- @localShift:xyz平移量
-- @dir:direction,方向,-1表示反向
-- @return:平移后的矩阵
getShiftedMatrix = function(matrix, localShift, dir)
    local m = {}
    for i = 1, 12, 1 do
        m[i] = matrix[i]
    end
    m[4] = m[4] + dir * (m[1] * localShift[1] + m[2] * localShift[2] + m[3] * localShift[3])
    m[8] = m[8] + dir * (m[5] * localShift[1] + m[6] * localShift[2] + m[7] * localShift[3])
    m[12] = m[12] + dir * (m[9] * localShift[1] + m[10] * localShift[2] + m[11] * localShift[3])
    return m
end

--开启或者禁止线程自动切换，多次开启/禁止时只会调用一次simSetThreadAutomaticSwitch
--用于占用系统资源，例如想临时设置目标位置，但是不想立即显示，如果不禁用线程切换则会
--@forbid:是否禁止切花线程
forbidThreadSwitches = function(forbid)
    if forbid then
        forbidLevel = forbidLevel + 1
        if forbidLevel == 1 then
            sim.setThreadAutomaticSwitch(false)
        end
    else
        forbidLevel = forbidLevel - 1
        if forbidLevel == 0 then
            sim.setThreadAutomaticSwitch(true)
        end
    end
end

--获取当关节的限位范围
--@return:jointLimitsL/jointRanges 关节下限/关节范围
function getJointLimits()
    local jointLimitsL = {}
    local jointRanges = {}
    local jointLimitsH={}
    for i = 1, #jhandles, 1 do
        --获取关节的参数:是否循环/下限/区间
        cyclic, interval = sim.getJointInterval(jhandles[i])
        if cyclic then
            --设置范围为[-pi,pi]
            jointLimitsL[i], jointRanges[i] = -math.pi, 2 * math.pi
        else
            jointLimitsL[i], jointRanges[i] = interval[1], interval[2]
        end
        jointLimitsH[i]=jointLimitsL[i]+jointRanges[i]
    end
    return jointLimitsL, jointRanges,jointLimitsH
end

-- 寻找1个无碰撞且满足目标位姿构型
-- @matrix:指定末端的位姿
findCollisionFreeConfig = function(matrix)
    sim.setObjectMatrix(ikTarget, -1, matrix)
    --修改关节的运动范围,减少搜索空间
    -- local cc = getConfig()
    --jhandles:关节句柄
    -- 0.65：当distance小于该值时，进行IK解算至目标位置，过大会导致IK计算量过大，过小会导致小步的迭代
    -- 10:maxTimeInMs，超过10ms无结果则停止搜索
    -- nil:metric default:{1.0,1.0,1.0,0.1}.
    -- distance=sqrt((dx*metric[0])^2+(dy*metric[1])^2+(dz*metric[2])^2+(angle*metric[3])^2)
    -- local c = sim.getConfigForTipPose(ikPINV, jhandles, 0.65, 100, nil, collisionPairs, nil, jointLimitsL, jointRanges)
    local c = sim.getConfigForTipPose(ikPINV, jhandles, 0.65, 100, nil, {robotCollectionHandle,collisionCollectionHandle}, nil, jointLimitsL, jointRanges)
    return c
    
end

-- 搜索TopN无碰撞且满足目标位姿构型
-- @matrix:目标姿态
-- @trialCnt:尝试寻找目标构型的次数
-- @maxConfigs:返回成功构型的最大个数
-- @return:cs,返回计算出的目标构型的table，大小为N*6的二维数组,N<=maxConfigs
findTopNCollisionFreeConfigs = function(matrix, trialCnt, maxConfigs)
    sim.setObjectMatrix(ikTarget, -1, matrix) --将目标设置为target
    local cc = getConfig()
    local cs = {}
    local l = {}
    sim.addStatusbarMessage("开始寻找可用的目标构型:")
    for i = 1, trialCnt, 1 do
        --寻找最多maxConfigs个目标构型
        local c = findCollisionFreeConfig(matrix)
        -----------------打印消息----------------
        if c then
            sim.addStatusbarMessage("第 " .. i .. " 次搜索目标构型: Success")
        else
            sim.addStatusbarMessage("第 " .. i .. " 次搜索目标构型: Falied")
        end
        ---------------------------------------
        if c then
            --计算构型的能量距离(构型空间)
            local dist = getConfigConfigDistance(cc, c)
            local p = 0
            local same = false
            -- 有可能获得的构型相同，为避免返回多个相同的目标构型
            -- 先检查构型能量，然后检查每个关节运动角度的偏差是否大于阈值
            for j = 1, #l, 1 do
                if math.abs(l[j] - dist) < 0.001 then
                    same = true
                    for k = 1, #jhandles, 1 do
                        if math.abs(cs[j][k] - c[k]) > 0.01 then
                            same = false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs + 1] = c
                l[#l + 1] = dist
            end
        end
        if #l >= maxConfigs then
            break
        end
    end
    --如果无可以的构型，则返回空
    if #cs == 0 then
        cs = nil
    end
    return cs
end

-- 获得机械臂的当前构型
-- @return:当前构型,table,size=joint_size
getConfig = function()
    local config = {}
    for i = 1, #jhandles, 1 do
        config[i] = sim.getJointPosition(jhandles[i])
    end
    return config
end

-- 设置机械臂的当前构型
-- @config:机械臂的关节值
setConfig = function(config)
    -- 执行机械臂到指定构型
    if config then
        for i = 1, #jhandles, 1 do
            sim.setJointPosition(jhandles[i], config[i])
        end
    end
end

--计算2个目标构型之间的距离(构型空间),sqrt(dx**2)
--@config1:构型1
--@config2:构型2
--@return:2个构型之间的伪距离
getConfigConfigDistance = function(config1, config2)
    local d = 0
    for i = 1, #jhandles, 1 do
        --每个关节*对应的权值
        local dx = (config1[i] - config2[i]) * metric[i]
        d = d + dx * dx
    end
    return math.sqrt(d)
end

-- 计算整条路径的能量距离(构型空间),用于选择规划出的最短路径
-- @path:规划出的路径(关节空间/构型空间)
-- @return:路径的伪长度
getPathLength = function(path)
    local d = 0
    local l = #jhandles
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
        d = d + getConfigConfigDistance(config1, config2)
    end
    return d
end

--计算起始构型到目标构型的路径，每个目标构型计算cnt次
--返回最短的路径以及长度(构型空间的能量距离)
--部分关节的运动范围过大,例如+-10'000,将会导致搜索空间过大/速度过慢/效率降低,所以限制关节的运动范围
--@startConfig:初始构型
--@goalConfigs:目标构型
--@cnt:
findPath = function(startConfig, goalConfigs, cnt)
    --重新调整2/3关节的运动范围(这2个关节的能量消耗最大)
    -- local jointLimitsL = {}
    -- local jointLimitsH = {}
    -- for i = 1, #jhandles, 1 do
    --     jointLimitsL[i] = startConfig[i] - 360 * math.pi / 180
    --     if jointLimitsL[i] < -10000 then
    --         jointLimitsL[i] = -10000
    --     end
    --     jointLimitsH[i] = startConfig[i] + 360 * math.pi / 180
    --     if jointLimitsH[i] > 10000 then
    --         jointLimitsH[i] = 10000
    --     end
    -- end
    -- jointLimitsL[2] = 47 * math.pi / 180
    -- jointLimitsH[2] = 313 * math.pi / 180
    -- jointLimitsL[3] = 19 * math.pi / 180
    -- jointLimitsH[3] = 341 * math.pi / 180

    local task = simOMPL.createTask("task") --创建任务
    simOMPL.setAlgorithm(task, OMPLAlgo) --设置算法
    simOMPL.setVerboseLevel(task, 0)     --设置消息级别

    local jSpaces = {}
    for i = 1, #jhandles, 1 do
        local proj = i
        if i > 3 then
            proj = 0
        end
        --设置关节状态空间：名称，类型，关节句柄table,关节下限table,关节上限table,是否用于计算关节映射table(为true时)：1,2,3关节映射，其他几个关节不映射
        --weight:默认为1.0，用于计算不同构型之间的距离(在后面自定义)。返回值为该关节空间的句柄
        jSpaces[#jSpaces + 1] =simOMPL.createStateSpace(
            "j_space" .. i,
            simOMPL.StateSpaceType.joint_position,
            jhandles[i],
            {jointLimitsL[i]},
            {jointLimitsH[i]},
            proj
        )
    end
    simOMPL.setStateSpace(task, jSpaces)
    -- simOMPL.setCollisionPairs(task, collisionPairs) --设置碰撞对
    simOMPL.setStartState(task, startConfig) --设置初始构型
    simOMPL.setGoalState(task, goalConfigs[1]) --设置目标构型
    for i = 2, #goalConfigs, 1 do
        simOMPL.addGoalState(task, goalConfigs[i]) --添加其他的目标构型
    end
    local path = nil
    local l = 999999999999
    --    forbidThreadSwitches(true)
    --计算CNT次目标构型
    for i = 1, cnt, 1 do
        -- 等价于:
        -- simExtOMPL_setup(task)
        -- if simExtOMPL_solve(task, maxTime) then
        --     simExtOMPL_simplifyPath(task, maxSimplificationTime)
        --     simExtOMPL_interpolatePath(task, stateCnt)
        --     result,path = simExtOMPL_getPath(task)
        -- end
        --参数：maxSimplificationTime,用于简化路径的时间，-1表示默认；stateCnt：返回的差值路径点(构型)数量
        local res, _path = simOMPL.compute(task, maxOMPLCalculationTime, -1, configCount)
        if res and _path then
            local _l = getPathLength(_path)
            if _l < l then
                l = _l
                path = _path
            end
        end
        if path then
            visualizePath(path)
        end
    end
    --    forbidThreadSwitches(false)
    simOMPL.destroyTask(task)
     --清除任务
    return path, l
end

findShortestPath = function(startConfig, goalConfigs, searchCntPerGoalConfig)
    -- 计算起始构型到目标构型的路径，每个目标构型计算cnt次
    --返回最短的路径以及长度(构型空间的能量距离)
    --其中onepath 为N*6=1200,即路径上200个构型*6个关节的角度
    local onePath, onePathLength = findPath(startConfig, goalConfigs, searchCntPerGoalConfig)
    if onePath then
        return onePath, generatePathLengths(onePath)
    end
    sim.addStatusbarMessage("寻找最短路径失败")
    return nil,nil
end

generateIkPath = function(startConfig, goalPose, steps, ignoreCollisions)
    --生成从当前构型到目标位姿的、线性的、无碰撞的构型
    forbidThreadSwitches(true)
    local currentConfig = getConfig()
    setConfig(startConfig)
    sim.setObjectMatrix(ikTarget, -1, goalPose)
    local coll = collisionPairs
    if ignoreCollisions then
        coll = nil
    end
    local c = sim.generateIkPath(ikPINV, jhandles, steps, coll)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    if c then
        return c, generatePathLengths(c)
    end
end

executeMotion = function(path, lengths, maxVel, maxAccel, maxJerk, pathName)
    dt = sim.getSimulationTimeStep()

    -- 1.折算出每个关节最大的速度
    jointsUpperVelocityLimits = {}
    for j = 1, 7, 1 do
        res, jointsUpperVelocityLimits[j] = sim.getObjectFloatParameter(jhandles[j], sim.jointfloatparam_upper_limit)
    end
    velCorrection = 1

    sim.setThreadSwitchTiming(200)
    while true do
        posVelAccel = {0, 0, 0}
        targetPosVel = {lengths[#lengths], 0} --终点的位置和速度
        pos = 0
        res = 0
        previousQ = {path[1], path[2], path[3], path[4], path[5], path[6],path[7]}
        local rMax = 0
        rmlHandle =sim.rmlPos(1, 0.0001, -1, posVelAccel, {maxVel * velCorrection, maxAccel, maxJerk}, {1}, targetPosVel)
        while res == 0 do
            res, posVelAccel, sync = sim.rmlStep(rmlHandle, dt)
            if (res >= 0) then
                l = posVelAccel[1]
                for i = 1, #lengths - 1, 1 do
                    l1 = lengths[i]
                    l2 = lengths[i + 1]
                    if (l >= l1) and (l <= l2) then
                        t = (l - l1) / (l2 - l1)
                        for j = 1, 7, 1 do
                            q =
                                path[7 * (i - 1) + j] +
                                getJointPosDifference(
                                    path[7 * (i - 1) + j],
                                    path[7 * i + j],
                                    jtypes[j] == sim.joint_revolute_subtype
                                )*t
                            dq = getJointPosDifference(previousQ[j], q, jtypes[j] == sim.joint_revolute_subtype)
                            previousQ[j] = q
                            r = math.abs(dq / dt) / jointsUpperVelocityLimits[j]
                            if (r > rMax) then
                                rMax = r
                            end
                        end
                        break
                    end
                end
            end
        end
        sim.rmlRemove(rmlHandle)
        if rMax > 1.001 then
            velCorrection = velCorrection / rMax
        else
            break
        end
    end
    sim.setThreadSwitchTiming(2)

    -- 2. 执行动作
    posVelAccel = {0, 0, 0}
    targetPosVel = {lengths[#lengths], 0}
    pos = 0
    res = 0
    jointPos = {}
    rmlHandle = sim.rmlPos(1, 0.0001, -1, posVelAccel, {maxVel * velCorrection, maxAccel, maxJerk}, {1}, targetPosVel)
    while res == 0 do
        dt = sim.getSimulationTimeStep()
        res, posVelAccel, sync = sim.rmlStep(rmlHandle, dt)
        -- sim.setGraphUserData(ghandle,"rml_pos",posVelAccel[1])
        -- sim.setGraphUserData(ghandle,"rml_speed",posVelAccel[2])
        -- sim.setGraphUserData(ghandle,"rml_acc",posVelAccel[3])
        if (res >= 0) then
            l = posVelAccel[1]
            for i = 1, #lengths - 1, 1 do
                l1 = lengths[i]
                l2 = lengths[i + 1]
                if (l >= l1) and (l <= l2) then
                    t = (l - l1) / (l2 - l1)
                    for j = 1, 7, 1 do
                        jointPos[j] =path[7 * (i - 1) + j] +getJointPosDifference(path[7 * (i - 1) + j],path[7 * i + j],jtypes[j] == sim.joint_revolute_subtype)*t
                    end
                    applyJoints(jhandles, jointPos)
                    break
                end
            end
        end
        sim.switchThread()
    end
    sim.rmlRemove(rmlHandle)
end

-- 保存自定义路径
savePath = function(filename, path, lengths)
    sim.writeCustomDataBlock(rdsHandle, filename .. ".pathData1", sim.packFloatTable(path))
    sim.writeCustomDataBlock(rdsHandle, filename .. ".pathLength1", sim.packFloatTable(lengths))
end

--加载自定义路径
loadPath = function(filename)
    path = sim.readCustomDataBlock(rdsHandle, filename .. ".pathData1")
    if (not path) then
        return nil
    end
    path = sim.unpackFloatTable(path)

    lengths = sim.readCustomDataBlock(rdsHandle, filename .. ".pathLength1")
    if (not lengths) then
        return nil
    end
    lengths = sim.unpackFloatTable(lengths)
    return path, lengths
end

--路径规划
function pathPlaning()
    -- path,lengths=loadPath('jacoPath_1')     --加载保存的路径
    --进行路径规划
    sim.addStatusbarMessage("路径规划开始")
    local m = getShiftedMatrix(simGetObjectMatrix(target1, -1), {0, 0, 0.1}, -1)
    sim.addStatusbarMessage("开始搜索可用目标构型...")

    --开始搜索目标构型，搜索300次，找到最多5个可用的目标构型
    local configs = findTopNCollisionFreeConfigs(m, 300, 5)

    --如果没有找到可用的目标构型则返回
    if configs == nil then
        sim.addStatusbarMessage("未搜索到可用的目标构型，无法进行路径规划")
        return nil, nil
    end
    sim.addStatusbarMessage("搜索到 " .. #configs .. " 个可用的目标构型")

    --计算路径，返回200个构型*6个关节的角度值，200个构型对应的关节距离累加值
    sim.addStatusbarMessage("开始进行路径规划")
    path, lengths = findShortestPath(getConfig(), configs, numberOfOMPLCalculationsPasses)
    return path, lengths
end


function checkSignal()
    ---------------设置当前的构型的步数-----------
    -- 当command!="start"时会卡在这里
    repeat
        command = simGetStringSignal("command")
        --当命令为replan时,直接退出并返回标识
        if command == "replan" then
            sim.rmlRemove(rmlHandle)
            return 1
        end
    until (command == "start")
    sim.setIntegerSignal(pathName, i)
    -----------------------------------------
end
--################主程序########################--
function sysCall_threadmain()
    ghandle=sim.getObjectHandle("Graph")
    jhandles = {-1, -1, -1, -1, -1, -1, -1} --获取关节句柄
    jtypes = {-1, -1, -1, -1, -1, -1, -1} --获取关节类型
    for i = 1, 7, 1 do
        jhandles[i] = sim.getObjectHandle("j" .. i)
        jtypes[i] = sim.getJointType(jhandles[i])
    end
    rdsHandle = sim.getObjectHandle("RDS_01")
    ikTarget = sim.getObjectHandle("RDS_01_target") --ikTarget
    ikTip = sim.getObjectHandle("RDS_01_tip") --ikTip
    ikPINV = sim.getIkGroupHandle("RDS_IK_PINV") --Ik伪逆
    ikDLS = sim.getIkGroupHandle("RDS_IK_DLS") --IK DLS
    jointLimitsL, jointRanges,jointLimitsH=getJointLimits()
    target1 = sim.getObjectHandle("target") --位置1
    -- target2=sim.getObjectHandle('jacoTarget1') --目标位置2
    -- target3=sim.getObjectHandle('jacoTarget2') --目标位置3
    ------------------------------------------------------
    collisionCollectionHandle = sim.getCollectionHandle("CollisionObjects") --机械臂的碰撞对象集合的句柄
    robotCollectionHandle=sim.getCollectionHandle("RDS_01")                 --机械臂本体的对象集合
    configCount = 200 --构型检测的数目
    sim.setStringSignal("command", "start") --初始化启动信号
    sim.setIntegerSignal("configNumer_1", 1) --初始当前路径点标识(1~configCount)
    --获得目标

    --@@@@@@@@@@@@@@@@@@@@@@@@参数表@@@@@@@@@@@@@@@@@@@@@@@@
    --碰撞对：1-2:机械臂本身不发生碰撞，3-4：机械臂和其他的对象不发生碰撞
    collisionPairs = {
        robotCollectionHandle,
        robotCollectionHandle,
        robotCollectionHandle,
        collisionCollectionHandle
    }
    maxVel = 1 --最大速度
    maxAccel = 1 --最大加速度
    maxJerk = 8000 --最大加加速度
    forbidLevel = 0
    metric = {0.2, 1, 0.8, 0.1, 0.1, 0.1,0.1} --关节能量权重
    ikSteps = 20 --Ik
    maxOMPLCalculationTime = 4 -- 单次路径规划的最长允许时间，单位s
    -- sim_ompl_algorithm_BKPIECE1
    OMPLAlgo = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
    numberOfOMPLCalculationsPasses = 4 -- 单个目标构型的路径规划最大次数
    -- @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    --清除上次生成的路径
    sim.writeCustomDataBlock(rdsHandle, "", nil)

    repeat
        path, lengths = pathPlaning()
        --如果找到最短路径则保存路径和能量累加值
        if path then
            sim.addStatusbarMessage("路径规划成功，保存该路径")
            savePath("jacoPath_1", path, lengths)
            visualizePath(path)
             --可视化路径
            --路径参数化，然后执行
            sim.addStatusbarMessage("执行动作1: 达到目标姿态点")
            replan = executeMotion(path, lengths, maxVel, maxAccel, maxJerk, "configNumer_1")
        else
            sim.addStatusbarMessage("未搜索到可用路径,路径规划失败")
            return
        end
    until (replan ~= 1)

    -- -- --靠近杯子(IK)
    -- sim.addStatusbarMessage("执行动作2：靠近杯子(逆解算)")
    -- local m=getShiftedMatrix(sim.getObjectMatrix(target1,-1),{0,0.01,0},-1)
    -- path,lengths=generateIkPath(getConfig(),m,ikSteps,true) --不考虑避障
    -- if path then
    --     executeMotion(path,lengths,maxVel,maxAccel,maxJerk,"configNumer_2")
    -- end

    -- -- -- -- 合并机械手,抓住杯子
    -- sim.addStatusbarMessage("执行动作3：合并机械手")
    -- sim.setIntegerSignal('RG2_open',1)
    -- sim.wait(1.5)

    -- -- -- 举起杯子(IK)
    -- sim.addStatusbarMessage("执行动作4：举起杯子(逆解算)")
    -- local m=sim.getObjectMatrix(target2,-1)
    -- path,lengths=generateIkPath(getConfig(),m,ikSteps,true)
    -- if path then
    --     executeMotion(path,lengths,maxVel,maxAccel,maxJerk,"configNumer_3")
    -- end
end