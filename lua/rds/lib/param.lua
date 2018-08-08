--@@机械臂参数

local param={}
local robot={}
local vrobot={}
robot.VERSION = 1.0 
-- robot.isSimulate=true   --是否执行规划出的路径/轨迹
robot.isCalculateTraj=false      --是否进行轨迹参数化
robot.isCalculateMode=false      --是否是计算模式(取消sleep/仿真操作))
robot.maxVel = 0.8 --最大速度
robot.maxAcc = 0.8 --最大加速度
robot.maxJerk = 1000 --最大加加速度
robot.forbidLevel = 0
robot.metric = {0.2, 1, 0.8, 0.1, 0.1, 0.1,0.1} --关节能量权重
robot.algoOMPL = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
-- simOMPL.Algorithm.RRTConnect
robot.singlePlanTime = 4       -- 单次路径规划的最长允许时间，单位s
robot.configPlanAttempts = 4 -- 单个目标构型的路径规划最大次数(OMPL)
robot.singlePosePlanAttempts=10    --单个目标位姿进行路径规划的次数：nil表示无穷次
robot.maxConfigs=5    --目标构型：找到的目标构型最大数目
robot.attemptCount=300  --目标构型：寻找目标构型的尝试次数
-- robot.configCount = 200 --构型数目:OMPL单次路径规划生成的轨迹点的数目(构型数目)
robot.stepOMPL=0.02  --OMPL规划出的轨迹的单位步长(关节空间单位能量距离))

robot.dt=nil             --单位s,轨迹参数化的时间间隔(为nil时dt=sim.getSimulationTimeStep()))

--joints
robot.joint={}
robot.joint.num=7
robot.joint.handles={-1,-1,-1,-1,-1,-1,-1}
robot.joint.types = {-1,-1,-1,-1,-1,-1,-1}
robot.joint.limitsL={-1,-1,-1,-1,-1,-1,-1}
robot.joint.limitsH={-1,-1,-1,-1,-1,-1,-1}
robot.joint.ranges={-1,-1,-1,-1,-1,-1,-1}
for i=1,robot.joint.num do
    local handle=sim.getObjectHandle("j" .. i)
    robot.joint.handles[i]=handle
    robot.joint.types[i]=sim.getJointType(handle)
    local cyclic, interval = sim.getJointInterval(handle)
    if cyclic then
        --设置范围为[-pi,pi]
        robot.joint.limitsL[i], robot.joint.ranges[i] = -math.pi, 2 * math.pi
    else
        robot.joint.limitsL[i], robot.joint.ranges[i] = interval[1], interval[2]
    end
    robot.joint.limitsH[i]=robot.joint.limitsL[i]+robot.joint.ranges[i]
end

--robotHandle
robot.robotHandle=sim.getObjectHandle("RDS_01")

--ik
robot.ik={
    tipHandle=sim.getObjectHandle("RDS_01_tip"),
    targetHandle=sim.getObjectHandle("RDS_01_target"),
    pinvHandle=sim.getIkGroupHandle("RDS_IK_PINV"),
    dlsHandle=sim.getIkGroupHandle("RDS_IK_DLS"),
    ignoreCollisions=true,
    pathFactor={1,0.1,0.05}
}


--collisions
robot.collisions={
    robotColHandle=sim.getCollectionHandle("RDS_01"),
    objectColHandle=sim.getCollectionHandle("CollisionObjects")
}

setmetatable(param, {__index={robot=robot,vrobot=vrobot}})


-- param.new=function(self)
--     return setmetatable(param, {__index={robot=robot}})
-- end
return param