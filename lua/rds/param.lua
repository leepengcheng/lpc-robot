--配置参数
local param={}
local robot={}
robot.paramCount = 200 --构型检测的数目
robot.maxVel = 1 --最大速度
robot.maxAcc = 1 --最大加速度
robot.maxJerk = 8000 --最大加加速度
robot.forbidLevel = 0
robot.metric = {0.2, 1, 0.8, 0.1, 0.1, 0.1,0.1} --关节能量权重
robot.maxOMPLCalculationTime = 4 -- 单次路径规划的最长允许时间，单位s
robot.OMPLAlgo = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
robot.OMPLAlgo = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
-- simOMPL.Algorithm.RRTConnect
robot.numberOfOMPLCalculationsPasses = 4 -- 单个目标构型的路径规划最大次数
robot.maxConfigs=5    --寻找的构型数目
robot.trialCount=300  --尝试的次数
robot.configCount = 200 --构型检测的数目
robot.forbidLevel=0 
robot.maxPlanAttempts=10 --单个位姿进行路径规划的次数：nil表示无穷次


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
    pathPointCount=20
}


--collisions
robot.collisions={
    robotColHandle=sim.getCollectionHandle("RDS_01"),
    objectColHandle=sim.getCollectionHandle("CollisionObjects")
}

setmetatable(param, {__index={robot=robot}})


-- param.new=function(self)
--     return setmetatable(param, {__index={robot=robot}})
-- end
return param