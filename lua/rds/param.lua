--配置参数
local param={}

param.paramCount = 200 --构型检测的数目
param.maxVel = 1 --最大速度
param.maxAcc = 1 --最大加速度
param.maxJerk = 8000 --最大加加速度
param.forbidLevel = 0
param.metric = {0.2, 1, 0.8, 0.1, 0.1, 0.1,0.1} --关节能量权重
param.ikSteps = 20 --Ik
param.maxOMPLCalculationTime = 4 -- 单次路径规划的最长允许时间，单位s
param.OMPLAlgo = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
param.OMPLAlgo = simOMPL.Algorithm.BKPIECE1 -- OMPL路径规划所用的算法
-- simOMPL.Algorithm.RRTConnect
param.numberOfOMPLCalculationsPasses = 4 -- 单个目标构型的路径规划最大次数
param.maxConfigs=5    --寻找的构型数目
param.trialCount=300  --尝试的次数
param.configCount = 200 --构型检测的数目
param.forbidLevel=0 


--joints
param.joint={}
param.joint.num=7
param.joint.handles={-1,-1,-1,-1,-1,-1,-1}
param.joint.types = {-1,-1,-1,-1,-1,-1,-1}
param.joint.limitsL={-1,-1,-1,-1,-1,-1,-1}
param.joint.limitsH={-1,-1,-1,-1,-1,-1,-1}
param.joint.ranges={-1,-1,-1,-1,-1,-1,-1}
for i=1,param.joint.num do
    local handle=sim.getObjectHandle("j" .. i)
    param.joint.handles[i]=handle
    param.joint.types[i]=sim.getJointType(handle)
    local cyclic, interval = sim.getJointInterval(handle)
    if cyclic then
        --设置范围为[-pi,pi]
        param.joint.limitsL[i], param.joint.ranges[i] = -math.pi, 2 * math.pi
    else
        param.joint.limitsL[i], param.joint.ranges[i] = interval[1], interval[2]
    end
    param.joint.limitsH[i]=param.joint.limitsL[i]+param.joint.ranges[i]
end

--robotHandle
param.robotHandle=sim.getObjectHandle("RDS_01")

--ik
param.ik={
    tipHandle=sim.getObjectHandle("RDS_01_tip"),
    targetHandle=sim.getObjectHandle("RDS_01_target"),
    pinvHandle=sim.getIkGroupHandle("RDS_IK_PINV"),
    dlsHandle=sim.getIkGroupHandle("RDS_IK_DLS")
}


--collisions
param.collisions={
    robotColHandle=sim.getCollectionHandle("RDS_01"),
    objectColHandle=sim.getCollectionHandle("CollisionObjects")
}
      

return param