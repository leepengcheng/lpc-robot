--初始化
function sysCall_init()
    --常亮
    ang_min_sinc = 1.0e-8
    ang_min_mc = 2.5e-4

    matrix = require "matrix" --只支持方阵
    ik_pinv = sim.getIkGroupHandle("IK_PINV")
    ik_dls = sim.getIkGroupHandle("IK_DLS")
    fJe = matrix(6, 6)
    camHandle = sim.getObjectHandle("camera")
    UR5Handle= sim.getObjectHandle("UR5")
    targetHandle = sim.getObjectHandle("target")
    tiptHandle = sim.getObjectHandle("tip")
    jointHandles = {-1, -1, -1, -1, -1, -1}
    V_Max={0,0,0,0,0,0} --每个关节的最大允许速度
    for i = 1, 6, 1 do
        jointHandles[i] = sim.getObjectHandle("UR5_joint" .. i)
        V_Max[i]=math.rad(180)
    end

    featureHandles = {-1, -1, -1}
    for i = 1, 3, 1 do
        featureHandles[i] = sim.getObjectHandle("feature" .. i)
    end
    local IMAGE_SIZE = 1024 --图像分辨率1024*1024
    local AGNLE = 60
    UV_0 = IMAGE_SIZE / 2 --图像的中心
    FXY = (IMAGE_SIZE / 2) * math.tan(math.rad(90 - AGNLE / 2)) --焦距的像素距离

    --期望的像素点的坐标(距离相机0.1)
    local target_locs = {{0.05, -0.05, 0.1}, {-0.05, -0.05, 0.1}, {-0.05, 0.05, 0.1}}
    target_pixels = {}
    for i = 1, 3, 1 do
        target_pixels[i] = getFeaturePixelPosition(target_locs[i])
    end
    lambda = 0.1
    step = 0.01
end


--sin(x)/x
function sinc(sinx, x)
    if math.abs(x) < ang_min_sinc then
        return 1.0
    end
        return sinx / x
end

--$ (1-cos(x))/x^2
function mcosc(cosx, x)
    if math.abs(x) < ang_min_mc then
        return 0.5
    end
    return (1.0 - cosx) / x / x
end

-- $(1-sinc(x))/x^2$ 
function msinc(sinx, x)
    if math.abs(x) < ang_min_mc then
        return 1.0 / 6.0
    end
    return (1.0 - sinx / x) / x / x
end





function vectorToMatrix(v, delta_t)
    --速度*时间=姿态变化量
    local v_dt = v * delta_t

    local t = {v_dt[1][1], v_dt[2][1], v_dt[3][1]} --平移向量
    local r = {v_dt[4][1], v_dt[5][1], v_dt[6][1]} --旋转向量

    --平移向量
    local theta = math.sqrt(r[1] * r[1] + r[2] * r[2] + r[3] * r[3])
    local si = math.sin(theta)
    local co = math.cos(theta)
    local sinc = sinc(si, theta)
    local mcosc = mcosc(co, theta)
    local msinc = msinc(si, theta)


    local R = matrix(4, 4)
    --旋转部分
    R[1][1] = co + mcosc * r[1] * r[1]
    R[1][2] = -sinc * r[3] + mcosc * r[1] * r[2]
    R[1][3] = sinc * r[2] + mcosc * r[1] * r[3]
    R[2][1] = sinc * r[3] + mcosc * r[2] * r[1]
    R[2][2] = co + mcosc * r[2] * r[2]
    R[2][3] = -sinc * r[1] + mcosc * r[2] * r[3]
    R[3][1] = -sinc * r[2] + mcosc * r[3] * r[1]
    R[3][2] = sinc * r[1] + mcosc * r[3] * r[2]
    R[3][3] = co + mcosc * r[3] * r[3]
    --平移部分
    R[4][1] =
        t[1] * (sinc + r[1] * r[1] * msinc) + t[2] * (r[1] * r[2] * msinc - r[3] * mcosc) +
        t[3] * (r[1] * r[3] * msinc + r[2] * mcosc)
    R[4][2] =
        t[1] * (r[1] * r[2] * msinc + r[3] * mcosc) + t[2] * (sinc + r[2] * r[2] * msinc) +
        t[3] * (r[2] * r[3] * msinc - r[1] * mcosc)
    R[4][3] =
        t[1] * (r[1] * r[3] * msinc - r[2] * mcosc) + t[2] * (r[2] * r[3] * msinc + r[1] * mcosc) +
        t[3] * (sinc + r[3] * r[3] * msinc)
    R[4][4] = 1
    return R
end


-- 限制每个关节的最大速度
-- v_in:matrix:N*1
-- v_max:table
function saturateVelocities(v_in, v_max, verbose)
    local  size = #v_in
    assert(size == #v_max,"Velocity vectors should have the same dimension")
    local scale = 1  --global scale factor to saturate all the axis
    for i = 1,size,1 do
        local v_i = math.abs(v_in[i][1])
        local v_max_i = math.abs(v_max[i])
        if (v_i > v_max_i) then --Test if we should saturate the axis
            local  scale_i = v_max_i / v_i
            if (scale_i < scale) then
                scale = scale_i
            end
            if (verbose) then
                print("Excess velocity "..v_in[i][1].. " axis nr. ".. i)
            end 
        end
    end
    return v_in*scale;
end

--计算特征点的当前的像素位置
--@loc:特征在相机坐标系的坐标
function getFeaturePixelPosition(loc)
    local X, Y, Z = loc[1], loc[2], loc[3]
    local u = X / Z * FXY + UV_0
    local v = Y / Z * FXY + UV_0
    return {u, v}
end

--计算像素的误差
--@current_pos:目标特征点在相机坐标系下的当前坐标(x,y,z)
--@target_pixel:目标特征点在像素坐标下的期望坐标(u,v)
function calculateSinglePixelError(current_pos, target_pixel)
    local current_pixel = getFeaturePixelPosition(current_pos)
    local pixel_dis = {current_pixel[1] - target_pixel[1], current_pixel[2] - target_pixel[2]}
    return pixel_dis
end

--
--@target_pixels:每个特征点期望的像素坐标
function calculatePixelsError()
    local err = matrix(6, 1)
    for i = 1, 3, 1 do
        local loc = sim.getObjectPosition(featureHandles[i], camHandle)
        local err_ = calculateSinglePixelError(loc, target_pixels[i])
        err[(i - 1) * 2 + 1][1], err[i * 2][1] = err_[1], err_[2]
    end
    return err
end

--计算交互矩阵
function getInteractionMatrix()
    local Lx = matrix(6, 6)
    for i = 1, 3, 1 do
        local loc = sim.getObjectPosition(featureHandles[i], camHandle)
        local X, Y, Z = loc[1], loc[2], loc[3]
        Lx[(i - 1) * 2 + 1] = {-1 / Z, 0, X / Z, X * Y, -(1 + X ^ 2), Y}
        Lx[i * 2] = {0, -1 / Z, Y / Z, 1 + Y ^ 2, -X * Y, -X}
    end
    return Lx
end





--向量转反对称矩阵
function skewMatrix(t)
    local mat=matrix(3,3)
    mat[1]={0,-t[3],t[2]}
    mat[2]={t[3],0,-t[1]}
    mat[3]={-t[2],t[1],0}
    return mat
end




--奇异矩阵->速度扭转矩阵
function hom2vec(homMat)
    local twistMat=matrix(6,6)
    local T={homMat[4],homMat[8],homMat[12]}
    local R=matrix{{homMat[1],homMat[2],homMat[3]},{homMat[5],homMat[6],homMat[7]},{homMat[9],homMat[10],homMat[11]}}
    local skewaR = skewMatrix(T) * R
    for i = 1,3,1 do 
        for j=1,3,1 do 
            twistMat[i][j] = R[i][j]
            twistMat[i + 3][j + 3] = R[i][j];
            twistMat[i][j + 3] = skewaR[i][j];
        end
    end
    return twistMat
end


function sysCall_actuation()
    sim.setObjectMatrix(targetHandle, -1, sim.getObjectMatrix(tiptHandle, -1))
    local res = sim.computeJacobian(ik_pinv, 0)
    local mat, s = {}, {}
    if res ~= -1 then
        --s=(column,rows) #mat=column*rows
        mat, s = sim.getIkGroupMatrix(ik_pinv, 0)
    else
        res = sim.computeJacobian(ik_dls, 1)
        mat, s = sim.getIkGroupMatrix(ik_dls, 0)
    end

    for i = 1, 6, 1 do
        for j = 1, 6, 1 do
            fJe[i][j] = mat[i + (j - 1) * 6]
        end
    end
    


    if Fisrt==nil then
        local pos=matrix(6,1)
        for i=1,6,1 do
            pos[i]={sim.getJointPosition(jointHandles[i])}
        end
        Fisrt=1
        local pos_=fJe*pos
        print(matrix.tostring(pos_))
    end
    --公式
    -- J1 = L * cVa * aJe
    -- e1 = J1p * error; 
    -- v = -lambda(e1) * e1 + (e_dot_init + lambda(e1) * e1_initial) * exp(-mu * t);
    -- local Err = calculatePixelsError() --误差
    -- local L = getInteractionMatrix() --图像雅克比
    -- local cMf=sim.getObjectMatrix(UR5Handle,camHandle)
    -- local cVf=hom2vec(cMf)
    -- local J1=L*cVf*fJe   --任务雅克比矩阵
    -- local J1_p = J1 ^ -1 --任务雅克比矩阵的逆
    -- local Vrobot = -lambda*J1_p*Err*0.05
    
    -- print("****************")
    -- -- print("Error:",E)
    -- -- print("Vcamera:", Vcam)
    -- -- print("Vrobot :",Vrobot)
    -- -- local theata=matrix(6,1)
    -- local Vrobot=saturateVelocities(Vrobot,V_Max,true) --限速
    -- ----
    -- -- local pos=sim.getObjectPosition(camHandle,-1)
    -- -- for i=1,3,1 do
    -- --     pos[i]=pos[i]+Vrobot[i][1]
    -- -- end
    -- -- sim.setObjectPosition(camHandle,-1,pos)
    -- for i = 1, 6, 1 do
    --     -- print(Vrobot[i][1])
    --     sim.setJointPosition(jointHandles[i], sim.getJointPosition(jointHandles[i]) + Vrobot[i][1])
    -- end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_dynCallback(inData)
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end
--]]
