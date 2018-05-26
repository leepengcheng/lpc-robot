function sysCall_init()
    matrix = require "matrix" --只支持方阵
    ik_pinv = sim.getIkGroupHandle("IK_PINV")
    ik_dls = sim.getIkGroupHandle("IK_DLS")
    J = matrix(6, 6)
    camHandle = sim.getObjectHandle("camera")

    jointHandles = {-1, -1, -1, -1, -1, -1}
    for i = 1, 6, 1 do
        jointHandles[i] = sim.getObjectHandle("UR5_joint" .. i)
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
    local target_locs = {{0.05, 0.05, 0.1}, {-0.05, 0.05, 0.1}, {-0.05, -0.05, 0.1}}
    target_pixels = {}
    for i = 1, 3, 1 do
        target_pixels[i] = getFeaturePixelPosition(target_locs[i])
    end
    lambda = 0.01
    step = 0.05
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
    current_pixel = getFeaturePixelPosition(current_pos)
    return {current_pixel[1] - target_pixel[1], current_pixel[2] - target_pixel[2]}
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

function sysCall_actuation()
    local res = sim.computeJacobian(ik_pinv, 0)
    local mat, s = {}, {}
    if res ~= -1 then
        --s=(column,rows) #mat=column*rows
        mat, s = sim.getIkGroupMatrix(ik_pinv, 0)
    else
        res = sim.computeJacobian(ik_dls, 0)
        mat, s = sim.getIkGroupMatrix(ik_dls, 0)
    end

    for i = 1, 6, 1 do
        for j = 1, 6, 1 do
            J[i][j] = mat[i + (j - 1) * 6]
        end
    end
    local E = calculatePixelsError()
    local Lx = getInteractionMatrix()
    local J_p = J ^ -1 --机械臂雅克比矩阵逆
    local Lx_p = Lx ^ -1 --图像雅克比逆
    local V = -lambda * J_p * Lx_p * E
    local D = V * step
    print(E)
    -- for i = 1, 6, 1 do        
    --     sim.setJointPosition(jointHandles[i], sim.getJointPosition(jointHandles[i]) + D[i][1])
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
