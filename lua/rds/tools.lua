--通用函数
local tools={}

--清除路径数据
function tools.clearPath (self,handle)
    sim.writeCustomDataBlock(handle, "", nil)
end

function tools.new(self)
    return setmetatable(tools,{__index={version=1.0}})
end


-- 保存自定义路径
function tools.savePath (self,handle,filename, path, lengths)
    sim.writeCustomDataBlock(handle, filename .. ".pathData1", sim.packFloatTable(path))
    sim.writeCustomDataBlock(handle, filename .. ".pathLength1", sim.packFloatTable(lengths))
end

--加载自定义路径
function tools.loadPath (self,handle,filename)
    local path = sim.readCustomDataBlock(handle, filename .. ".pathData1")
    if (not path) then
        return nil
    end
    path = sim.unpackFloatTable(path)

    local lengths = sim.readCustomDataBlock(handle, filename .. ".pathLength1")
    if (not lengths) then
        return nil
    end
    lengths = sim.unpackFloatTable(lengths)
    return path, lengths
end

-- 获得沿着指定矩阵的坐标轴放心进行平移后的矩阵
-- @matrix:指定的矩阵
-- @trans:xyz平移量
-- @return:平移后的矩阵
function tools.getTransMatrix(self,matrix, trans)
    local m = {}
    for i = 1, 12, 1 do
        m[i] = matrix[i]
    end
    m[4] = m[4] + m[1] * trans[1] + m[2] * trans[2] + m[3] * trans[3]
    m[8] = m[8] + m[5] * trans[1] + m[6] * trans[2] + m[7] * trans[3]
    m[12] = m[12] + m[9] * trans[1] + m[10] * trans[2] + m[11] * trans[3]
    return m
end



function tools.getTransRotMatrix(self,handle,pos,euler)
    local pos=pos or {0,0,0}
    local euler=euler or {0,0,0}
    local baseMatrix=sim.getObjectMatrix(handle,-1)
    local matrix=sim.buildMatrix(pos,euler)
    local targetMatrix=sim.multiplyMatrices(baseMatrix,matrix)
    return targetMatrix
end




return tools