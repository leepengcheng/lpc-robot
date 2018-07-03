--通用函数
local common={}

--command
common.REPLAN=1
common.START=2
common.STOP=3



--清除路径数据
function common.clearPath (self,handle)
    sim.writeCustomDataBlock(handle, "", nil)
end


-- 保存自定义路径
function common.savePath (self,handle,filename, path, lengths)
    sim.writeCustomDataBlock(handle, filename .. ".pathData1", sim.packFloatTable(path))
    sim.writeCustomDataBlock(handle, filename .. ".pathLength1", sim.packFloatTable(lengths))
end

--加载自定义路径
function loadPath (self,handle,filename)
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
function common.getTranslatedMatrix(self,targetHandle, trans)
    local matrix=sim.getObjectMatrix(targetHandle, -1)
    local m = {}
    for i = 1, 12, 1 do
        m[i] = matrix[i]
    end
    m[4] = m[4] + m[1] * trans[1] + m[2] * trans[2] + m[3] * trans[3]
    m[8] = m[8] + m[5] * trans[1] + m[6] * trans[2] + m[7] * trans[3]
    m[12] = m[12] + m[9] * trans[1] + m[10] * trans[2] + m[11] * trans[3]
    return m
end









-- --返回每个构型距离依次递增table,用于RML速度映射
-- --小等于规划的目标构型的数目，此处为200
-- --@path:规划出的关节控制的路径点,size=joint_size*config_size
-- function common.generatePathLengths(self,path)
--     local d = 0
--     local l = #jhandles
--     local pc = #path / l
--     local retLengths = {0}
--     for i = 1, pc - 1, 1 do
--         local config1 = {
--             path[(i - 1) * l + 1],
--             path[(i - 1) * l + 2],
--             path[(i - 1) * l + 3],
--             path[(i - 1) * l + 4],
--             path[(i - 1) * l + 5],
--             path[(i - 1) * l + 6],
--             path[(i - 1) * l + 7]
--         }
--         local config2 = {
--             path[i * l + 1],
--             path[i * l + 2],
--             path[i * l + 3],
--             path[i * l + 4],
--             path[i * l + 5],
--             path[i * l + 6],
--             path[i * l + 7]
--         }
--         d = d + getCurrentConfigDistance(config1, config2)
--         retLengths[i + 1] = d
--     end
--     return retLengths
-- end

return common