--抓取类
local grasp={}

grasp.frame_id=-1
grasp.grasp_pose={}
grasp.pre_grasp_approach={
    direction={0,0,1},
    frame_id=-1,
    desired_distance=0.4
}
grasp.post_grasp_retreat={
    direction={0,0,1},
    frame_id=-1,
    desired_distance=0.4
}

return grasp