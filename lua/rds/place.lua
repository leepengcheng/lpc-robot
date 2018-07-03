--放置类
local place={}

place.frame_id=-1
place.place_pose={}
place.pre_place_approach={
    direction={0,0,1},
    frame_id=-1,
    desired_distance=0.4
}
place.post_place_retreat={
    direction={0,0,1},
    frame_id=-1,
    desired_distance=0.4
}
place.method="ik"

return place