
function createShapeBoundingBoxAndJoints(linkHandles,color)
    color=color or {0,0,1}

    for i=1,#linkHandles,1 do
        linkHandle=linkHandles[i]
        pos=sim.getObjectPosition(linkHandle,-1) 
        ori=sim.getObjectOrientation(linkHandle,-1) 
        start=14
        res,xmin=sim.getObjectFloatParameter(linkHandle,start+1)
        res,ymin=sim.getObjectFloatParameter(linkHandle,start+2 )
        res,zmin=sim.getObjectFloatParameter(linkHandle,start+3 )
        res,xmax=sim.getObjectFloatParameter(linkHandle,start+4 )
        res,ymax=sim.getObjectFloatParameter(linkHandle,start+5 )
        res,zmax=sim.getObjectFloatParameter(linkHandle,start+6 )
        xsize=xmax-xmin
        ysize=ymax-ymin
        zsize=zmax-zmin
        boxHandle=sim.createPureShape(0,16,{xsize,ysize,zsize},0.0)
        sim.setObjectPosition(boxHandle,-1,pos)
        sim.setObjectOrientation(boxHandle,-1,ori)
        sim.setShapeColor(boxHandle,nil,sim.colorcomponent_ambient_diffuse,color)
        sim.setShapeColor(boxHandle,nil,sim.colorcomponent_transparency,{0.5})
    end
end





sim.setBoolParameter(sim.boolparam_dynamics_handling_enabled,false)
roboLinkHandles={}
roboLinkNames={"base"}

for i=1,7,1 do
roboLinkNames[i+1]="link"..i
end

for i=1,#roboLinkNames,1 do
    roboLinkHandles[i]=sim.getObjectHandle(roboLinkNames[i])
end

createShapeBoundingBoxAndJoints(roboLinkHandles)


