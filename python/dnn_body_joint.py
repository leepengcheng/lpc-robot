#coding:utf-8
import os
import math
import time
import vrepUtil



dir_vrep=os.getenv("VREP_ROOT")
venv = vrepUtil.vrepper(dir_vrep=dir_vrep,headless=False)
venv.start()

#加载场景
venv.load_scene(os.getcwd() + '../scenes/body_joint_wheel.ttt')

#加载
body = venv.get_object_by_name('body')
wheel = venv.get_object_by_name('wheel')
joint = venv.get_object_by_name('joint')

print(body.handle,wheel.handle,joint.handle)

for j in range(5):
    venv.start_blocking_simulation()

    for i in range(20):
        print('simulation step',i)
        print('body position',body.get_position())
        print('wheel orientation',wheel.get_orientation())

        joint.set_velocity(10 * math.sin(i/5))
        # you should see things moving back and forth

        venv.step_blocking_simulation() # forward 1 timestep

    # stop the simulation and reset the scene:
    venv.stop_simulation()

print('simulation ended. leaving in 5 seconds...')
time.sleep(5)
