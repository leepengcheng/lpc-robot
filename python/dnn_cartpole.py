#coding:utf-8

import os,time
import numpy as np
import gym
from gym import spaces
# from gym.utils import colorize, seeding

import  vrep

from vrepUtil import  blocking,onshot,init


class CartPoleVREPEnv(gym.Env):
    def __init__(self):

        #初始化连接
        self.clientID = init()
        vrep.simxSynchronous(self.clientID,True) #激活同步模式
        self.updateObjectHandles() #获取对象的句柄

        #小车的X位置/X速度
        #质量块X位置/Z位置(垂直向上)
        #质量块X速度/Z速度(垂直向上)
        obs = np.array([np.inf]*6) #观测值
        act = np.array([1.])       #执行

        self.action_space = spaces.Box(-act,act,dtype=np.float32)
        self.observation_space = spaces.Box(-obs,obs,dtype=np.float32)

    def updateObjectHandles(self):
        _,self.cartpole_handle=vrep.simxGetObjectHandle(self.clientID,'cart_pole',blocking)
            #小车的滑动关节
        _,self.slider_Jhandle=vrep.simxGetObjectHandle(self.clientID,'car_joint_slider',blocking)
        #小车本体
        _,self.cart_handle=vrep.simxGetObjectHandle(self.clientID,'cart',blocking)
        #质量块(倒立摆)
        _,self.mass_handle= vrep.simxGetObjectHandle(self.clientID,'mass',blocking)

    def observe(self):

        # 观测action结果
        _,cartpos = vrep.simxGetObjectPosition(self.clientID,self.cart_handle,-1,blocking)
        _,masspos = vrep.simxGetObjectPosition(self.clientID,self.mass_handle,-1,blocking)
        _,cartvel,cart_angvel = vrep.simxGetObjectVelocity(self.clientID,self.cart_handle,blocking)
        _,massvel,mass_angvel = vrep.simxGetObjectVelocity(self.clientID,self.mass_handle,blocking)

        self.observation = np.array([
            cartpos[0],cartvel[0],
            masspos[0],masspos[2],
            massvel[0],massvel[2]
            ],dtype=np.float32)

    def step(self,actions):
        #将运动限制在-1,1
        actions = np.clip(actions, -1, 1)

        #小车随机速度值
        v = actions[0]

        #执行:设置小车速度
        vrep.simxSetJointTargetVelocity(self.clientID,self.slider_Jhandle,v,onshot)
        vrep.simxSynchronousTrigger(self.clientID)

        #观测结果
        self.observe()

        #质量块的Z值
        mass_pos_z = self.observation[3] # masspos[2]

        ##############Reward##############
        #Reward:Z越大/
        cost = mass_pos_z - (v**2) * 0.001


        #观测值，reward,done,info
        return self.observation, cost, False, {}

    def reset(self):

        #停止仿真
        vrep.simxStopSimulation(self.clientID,blocking)

        #暂停(必须）
        time.sleep(0.5)
        # vrep.simxRemoveModel(self.clientID,self.cartpole_handle,blocking)
        # vrep.simxLoadModel(self.clientID,"/media/lee/workspace/GitWorkSpace/lpc-robot/scenes/cart_pole.ttm",1,blocking)
        # self.getObjectHandles()
        #开始仿真
        vrep.simxStartSimulation(self.clientID,blocking)
        self.observe()

        return self.observation

    def close(self):
        print("Stop Simulation")
        vrep.simxStopSimulation(self.clientID,blocking)

if __name__ == '__main__':
    env = CartPoleVREPEnv()
    for k in range(5):
        observation = env.reset()
        for _ in range(20):
        #   env.render()
          action = env.action_space.sample() #均匀分布随机运动
          observation, reward, done, info = env.step(action)
          print(reward)
        print("*"*30)
    env.close()