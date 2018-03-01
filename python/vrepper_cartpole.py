#coding:utf-8
import vrepper
from vrepper.core import vrepper

import os,time
import numpy as np

import gym
from gym import spaces
# from gym.utils import colorize, seeding

class CartPoleVREPEnv(gym.Env):
    def __init__(self,port_num=19997,headless=False):
        self.venv = venv = vrepper(port_num,headless=headless)
        venv.start()
        #加载模型
        venv.load_scene(os.getcwd() + '/scenes/cart_pole.ttt')

        #小车的滑动关节
        self.slider = venv.get_object_by_name('car_joint_slider')
        #小车本体
        self.cart = venv.get_object_by_name('cart')
        #质量块(倒立摆)
        self.mass = venv.get_object_by_name('mass')


        #小车的X位置/X速度
        #质量块X位置/Z位置(垂直向上)
        #质量块X速度/Z速度(垂直向上)
        obs = np.array([np.inf]*6) #观测值
        act = np.array([1.])       #执行

        self.action_space = spaces.Box(-act,act)
        self.observation_space = spaces.Box(-obs,obs)

    def _self_observe(self):
        # 观测action结果,小车和
        cartpos = self.cart.get_position()
        masspos = self.mass.get_position()
        cartvel,cart_angvel = self.cart.get_velocity()
        massvel,mass_angvel = self.mass.get_velocity()

        self.observation = np.array([
            cartpos[0],cartvel[0],
            masspos[0],masspos[2],
            massvel[0],massvel[2]
            ]).astype('float32')

    def _step(self,actions):
        #将运动限制在-1,1
        actions = np.clip(actions, -1, 1)

        #小车随机速度值
        v = actions[0]

        #执行:设置小车速度
        self.slider.set_velocity(v)
        self.venv.step_blocking_simulation()

        #观测结果
        self._self_observe()

        #质量块的Z值
        mass_pos_z = self.observation[3] # masspos[2]

        ##############Reward##############
        #Reward:Z越大/
        cost = mass_pos_z - (v**2) * 0.001



        #观测值，reward,done,info
        return self.observation, cost, False, {}

    def _reset(self):
        self.venv.stop_simulation()
        self.venv.start_simulation(True)
        self._self_observe()
        return self.observation

    def _destroy(self):
        self.venv.stop_simulation()
        self.venv.end()

if __name__ == '__main__':
    env = CartPoleVREPEnv(headless=True)
    for k in range(5):
        observation = env.reset() 
        for _ in range(20):
        #   env.render()
          action = env.action_space.sample() #均匀分布随机运动
          observation, reward, done, info = env.step(action)
          print(reward)
        print("*"*30)

    print('simulation ended. leaving in 5 seconds...')
    time.sleep(5)