#coding:utf-8

import os,time
import numpy as np
import gym
from gym import spaces
import argparse
from itertools import count
#torch
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical #类别分布

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

        #执行空间和观测空间的区间范围上下限
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

        # 观测action后的结果
        _,cartpos = vrep.simxGetObjectPosition(self.clientID,self.cart_handle,-1,blocking)
        _,masspos = vrep.simxGetObjectPosition(self.clientID,self.mass_handle,-1,blocking)
        _,cartvel,cart_angvel = vrep.simxGetObjectVelocity(self.clientID,self.cart_handle,blocking)
        _,massvel,mass_angvel = vrep.simxGetObjectVelocity(self.clientID,self.mass_handle,blocking)

        #观测空间是6维的np.array
        self.observation = np.array([
            cartpos[0],cartvel[0],
            masspos[0],masspos[2],
            massvel[0],massvel[2]
            ],dtype=np.float32)

    def step(self,actions):

        #将运动限制在-1,1(貌似没必要,除非外部认为指定时超过范围)
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
        reward = mass_pos_z - (v**2) * 0.001


        #观测值/reward/done/info
        return self.observation, reward, False, {}

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
        vrep.simxStopSimulation(self.clientID,blocking)


parser = argparse.ArgumentParser(description='PyTorch REINFORCE example')
parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
                    help='discount factor (default: 0.99)')
parser.add_argument('--seed', type=int, default=543, metavar='N',
                    help='random seed (default: 543)')
parser.add_argument('--render', action='store_true',
                    help='render the environment')
parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                    help='interval between training status logs (default: 10)')
parser.add_argument("--model-path",type=str,default="",help="forward test")
args = parser.parse_args()

env = gym.make('CartPole-v0')
env.seed(args.seed)
torch.manual_seed(args.seed)


#直接输出速度
class CartPoleNet(nn.Module):
    def __init__(self):
        super(CartPoleNet, self).__init__()
        self.affine1 = nn.Linear(4, 128)
        self.affine2 = nn.Linear(128, 2)

        self.saved_log_probs = []
        self.rewards = []

    def forward(self, x):
        x = F.relu(self.affine1(x))
        action_scores = self.affine2(x)
        return F.softmax(action_scores, dim=1) #注意dim=1


model = CartPoleNet()
optimizer = optim.Adam(model.parameters(), lr=1e-2)
eps = np.finfo(np.float32).eps.item() #float32的最小值
model_path=args.model_path if os.path.exists(args.model_path) else "./data1.pkl"



def select_action(state):
    '''
    选择action=0,1
    probs=torch.Tensor([0.2,0.4,0.4])
    当action=m.sample()=0时,表示随机到了index=0的对象,该对象的概率p=0.2
    此时：m.log_prob(action)=math.log(0.2)
    '''
    state = torch.from_numpy(state).float().unsqueeze(0)
    probs = model(state) #probs.shape=(1,2)
    m = Categorical(probs) #生成概率分布(归一化)，对应的概率值为probs
    action = m.sample()    #根据对应的概率选择index=0~len(probs)
    model.saved_log_probs.append(m.log_prob(action)) #选择对应action的概率,保存到list中
    return action.item()



#将1个episode中的所有迭代项作为1个整体输入
#例如某个仿真过程共11步,则该batch_size=11
#其中rewards是累计计算的,然后标准化
def finish_episode():
    R = 0
    policy_loss = []
    rewards = []
    #rewards,倒序
    for r in model.rewards[::-1]:
        R = r + args.gamma * R #reward=q+gamma*q'
        rewards.insert(0, R) #注意：正序排列,不是append
    rewards = torch.tensor(rewards)
    #专业术语：标准分数->转换为均值为0/方差为1的序列
    rewards = (rewards - rewards.mean()) / (rewards.std() + eps)
    for log_prob, reward in zip(model.saved_log_probs, rewards):
        policy_loss.append(-log_prob * reward) #公式参见torch.distributions
    optimizer.zero_grad() #梯度清零
    policy_loss = torch.cat(policy_loss).sum()
    policy_loss.backward()
    optimizer.step() #更新参数值
    del model.rewards[:]  #注意不是del policy.rewards
    del model.saved_log_probs[:]


def train():
    running_reward = 10
    for i_episode in count(1):
        state = env.reset()
        for t in range(10000):  # Don't infinite loop while learning
            action = select_action(state) #选择action
            state, reward, done, _ = env.step(action) #reward=0.0|1.0
            if args.render:
                env.render()
            model.rewards.append(reward) #保存reward
            if done:
                break

        running_reward = running_reward * 0.99 + t * 0.01
        finish_episode()
        if i_episode % args.log_interval == 0:
            print('Episode {}\tLast length: {:5d}\tAverage length: {:.2f}'.format(
                i_episode, t, running_reward))
        #gym/envs/__init__.py,CartPole-v0,reward_threshold=195.0
        if running_reward > env.spec.reward_threshold:
            print("Solved! Running reward is now {} and "
                  "the last episode runs to {} time steps!".format(running_reward, t))
            torch.save(model.state_dict(), model_path)
            break



def test():
    env = CartPoleVREPEnv()
    for k in count(1):
        state = env.reset()
        for _ in range(20):
            # env.render()
            action = env.action_space.sample() #均匀分布随机运动
            state, reward, done, info = env.step(action)
            print(reward)
        print("*"*30)
        env.close()


if __name__ == '__main__':
    test()
