#coding:utf-8
import argparse
import gym
import numpy as np
from itertools import count
import os

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical #类别分布


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


class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.affine1 = nn.Linear(4, 128)
        self.affine2 = nn.Linear(128, 2)

        self.saved_log_probs = []
        self.rewards = []

    def forward(self, x):
        x = F.relu(self.affine1(x))
        action_scores = self.affine2(x)
        return F.softmax(action_scores, dim=1)


policy = Policy()
optimizer = optim.Adam(policy.parameters(), lr=1e-2)
eps = np.finfo(np.float32).eps.item() #float32的最小值


def select_action(state):
    '''
    probs=torch.Tensor([0.2,0.4,0.4])
    当action=m.sample()=0时,表示随机到了index=0的对象,该对象的概率p=0.2
    此时：m.log_prob(action)=math.log(0.2)
    '''
    state = torch.from_numpy(state).float().unsqueeze(0)
    probs = policy(state)
    m = Categorical(probs) #生成概率分布(归一化)，对应的概率值为probs
    action = m.sample()    #根据对应的概率选择index=0~len(probs)
    policy.saved_log_probs.append(m.log_prob(action)) #选择对应action的概率,保存到list中
    return action.item()



#将1个episode中的所有迭代项作为1个整体输入
#例如某个仿真过程共11步,则该batch_size=11
#其中rewards是累计计算的,然后标准化
def finish_episode():
    R = 0
    policy_loss = []
    rewards = []
    #rewards,倒序
    for r in policy.rewards[::-1]:
        R = r + args.gamma * R #reward=q+gamma*q'
        rewards.insert(0, R) #注意：正序排列,不是append
    rewards = torch.tensor(rewards)
    #专业术语：标准分数->转换为均值为0/方差为1的序列
    rewards = (rewards - rewards.mean()) / (rewards.std() + eps)
    for log_prob, reward in zip(policy.saved_log_probs, rewards):
        policy_loss.append(-log_prob * reward) #公式参见torch.distributions
    optimizer.zero_grad()
    #size=[1]*12 -> [12]
    policy_loss = torch.cat(policy_loss).sum()
    policy_loss.backward()
    optimizer.step()
    del policy.rewards[:]  #注意不是del policy.rewards
    del policy.saved_log_probs[:]


def main():
    running_reward = 10
    for i_episode in count(1):
        state = env.reset()
        for t in range(10000):  # Don't infinite loop while learning
            action = select_action(state) #选择action
            state, reward, done, _ = env.step(action) #reward=0.0|1.0
            if args.render:
                env.render()
            policy.rewards.append(reward) #保存reward
            if done:
                break

        running_reward = running_reward * 0.99 + t * 0.01
        finish_episode()
        if i_episode % args.log_interval == 0:
            print('Episode {}\tLast length: {:5d}\tAverage length: {:.2f}'.format(
                i_episode, t, running_reward))
        #gym/envs/__init__.py,CartPole-v0,reward_threshold=195.0
        if t > 1000:
        # if running_reward > env.spec.reward_threshold:
            print("Solved! Running reward is now {} and "
                  "the last episode runs to {} time steps!".format(running_reward, t))
            if not os.path.exists(model-path):
                torch.save(policy.state_dict(),"./data.pkl")
            break



if __name__ == '__main__':
    main()
