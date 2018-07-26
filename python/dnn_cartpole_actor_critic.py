#coding:utf-8
import argparse
import gym
import numpy as np
from itertools import count
from collections import namedtuple
import  os
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical



parser = argparse.ArgumentParser(description='PyTorch actor-critic example')
parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
                    help='discount factor (default: 0.99)')
parser.add_argument('--seed', type=int, default=543, metavar='N',
                    help='random seed (default: 1)')
parser.add_argument('--render', action='store_true',
                    help='render the environment')
parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                    help='interval between training status logs (default: 10)')
parser.add_argument("--model-path",type=str,default="",help="forward test")
args = parser.parse_args()


env = gym.make('CartPole-v0')
env.seed(args.seed)
torch.manual_seed(args.seed)
model_path=args.model_path if os.path.exists(args.model_path) else "./data2.pkl"

SavedAction = namedtuple('SavedAction', ['log_prob', 'value'])


#输出1：向左/向右(0/1)的概率
#输出2：状态值
class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.affine1 = nn.Linear(4, 128)
        self.action_head = nn.Linear(128, 2)
        self.value_head = nn.Linear(128, 1)

        self.saved_actions = []
        self.rewards = []

    def forward(self, x):
        x = F.relu(self.affine1(x))
        action_scores = self.action_head(x)
        state_values = self.value_head(x)
        return F.softmax(action_scores, dim=-1), state_values


model = Policy()
optimizer = optim.Adam(model.parameters(), lr=3e-2)
eps = np.finfo(np.float32).eps.item()


def select_action(state):
    state = torch.from_numpy(state).float()
    probs, state_value = model(state)
    m = Categorical(probs)
    action = m.sample()
    model.saved_actions.append(SavedAction(m.log_prob(action), state_value))
    return action.item()




def finish_episode():
    R = 0
    saved_actions = model.saved_actions
    policy_losses = []
    value_losses = []
    rewards = []
    for r in model.rewards[::-1]:
        #gamma越大说明越往后的行为加分越高
        R = r + args.gamma * R
        rewards.insert(0, R)
    rewards = torch.tensor(rewards)
    rewards = (rewards - rewards.mean()) / (rewards.std() + eps)
    #actor:输入state-输出action
    #critic:输入(state,action)-输出Q值
    for (log_prob, value), r in zip(saved_actions, rewards):
        reward = r - value.item() #实际reward=仿真reward-网络预测reward
        policy_losses.append(-log_prob * reward)
        value_losses.append(F.smooth_l1_loss(value, torch.tensor([r])))
    optimizer.zero_grad()
    loss = torch.stack(policy_losses).sum() + torch.stack(value_losses).sum()
    loss.backward()
    optimizer.step()
    del model.rewards[:]
    del model.saved_actions[:]


def train():
    running_reward = 10
    for i_episode in count(1):
        state = env.reset()
        for t in range(10000):  # Don't infinite loop while learning
            action = select_action(state)
            state, reward, done, _ = env.step(action)
            if args.render:
                env.render()
            model.rewards.append(reward)
            if done:
                break

        running_reward = running_reward * 0.99 + t * 0.01
        finish_episode()
        if i_episode % args.log_interval == 0:
            print('Episode {}\tLast length: {:5d}\tAverage length: {:.2f}'.format(
                i_episode, t, running_reward))
        if running_reward > env.spec.reward_threshold:
            print("Solved! Running reward is now {} and "
                  "the last episode runs to {} time steps!".format(running_reward, t))
            torch.save(model.state_dict(),model_path)
            break




def test2(loc=0.5):
    state = env.reset()
    for i in count(1):
        pos,pos_v,ang,ang_v=state
        action=ang+ang_v*0.1+(pos-loc)*0.1+pos_v*0.1
        state, reward, done, _ = env.step(0 if action<=0 else 1)
        env.render()


def test1():
    model.load_state_dict(torch.load(model_path))
    ##训练参数的个数
    ##np.sum([p.numel() for p in policy.parameters() if p.requires_grad])
    state = env.reset()
    for t in count(1):
        state = torch.from_numpy(state).float()
        probs, state_value = model(state)
        print (probs, state_value)
        m = Categorical(probs)
        action = m.sample().item()
        state, reward, done, _ = env.step(action)
        env.render()
        # print("count %d"%t)

if __name__ == '__main__':
    # train()
    test1()
