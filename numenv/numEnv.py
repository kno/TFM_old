import gym
from gym import spaces, logger
import numpy as np

class NumEnv(gym.Env):
    number = 0
    
    def __init__(self):
        self.action_space = spaces.Discrete(7)
        l = np.array([0])
        h = np.array([100])
        self.observation_space = spaces.Box(l,h)

    def reset(self):
        self.number = np.random.randint(0,100)
        self.gnumber = np.random.randint(0,100)
        self.maxReward = abs(self.number - self.gnumber)
        print("Find the number", self.number,  " from " , self.gnumber, " for " , self.maxReward)
        self.steps = 0
        return [0]

    def render(self,mode):
        None

    def step(self, action):
        print("Action->" , action, self.gnumber)
        done = False
        reward = -1
        status = [3]
        self.steps += 1
        self.gnumber += action - 4
        #if (action == 0):
        #    self.gnumber += 1
        #elif (action == 1):
        #    self.gnumber -= 1
        #elif (action == 2):
        #    self.gnumber += 2
        #elif (action == 3):
        #    self.gnumber -= 2
        if (self.gnumber == self.number):
            reward = 10#self.maxReward
            done = True
            status = [0]
        #elif (self.gnumber > self.number):            
        #    status = [1]
        #else:
        #    status = [2]
        if (self.steps >= 75):
            reward = -10
            done = True
        status = [self.gnumber - self.number]
        return status, reward, done, {}

    def stop(self):
        None