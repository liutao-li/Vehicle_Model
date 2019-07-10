import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0  # the desired point at the center of gravity
        self.yc = 0
        self.theta = 0  #the heading angle(rad)
        self.delta = 0
        self.beta = 0  #the side slip angle(rad)

        self.L = 2  #the length of wheelbase
        self.lr = 1.2  #the distance between the center of  mass and the rear axle
        self.w_max = 1.22  #w_max:the maximum of steering rate

        self.sample_time = 0.01

    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta
        self.beta = 0

    def step(self, v, w):
        if w > self.w_max:
            w = self.w_max 
        self.xc = self.xc + v * np.cos(self.theta + self.beta) * self.sample_time
        self.yc = self.yc + v * np.sin(self.theta + self.beta) * self.sample_time
        self.theta = self.theta + v * np.cos(self.beta) * np.tan(self.delta) / self.L * self.sample_time
        self.delta = self.delta + w * self.sample_time
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)
        pass
