import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

    def step(self, throttle, alpha):
        # update
        Te = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e * self.w_e)
        Faero = self.c_a * self.v * self.v
        Rx = self.c_r1 * self.v
        Fg = self.m * self.g * alpha
        Fload = Faero + Rx + Fg
        self.x = self.x + self.v * self.sample_time
        self.w_e_dot = (Te - self.GR * self.r_e * Fload) / self.J_e 
        self.w_e = self.w_e + self.w_e_dot * self.sample_time
        w_w = self.w_e * self.GR
        s = (w_w * self.r_e - self.v) / self.v
        if np.abs(s) < 1:
            Fx = self.c * s
        else:
            Fx = self.F_max
        self.a = (Fx - Fload) / self.m
        self.v = self.v + self.a * self.sample_time
        pass
