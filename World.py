# -*- coding: utf-8 -*-
"""
Created on Tue Sep 11 16:00:54 2018

@author: nathan
"""

import numpy as np
import random
import math
import PIL
from PIL import *

class World(object):
    def __init__(self):
        self.width = 540
        self.height = 360
        self.surface = np.zeros([self.height, self.width])
    
    def Show(self):
        Screen = PIL.Image.fromarray(self.surface)
        Screen.show()
    
    def World_gen_init(self, tileable):
        
        self.node_loc = np.zeros([16,2])
        self.node_rand = np.zeros([16,2])
        
        self.node_loc[:7,1] = 0.463647
        self.node_loc[7:14,1] = -0.463647
        self.node_loc[14,1] = math.pi/2
        self.node_loc[15,1] = -math.pi/2
        
        for n in range(7):
            self.node_loc[n,0] = (n - 3)*2/5*math.pi
            self.node_loc[n + 7,0] = ((n - 3)*2/5 + 0.2)*math.pi
        
        for n in range(16):
            A = random.randint(0,360)/180*math.pi
            self.node_rand[n,0] = math.cos(A)
            self.node_rand[n,1] = math.sin(A)
        if tileable:
            self.node_rand[5,:] = self.node_rand[0,:]
            self.node_rand[6,:] = self.node_rand[1,:]
            self.node_rand[12,:] = self.node_rand[7,:]
            self.node_rand[13,:] = self.node_rand[8,:]
            
    def World_gen(self, threshold):
        for x in range(self.width):
            lon = (2*x/self.width - 1)*math.pi
            for y in range(self.height):
                lat = (y/self.height - 0.5)*math.pi
                self.buffer = np.zeros([3,2]) + 4
                for n in range(16):
                    
                    if n == 5:
                        DUPLICATE = False
                        for rank in range(3):
                            if self.buffer[rank,0] == 0:
                                DUPLICATE = True
                                break
                        if DUPLICATE:
                            continue
                    if n == 6:
                        DUPLICATE = False
                        for rank in range(3):
                            if self.buffer[rank,0] == 1:
                                DUPLICATE = True
                                break
                        if DUPLICATE:
                            continue
                    if n == 12:
                        DUPLICATE = False
                        for rank in range(3):
                            if self.buffer[rank,0] == 7:
                                DUPLICATE = True
                                break
                        if DUPLICATE:
                            continue
                    if n == 13:
                        DUPLICATE = False
                        for rank in range(3):
                            if self.buffer[rank,0] == 8:
                                DUPLICATE = True
                                break
                        if DUPLICATE:
                            continue
                    if n > 13:
                        self.node_loc[n,0] = lon
                    a = math.sin(abs(lat - self.node_loc[n,1])/2)**2 + math.cos(lat)*math.cos(self.node_loc[n,1])*math.sin(abs(lon - self.node_loc[n,0])/2)**2
                    d = 2*math.atan2(math.sqrt(a), math.sqrt(1 - a))
                    for rank in range(3):
                        if d <= self.buffer[rank, 1]:
                            for moves in range(2 - rank):
                                self.buffer[-moves - 1, :] = self.buffer[-moves - 2, :]
                            self.buffer[rank,:] = [n, d]
                            break    
                """end of for range(16) loop"""
                
                A_dir = math.asin(math.sin(abs(lat - self.node_loc[int(self.buffer[0,0]),1]))*math.sin(self.buffer[0,1]))
                A_x = self.buffer[0,1]*math.cos(A_dir)
                A_y = self.buffer[0,1]*math.sin(A_dir)
                
                B_dir = math.asin(math.sin(abs(lat - self.node_loc[int(self.buffer[1,0]),1]))*math.sin(self.buffer[1,1]))
                B_x = self.buffer[0,1]*math.cos(B_dir)
                B_y = self.buffer[0,1]*math.sin(B_dir)
                
                C_dir = math.asin(math.sin(abs(lat - self.node_loc[int(self.buffer[2,0]),1]))*math.sin(self.buffer[2,1]))
                C_x = self.buffer[0,1]*math.cos(C_dir)
                C_y = self.buffer[0,1]*math.sin(C_dir)
                
                s = np.dot([A_x, A_y],self.node_rand[int(self.buffer[0,0]),:])
                t = np.dot([B_x, B_y],self.node_rand[int(self.buffer[1,0]),:])
                u = np.dot([C_x, C_y],self.node_rand[int(self.buffer[2,0]),:])
                
                UNIT_RADIAN = 1.6-math.cos(2*math.pi/5)*1.6
                
                f_s = 3*(self.buffer[0,1]/UNIT_RADIAN - 1)**2 + 2*(self.buffer[0,1]/UNIT_RADIAN - 1)**3
                f_t = 3*(self.buffer[1,1]/UNIT_RADIAN - 1)**2 + 2*(self.buffer[1,1]/UNIT_RADIAN - 1)**3  
                f_u = 3*(self.buffer[2,1]/UNIT_RADIAN - 1)**2 + 2*(self.buffer[2,1]/UNIT_RADIAN - 1)**3   
                
                self.surface[y, x] = (0.5 + (f_s*s + f_t*t + f_u*u)/3)*255
                
                if threshold > 0:
                    if self.surface[y,x] > threshold:
                        self.surface[y,x] = 255
                    else:
                        self.surface[y,x] = 0 
                
def main():
    test = World()
    test.World_gen_init(True)
    test.World_gen(0)
    test.Show()
    
if __name__ == '__main__':
    main()