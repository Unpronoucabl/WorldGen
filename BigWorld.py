# -*- coding: utf-8 -*-
"""
Created on Tue Sep 11 16:00:54 2018

@author: nathan
"""

import numpy as np
import random
import math
import PIL
import time
import imageio
from PIL import *

class BigWorld(object):
    def __init__(self, width,height):
        self.width = width
        self.height = height
        self.surface = np.zeros([self.height, self.width])
    
    def Show(self):
        Screen = PIL.Image.fromarray(self.surface)
        Screen.show()
    
    def World_gen_init(self, gridsize, tileable):
        DISC_HEIGHT = math.atan(0.5)
        self.UNIT_RADIAN = (1 - math.cos(2*math.pi/5))*1.6/(gridsize + 1)

        self.lon_list = [None]*(4 + 3*gridsize)
        for num_lon in range(gridsize + 2):
            if num_lon == 0:
                self.lon_list[0] = 0
                self.lon_list[-1] = 0
                continue
            self.lon_list[num_lon] = np.arange(-math.pi,math.pi,2*math.pi/5/num_lon)
            self.lon_list[-num_lon - 1] = np.arange(-math.pi,math.pi,2*math.pi/5/num_lon) + math.pi/5/num_lon
        for num_lon in range(gridsize):
            self.lon_list[num_lon + gridsize + 2] = np.arange(-math.pi,math.pi,2*math.pi/5/(gridsize + 1)) + math.pi/5/(gridsize + 1)*((1 + num_lon) % 2)  
        
        """self.lat_list = np.arange(-math.pi/2,math.pi/2,math.pi/(3*gridsize))"""
        self.lat_list = [None]*(4 + 3*gridsize)
        for lats in range(gridsize + 2):
            self.lat_list[lats] = np.zeros([max(1,5*lats)]) + (math.pi/2 - lats*(math.pi/2 - DISC_HEIGHT)/(gridsize + 1))
            self.lat_list[-lats - 1] = -self.lat_list[lats]
        for lats in range(gridsize):
            self.lat_list[lats + gridsize + 2] = np.zeros((gridsize + 1)*5) + DISC_HEIGHT - (lats + 1)*2*DISC_HEIGHT/(gridsize + 1)
            
        self.rand_list = [None]*(4 + 3*gridsize)
        for num_lon in range(gridsize + 2):
            self.rand_list[num_lon] = np.zeros([max(1,5*num_lon),2])
            self.rand_list[-num_lon - 1] = np.zeros([max(1,5*num_lon),2])
        for num_lon in range(gridsize):
            self.rand_list[num_lon + gridsize + 2] = np.zeros([(gridsize + 1)*5,2])
        
        for lons in range(len(self.rand_list)):
            Buf = np.zeros(self.rand_list[lons].shape)
            for n in range(len(self.rand_list[lons])):
                A = random.randint(0,360)/180*math.pi
                Buf[n,:] = [math.cos(A), math.sin(A)]
            self.rand_list[lons] = Buf 
                        
    def World_gen(self):
        lat_counter = 1
        for y in range(self.height):
            lat = (0.5 - y/self.height)*math.pi
            Lon_loc = np.append(self.lon_list[lat_counter - 1], self.lon_list[lat_counter])
            Lat_loc = np.append(self.lat_list[lat_counter - 1], self.lat_list[lat_counter])
            self.node_loc = np.transpose(np.vstack((Lon_loc, Lat_loc)))
            
            self.node_rand = np.vstack((self.rand_list[lat_counter - 1], self.rand_list[lat_counter]))
            for x in range(self.width):
                lon = (2*x/self.width - 1)*math.pi
                self.buffer = np.zeros([3,2]) + 4
                for n in range(len(self.node_loc)):
                    a = math.sin(abs(lat - self.node_loc[n,1])/2)**2 + math.cos(lat)*math.cos(self.node_loc[n,1])*math.sin(abs(lon - self.node_loc[n,0])/2)**2
                    d = 2*math.atan2(math.sqrt(a), math.sqrt(1 - a))
                    for rank in range(3):
                        if d <= self.buffer[rank, 1]:
                            for moves in range(2 - rank):
                                self.buffer[-moves - 1, :] = self.buffer[-moves - 2, :]
                            self.buffer[rank,:] = [n, d]
                            break    
                
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
                
                f_s = max(3*(self.buffer[0,1]/self.UNIT_RADIAN - 1)**2 + 2*(self.buffer[0,1]/self.UNIT_RADIAN - 1)**3,0)
                f_t = max(3*(self.buffer[1,1]/self.UNIT_RADIAN - 1)**2 + 2*(self.buffer[1,1]/self.UNIT_RADIAN - 1)**3,0)  
                f_u = max(3*(self.buffer[2,1]/self.UNIT_RADIAN - 1)**2 + 2*(self.buffer[2,1]/self.UNIT_RADIAN - 1)**3,0)   
                
                self.surface[y, x] = (0.5 + (f_s*s + f_t*t + f_u*u))*255
                
            """End x loop"""
            if lat < self.lat_list[lat_counter][0]:
                lat_counter = lat_counter + 1
                
    def Threshold(self,level):
        for x in range(self.width):
            for y in range(self.height):
                if self.surface[y,x] > level:
                    self.surface[y,x] = 255
                else:
                    self.surface[y,x] = 0                 
                
def main(width, height, iterations):
    
    collector = [None]*iterations
    weight = 0
    time.clock()
    for n in range(iterations):
        collector[n] = BigWorld(width,height)
        collector[n].World_gen_init(iterations - n - 1,True)
        collector[n].World_gen()
        weight = weight + n + 1
        print(time.clock())
    final = BigWorld(width,height)
    for n in range(iterations):
        final.surface = np.add(np.roll(final.surface, 2, axis=1), (n + 1)/weight*collector[n].surface)
    imageio.imwrite('Polar1.png', final.surface)
    final.Show()
    
if __name__ == '__main__':
    main(540,360,1)