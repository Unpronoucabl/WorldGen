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

class FastWorld(object):
    def __init__(self, width,height):
        self.width = width
        self.height = height
        self.surface = np.zeros([self.height, self.width])
    
    def Show(self):
        Screen = PIL.Image.fromarray(self.surface)
        Screen.show()
    
    def World_gen_init(self, gridsize):
        self.DISC_HEIGHT = math.atan(0.5)
        self.UNIT_RADIAN = (1 - math.cos(2*math.pi/5))*1.6/(gridsize + 1)    
        self.lon_hdist = math.pi/5/(gridsize + 1)
        """self.lat_list = np.arange(-math.pi/2,math.pi/2,math.pi/(3*gridsize))"""
        self.lat_list = [None]*(4 + 3*gridsize)
        for lats in range(gridsize + 2):
            self.lat_list[lats] = math.pi/2 - lats*(math.pi/2 - self.DISC_HEIGHT)/(gridsize + 1)
            self.lat_list[-lats - 1] = -self.lat_list[lats]
        for lats in range(gridsize):
            self.lat_list[lats + gridsize + 2] = self.DISC_HEIGHT - (lats + 1)*2*self.DISC_HEIGHT/(gridsize + 1)
            
        self.rand_list = [None]*(4 + 3*gridsize)
        for num_lon in range(gridsize + 2):
            self.rand_list[num_lon] = np.zeros([max(1,5*num_lon),2])
            self.rand_list[-num_lon - 1] = np.zeros([max(1,5*num_lon),2])
        for num_lon in range(gridsize):
            self.rand_list[num_lon + gridsize + 2] = np.zeros([(gridsize + 1)*5,2])
        
        for lats in range(len(self.rand_list)):
            Buf = np.zeros(self.rand_list[lats].shape)
            for n in range(len(self.rand_list[lats])):
                A = random.randint(0,360)/180*math.pi
                Buf[n,:] = [math.cos(A), math.sin(A)]
            self.rand_list[lats] = Buf 
                        
    def World_gen(self):
        lat_counter = 1
        self.node_loc = np.zeros([4,3])
        POLAR = True
        layerlim = 0
        for y in range(self.height):
            lat = (0.5 - y/self.height)*math.pi
            if lat < self.lat_list[lat_counter]:
                lat_counter = lat_counter + 1
                if lat < self.DISC_HEIGHT:
                    if lat <= -self.DISC_HEIGHT:
                        POLAR = True
                        layerlim = layerlim - 1
                    elif POLAR == True:
                        layerlim = lat_counter
                        POLAR = False
            odd = lat_counter % 2
            lat1 = self.lat_list[lat_counter - 1]
            lat2 = self.lat_list[lat_counter]
            for x in range(self.width):
                lon = (2*x/self.width - 1)*math.pi
                if POLAR:
                    if layerlim > 0:
                        hdist1 = 0.4*math.pi/layerlim
                        if layerlim == 1:
                            hdist2 = 1
                        else:
                            hdist2 = 0.4*math.pi/(layerlim - 1)
                    else:
                        if lat_counter == 1:
                            hdist1 = 1
                        else:
                            hdist1 = 0.4*math.pi/(lat_counter - 1)
                        hdist2 = 0.4*math.pi/lat_counter
                else:
                    hdist1 = 2*self.lon_hdist
                    hdist2 = hdist1
                lon1 = lon/hdist1
                lon2 = lon/hdist2
                nodeLons = np.array([math.ceil(lon1)*hdist1, math.floor(lon1)*hdist1, math.ceil(lon2)*hdist2, math.floor(lon2)*hdist2])
                nodeNo = np.array([[math.ceil(lon1) + math.floor(len(self.rand_list[lat_counter - 1])/2),
                                   math.floor(lon1) + math.floor(len(self.rand_list[lat_counter - 1])/2)],
                                   [math.ceil(lon2) + math.floor(len(self.rand_list[lat_counter])/2),
                                   math.floor(lon2) + math.floor(len(self.rand_list[lat_counter])/2)]])
                if POLAR:
                    if layerlim > 0:
                        nodeLons[1] = nodeLons[1] + hdist1/2
                        nodeLons[0] = nodeLons[0] + hdist1/2
                        nodeLons[3] = nodeLons[3] + hdist2/2
                        nodeLons[2] = nodeLons[2] + hdist2/2
                elif odd == 1:
                    nodeLons[1] = nodeLons[1] + hdist1/2
                    nodeLons[0] = nodeLons[0] + hdist1/2
                    
                else:
                    nodeLons[3] = nodeLons[3] + hdist2/2
                    nodeLons[2] = nodeLons[2] + hdist2/2
                    
                if nodeLons[0] == nodeLons[1]:
                    nodeLons[1] = nodeLons[0] + hdist1
                    nodeNo[0,1] = nodeNo[0,0] + 1
                if nodeLons[3] == nodeLons[2]:
                    nodeLons[3] = nodeLons[2] + hdist2
                    nodeNo[1,1] = nodeNo[1,0] + 1
                    
                if nodeLons[1] >= lon:
                    nodeLons[0] = nodeLons[0] - hdist1
                    nodeLons[1] = nodeLons[1] - hdist1
                    nodeNo[0,0] = nodeNo[0,0] - 1
                    nodeNo[0,1] = nodeNo[0,1] - 1
                elif nodeLons[0] < lon:
                    nodeLons[0] = nodeLons[0] + hdist1
                    nodeLons[1] = nodeLons[1] + hdist1
                    nodeNo[0,0] = nodeNo[0,0] + 1
                    nodeNo[0,1] = nodeNo[0,1] + 1
                if nodeLons[3] >= lon:
                    nodeLons[3] = nodeLons[3] - hdist2
                    nodeLons[2] = nodeLons[2] - hdist2
                    nodeNo[1,0] = nodeNo[1,0] - 1
                    nodeNo[1,1] = nodeNo[1,1] - 1
                elif nodeLons[2] < lon:
                    nodeLons[3] = nodeLons[3] + hdist2
                    nodeLons[2] = nodeLons[2] + hdist2
                    nodeNo[1,0] = nodeNo[1,0] + 1
                    nodeNo[1,1] = nodeNo[1,1] + 1
                
                for wrap in range(2):
                    if nodeNo[0, wrap] >= len(self.rand_list[lat_counter - 1]):
                        nodeNo[0, wrap] = nodeNo[0, wrap] - len(self.rand_list[lat_counter - 1])
                    if nodeNo[1, wrap] >= len(self.rand_list[lat_counter]):
                        nodeNo[1, wrap] = nodeNo[1, wrap] - len(self.rand_list[lat_counter])    
                
                if lat > self.lat_list[1]:
                    nodeLons[0] = 0
                    nodeLons[1] = 0
                    nodeNo[0,:] = [0,0]
                elif lat < self.lat_list[-2]:
                    nodeLons[2] = 0
                    nodeLons[3] = 0
                    nodeNo[1,:] = [0,0]
                    
                self.node_loc = np.array([[nodeLons[0], lat1, nodeNo[0,0], lat_counter - 1, 0],
                                         [nodeLons[1], lat1, nodeNo[0,1], lat_counter - 1, 1],
                                         [nodeLons[2], lat2, nodeNo[1,0], lat_counter, 2],
                                         [nodeLons[3], lat2, nodeNo[1,1], lat_counter, 3]])
                self.buffer = np.zeros([3,2]) + 4
                for n in range(len(self.node_loc)):
                    if lat > self.lat_list[1] and self.node_loc[n,4] == 1:
                        continue
                    if lat < self.lat_list[-2] and self.node_loc[n,4] == 3:
                        continue
                    a = math.sin(abs(lat - self.node_loc[n,1])/2)**2 + math.cos(lat)*math.cos(self.node_loc[n,1])*math.sin(abs(lon - self.node_loc[n,0])/2)**2
                    d = abs(2*math.atan2(math.sqrt(a), math.sqrt(1 - a)))
                    for rank in range(3):
                        if d <= self.buffer[rank, 1]:
                            for moves in range(2 - rank):
                                self.buffer[-moves - 1, :] = self.buffer[-moves - 2, :]   
                            """if d + 2 == 2:
                                    d = 0"""
                            self.buffer[rank,:] = [self.node_loc[n,4], d]
                            """if abs(self.node_loc[n,1]) == math.pi:
                                self.buffer[rank,:] = [0, d]"""
                            break
                    
                s_n = int(self.buffer[0,0])
                t_n = int(self.buffer[1,0])
                u_n = int(self.buffer[2,0])
                
                A_dir = math.asin(math.sin(abs(lat - self.node_loc[s_n,1]))*math.sin(self.buffer[0,1]))
                A_x = self.buffer[0,1]*math.cos(A_dir)
                A_y = self.buffer[0,1]*math.sin(A_dir)
                
                B_dir = math.asin(math.sin(abs(lat - self.node_loc[t_n,1]))*math.sin(self.buffer[1,1]))
                B_x = self.buffer[1,1]*math.cos(B_dir)
                B_y = self.buffer[1,1]*math.sin(B_dir)
                
                C_dir = math.asin(math.sin(abs(lat - self.node_loc[u_n,1]))*math.sin(self.buffer[2,1]))
                C_x = self.buffer[2,1]*math.cos(C_dir)
                C_y = self.buffer[2,1]*math.sin(C_dir)

                s = np.dot([A_x, A_y],self.rand_list[int(self.node_loc[s_n,3])][int(self.node_loc[s_n,2])])
                t = np.dot([B_x, B_y],self.rand_list[int(self.node_loc[t_n,3])][int(self.node_loc[t_n,2])])
                u = np.dot([C_x, C_y],self.rand_list[int(self.node_loc[u_n,3])][int(self.node_loc[u_n,2])])
                """
                if POLAR and layerlim > 0:
                        
                    s_dist = self.buffer[0,1]/hdist1
                    t_dist = self.buffer[1,1]/hdist1
                    u_dist = self.buffer[2,1]/hdist1
                else:
                    s_dist = self.buffer[0,1]/hdist2
                    t_dist = self.buffer[1,1]/hdist2
                    u_dist = self.buffer[2,1]/hdist2
                """
                s_dist = self.buffer[0,1]
                t_dist = self.buffer[1,1]
                u_dist = self.buffer[2,1]
                
                """
                elif t_dist + 2 == 2:
                    t_dist = 0
                if u_dist + 2 ==2:
                    u_dist = 0
                """
                """
                f_s = max(3*(s_dist-1)**2 + 2*(s_dist-1)**3,0)
                f_t = max(3*(t_dist-1)**2 + 2*(t_dist-1)**3,0)  
                f_u = max(3*(u_dist-1)**2 + 2*(u_dist-1)**3,0)   
                
                
                tot_dist = 1/(s_dist + t_dist + u_dist)
                f_s = max(1 - 2*s_dist*tot_dist,0)
                f_t = max(1 - 2*t_dist*tot_dist,0)
                f_u = max(1 - 2*u_dist*tot_dist,0)
                """
            
                f_s = max(1 - s_dist/(t_dist + u_dist),0)**2
                f_t = max(1 - t_dist/(s_dist + u_dist),0)**2
                f_u = max(1 - u_dist/(s_dist + t_dist),0)**2
                """
                """
                self.surface[y, x] = (0.5 + (f_s*s + f_t*t + f_u*u))*255
                
            """End x loop"""
                
    def Threshold(self,level):
        for x in range(self.width):
            for y in range(self.height):
                if self.surface[y,x] > level:
                    self.surface[y,x] = 255
                else:
                    self.surface[y,x] = 0                 
                
def main(width, height):
    
    Iterate(width, height, 12)
    """
    Just_one(width, height)
    """
def scale(array, depth):
    Min = array.min()
    array = (array - Min)
    Max = array.max()
    array = array*depth/Max
    return array
    
def Iterate(width, height, iterations):
    collector = [None]*iterations
    weight = 0
    time.clock()
    for n in range(iterations):
        collector[n] = FastWorld(width,height)
        collector[n].World_gen_init(iterations - n - 1)
        collector[n].World_gen()
        weight = weight + n + 1
        print(time.clock())
    final = FastWorld(width,height)
    for n in range(iterations):
        final.surface = np.add(np.roll(final.surface, int(width/137), axis=1), (n + 1)/weight*collector[n].surface)
    final.surface = scale(final.surface, 255)
    imageio.imwrite('Polar7.png', final.surface)
    final.Show()
    return final
    
def Just_one(width, height):
    test = FastWorld(width, height)
    test.World_gen_init(2)
    test.World_gen()
    test.surface = scale(test.surface,255)
    test.Show()
    
if __name__ == '__main__':
    main(540,360)