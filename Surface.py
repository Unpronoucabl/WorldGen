# -*- coding: utf-8 -*-
"""
Created on Sun Sep  2 15:34:51 2018

@author: nathan
"""

import numpy as np
import random
import math
import PIL
from PIL import *


class Surface(object):
           
    def __init__(self, size, depth):
        self.size = size
        self.depth = depth
        self.surface = np.zeros(size)
        
    def Simplex_gen(self):
        UNIT_RADIAN = 1.10557
        """72/180*math.pi"""
        self.node_loc = np.zeros((15,2))
        
        """ East/West (36 (72) degrees apart)"""
        self.node_lon = np.zeros([15])
        """North/South (tan^-1(1/2)N/S)"""
        self.node_lat = np.zeros([15])
        
        """ node locations                      Rad
                     tan(0.5) ~  26.56   ~            0.463647
        0 (-216, np.arctan(0.5))     5     -3.769911
        1 (-144, np.arctan(0.5))   1       -2.513274
        2 (-72,  np.arctan(0.5))   2       -1.256637
        3 (0,    np.arctan(0.5))   3        0.000000
        4 (72,   np.arctan(0.5))   4        1.256637
        5 (144,  np.arctan(0.5))   5        2.513274
        6 (216,  np.arctan(0.5))     1      3.769911
                                                     -0.0463647
        7 (-180, -np.arctan(0.5))   1       -pi
        8 (-108, -np.arctan(0.5))   2       -1.884956
        9 (-36,  -np.arctan(0.5))   3       -0.628319
        10 (36,  -np.arctan(0.5))   4        0.628319
        11 (108, -np.arctan(0.5))   5        1.884956
        12 (180, -np.arctan(0.5))   1        pi
        
        13 (n/a ("0"), 90)
        14 (n/a ("0"), -90)
        """
        self.node_loc[:7,1] = 0.463647
        self.node_loc[7:13,1] = -0.463647
        self.node_loc[13,1] = math.pi/2
        self.node_loc[14,1] = -math.pi/2
        
        for n in range(7):
            self.node_loc[n,0] = (n - 3)*2/5*math.pi
        for n in range(6):
            self.node_loc[n + 7,0] = ((n - 3)*2/5 + 0.2)*math.pi
        B = 0
        
        for n in range(11):
            if n == 5:
                """wrapround"""
                self.node_lon[n + 1] = self.node_lon[1]
                self.node_lat[n + 1] = self.node_lat[1]
                continue
            B = random.randint(0,360)/180*math.pi
            """
            b = math.asin(math.sin(B)*math.sin(UNIT_RADIAN))
            a = math.acos(math.cos(UNIT_RADIAN)/math.cos(b))
            """
            b = math.cos(B)
            a = math.sin(B)
            
            self.node_lon[n + 1] = a
            self.node_lat[n + 1] = b
            """
            self.node_lon[n + 1] = self.node_loc[n + 1,0] + a
            self.node_lat[n + 1] = self.node_loc[n + 1,1] + b
            
            """
        """more wraparound"""
        self.node_lon[0] = self.node_lon[5]
        self.node_lat[0] = self.node_lat[5]
        self.node_lon[12] = self.node_lon[7]
        self.node_lat[12] = self.node_lat[7]
        """Poles"""
        self.node_lon[13] = random.randint(-180,180)/180*math.pi
        self.node_lat[13] = 7/45*math.pi
        
        self.node_lon[14] = random.randint(-180,180)/180*math.pi
        self.node_lat[14] = -7/45*math.pi
        
        for y in range(self.size[0]):
            lat = (0.5 - y/self.size[0])*np.pi
            """
            if lat < -0.463647:
                layer = 3
            elif lat > 0.463647:
                layer = 1
            else:
                layer = 2
               """ 
            for x in range(self.size[1]):
                lon = (2*x/self.size[1] - 1)*np.pi
                """
                Triangles                   Layers
                 /\  /\  /\  /\  /\  /        1
                /1_\/2_\/_3\/_4\/_5\/_1     
                \6 /\ 8/\10/\12/\14/\6        2
               15\/7_\/9_\/11\/13\/15\
               20/\16/\17/\18/\19/\20/        3
                /  \/  \/  \/  \/  \/
                """
                """
                FIRST = True
                SECOND = False
                """
                self.buffer = np.zeros([3,5])
                for n in range(14):
                    a = lon - self.node_loc[n,0]
                    b = lat - self.node_loc[n,1]
                    """if abs(a) > UNIT_RADIAN or abs(b) > UNIT_RADIAN:
                        continue"""
                    d = math.sin(b/2)**2 + math.cos(lat)*math.cos(self.node_loc[n,1])*math.sin(a/2)**2
                    c = 2*math.atan2(math.sqrt(d), math.sqrt(1 - d))
                    """if c > UNIT_RADIAN:
                        continue
                        """
                    """if FIRST:"""
                    """
                    FIRST = False
                    SECOND = True
                    """
                    if self.node_lon[n] == self.buffer[0,2] and self.node_lat[n] == self.buffer[0,3]:
                        continue
                    elif self.node_lon[n] == self.buffer[1,2] and self.node_lat[n] == self.buffer[1,3]:
                        continue
                    elif self.node_lon[n] == self.buffer[2,2] and self.node_lat[n] == self.buffer[2,3]:
                        continue
                    for rank in range(3):
                        if self.buffer[rank,4] == 0:
                            self.buffer[rank,:] = [a, b, self.node_lon[n] - lon, self.node_lat[n] - lat, c/UNIT_RADIAN]
                            break                    
                        if self.buffer[rank,4] > c:
                            for moves in range(2 - rank):
                                self.buffer[-moves-1,:] = self.buffer[-moves-2,:]
                            self.buffer[rank,:] = [a, b, self.node_lon[n] - lon, self.node_lat[n] - lat, c/UNIT_RADIAN]
                        break
                    """
                    elif self.buffer[1,4] == 0:
                        self.buffer[1,:] = [a, b, self.node_lon[n], self.node_lat[n], c/UNIT_RADIAN]
                        continue
                    elif self.buffer[1,4] > c:
                        self.buffer[2,:] = self.buffer[1,:]
                        self.buffer[1,:] = [a, b, self.node_lon[n], self.node_lat[n], c/UNIT_RADIAN]
                        continue
                    elif self.buffer[2,4] == 0:
                        self.buffer[2,:] = [a, b, self.node_lon[n], self.node_lat[n], c/UNIT_RADIAN]
                    elif self.buffer[2,4] > c:
                        self.buffer[2,:] = [a, b, self.node_lon[n], self.node_lat[n], c/UNIT_RADIAN]
                        else:                            
                            SECOND = False
                            
                        if layer == 1:
                            a = lon - self.node_loc[13,0]
                            b = lat - self.node_loc[13,1]
                            d = math.sin(b/2)**2 + math.cos(lat)*math.cos(self.node_loc[13,1])*math.sin(a/2)**2
                            c = 2*math.atan2(math.sqrt(d), math.sqrt(1 - d))
                            self.buffer[2,:] = [0, math.pi/2 - lat, self.node_lon[13], self.node_lat[13], c/UNIT_RADIAN]
                            break
                        elif layer == 3:
                            a = lon - self.node_loc[14,0]
                            b = lat - self.node_loc[14,1]
                            d = math.sin(b/2)**2 + math.cos(lat)*math.cos(self.node_loc[14,1])*math.sin(a/2)**2
                            c = 2*math.atan2(math.sqrt(d), math.sqrt(1 - d))
                            self.buffer[2,:] = [0, -math.pi/2 - lat, self.node_lon[14], self.node_lat[14], c/UNIT_RADIAN]
                            break
                    elif self.node_lon[n] == self.buffer[0,2] and self.node_lat[n] == self.buffer[0,3]:
                        continue
                    
                    else:
                        self.buffer[2,:] = [a, b, self.node_lon[n], self.node_lat[n], c/UNIT_RADIAN]
                        break                    
                    """
                s = np.dot(self.buffer[0,0:1],self.buffer[0,2:3]) 
                t = np.dot(self.buffer[1,0:1],self.buffer[1,2:3])
                u = np.dot(self.buffer[2,0:1],self.buffer[2,2:3])        
                """
                Sum = np.sum(self.buffer[:,4])
                S_s = 1-2*self.buffer[0,4]/Sum
                S_t = 1-2*self.buffer[1,4]/Sum
                S_u = 1-2*self.buffer[2,4]/Sum
                """
                S_s = 3*(self.buffer[0,4]/UNIT_RADIAN)**2-2*(self.buffer[0,4]/UNIT_RADIAN)**3
                S_t = 3*(self.buffer[1,4]/UNIT_RADIAN)**2-2*(self.buffer[1,4]/UNIT_RADIAN)**3
                S_u = 3*(self.buffer[2,4]/UNIT_RADIAN)**2-2*(self.buffer[2,4]/UNIT_RADIAN)**3
                """
                """
                self.surface[y, x] = (0.5 + (s*S_s + t*S_t + u*S_u)/3)*255
                """
                self.surface[y, x] = (0.5 + (s + t + u)/3)*255
                """
        
    def Perlin_gen(self, grid_x_nodes, grid_y_nodes, tileable):
        self.gridsize = [grid_x_nodes - 1, grid_y_nodes - 1]
        self.x_node = np.zeros((grid_x_nodes, grid_y_nodes))
        self.y_node = np.zeros((grid_x_nodes, grid_y_nodes))
        self.cellsize_x = int(np.floor(self.size[0]/self.gridsize[0]))
        self.cellsize_y = int(np.floor(self.size[1]/self.gridsize[1]))
        
        if self.size[0] % self.gridsize[0] != 0 or self.size[1] % self.gridsize[1] != 0:
            print('Trimming canvas to fit grid')
            self.size = [self.cellsize_x * self.gridsize[0], self.cellsize_y * self.gridsize[1]]
            self.surface = np.zeros(self.size)
        
        A = 0
        for x in range(self.gridsize[0] + 1):
            for y in range(self.gridsize[1] + 1):
                A = random.randint(0,360)/180*np.pi
                self.x_node[x,y] = np.cos(A)
                
                self.y_node[x,y] = np.sin(A)
        """         
        """
        if tileable:
            for x in range(self.gridsize[0]):
                self.x_node[x, self.gridsize[0]] = self.x_node[x,0]
                self.y_node[x, self.gridsize[0]] = self.y_node[x,0]
            for y in range(self.gridsize[1]):
                self.y_node[self.gridsize[1], y] = self.y_node[0,y]
                self.x_node[self.gridsize[1], y] = self.x_node[0,y]
        
        """ Old method 
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                vec2_x0y0 = ((x % self.cellsize_x)/self.cellsize_x, (y % self.cellsize_y)/self.cellsize_y)
                vec2_x1y0 = (-(self.cellsize_x - vec2_x0y0[0])/self.cellsize_x, vec2_x0y0[1])
                vec2_x0y1 = (vec2_x0y0[0], -(self.cellsize_y - vec2_x0y0[1])/self.cellsize_y)
                vec2_x1y1 = (vec2_x1y0[0],vec2_x0y1[1])
                
                grid_x0 = int(np.floor(x/self.cellsize_x))
                grid_y0 = int(np.floor(y/self.cellsize_y))
                
                s = np.dot([self.x_node[grid_x0, grid_y0], self.y_node[grid_x0, grid_y0]], vec2_x0y0)
                t = np.dot([self.x_node[grid_x0 + 1, grid_y0], self.y_node[grid_x0 + 1, grid_y0]], vec2_x1y0)
                u = np.dot([self.x_node[grid_x0, grid_y0 + 1], self.y_node[grid_x0, grid_y0 + 1]], vec2_x0y1)
                v = np.dot([self.x_node[grid_x0 + 1, grid_y0 + 1], self.y_node[grid_x0 + 1, grid_y0 + 1]], vec2_x1y1)
                            
                S_x = 3*(vec2_x0y0[0])**2 - 2*vec2_x0y0[0]**3
                S_y = 3*(vec2_x0y0[1])**2 - 2*vec2_x0y0[1]**3
                                
                self.surface[x,y] = ((s + S_x*(t - s)) + S_y*(-(u + S_x*(v - u)) + (s + S_x*(t - s))))*self.depth
         """
        for G_x in range(self.gridsize[0]):
             for G_y in range(self.gridsize[1]):
                 for C_x in range(self.cellsize_x):
                     for C_y in range(self.cellsize_y):
                         x0y0 = np.array([[C_x/self.cellsize_x], [C_y/self.cellsize_y]])
                         x1y0 = np.array([[C_x/self.cellsize_x - 1], [C_y/self.cellsize_y]])
                         x0y1 = np.array([[C_x/self.cellsize_x], [C_y/self.cellsize_y - 1]])
                         x1y1 = np.array([[C_x/self.cellsize_x - 1], [C_y/self.cellsize_y - 1]])
                         
                         s = np.dot([[self.x_node[G_x, G_y],self.y_node[G_x, G_y]]], x0y0)
                         t = np.dot([[self.x_node[G_x + 1, G_y],self.y_node[G_x + 1, G_y]]], x1y0)
                         u = np.dot([[self.x_node[G_x, G_y + 1],self.y_node[G_x, G_y + 1]]], x0y1)
                         v = np.dot([[self.x_node[G_x + 1, G_y + 1],self.y_node[G_x + 1, G_y + 1]]], x1y1)
                         
                         S_x = 3*(x0y0[0])**2 - 2*x0y0[0]
                         S_y = 3*(x0y0[1])**2 - 2*x0y0[1]
                         """
                         S_x = x0y0[0]
                         S_y = x0y0[1]
                         """
                         X = G_x*self.cellsize_x + C_x
                         Y = G_y*self.cellsize_y + C_y
                         
                         self.surface[X, Y] = (0.5+(s + S_x*(t - s)) + S_y*((u + S_x*(v - u)) - (s + S_x*(t - s))))*255
                         """
                         self.surface[X, Y] = (0.5+(s + t + u + v)/4)*255
                         
                         """
            
    def Show(self):
        Screen = PIL.Image.fromarray(self.surface)
        Screen.show()
def main():
    
    test = Surface([360,540],255)
    """test.Perlin_gen(21,21,0)"""
    
    test.Simplex_gen()
    
    test.Show()
    
    
    def Fractal_gen(size, init_grid_x, init_grid_y,iterations, tileable):
        collector = [None]*iterations
        tot = 0
        for n in range(iterations):
            collector[n] = Surface(size,255)
            collector[n].Perlin_gen(int((init_grid_x - 1)/(2**n) + 1),int((init_grid_y - 1)/(2**n) + 1),1)
            tot = tot + n + 1
        Final = Surface(size, 255)
        for n in range(iterations):
            Final.surface = Final.surface + (n+1)/tot*collector[n].surface
        Screen = PIL.Image.fromarray(Final.surface)
        Screen.show()
        
    """Fractal_gen([1024,1024],129,129,7,1)"""
            
if __name__ == '__main__':
    main()
    