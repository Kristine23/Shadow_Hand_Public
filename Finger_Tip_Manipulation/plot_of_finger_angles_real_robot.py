import String_Handler
import matplotlib.pyplot as plt
import Mujoco_Simulator
import Shadow_Hand

from mpl_toolkits.mplot3d import axes3d
from scipy.optimize import curve_fit

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error





class grid_interpolation():
    
    def __init__(self, grid_file_name, is_little_finger):
        
        self.is_little_finger = is_little_finger
        self.grid_from_file(grid_file_name)
    
    def grid_from_file(self, file_name):
        
        values = []
        with open(file_name, 'r') as file:
            # Read each line in the file
            for line in file:
                a  = String_Handler.array_from_sting(line)
                values.append(a)

        x_i = 1
        y_i = 1
        z_i = 0        
        
        x_start = values[0][0]
        y_start = values[0][1]
        z_start = values[0][2]
        i  = 0   
        
        
        if self.is_little_finger:
            z_i = 1
            t_i = 0
            t_start = values[0][3]
            self.min_limits = [x_start, y_start, z_start,t_start]
        else: 
            self.min_limits = [x_start, y_start, z_start]
        
        print('min', self.min_limits)
        
        
        
        self.stride = [0  for i in range(len(self.min_limits))]
        
        #print(values[0])
        
        for i in range(len(values)):
            x = values[i][0]
            if x == values[0][0]:
                #print(x)
                y = values[i][1]
                if y == values[0][1]:
                    z = values[i][2]
                    if self.is_little_finger:
                        if z == values[0][2]:
                            t =  values[i][3]
                            self.stride[3] =  t- t_start
                            t_start = t
                            t_i += 1 
                        
                        if z != z_start:
                            self.stride[2] =  z- z_start
                            z_start = z
                            z_i += 1 
                    
                    else: 
                        self.stride[2] =  z- z_start
                        z_start = z
                        z_i += 1 
                
                if y != y_start:
                    #print(y)
                    self.stride[1] = y - y_start
                    y_start = y
                    y_i += 1
            
            
            if x != x_start: 
                #print('x') #_start)
                
                self.stride[0] = x - x_start
                x_start = x 
                x_i +=1
        
        if self.is_little_finger:
            self.max_limits = [x_start, y_start, z_start, t_start]
        else: 
            self.max_limits = [x_start, y_start, z_start]
        
        
        #print(self.max_limits)
        #print('finger is littel finger', self.is_little_finger)
        #print(x_i, y_i, z_i, t_i)
        #print(x_i*y_i*z_i*t_i)
        
        #print(len(values))
        self.len_grid = [x_i, y_i, z_i]
        if self.is_little_finger: 
            self.len_grid.append(t_i)
    
        self.grid = []
        c = 0
        for i in range(x_i):
            self.grid.append([])
            for j in range(y_i):
                self.grid[i].append([])
                for p in range(z_i): 
                    if self.is_little_finger: 
                         self.grid[i][j].append([])
                         for l in range(t_i): 
                            
                            #print('val', values[c][-1])
                            #print(self.grid)
                            #print(i, j, p)
                            #print(c, x_i*y_i*z_i*t_i)
                            #print('grid',self.grid[i][j][p])
                            self.grid[i][j][p].append(values[c][-1])
                            c += 1
                    else: 
                        self.grid[i][j].append(values[c][-1])
                       # if i == 20 and j == 90 and p == 19: 
                        #    print('value at index 20, 90, 19', c, values[c])
                        #    print(self.grid[i][j][p])
                        c += 1
        
        #print(self.grid)
        
        
        #print(self.min_limits, self.max_limits, self.stride)
        
        
        return
        
    
    
    def estimate_value(self, point):
        indexes = []
        #print(point)
        #print('test_values')
        #print(self.grid[0][0][0][0])
        #print(len(self.grid[0][0][0]))
        #print(len(self.grid[0][0]))
        #print(len(self.grid[0]))
        #print(len(self.grid))
        #idx_test = np.floor((3.06 - self.min_limits[-1])/ self.stride[-1])
        #print('idx', idx_test)
        
        
        
        for i in range(len(point)):
            '''
            index = (point[i] - self.min_limits[i])/ self.stride[i]
            
            
            floor_index = np.floor(index)
            ceil = np.ceil(index)
            if ceil- index < 10**(-6):
                start_index = ceil
            else: 
                start_index = floor_index
            '''
            start_index = np.floor((point[i] - self.min_limits[i])/ self.stride[i])
            idx = np.int16(start_index)
            #print('idx: ', start_index, (point[i] - self.min_limits[i])/ self.stride[i])
            #print(point[i] - self.min_limits[i])
            #print(self.len_grid)
            #print(point[i]- -0.262)
            #print(self.min_limits[i]- -0.262)
            if idx >= self.len_grid[i]-1:
                idx -= 1
            if idx < 0: 
                idx = 0
            indexes.append(idx)
        #print('point', point)
        #print('idx', indexes)
        #print(self.grid[9][8][0][8])
        #print(self.grid[9][8][0][8])
        #print(self.grid[indexes[0]][indexes[1]][indexes[2]])
        if not self.is_little_finger: 
            res = self.trilinear_interpolation(point, indexes)
        else: 
            res = self.quadro_interpolation(point, indexes)
        
        
        return res


    
    
    def quadro_interpolation(self, point, indexes):
        #https://en.wikipedia.org/wiki/Trilinear_interpolation
        
        
        x_0 = self.min_limits[0] +  self.stride[0]*indexes[0]
        x_d = (point[0] - x_0)/ self.stride[0]
        
        
        y_0 = self.min_limits[1] +  self.stride[1]*indexes[1]
        y_d = (point[1] - y_0)/ self.stride[1]
        
        z_0 = self.min_limits[2] +  self.stride[2]*indexes[2]
        z_d = (point[2] - z_0)/ self.stride[2]
        
        t_0 = self.min_limits[3] +  self.stride[3]*indexes[3]
        t_d = (point[3] - t_0)/ self.stride[3]
        
        c000 = self.grid[indexes[0]][indexes[1]][indexes[2]][indexes[3]]*(1-x_d) +      self.grid[indexes[0]+1][indexes[1]][indexes[2]][indexes[3]]*x_d
        c001 = self.grid[indexes[0]][indexes[1]][indexes[2]][indexes[3]+1]*(1-x_d) +    self.grid[indexes[0]+1][indexes[1]][indexes[2]][indexes[3]+1]*x_d
        c010 = self.grid[indexes[0]][indexes[1]][indexes[2]+1][indexes[3]]*(1-x_d) +    self.grid[indexes[0]+1][indexes[1]][indexes[2]+1][indexes[3]]*x_d
        c011 = self.grid[indexes[0]][indexes[1]][indexes[2]+1][indexes[3]+1]*(1-x_d) +  self.grid[indexes[0]+1][indexes[1]][indexes[2]+1][indexes[3]+1]*x_d
        c100 = self.grid[indexes[0]][indexes[1]+1][indexes[2]][indexes[3]]*(1-x_d) +    self.grid[indexes[0]+1][indexes[1]+1][indexes[2]][indexes[3]]*x_d
        c101 = self.grid[indexes[0]][indexes[1]+1][indexes[2]][indexes[3]+1]*(1-x_d) +  self.grid[indexes[0]+1][indexes[1]+1][indexes[2]][indexes[3]+1]*x_d
        c110 = self.grid[indexes[0]][indexes[1]+1][indexes[2]+1][indexes[3]]*(1-x_d) +  self.grid[indexes[0]+1][indexes[1]+1][indexes[2]+1][indexes[3]]*x_d
        c111 = self.grid[indexes[0]][indexes[1]+1][indexes[2]+1][indexes[3]+1]*(1-x_d)+ self.grid[indexes[0]+1][indexes[1]+1][indexes[2]+1][indexes[3]+1]*x_d
        
        c00 = c000*(1-y_d) + c100*y_d
        c01 = c001*(1-y_d) + c101*y_d
        c10 = c010*(1-y_d) + c110*y_d
        c11 = c011*(1-y_d) + c111*y_d

        c0 = c00*(1-z_d) + c10*z_d
        c1 = c01*(1-z_d) + c11*z_d

        c = c0 *(1-t_d) + c1*t_d
        
        return c

    
    

    def trilinear_interpolation(self, point, indexes):
        #https://en.wikipedia.org/wiki/Trilinear_interpolation
        
        
        x_0 = self.min_limits[0] +  self.stride[0]*indexes[0]
        x_d = (point[0] - x_0)/ self.stride[0]
        
        
        y_0 = self.min_limits[1] +  self.stride[1]*indexes[1]
        y_d = (point[1] - y_0)/ self.stride[1]
        
        z_0 = self.min_limits[2] +  self.stride[2]*indexes[2]
        z_d = (point[2] - z_0)/ self.stride[2]
        
        
        #print('x_d', x_d)
        #print(x_0, point[0])
        
        
        c00 = self.grid[indexes[0]][indexes[1]][indexes[2]]*(1-x_d) + self.grid[indexes[0]+1][indexes[1]][indexes[2]]*x_d
        c01 = self.grid[indexes[0]][indexes[1]][indexes[2]+1]*(1-x_d) + self.grid[indexes[0]+1][indexes[1]][indexes[2]+1]*x_d
        c10 = self.grid[indexes[0]][indexes[1]+1][indexes[2]]*(1-x_d) + self.grid[indexes[0]+1][indexes[1]+1][indexes[2]]*x_d
        c11 = self.grid[indexes[0]][indexes[1]+1][indexes[2]+1]*(1-x_d) + self.grid[indexes[0]+1][indexes[1]+1][indexes[2]+1]*x_d

        c0 = c00*(1-y_d) + c10*y_d
        c1 = c01*(1-y_d) + c11*y_d

        c = c0 *(1-z_d) + c1*z_d

        return c


def increas_func(xyz, a, b,c, d, e, f, g, h, i, j, k, l):
    inp, angle1, angle2 = xyz
    return a*inp+ b*inp**2 + c*inp**3 + d*inp**4 +e*angle1 + f*angle1**2 + g*angle1**3 + h*angle1**4 + i*angle2 + j*angle2**2 + k*angle2**3 + l*angle2**4 



def decreas_func(xyz, a, b,c, d, e, f, g):
    input, angle1, angle2 = xyz
    return a*input + b*input**2 + c*angle1 + d*angle2 + e*angle2**2 + f*angle2**3 + g*angle2**4



def plot_max_angles(inp, res): 
    
    start_value = inp[0][-3]
    
    i = 0
    input = []
    output = []
    while i < len(inp): 
        if inp[i][-3] != start_value: 
            start_value = inp[i][-3]
            plt.plot(input, output)
            input = []
            output = []
        input.append(inp[i][-2])
        output.append(res[i][-2])
        i += 1
    plt.show()


def add_to_plot(inp, res, ax, _color):
    
    test_values = [1.40, 1.60, 1.8, 2.0 , 2.2,] # 2.4, 2.6, 2.8, 3]
    
    colors = ['r', 'b', 'g', 'black', 'yellow', 'aqua', 'pink', 'darkgreen', 'purple', 'darkred', 'darkorange']
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    
    for r in range(len(test_values)):
        j = test_values[r]
        max_joint = []
        angle1 = []
        angle2 = []
        i = 0
        
        #max_joint.append(res[i][-2])#res[i][-1] + res[i][-2]
        #angle1.append(test_values[r])
        #angle1.append(res[i][0])
        #angle2.append(res[i][1])
        
            
        for i in range(1,len(inp)): 
            if abs(inp[i][-2]-j) < 0.01:
                if  inp[i-1][-2] >= inp[i][-2]: 
                    max_joint.append(res[i][-2])#res[i][-1] + res[i][-2]
                    #angle1.append(test_values[r])
                    angle1.append(res[i][0])
                    angle2.append(res[i][1])
                
        # Plot a basic wireframe.
        #ax.plot_wireframe(angle1, angle2, max_joint)

        ax.scatter(angle1, angle2, max_joint, color = colors[r])








def get_values(inp, res, return_increase = True):
    test_values = [1.40, 1.60, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3]
    
    
    max_joint_i = []
    angle1_i = []
    angle2_i =[]
    input_i = []
        
    max_joint_d = []
    angle1_d = []
    angle2_d =[]
    input_d= []
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    colors = ['r', 'b', 'g', 'black', 'yellow', 'aqua', 'pink', 'darkgreen', 'purple', 'darkred', 'darkorange']

    for r in range(len(test_values)):
        j = test_values[r]
        
        increase = True
        for i in range(len(inp)): 
            if abs(inp[i][-2]-j) < 0.01 and inp[i][1] > -0.25 and inp[i][1] < 1.57 : 
                #if inp[i][0] <0.02 and inp[i][0] > -0.02: 
               
                        #angle1.append(res[i][0])
                    
                    if i != 0:
                        #print('ind:', inp[i][0], inp[i][-2], inp[i-1][-2])
                        #print('res', res[i])
                        if inp[i][-2] > inp[i-1][-2]:
                            #print(True)
                            increase = True
                        else: 
                            increase = False
                    if increase: 
                        max_joint_i.append(res[i][-2])#
                        angle1_i.append(res[i][0])
                        angle2_i.append(res[i][1])
                        input_i.append(inp[i][-2])
                    else: 
                        max_joint_d.append(res[i][-2])#
                        angle1_d.append(res[i][0])
                        angle2_d.append(res[i][1])
                        input_d.append(inp[i][-2])
                

    ax.scatter(angle1_i, angle2_i, max_joint_i, color = colors[r])
    #ax.scatter(angle1_d, angle2_d, max_joint_d, color = colors[r])

    ax.set_xlabel('angel1')
    #ax.set_xlabel('input')
    ax.set_ylabel('angel2')
    ax.set_zlabel('max_joint')
    plt.show()
    if return_increase: 
        return input_i, angle1_i, angle2_i, max_joint_i
    else: 
        return input_d, angle1_d, angle2_d, max_joint_d
    




def show_relation_angle1_max(inp, res):
    
    test_values = [1.50, 1.65, 1.8, 1.95, 2.1, 2.25, 2.4, 2.55, 2.7, 2.85, 3]
    test_values = [1.40, 1.60, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3]
    
    test_values = [1.8, 2.4, 3]
    test_values = [1.6, 2.2, 3]
    angle1 = [-0.3 , 0.3]

    
    colors = ['r', 'b', 'g', 'black', 'yellow', 'aqua', 'pink', 'darkgreen', 'purple', 'darkred', 'darkorange']
    #fig = plt.figure()
    #ax = fig.add_subplot(projection='3d')
    

    for r in range(len(test_values)):
        j = test_values[r]
        
        max_joint_i = []
        max_joint_d = []
            #angle1 = []
        increase_val = []
        decrese =[]
        increase = True
        for i in range(len(inp)): 
            if abs(inp[i][-2]-j) < 0.01: 
                if inp[i][0] <0.02 and inp[i][0] > -0.2: 
                    res[i][-1] + res[i][-2]
                        #angle1.append(res[i][0])
                    print('ind:', inp[i][-1], inp[i][-2])
                    print('res', res[i])
                    if i != 0:
                        if inp[i][-2] > inp[i-1][-2]:
                            increase = True
                        else: 
                            increase = False
                    if increase: 
                        max_joint_i.append(res[i][-2])#
                        increase_val.append(res[i][1])
                    else: 
                        max_joint_d.append(res[i][-2])#
                        decrese.append(res[i][1])
                
        
        plt.plot(increase_val, max_joint_i, 'o', color = colors[r])
        plt.plot(decrese, max_joint_d, 'o', color = colors[len(test_values)+r])
        # Plot a basic wireframe.
        #ax.plot_wireframe(angle1, angle2, max_joint)

    plt.show()
    
    


def show_relation_3D(inp, res, old_4_fingers = False):
    
    test_values = [1.50, 1.65, 1.8, 1.95, 2.1, 2.25, 2.4, 2.55, 2.7, 2.85, 3]
    #test_values = [1.40, 0.4, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3]
    
    test_values = [1.40, 1.6, 1.8, 2., 2.2, 2.4, 2.6]
    test_values = [1.30, 1.5, 1.7, 1.9, 2.1, 2.3, 2.5, 2.7]
    
    
    test_values = [1.30, 1.54, 1.7, 1.94, 2.1, 2.34, 2.5, 2.74]
    
    if old_4_fingers: 
        test_values = [1.45, 1.55, 1.7, 1.95, 2.1, 2.5]
    
    
    #test_values = [1.8, 2.4, 3]
    #test_values = [1.6]
    

    
    colors = ['r', 'b', 'g', 'black', 'yellow', 'aqua', 'pink', 'darkgreen', 'purple', 'darkred', 'darkorange']
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    
    for r in range(len(test_values)):
        j = test_values[r]
        max_joint = []
        angle1 = []
        angle2 = []
        i = 0
        
        #max_joint.append(res[i][-2])#res[i][-1] + res[i][-2]
        #angle1.append(test_values[r])
        #angle1.append(res[i][0])
        #angle2.append(res[i][1])
        
            
        for i in range(1,len(inp)): 
            if abs(inp[i][-2]-j) < 0.01:
                if  inp[i-1][-2] >= inp[i][-2]: 
                    max_joint.append(res[i][-2])#res[i][-1] + res[i][-2]
                    #angle1.append(test_values[r])
                    angle1.append(res[i][0])
                    angle2.append(res[i][2])
                    
                
        # Plot a basic wireframe.
        #ax.plot_wireframe(angle1, angle2, max_joint)

        ax.scatter(angle1, angle2, max_joint, color = colors[r])
    
    ax.set_xlabel('angel1')
    #ax.set_xlabel('input')
    ax.set_ylabel('angel2')
    ax.set_zlabel('max_joint')
    #plt.show()
    
    
        
    



def show_max_relations_one_joint(inp, res, joint_of_intrest = 0):
    
    
    
    test_values = [1.80, 2.4, 3.0]
    
    for j in test_values:
        max_joint = []
        angle = []
        for i in range(len(inp)): 
            if abs(inp[i][-2]-j):
                max_joint.append(res[i][-2])
                angle.append(res[i][joint_of_intrest])
                
                
        plt.plot(angle, max_joint, 'go--', linewidth=2, markersize=12)

    plt.show()
    
    




def clean(inp, res):
    clean_inp = []
    clean_res = []
    inp_val = inp[0][-2]
    inp_val1 = inp[0][-3]
    print('cleaning --')
    for i in range(len(inp)):
        #print(inp[i][-2])
        #print(i, '/', len(inp)-1)
        if abs(inp_val- inp[i][-2]) > 0.005 or abs(inp_val1- inp[i][-3]): 
            #print('inside', inp_val)
            clean_inp.append(inp[i-1])
            clean_res.append(res[i])
            inp_val = inp[i][-2]
            inp_val1 = inp[i][-3]
            
    clean_inp.append(inp[-1])
    clean_res.append(res[-1])
            
    return clean_inp, clean_res




def read_file(index, file_name):
    
    
    inp = []
    res = []
    with open(file_name, 'r') as file:
        # Read each line in the file
        for line in file:
            # Print each line
            l = line.strip()
            
            
            values = l.split(':')
            a  = String_Handler.array_from_sting(values[1])
            if values[0] == 's': # send
                inp.append(a[index])
            else: #'res'
                res.append(a[index])
    return inp, res




def show_function(inp, angle1, angle2, max_joint, values):
    
    test = inp[0] 
    
    inpu = []
    ang1 = []
    ang2 =  []
    max_j = []
    res = []
    for i in range(len(inp)):
        if inp[i] == test: 
            
            inpu.append(inp[i])
            ang1.append(angle1[i])
            ang2.append(angle2[i])
            max_j.append(max_joint[i])
            xyz = [inp[i], angle1[i], angle2[i]]
            res.append(increas_func(xyz, values[0], values[1], values[2], values[3], values[4], values[5], values[6],values[7], values[8], values[9], values[10], values[11]))
     
     
    colors = ['r', 'b', 'g', 'black', 'yellow', 'aqua', 'pink', 'darkgreen', 'purple', 'darkred', 'darkorange']
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
            
    ax.scatter(ang1, ang2, max_j, color = colors[0])
    ax.scatter(ang1, ang2, res, color = colors[1])

    ax.set_xlabel('angel1')
    #ax.set_xlabel('input')
    ax.set_ylabel('angel2')
    ax.set_zlabel('max_joint')
    plt.show()


def create_file_inp_result_data(inp, res, file_name, middel_finger = True):
    f = open(file_name, 'w')
    
    inp_res = []
    for i in range(len(inp)):
        #print(res[i])
        values = [inp[i][0], inp[i][1], inp[i][2], res[i][-2]]
        if not middel_finger: 
            values = [ inp[i][0], inp[i][1], inp[i][2], inp[i][3], res[i][-2]]
        s = String_Handler.array2string(values)
        f.write(s)
        f.write('\n')
        #print(s)
    f.close()

def read_inp_result_data(file_name):
    data = []
    with open(file_name, 'r') as file:
        # Read each line in the file
        for line in file:
            # Print each line
            a  = String_Handler.array_from_sting(line)
            data.append(a)
    return data
    

def find_surounding_data(data, point, middel_finger):
    if middel_finger: 
        indicator = [[0,0,0],[0,0,1], [0,1,0],[0,1,1],[1,0,0],[1,0,1], [1,1,0],[1,1,1]]
        numb_indicators = 3
    else: 
        indicator = [[0,0,0,0],[0,0,0,1], [0,0,1,0],[0,0,1,1],[0,1,0,0],[0,1,0,1], [0,1,1,0],[0,1,1,1],[1,0,0,0],[1,0,0,1], [1,0,1,0],[1,0,1,1],[1,1,0,0],[1,1,0,1], [1,1,1,0],[1,1,1,1]]
        numb_indicators = 4
    
    current_points = []
    for i in range(len( indicator )):
        cur_point = []
        for j in range(numb_indicators):
            if  indicator[i][j] == 0: 
                cur_point.append(-100)
            else:
                cur_point.append(100)
        current_points.append(cur_point)
    print(current_points)        
                
                
    for i in range(len(data)):
        for j in range(len(indicator)):
            is_candidate = True
            for r in range(numb_indicators):
                if indicator[j][r] == 0 and data[i][r] > point[r]:
                    is_candidate = False
                elif indicator[j][r] == 1 and data[i][r] < point[r]:
                    is_candidate = False
            if is_candidate:
                #print(j)
                value1 = np.linalg.norm([current_points[j][p]- point[p] for p in range(numb_indicators)])
                value2 = np.linalg.norm([data[i][p] - point[p] for p in range(numb_indicators)])
                #print(value1, value2)
                if value2 < value1:
                    current_points[j] = data[i]
                break
    #print('current_points', current_points)
    
    for c in current_points: 
        #print(c)
        for j in range(numb_indicators):
            #print(abs(c[j]))
            
            if abs(c[j]) > 50:
                #print(point, current_points) 
                return False
    
    return current_points
    
    
    
    
    
    #return


def estimate_value(point, sorounding, is_middle_finger):
    
    # https://en.wikipedia.org/wiki/Trilinear_interpolation larst interpolation
    
    if is_middle_finger: 
        indicator = 8
    else:
        indicator = 16
    
    values = np.empty((indicator,indicator))
    res = np.empty((indicator,1))
    
    
    for i in range(indicator):
        res[i] = sorounding[i][-1]
        
        x = sorounding[i][0]
        y = sorounding[i][1]
        z = sorounding[i][2]
        
        values[i][0] = 1
        values[i][1] = x
        values[i][2] = y
        values[i][3] = z
        values[i][4] = x*y
        values[i][5] = x*z
        values[i][6] = y*z
        values[i][7] = x*y*z
        
        if not is_middle_finger: 
            t  = sorounding[i][3]
            
            values[i][8] = t
            values[i][9] = t*x
            values[i][10] = t*y
            values[i][11] = t*z
            values[i][12] = t*x*y
            values[i][13] = t*x*z
            values[i][14] = t*y*z
            values[i][15] = t*x*y*z
    
    
    pinv = np.linalg.pinv(values)
    
    coef = np.matmul(pinv, res)
    
    
    x_p = point[0]
    y_p = point[1]
    z_p = point[2]
    
    value = coef[0] + coef[1]*x_p+ coef[2]*y_p+ coef[3]*z_p+ coef[4]*x_p*y_p+ coef[5]*x_p*z_p+ coef[6]*y_p*z_p +  coef[7]*x_p*y_p*z_p
    
    if not is_middle_finger: 
        t_p = point[3]
        value += coef[8]*t_p + coef[9]*t_p*x_p+ coef[10]*t_p*y_p+ coef[11]*t_p*z_p+ coef[12]*t_p*x_p*y_p+ coef[13]*t_p*x_p*z_p+ coef[14]*t_p*y_p*z_p +  coef[15]*t_p*x_p*y_p*z_p
    
    return value[0]

def find_data_point(point, data, stride):
    dif = stride/2.
    #print('point', point)
    
    for d in data: 


        is_point = True
        val = 0
        for j in range(len(point)):
            val += np.abs(point[j]-d[j])
            if point[j] > d[j]+ dif or point[j] < d[j]- dif: 
                is_point = False
        #if val < 0.01:
        #    print('point', point)
        #    print(d)
        #    print(is_point)
        if is_point: 
            return d
        
    return [100]



    
    
# Array A[] has the items to sort; array B[] is a work array.
def TopDownMergeSort(A):
    
    B = A.copy()
    n = len(A)

    CopyArray(A, 0, n, B)           # one time copy of A[] to B[]
    TopDownSplitMerge(A, 0, n, B)   # sort data from B[] into A[]
    return A

# Split A[] into 2 runs, sort both runs into B[], merge both runs from B[] to A[]
# iBegin is inclusive; iEnd is exclusive (A[iEnd] is not in the set).
def TopDownSplitMerge(B, iBegin, iEnd, A):

    if (iEnd - iBegin <= 1):                     # if run size == 1
        return                                  #   consider it sorted
    # split the run longer than 1 item into halves
    iMiddle = int(np.ceil((iEnd + iBegin) / 2.))              # iMiddle = mid point
    
    #print(type(iMiddle))
    print(iMiddle)
    # recursively sort both runs from array A[] into B[]
    TopDownSplitMerge(A, iBegin,  iMiddle, B)  # sort the left  run
    TopDownSplitMerge(A, iMiddle,    iEnd, B)  # sort the right run
    
    print(iMiddle)
    print(iBegin, iEnd)
    # merge the resulting runs from array B[] into A[]
    TopDownMerge(B, iBegin, iMiddle, iEnd, A)

#  Left source half is A[ iBegin:iMiddle-1].
# Right source half is A[iMiddle:iEnd-1   ].
# Result is            B[ iBegin:iEnd-1   ].
def TopDownMerge(B, iBegin, iMiddle, iEnd, A):

    i = iBegin
    j = iMiddle
 
    # While there are elements in the left or right runs...
    for k in range( iBegin, iEnd):
        # If left run head exists and is <= existing right run head.

        if (i < iMiddle and (j >= iEnd or is_Ai_smaller_than_Aj(A[i], A[j]))): 
            B[k] = A[i]
            i = i + 1
        else: 
            B[k] = A[j]
            j = j + 1
        

def is_Ai_smaller_than_Aj(A_i, A_j):
    is_smaller = True
    diff = 0.005
    for t in range(len(A_i)):
        if A_i[t] > A_j[t] + diff:
            return False
        elif A_i[t] < A_j[t] - diff:
            return True
    return True
        
    
    
    

def CopyArray(A, iBegin, iEnd, B):

    for k in range(iBegin, iEnd):
        B[k] = A[k]

    
    
    
    
    
    
    
    
    
    
    
    
    
    



def sort_data(data, file_name):
    
    data_sorted = TopDownMergeSort(data)
    
    '''
    data_sorted = data.copy()
    l = len(data_sorted)
    r = len(data_sorted[0])-1
    
    diff = np.abs(data_sorted[0][0]-data_sorted[1][0])/2
    
    
    
    
    for i in range(l):
        swap_idx = i
        min_data = data_sorted[i]
        if i % 100 == 0: 
            print(i)
        for j in range(i+1, l):
           
            compare_data = data_sorted[j]
       
            compare_is_min = True
            #print(min_data, compare_data)
            
            for t in range(r):
                if compare_data[t] > min_data[t] + diff:
                    compare_is_min = False
                    break
                elif compare_data[t] < min_data[t] - diff:
                    compare_is_min = True
                    break
           
            if compare_is_min: 
                swap_idx = j
                min_data = compare_data
        if swap_idx != i: 
            temp = data_sorted[i]
            data_sorted[i] = data_sorted[swap_idx]
            data_sorted[swap_idx] = temp
    
    

    #remove dublicates:
    resulting_data = [data_sorted[0]]

    data_to_consider = resulting_data[0]
    for i in range(l): 
        is_identical = True 
        compare_data = data_sorted[i]
        for j in range(r):
            if compare_data[t] > data_to_consider[t] + diff or compare_data[t] < data_to_consider[t] - diff:
                is_identical = False
                break
        if not is_identical:
            resulting_data.append(compare_data)
            data_to_consider = compare_data 
    '''    
            
            
    
    
    
    f = open(file_name, 'w')
    
    for i in range(len(data_sorted)):
        data_point = data_sorted[i]
        s = String_Handler.array2string(data_point)
        f.write(s)
        f.write('\n') 
        
        
    f.close()
    
      
            
        
def create_grid_from_sorted_data(sorte_data_file, grid_file_name, is_little_finger):
    
    data = []
    with open(sorte_data_file, 'r') as file:
        # Read each line in the file
        for line in file:
            a  = String_Handler.array_from_sting(line)
            data.append(a)
    
    print('has been reading')
    n_val = len(data[0])-1
    
    
    min_limits = [data[0][i] for i in range(  n_val )]
    max_limits = [0 for i in range(n_val)]
    #print( min_limits)
    
    #diff = stride/2
    #min_limits[0] += stride
    
    diff = 0.005
    
    for i in range(len(data)):
        for j in range(n_val):
            if data[i][j]> max_limits[j]: 
                max_limits[j] = data[i][j]
    
    print('min, max', min_limits, max_limits)
    
    stride = np.abs(data[0][-2]- data[1][-2])
    diff = stride/2
    min_limits = [min_limits[i]  for i in range(len(min_limits))]
    max_limits = [max_limits[i] for i in range(len(max_limits))]
    #max_limits[0]-=stride
    if is_little_finger: # mangler et resultat... de aller sidste  [0.7200000000000006, -0.349, -0.262, 1.3]
        min_limits[-1] += stride
    #
    #min_limits[0] += stride
    #min_limits[2] += stride
    
    #print(max_limits)
    
    #stride *= 2

    

    val = min_limits.copy()
    idx_est = 0
    data_point = data[0]
    f = open( grid_file_name, 'w')
    while val[0] <= max_limits[0]+diff:  
        while val[1] <= max_limits[1]+diff:
            while val[2] <= max_limits[2]+diff:
                if is_little_finger:
                    while val[3] <= max_limits[3]+diff:
                        
                        exists = False
                        for i in range(idx_est, len(data)):
                            res = [data[i][:-1][j]-val[j] for j in range(len(val))]
                            if np.linalg.norm(res)< diff: 
                                exists = True
                                idx_est = i
                                #print(i, val, data[i])
                                break
                        if not exists: 
                            print('can not find', idx_est, val)
                        
                        s = String_Handler.array2string(data[idx_est])
                        f.write(s)
                        f.write('\n')
                        
                        
                        val[3] += stride
                        
                        
                        
                    val[3] = min_limits[3]
                else: 
                
                    exists = False
                    for i in range(idx_est, len(data)):
                        res = [data[i][:3][j]-val[j] for j in range(len(val))]
                        if np.linalg.norm(res)< diff: 
                            exists = True
                            idx_est = i
                            #print(i, val, data[i])
                            break
                    if not exists: 
                        print('can not find', idx_est, val)
                    
                    s = String_Handler.array2string(data[idx_est])
                    f.write(s)
                    f.write('\n')
                
                
                val[2] += stride
            val[2] = min_limits[2]
            val[1] +=stride
        val[1] = min_limits[1]
        val[0] += stride
    print('all exists')
    f.close()
    
    return
    
    
    
    



def sort_data_old(data, finger, file_name, is_middle_finger): 
    
    
    start_data = data[0]
    stride = np.abs(data[0][0]- data[1][0])
    min_limits = [start_data[i]+ stride for i in range(len(start_data)-1)]
    max_limits = [0 for i in range(len(start_data)-1) ]
    #print( min_limits)
    
    #diff = stride/2
    #min_limits[0] += stride
    
    diff = 0
    
    for i in range(len(data)):
        for j in range(len(data[i])-1):
            if data[i][j]> max_limits[j]: 
                max_limits[j] = data[i][j]
    
    
    val = min_limits.copy()
    
    f = open(file_name, 'w')
    
    while val[0] <= max_limits[0]+diff:
        while val[1] <= max_limits[1]+diff:
            while val[2] <= max_limits[2]+diff:
                if not is_middle_finger: 
                    while val[3] <= max_limits[3]+diff:
                        point = val[:]
                        data_point =  find_data_point(point, data, stride)
                        if data_point[0] > 50: 
                            
                            print('warning not found', val)
                        s = String_Handler.array2string(data_point)
                        f.write(s)
                        f.write('\n') 
                        
                        val[3] += stride    
                        
                    val[3] = min_limits[3]    

                    
                else: 
                    point = val[:]
                    data_point =  find_data_point(point, data, stride)
                    if data_point[0] > 50: 
                        print('warning not found', val)
                    s = String_Handler.array2string(data_point)
                    f.write(s)
                    f.write('\n') 
                                                
                    
                
                    
                    
                    
                print(val)
                val[2] +=stride 
            val[2] = min_limits[2]
            val[1] += stride
        val[1] = min_limits[1]
        val[0] += stride
    
    
    
    return




def create_data_from_res_fingers(data, finger, file_name, middel_finger = True):
    
    f = open(file_name, 'w')
    
    
    angle_0 = [finger.min_limits[0], finger.max_limits[0]]
    angle_1 = [finger.min_limits[1], finger.max_limits[1]]
    angle_2 = [finger.min_limits[2], finger.max_limits[2]]
    input_angle = [1.4,  3]
    
    
    
    
    #input_angle = [1.49, 1.71]
    
    print(angle_0, angle_1, input_angle)
    
    x_start = 0.05
    y_start = 0.05
    z_start = 0.05
    t_start = 0.05
    
    val = 0.2
    
    x_diff = val#0.04
    y_diff = val #0.04
    z_diff = val #0.04
    t_diff = val #0.04

    
    x = input_angle[0] #+ x_start
    y = angle_0[0] #+ y_start
    z = angle_1[0] #+ z_start
    t = angle_2[0] #+ t_start
    
    
    resulting_matrix = []
    
    while x <= input_angle[1] - x_start:
        
        while y <= angle_0[1] -y_start:
            while z <= angle_1[1]-  z_start:
                
                if not middel_finger: 
                    while t <= angle_2[1] - t_start:
                        point = [x,y,z,t]
                        sorounding =  find_surounding_data(data, point, middel_finger)
                        if sorounding == False: 
                            print('warning', [x,y,z])
                            s = String_Handler.array2string([x,y,z, t, -100])
                        else: 
                            print(point)
                            print('sorounding', sorounding)
                            value = estimate_value(point, sorounding, middel_finger)
                            s = String_Handler.array2string([x,y,z, t, value])
                        f.write(s)
                        f.write('\n') 
                            
                        
                        
                        
                        
                        
                        t+= t_diff

                    t = angle_2[0] + t_start
                    
                else: 
                    print(x, y, z)
                    point = [x,y,z]
                    
                    #find i input og res
                    
                    sorounding =  find_surounding_data(data, point)
                    if sorounding == False: 
                        print('warning', [x,y,z])
                        s = String_Handler.array2string([x,y,z, -100])
                    else: 
                        print('sorounding', sorounding)
                        value = estimate_value(point, sorounding)
                        s = String_Handler.array2string([x,y,z, value])
                    f.write(s)
                    f.write('\n') 
                                                    
                    
                
                    
                    
                    
            
                z += z_diff
                print(x, y, z)
            z = angle_1[0] + z_start
            y += y_diff
           
        y = angle_0[0] + y_start
        x += x_diff
        
    
    
    f.close()



def clean_grid():
    file_name = 'grid_finger_1_original.txt'
    resfile_name = 'grid_finger_1.txt'
    
    values = []
    with open(file_name, 'r') as file:
        # Read each line in the file
        for line in file:
            a  = String_Handler.array_from_sting(line)
            values.append(a)

    
    f = open(resfile_name, 'w')
    for i in range(len(values)):
        a = values[i]
        if np.abs(a[-1]) < 10:
            s = String_Handler.array2string(a)
            f.write(s)
            f.write('\n') 

    f.close()
        

def calculate_residual(finger_idx, is_middle_finger):
    file_name = 'grid_'+ str(finger_idx) +'.txt'
    file_name1 = 'intermediate_results_'+ str(finger_idx) +'.txt'

    mj_sim = Mujoco_Simulator.Mujoco_Simulator()
    shadow_hand = Shadow_Hand.Shadow_hand(mj_sim)
    
   
    finger = shadow_hand.fingers[finger_idx]
    
    
    data = read_inp_result_data(file_name1)   
    
    g = grid_interpolation(file_name, not is_middle_finger)
    
    #data_point = [0.72, 0.29100000000000004, -0.2620000000000003, 2.019999999999999] #1.5915640666924253
    #[0.72,0.29100000000000004,-0.2620000000000003,2.099999999999999,1.6255476410701994]
   
    #res1 = 1.5915640666924253
    
    #res  = g.estimate_value(data_point)
    
    #print('real', res1)
    #print('res', res)
    
    
    max_residual = 0
    #indx = 0
    #print('data', len(data))
    #print(g.min_limits)
    #print(g.max_limits)
    #return
    for i in range(len(data)):
        
        point = data[i][:-1]
        #point = [data[i][-1], data[i][0], data[i][1]]
        #print(point)
        within_limits = True
        for j in range(len(point)):
            bias = 0.005
            
            if point[j] < g.min_limits[j] - bias or point[j] > g.max_limits[j]+ bias : 
                within_limits = False
        if within_limits: 
            est = g.estimate_value(point)
            
            r = np.abs(data[i][-1]-est)
            #r = np.abs(res[i][-2]-1.57)
            #print(r)
            if r > max_residual: 
                #print('point', point)
                #print(est)
                #print('res', data[i][-1])
                #print(r)
                max_residual = r
                indx = i
    print(g.min_limits)
    print(g.max_limits)
    print('max_residual finger', finger_idx, ':  ', max_residual)
    print('inp max', data[indx])
    print(point)
    print('est', g.estimate_value(point = data[indx][:-1]))

        
        



def main():
    
    #calculate_residual()
    
    
    

    
    mj_sim = Mujoco_Simulator.Mujoco_Simulator()
    shadow_hand = Shadow_Hand.Shadow_hand(mj_sim)
    
    finger_idx = 4
    middel_finger = False
    finger = shadow_hand.fingers[finger_idx]
    
    
    if finger_idx  == 4: 
        file_name0 = 'robot_config_finger_4_results.txt'
    
    if finger_idx  == 1 or finger_idx  == 2: 
        file_name0 = 'res_config_1_2.txt'
        #file_name = 'robot_config_finger_1_2_results.txt'
    else: 
        file_name0 = 'res_config_' + str(finger_idx ) + '.txt'
    
    
    file_name = 'intermediate_results_'+ str(finger_idx) +'.txt'
   
    
    #initial createion of result and input:
    
    #_rinp, _rres = read_file(finger_idx, file_name0)
    #inp, res = clean(_rinp, _rres)
    
    
    '''
   
    show_relation_3D(inp, res)
  
    file_name0 = 'robot_config_finger_1_2_results.txt'
    file_name0 = 'robot_config_finger_4_results.txt'
    _rinp, _rres = read_file(finger_idx, file_name0)
    inp, res = clean(_rinp, _rres)
    
    
    show_relation_3D(inp, res, True)
    
    plt.show()
    return
    
    file_name0 = 'robot_config_finger_1_all_results.txt'
    _rinp, _rres = read_file(finger_idx, file_name0)
    inp, res = clean(_rinp, _rres)
    
    
    show_relation_3D(inp, res)
   
    plt.show()
    return
    '''
    
    
    
    #converte to data input used and stor in file:
    
    #create_file_inp_result_data(inp, res, file_name, middel_finger)
    
    
    
    
    #sort data and create data: 
    
    file_name1 = 'sorted_'+ str(finger_idx) +'.txt'
    #data = read_inp_result_data(file_name)    
    #sort_data(data, file_name1)
    
    file_name_grid = 'grid_'+ str(finger_idx) +'.txt'
    
    create_grid_from_sorted_data(file_name1, file_name_grid, not middel_finger)
    
    
    
    
    #finger_idx = 2
    calculate_residual(finger_idx, middel_finger)
    


    

if __name__ == "__main__":
    main()
    