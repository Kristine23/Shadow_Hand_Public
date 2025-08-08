import String_Handler
import numpy as np



class Grid_Interpolation():
    
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
        
        #print('min', self.min_limits)
        
        
        
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
        #print('min grid', self.min_limits)
        #print('max_grid', self.max_limits)
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
                idx = self.len_grid[i]-2
            if idx < 0: 
                idx = 0
            indexes.append(idx)
        #print('point', point)
        #print(indexes)
        #print(self.grid[20][90][19])
        
        #print(self.grid[indexes[0]][indexes[1]][indexes[2]])
        if not self.is_little_finger: 
            res = self.trilinear_interpolation(point, indexes)
        else: 
            res = self.quadro_interpolation(point, indexes)
        
        
        return res



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
        #print('c3', c)
        return c

    
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
        
        #print('stride', self.stride)    
        #print('values', x_d, y_d,z_d,t_d )
    
            
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
        #print('c4', c)
        return c
