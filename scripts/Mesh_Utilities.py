from stl import mesh
import numpy as np
import re




def mesh_points_from_obj_file(path2File):
    reComp = re.compile("(?<=^)(v |vn |vt |f )(.*)(?=$)", re.MULTILINE)
    with open(path2File) as f:
        data = [txt.group() for txt in reComp.finditer(f.read())]
        
    v_arr, vn_arr, vt_arr, f_arr = [], [], [], []
    for line in data:
        tokens = line.split(' ')
        if tokens[0] == 'v':
            v_arr.append([float(c) for c in tokens[1:]])
        elif tokens[0] == 'vn':
            vn_arr.append([float(c) for c in tokens[1:]])
        elif tokens[0] == 'vt':
            vt_arr.append([float(c) for c in tokens[1:]])
        elif tokens[0] == 'f':
            f_arr.append([int(c.split('//')[0]) for c in tokens[1:]])
            #f_arr.append([[int(i) if len(i) else 0 for i in c.split('/')] for c in tokens[1:]])
    
    '''
    open('thumb_faces.txt', 'w').close()
    file = open("thumb_faces.txt", "a")
    file.write('Thumb points:\n\n')
    for i in range(len(v_arr)):
        p=v_arr[i]
        if p[1]< 4.5 and p[2] >10:
            if p[1]< -7 and p[2]<11:
                continue
            s = '{'
            for j in range(3):
                e = p[j]
                s += '{:.10f}'.format(e) 
                if j < 2:
                    s += ','
                
            
            s += '}'
            
            if i < len(v_arr)-1:
                s+= ','    
                
            file.write(s)
    
    file.write('\n\n Thumb faces:\n\n')
 
    for j in range(len(f_arr)):
        f = f_arr[j]
        print(f)
        sting = '{'
        face =[]
        for i in range(3):
            p = np.array(v_arr[f[i]-1])
            if p[1]< 4.5 and p[2] >10:
                if p[1]< -7 and p[2]<11:
                    continue
                face.append(p)
                s = '{'
                for j in range(3):
                    e = p[j]
                    s += '{:.10f}'.format(e) 
                    if j < 2:
                        s += ','
                
            
                s += '}'
                
                if i < 2:
                    s+= ','
                    
                sting += s
            
            
        sting += '} '
        
        if j < len(f_arr)-1:
            sting += ', '
        if len(face) == 3:
            file.write(sting)    
            
    file.close()
    '''
    
    '''
    open('mf_faces.txt', 'w').close()
    file = open("mf_faces.txt", "a")
    file.write('MF points:\n\n')
    for i in range(len(v_arr)):
        p=v_arr[i]
        if p[1]< 4:
            if p[0] > -8.5 and p[0]< 8.5 and p[1] > -4 and p[2]<10:
                continue 
            if p[0] > -7.5 and p[0]< 7.5 and p[1] > -6 and p[2]<10:
                continue 
            s = '{'
            for j in range(3):
                e = p[j]
                s += '{:.10f}'.format(e) 
                if j < 2:
                    s += ','
                
            
            s += '}'
            if i < len(v_arr)-1:
                s+= ','    
                
            file.write(s)
    
    file.write('\n\n MF faces:\n\n')
 
    for j in range(len(f_arr)):
        f = f_arr[j]
        print(f)
        sting = '{'
        face =[]
        for i in range(3):
            p = np.array(v_arr[f[i]-1])
            if p[1]< 4:
                if p[0] > -8.5 and p[0]< 8.5 and p[1] > -4 and p[2]<10:
                    continue 
                if p[0] > -7.5 and p[0]< 7.5 and p[1] > -6 and p[2]<10:
                    continue 
                face.append(p)
                s = '{'
                for j in range(3):
                    e = p[j]
                    s += '{:.10f}'.format(e) 
                    if j < 2:
                        s += ','
                
            
                s += '}'
                if i < 2:
                    s+= ','
                
                sting += s
            
            
        sting += '} '
        
        if j < len(f_arr)-1:
            sting += ', '
        if len(face) == 3:
            file.write(sting)    
            
    file.close()
    '''
    faces = []
    for j in range(len(f_arr)):
        f = f_arr[j]
        face =[]
        for i in range(3):
            p = np.array(v_arr[f[i]-1])
            faces.append(p)
        
            
    
    
    
    return v_arr #faces




  
def read_Stl_file(path2File):
    res_mesh = mesh.Mesh.from_file(path2File)
    return res_mesh 
        
def new_mesh_from_vectors(mesh_vectors):        
    l2 = len(mesh_vectors)
    data = np.zeros(l2, mesh.Mesh.dtype)
    for i in range(0, l2):
        data['vectors'][i] = mesh_vectors[i]
    new_mesh = mesh.Mesh(data)
    return new_mesh

def points_in_mesh(_mesh):
    points = []
    shrink = 0
    l = len(_mesh.vectors)
    for i in range(0, l):
        triangle= _mesh.vectors[i]
        for p in triangle:
            point = [p[0]-shrink, p[1]-shrink, p[2]-shrink]
            if point not in points: 
                points.append(point)
    result = np.empty([len(points),3])
    for i in range(0,len(points)): 
            for j in range(0, 3):
                result[i][j] = points[i][j]
    return result

    
    