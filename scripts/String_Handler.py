import numpy as np

def array2string(array, number_nested_arrays=0):
    # vector has 0 nested arrays
    # matrix has 1 nested arrays
    
    s = "["
    for i in range(len(array)):
        element = array[i]
        if number_nested_arrays == 0: 
            s+= str(element)
        else: 
            s+= array2string(element, number_nested_arrays - 1)
        if i < len(array)-1:
            s += ","
    
    s += "]"
    
    return s


def multiple_arrays_from_string(string):
    
    elements = string.split('\n')

    
    arrays = []
    for i in range(len(elements)-1):
        arrays.append(array_from_sting(elements[i]))
        
        
    return arrays
    
    
def array_from_sting(string):
    
    
    string.replace(" ", "")
    
    s = string.split("=", 1)
    arrays = s[-1]
    
    a = 0
    while arrays[a] != '[':
        a += 1
        if a > 100: 
            print('array_from_sting not working, somethings wrong!')
            return
    
    n = 0
    while arrays[n+a] == '[':
        n +=1 
    n -=1
    
    
    #print(s[0], 'n', n)
    #print(arrays)
    #print('0', arrays[0], '1', arrays[1], '2', arrays[2])

    if n == 0: 
        v = arrays.split('[')[-1]
        u = v.split(']')[0]
        return vector_from_string(u)
        
    
    res = []
    for i in range(n):
        res.append([]) # arrays outermost to inner most.
    
  
    
    
    depth = n-1
    
    elements = arrays.split("],[")
    
    
    for e in elements: 
        v = e.split('[')[-1]
        if ']' not in e:
            depth = n-1
            vector = vector_from_string(v)
            res[depth].append(vector)
        else: 
            u = v.split(']')
            v = u[0]
            vector = vector_from_string(v)
   
            res[depth].append(vector)
            c = len(u)-1
         
            while depth >= 1 and c > 0:
                res[depth-1].append(res[depth].copy())
                res[depth] = [] 
                c-= 1
                depth -= 1
                
        
    return res[0]
        
    


    

def vector_from_string(string):
    array = []
    values = string.split(',')
    for v in values:
        array.append(float(v))
    return array
    
    

