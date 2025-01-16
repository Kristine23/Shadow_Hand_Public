import numpy as np
import mujoco
import Constants
import PyPlotting

from scipy.spatial.transform import Rotation as Ro
from sympy import Matrix
import numpy as np


#https://danceswithcode.net/engineeringnotes/quaterniorollMatrins/quaternions.html



def skew_symmetric_cross_product(vector):
    matrix = np.zeros((3,3))

    matrix[0][1] = -vector[2]
    matrix[0][2] = vector[1]
    matrix[1][0] = vector[2]
    matrix[1][2] = -vector[0]
    matrix[2][0] = -vector[1]
    matrix[2][1] = vector[0]
    return matrix



def find_shortes_distance_between_vectors(v1_start, v2_start, v1, v2):
    
    #udspringer fra denne  https://math.stackexchange.com/questions/3081301/shortest-distance-between-two-vectors
    # samt geometisk analyse af linjer i 3D.
    
    # shortes distance of lines:
    A = v1_start
    a = v1
    B = v2_start
    b = v2
    
    '''
    mu_lambda_scalar = np.zeros((2,2))
    rigth_side = np.zeros(2)
     
    for i in range(3):
        #first equation:
        mu_lambda_scalar[0][0] += a[i]*b[i]
        mu_lambda_scalar[1][0]-= a[i]**2
        rigth_side[0] -= a[i]*(B[i]-A[i])
        
        #second_equation
        mu_lambda_scalar[0][1] += b[i]**2
        mu_lambda_scalar[1][1] -= b[i]*a[i]
        rigth_side[1] -= b[i]*(B[i]-A[i])
    
    print(mu_lambda_scalar)
    # res = [mu, lambda]
    
    
    [mu_val, lambda_val] = np.matmul(np.linalg.inv(mu_lambda_scalar), rigth_side)
    '''
    
    lambda_val, mu_val = calculate_lambda_and_mu(a, b, A, B)
    
    '''
    print('lambda_val, mu_val', lambda_val, mu_val)
    
    print(mu_val, lambda_val)
    print('points C D')
    C = []
    D = []
    for i in range(3):
        D.append(mu_val*b[i]+B[i])
        C.append(lambda_val*a[i]+ A[i])
    print('C', C)
    print('D', D)
    '''
    
    
    
    
    
    if mu_val < 0 or lambda_val < 0: 
        new_lambda_val = calculate_lambda_for_point(a, A, B)
        new_mu_val = calculate_lambda_for_point(b, B, A) 
        
        if new_mu_val > 1:
            new_mu_val = 1
        if new_lambda_val > 1: 
            new_lambda_val = 1
        
        if mu_val < 0 and lambda_val < 0:
            # så må det tættese de kan være på hinanden være enten A til den ene vector eller B til den anden vecor
            # either both zero or one is negative. Geometisk consideration da linjer går længre fra hinanden
            if new_mu_val <= 0 and new_lambda_val <= 0: 
                return np.linalg.norm(A-B)
            if new_mu_val > 0: 
                 # enshure not longer than vector
                D = np.array([new_mu_val*b[i] + B[i] for i in range(3)])
                return  np.linalg.norm(D-A)
            if new_lambda_val > 0: 
                C = np.array([new_lambda_val*a[i] + A[i] for i in range(3)])
                return np.linalg.norm(B-C)
        elif mu_val < 0: 
            C = np.array([new_lambda_val*a[i] + A[i] for i in range(3)])
            return np.linalg.norm(B-C)
        elif lambda_val < 0: 
            D = np.array([mu_val*b[i] + B[i] for i in range(3)])
            return  np.linalg.norm(D-A)      
    else: 
        # both are positive: 
        
        
        if lambda_val >1 or mu_val > 1:
            #test both endpotins
            C = np.array([a[i] + A[i] for i in range(3)])
            D = np.array([b[i] + B[i] for i in range(3)])
            new_lambda_val = calculate_lambda_for_point(a, A, D)
            new_mu_val = calculate_lambda_for_point(b, B, C)
            if new_lambda_val < 0: 
                new_lambda_val = 0
            if new_mu_val < 0:
                new_mu_val = 0
            
            if lambda_val >1 and mu_val > 1:
                # can only be that new valuse are both 1 or one is smaller. 
                if new_lambda_val >= 1 and new_mu_val >=1:
                    return np.linalg.norm(C-D)
                if new_lambda_val < 1:
                    
                    C = np.array([new_lambda_val*a[i] + A[i] for i in range(3)])
                    return  np.linalg.norm(C-D)  
                if new_mu_val < 1:
                    
                    D = np.array([new_mu_val*b[i] + B[i] for i in range(3)])
                    return  np.linalg.norm(C-D)
            if lambda_val > 1:
                D = np.array([new_mu_val*b[i] + B[i] for i in range(3)])
                return  np.linalg.norm(C-D)
            if mu_val > 1:
                
                C = np.array([new_lambda_val*a[i] + A[i] for i in range(3)])
                return  np.linalg.norm(C-D)  
        else:            
            D = np.array([mu_val*b[i] + B[i] for i in range(3)])
            C = np.array([lambda_val*a[i]+ A[i] for i in range(3)])
            return  np.linalg.norm(C-D) 
    
    
def calculate_lambda_for_point(a, A, B):
    
    lambda_val = sum([a[i]*(B[i]-A[i]) for i in range(3)]) / sum([a[i]**2 for i in range(3)])
    
    return lambda_val


def calculate_lambda_and_mu(a, b, A, B):
    left = np.zeros((2,2))
    rigth = np.zeros(2)
    
    for i in range(len(a)):
        left[0][0] += a[i]*b[i]
        left[0][1] -= a[i]**2
        left[1][0] += b[i]**2
        left[1][1] -= a[i]*b[i]
    
        rigth[0] += -B[i]*a[i]+A[i]*a[i]
        rigth[1] += -B[i]*b[i]+A[i]*b[i]
        
    mu_lambda = np.matmul(np.linalg.pinv(left), rigth)
    
    return mu_lambda[1] , mu_lambda[0]    

    


def vector_contain_only_positive_values(vector):
    
    for v in vector: 
        if not v > 0: 
            return False
    return True



def vector_does_not_contain_negative_values(vector):
    
    for v in vector: 
        if not v >= 0: 
            return False
    return True


def rotation_to_quat(rotation_matrix):
    r = Ro.from_matrix(rotation_matrix)
    return r.as_quat()

def angle_axis_to_quat(axis, theta):
    r = Ro.from_rotvec(theta * axis)
    return r.as_quat()
    
    
def rotation_to_mujoco_quat(rotation):
    axis, theta = axis_and_angle_from_rotation(rotation)
    #r = R.from_matrix(rotation).as_rotvec()
    #theta = np.linalg.norm(r)
    #axis = r/theta
    quat = np.zeros(4)
    quat[0] = np.cos(theta/2)
    quat[1:] = np.sin(theta/2)*axis
    return quat
    
    
    
 
def quat_to_transformation_matrix(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    
    T = np.array([[1-2*(q2**2)-2*(q3**2), 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2, 0],
                       [2*q1*q2+2*q0*q3, 1-2*(q1**2)-2*(q3**2), 2*q2*q3-2*q0*q1 , 0],
                       [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*(q1**2)-2*(q2**2), 0], 
                       [0,0,0,1]])

    return T

def transformation_matrix_for_mujoco(quat,pos):    
    T = quat_to_transformation_matrix(quat)
    T[:3,3]=pos*Constants.WOLD_SCALE
    return T

    

#https://scipp.ucsc.edu/~haber/ph216/rotation_12.pdf


def rotaion_transformation_matrix_from_axises_and_angle(axis, theta):
    T = np.identity(4)
    R = rotation_from_axises_and_angle(axis, theta)
    for i in range(3): 
        for j in range(3): 

            T[i][j] = R[i][j]

    #T[3][3] = 1
    return T
    


def rotation_from_axises_and_angle(axis, theta):
    v = normalize_vector(axis)
    
    n1 = v[0]
    n2 = v[1]
    n3 = v[2]
    #theta = truncate(theta)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    #print(cos_theta)
    #print(sin_theta)
    #print(cos_theta+(n1**2)*(1-cos_theta))
        
    R = np.array([[cos_theta+(n1**2)*(1-cos_theta), n1*n2*(1-cos_theta)-n3*sin_theta, n1*n3*(1-cos_theta)+n2*sin_theta],
                  [n1*n2*(1-cos_theta)+n3*sin_theta, cos_theta+(n2**2)*(1-cos_theta), n2*n3*(1-cos_theta)-n1*sin_theta],
                  [n1*n3*(1-cos_theta)-n2*sin_theta ,n2*n3*(1-cos_theta)+n1*sin_theta , cos_theta+(n3**2)*(1-cos_theta)]])
   
    #print(R)
    #for v in R:
    #    truncate_vector(v,8)
    rotation_vector = v*theta
    R = Ro.from_rotvec(rotation_vector)
    
    return R.as_matrix()


def is_identity(M): 
    for i in range(len(M)): 
        for j in range(len(M)): 
            eps = 10**(-12)
            if i == j and abs(1-M[i][j]) > eps: 
                return False
            elif i != j and abs(M[i][j]) > 10**(-12):
                return False
    return True
                 
def normalize_vector(v):
    n = np.linalg.norm(v)
    eps = 10**(-15)
    if (n < eps):
        normalized = v
    else: 
        normalized = v / n
    return normalized


def is_symmetric(M): 
    for i in range(len(M)): 
        for j in range(int(len(M)/2)):
            if M[i][j] != M[j][i]: 
                return False
    return True


def truncate(n, decimal = 15):
    d = np.power(10,decimal-1)
    res = (int(n * d))/d
    return res

def truncate_vector(v, decimal = 15):
    res = np.empty(len(v))
    for i in range(len(v)):
        res[i] = truncate(v[i], decimal) 
    return res


def scew_symetric_matrix_from_vector(vector):
    x, y, z = vector
    res = np.zeros((3,3))
    res[0][1] = -z
    res[0][2] = y
    res[1][0] = z
    res[1][2] = -x
    res[2][0] = -y
    res[2][1] = x
    return res
    
    
    
    
def nullspace(matrix):
    M = Matrix(matrix)
    null_space = M.nullspace()
    res = []

    for j in range(len(null_space)):
        element = []
        for i in null_space[j]:
            element.append(i)
        res.append(np.array(element))
    return res


    


## https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf

def axis_and_angle_from_rotation(R): 
    R = Ro.from_matrix(R)
    vec = R.as_rotvec()
    return normalize_vector(vec), np.linalg.norm(vec)
    
    '''
    if is_identity(R): 
        theta = 0
        axis = np.array([1, 0, 0])
        return axis, theta

    axis = np.array([R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]])
    axis_norm = np.linalg.norm(axis)
    x = axis_norm/2.
    trace = R[0][0] + R[1][1] + R[2][2]
    
    eps = 10**(-6)
    if x> eps:
        #print('her')
        #print((trace-1.)/2.)
        theta = np.arccos((trace-1.)/2.)
        axis = axis / axis_norm 
        #theta = 1.53
       
    elif trace -1 > 0:
        theta = 0
        axis = axis / axis_norm
    else: 
        theta = np.pi
        v_x = np.sqrt((R[0][0]+1)/2.)
        v_y = np.sqrt((R[1][1]+1)/2.)
        v_z = np.sqrt((R[2][2]+1)/2.)
        axis = np.array([v_x, v_y, v_z])
        axis =normalize_vector(axis)

    '''
    
    return axis, theta
    
    
    
    
    
    
    

def axis_and_angle_from_rotation_old(R):
    if is_identity(R): 
        theta = 0
        axis = np.array([1, 0, 0])
        
    trace = R[0][0] + R[1][1] + R[2][2]
    theta = np.arccos((trace-1.)/2)#np.arccos(truncate((trace-1.)/2,16))
    
    epsilon = 10**(-6)
    
    if theta > np.pi-epsilon: 
        alpha_0 = np.sign(R[2][1] - R[1][2])
        alpha_1 = np.sign(R[0][2] - R[2][0])
        alpha_2 = np.sign(R[1][0] - R[0][1])
        v_x = alpha_0*np.sqrt((R[0][0]+1)/2.)
        v_y = alpha_1*np.sqrt((R[1][1]+1)/2.)
        v_z = alpha_2* np.sqrt((R[2][2]+1)/2.)
        axis = np.array([v_x, v_y, v_z])
        axis =normalize_vector(axis)
        
        
    
    else: 
        v = np.array([R[2][1]- R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]])
        axis = normalize_vector(v)

    #theta = truncate(theta,14)
    return axis, theta
 


def rotation_matrix_2_transformation(R):
    T = np.zeros([4,4])
    T[:3,:3] = R
    T[3,3] = 1
    #print(T)
    return T
    

def rotation_from_transformation(T):
    R= T[:3,:3]
    return R




def transformation_matrix_for_body(model, body_name):
    return transformation_matrix_for_mujoco(model.body(body_name).quat.copy(), model.body(body_name).pos.copy())

def difference_between_rotations(start_rotation , end_rotation):
    rotation = np.matmul(np.transpose(start_rotation), end_rotation)
    #print('rotation',  rotation)
    axis, theta = axis_and_angle_from_rotation(rotation)
    return axis, theta
    


def point_to_4(p):
    p_res = np.ones(4)
    for i in range(3): 
        p_res[i] = p[i] 
    return p_res


def is_zero_vector(v, eps = 10**(-8)):
    for i in range(len(v)): 
        if abs(v[i]) > eps:
            return False
    return True


"""
    
def rpy_rotation_matrix(rpy):
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    
    rollMatrix = np.array([
    [1, 0, 0],
    [0, np.cos(roll), -np.sin(roll)],
    [0, np.sin(roll), np.cos(roll)]
    ])
    
    pitchMatrix = np.array([
    [np.cos(pitch), 0, np.sin(pitch)],
    [0, 1, 0],
    [-np.sin(pitch), 0, np.cos(pitch)]
    ])

   
    yawMatrix = np.array([
    [np.cos(yaw), -np.sin(yaw), 0],
    [np.sin(yaw), np.cos(yaw), 0],
    [0, 0, 1]
    ])
    
    return np.matmul(np.matmul(rollMatrix, pitchMatrix), yawMatrix)  # tjek op på dette. 



"""


"""_summary_
   
rpy = [1.5708, 0 , 3.14159]

rpy = [0, 0, np.pi/4]

T = np.linalg.inv(rpy_rotation_matrix(rpy))

p = np.array([1, 0 , 0])

print(np.dot(T, p))

 """
 
 
def estimation_end_point_of_tangent_vector_on_surface( tangent_origo, unit_tangent, norm_of_tangent, curvature, normal_to_origo):
        new_point = tangent_origo + unit_tangent*norm_of_tangent + curvature/2 *(norm_of_tangent**2)*(-normal_to_origo)
        return new_point
 

def array(X, i): 
    result = np.empty(len(X))
    for j in range(0,len(X)-1): 
       result[j] = X[j][i]
    return result

class Surface:
    
    eps = 10**(-5)  # for differentiation
    
    def __init__(self, funktion):     
        self.F = funktion
        
        
    def F_homogenuos(self,u,v):
        point = self.F(u,v)
        res_point = np.array([point[0],point[1], point[2], 1])
        return res_point
        
    def duF(self, u, v):
        res = (self.F(u+self.eps,v) - self.F(u-self.eps,v))/(2*self.eps)
        return res[:3]

    def dvF(self, u, v):
        res = (self.F(u,v+self.eps) - self.F(u,v-self.eps))/(2*self.eps)
        return res[:3]

    def duuF(self, u, v):
        res = (self.F(u+self.eps,v)-2*self.F(u,v)+self.F(u-self.eps,v))/(self.eps**2)
        return res[:3]
    
    def duvF(self, u, v):
        res = (self.F(u+self.eps,v+self.eps)- self.F(u+self.eps,v-self.eps)- self.F(u-self.eps,v+self.eps)+ self.F(u-self.eps,v-self.eps))/(4*(self.eps**2))
        return res[:3]
    
    def dvvF(self, u, v):
        res = (self.F(u,v+self.eps)-2*self.F(u,v)+self.F(u,v-self.eps))/(self.eps**2)
        return res[:3]
    
    
    
    def normal(self, u, v): 
        du = self.duF(u,v)
        dv = self.dvF(u,v)
        #eps = 10**(-12)
        vector = np.cross(self.duF(u,v), self.dvF(u,v))
        if not is_zero_vector(vector):
            normal = vector/ np.linalg.norm(vector)
        else: 
            #Warning(print('normal is zero', du, dv, vector, not is_zero_vector(vector) ))
            normal = vector
        return normal
       
    def shape_operator(self, u, v):
        f_u = self.duF(u,v)
        f_v = self.dvF(u,v)
        f_uv = self.duvF(u,v)
        f_uu = self.duuF(u,v)
        f_vv = self.dvvF(u,v)
        n= self.normal(u,v)
        
        #print(f_u, f_v, f_uv, f_uu, f_vv)
        
        
        
        of_diagonal_matrix1 = np.dot(f_u, f_v)
        matrix1 = np.array([[np.dot(f_u, f_u), of_diagonal_matrix1],
                             [of_diagonal_matrix1, np.dot(f_v, f_v)]])
        
        of_diagonal_matrix2 = np.dot(f_uv, n)
        matrix2 = np.array( [[np.dot(f_uu, n), of_diagonal_matrix2],
                             [of_diagonal_matrix2, np.dot(f_vv, n)]])
        
        shape_operator = np.matmul(np.linalg.inv(matrix1), matrix2)
        return shape_operator
    '''
    def principal_curvature(self, u,v):
        shape_operator = self.shape_operator(u,v)
        eigenvalues, eigenvectors  = np.linalg.eig(shape_operator)
        return eigenvalues
    '''
    def first_and_second_fundamental_form_values(self, u, v): #https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node28.html
        f_u = self.duF(u,v)
        f_v = self.dvF(u,v)
        f_uv = self.duvF(u,v)
        f_uu = self.duuF(u,v)
        f_vv = self.dvvF(u,v)
        n= self.normal(u,v)
        
        E = np.dot(f_u,f_u)
        F = np.dot(f_u,f_v)
        G = np.dot(f_v, f_v)
        L = np.dot(f_uu, n)
        M = np.dot(f_uv, n)
        N = np.dot(f_vv,n)
        
        K = (L*N-(M**2))/(E*G-(F**2))
        
        H = (E*N+G*L-2*F*M)/(2*(E*G-(F**2)))
        
        #print(f_u, f_v, f_uu, f_uv, f_vv)
        #print(E, F, G, L , M , N)
        
        return E, F, G, L , M , N, K, H
    
    
    
        
    def is_umbilica_point(self, k_min, k_max, K, H):
        eps = self.eps**2
        
        if abs(K)< eps and abs(H)<eps:
            print("warning: flat point on surface, can most likely not handle it")

        
        
        if abs(abs(K) - H**2)< eps or (abs(k_max)+abs(k_min)) < eps:
            return True
        
        return False
    
  
        
        
        
        
    
    def principal_curvature(self, u,v, principal_curvature_for_base_finding=False):
        E, F, G, L , M , N, K, H = self.first_and_second_fundamental_form_values(u,v)
        
        
        k_max = H + np.sqrt((H**2)-K)
        k_min = H - np.sqrt((H**2)-K)
        
        is_umbilica_point = self.is_umbilica_point(k_min, k_max, K, H)
        #print('is_umbilica_point', is_umbilica_point, k_min, k_max)
        
        if is_umbilica_point:
            k_max = -H
            k_min = k_max
        
        #print(k_max, k_min)
        if principal_curvature_for_base_finding:
            return is_umbilica_point,E, F, G, L , M , N, k_max, k_min
        else:
            return k_max, k_min
    
    
    def principal_axis(self,u,v, return_positive_curvature = False):   # get principal axis assume only convex surfaces
        
        is_umbilica_point, E, F, G, L , M , N, k_max, k_min= self.principal_curvature(u,v, True)
        
        #k_min, k_max = self.principal_curvature(u,v)

        '''
        p_c = [k_min, k_max]
        basis = [[],[]]
        #print(p_c)
        for i in range(2):
            #print('M-p_c[i]*F', M-p_c[i]*F)
            #print(N-p_c[i]*G)
            #print (p_c[i])
            #du/dv = value 
            a = 1
            b = -((M-p_c[i]*F)/(N-p_c[i]*G))
            #print('a, b ', a,b)
            #print(-(L-p_c[i]*E)/(M-p_c[i]*F))
            eigvec = normalize_vector(np.array([a,b]))
            print(eigvec)
            #du = normalize_vector(self.duF(u,v))
            a = eigvec[0]
            b = eigvec[1]
            #dv = normalize_vector(self.dvF(u,v))
            
            vector = a*self.duF(u,v) + b*self.dvF(u,v) #a*self.duF(u,v) + b*self.dvF(u,v)
            #print(vector)
            #print(np.linalg.norm( vector ))
            #print()
            base = vector / np.linalg.norm( vector )
            #print(base)
            #v = vector / np.linalg.norm( vector )
            basis[i] = base
            #print(basis)

        #print(basis)
        
        
        if is_umbilica_point:
            basis[1] = np.cross(self.normal(u,v), basis[0])
            
        
        print('dot product basis 1 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx', np.dot(basis[0],basis[1]))
        
        '''
        shape_operator = self.shape_operator(u,v)
        eigenvalues, eigenvectors  = np.linalg.eig(shape_operator)
        #print(k_max, k_min)
        
        eigenvectors = np.transpose(eigenvectors)
        #print('eig ', eigenvalues, eigenvectors)
        
        basis = []
        for i in range(2):
            e = eigenvectors[i]  # as smallest is first?
            
            #e = e/e[0]
            #print('e', e[0]/e[1])
            
            du = self.duF(u,v)
            
            dv = self.dvF(u,v) #normalize_vector(self.dvF(u,v))
            
            b = e[0]*du + e[1]*dv
            #print('b', b)
            b = b / np.linalg.norm(b)
            basis.append(b[:3])
        if is_umbilica_point:
            basis[1] = np.cross(self.normal(u,v), basis[0])
            
    
        
        #print('cross of du,dv', np.cross(self.duF(u,v), self.dvF(u,v)))
        
        #print('dot product basis 23333333333333333333333333333333333333333333333333333333333', np.dot(basis[0],basis[1]))
        
        #basis[1] = np.cross(self.normal(u,v), basis[0])
        #print('dot product basis 2', np.dot(basis[0],basis[1])
        
        #print('basis', basis)
        
        #print('dot product basis 1', np.dot(basis[0],basis[1]))
        
        x= basis[0]
        y = basis[1]
        k1 = eigenvalues[0]
        k2 = eigenvalues[1]
      
        
        
        
        
        if return_positive_curvature:
            if k1 < 0: 
                k1 = -k1
            if k2 < 0: 
                k2 = -k2
            axis = [x,y]
            curvature = [k1, k2]
            
            return axis, curvature
        else:
            return [x, y]
        
     


    def base_for_tangent_plane(self,u,v):
        x, y = self.principal_axis(u,v)
        n = self.normal(u,v)        
        n1 = np.cross(x, y)
        if np.dot(n,n1) < -.9 : 
            return y, x
        else:
            return x, y
        
    
    
    def directional_curvature(self, u, v, direction):
        axis, curvature = self.principal_axis(u,v, True)
        mu = normalize_vector(direction)
        directional_curvature = curvature[0] * (np.dot(mu, axis[0])**2) +  curvature[1] * (np.dot(mu, axis[1])**2)
        return directional_curvature
        
        
        
        
    
    def surface_distance1(self, uv_start, uv_target, h = 0.1):
        #ok for convex for non convex story is different
        
        p_target = self.F(uv_target[0], uv_target[1])
        p_current = self.F(uv_start[0], uv_start[1])
        
        u,v = uv_start
        
        current_direction = p_target - p_current
        distance = np.linalg.norm(current_direction)
        direction = current_direction / distance
        total_distance = 0
        while distance > h:
            
            
            p = p_current + h*direction
            u, v = self.uv_for_surface_point(p)
            p_new = self.F(u,v)
            
            total_distance += np.linalg.norm(p_new-p_current)
            p_current = p_new
            
            current_direction = p_target - p_current
            distance = np.linalg.norm(current_direction)
            direction = current_direction / distance
            
        total_distance +=distance
        

      
        return total_distance
        
        
        
        
        
        
        
        
      
    def surface_distance(self, uv_start, uv_target, h = 0.01):
        #Small-angle approximation see wiki
        
        
    
        p_target = self.F(uv_target[0], uv_target[1])[:3]
        
        distance = 0
        uv_current = uv_start
        p_current = self.F(uv_current[0], uv_current[1])[:3]
        
        n_current= self.normal(uv_current[0], uv_current[1])[:3]
        tangent = self.tangential_direction(uv_current,p_target, p_current, n_current)
        i = 0
        
        
        
        
        while np.linalg.norm(tangent) > h:

            
            
            curvature = self.directional_curvature(uv_current[0],uv_current[1],  tangent)
            

            distance +=  h
            
            
            unit_tangent = normalize_vector( tangent)
            p_current, uv_current = self.project_end_point_of_tangent_vector_to_surface( p_current, unit_tangent, h, curvature, n_current)
            
            
            
            n_current= self.normal(uv_current[0], uv_current[1])
            tangent = self.tangential_direction(uv_current,p_target, p_current,n_current)
     
            
            i+= 1
        
        distance += np.linalg.norm(tangent) 
            
        return distance

       
    
    def estimation_end_point_of_tangent_vector_on_surface(self, tangent_origo, unit_tangent, norm_of_tangent, curvature, normal_to_origo):
        new_point = tangent_origo + unit_tangent*norm_of_tangent + curvature/2 *(norm_of_tangent**2)*normal_to_origo
        return new_point
    
    
    def project_end_point_of_tangent_vector_to_surface(self, tangent_origo, unit_tangent, norm_of_tangent, curvature, normal_to_origo):
        
        new_point = self.estimation_end_point_of_tangent_vector_on_surface(tangent_origo, unit_tangent, norm_of_tangent, curvature, normal_to_origo)
        
        #pp.plot_point(new_point, 'b')
        
        u, v = self.uv_for_surface_point(new_point)
        res_point = self.F(u,v)[:3]
        return res_point, [u,v]
        
        
    

    
    
    def tangential_direction(self, uv, point_to_project, tangent_origo = None, tangent_normal= None):
        tangent_origo, tangent_normal = self.tangent_origo_and_normal_none_test(uv, tangent_origo, tangent_normal)
        
    
        
        p_target_on_tangent = self.projected_point_on_tangent_plane(uv, point_to_project, tangent_origo, tangent_normal)
        
        
        
        direction = p_target_on_tangent - tangent_origo
        
        #print('target', p_target_on_tangent)
        #print('point', point_to_project)
        
        return direction
        
    
    
    def projected_point_on_tangent_plane(self, uv, point_to_project, tangent_origo = None, tangent_normal= None):
        
        tangent_origo, tangent_normal = self.tangent_origo_and_normal_none_test(uv, tangent_origo, tangent_normal)
        
        v_origo_to_p = point_to_project - tangent_origo
        
        
        p_projected= point_to_project - np.dot(v_origo_to_p, tangent_normal)*tangent_normal
        
        
        return p_projected 
    
    
    def tangent_origo_and_normal_none_test(self, uv, tangent_origo, tangent_normal):
        if tangent_origo is None:
            tangent_origo = self.F(uv[0],uv[1])
        if tangent_normal is None: 
            tangent_normal= self.normal(uv[0],uv[1])
        return tangent_origo, tangent_normal
    
    
    
    def uv_for_surface_point(self, surface_point):
        #has to be implemented for all surfaces, not implicit.
        return
    
    
    
        
    #    return 0
    

class Ellipsoid(Surface):
    
    uv_limits = [(0,np.pi), (0,np.pi)]
    
    def __init__(self):
        return        
       
        #self.evecs = np.transpose(self.inv_evecs)
    '''  
    def fit_from_mesh(self, mesh_points):
        self.mesh_points = mesh_points
        self.center, self.evecs, self.radii, self.v = self.ellipsoid_least_square_fit()
        
        self.inv_evecs = np.transpose(self.evecs)
        
        #print(self.evecs)
        #print(self.center)
    '''
    
    
    
    
    def direct_fit(self, center, evecs, radii):
        self.center = center
        self.evecs = evecs
        self.radii = radii
        self.inv_evecs = np.transpose(self.evecs)
        
        #print('center', center)
        #print(evecs)
        #print(radii)
        
    def normal(self, u, v):
        p = self.F(u,v)
        res = 2*np.array([p[i]/ self.radii[i]**2 for i in range(3)])
        res = normalize_vector(res)
        return res
    
    
    
    def uv_for_surface_point(self, surface_point):
        
    
        
        p_centered =  surface_point - self.center
        
        p_not_rotated = np.matmul(self.inv_evecs, p_centered)
        
        #print('after_rotation', surface_point)
        
        circle_point = np.array([p_not_rotated[i]/ self.radii[i] for i in range(3)]) 
        
        #print('circle_point', circle_point)
        
        circle_point = normalize_vector(circle_point)
        
        y1 = (np.sqrt(circle_point[0]**2 + circle_point[1]**2))
        x1 =circle_point[2]
        
        v =  np.arccos(circle_point[2]) #+ np.pi*3/2
        
        #value = circle_point[0]/(np.sqrt(circle_point[0]**2 + circle_point[1]**2))
        #u = np.sign(circle_point[1])*np.arccos(value) 
        
        u =np.arctan2(circle_point[1],circle_point[0])  #+ np.pi
        
        
        
        #print('u, v', u,v)
        
        #print('recalculated', self.F(u,v))
        
        #pp.plot_point( self.F(u,v), 'r')
        #pp.show()
        
        
        return u,v
    
    def u_with_in_bounderies(self, u):
        if u < 0 or u >= 2*np.pi: 
            pi_billion = 3141592653
            u_billion = int(u*10**9)
            val = u_billion % 2*pi_billion
            u = val/10**(9)
        return u
    
    def v_with_in_bounderies(self, u, v):
        if v < 0 or v > np.pi:
            pi_billion = 3141592653
            v_billion = int(v*10**9)
            val = v_billion % 2*pi_billion
            
            if val > pi_billion:
                u = self.u_with_in_bounderies(u+np.pi)
                v = (2*pi_billion-val)/10**(9)
                
        return u,v
            
            
        
    def F(self,u,v):
       
        #write formula for u and v  # tjek op igen om hvad bounderyes has to be.
        # 0 <= u < 2*pi , 0 <= v <= pi
        
        #https://en.wikipedia.org/wiki/Ellipsoid Parameterisering
        
        #u  = self.u_with_in_bounderies(u)
        #u, v = self.v_with_in_bounderies(u,v)
        
            
        
        
        x = self.radii[0]*np.cos(u)*np.sin(v) 
        y = self.radii[1]*np.sin(u)*np.sin(v)
        z = self.radii[2]*np.cos(v)
        
        
        
        '''
        print('radii', self.radii)
        print('u,v', u,v )
        print('u', np.cos(u), np.sin(u))
        print('v', np.sin(v), np.cos(v))
        print(np.cos(u)*np.sin(v) )
        print(np.sin(u)*np.sin(v))
        print(np.cos(v))
        print('point', x, y, z)
        '''
        
        surface_point = [x, y, z]
        #print('not_rotated F(u,v)',u, v, surface_point)
        point = np.matmul(self.evecs, surface_point) + self.center
        #print('surface_point1', np.dot(surface_point, self.evecs))
        #print('surface_point2', np.matmul(self.inv_evecs, surface_point))
        
        
        
        return point


  
    
    
    #https://adamheins.com/blog/ellipsoidal-shell-inertia
    def inertia_tensor(self, mass):
        I = np.zeros((3,3))
        I[0][0] = (self.radii[1]**2 + self.radii[2]**2)
        I[1][1] = (self.radii[0]**2 + self.radii[2]**2)
        I[2][2] = (self.radii[0]**2 + self.radii[1]**2)
        I = mass/5. * I 
        return I
              
    '''   
    # http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
    # for arbitrary axes
    def ellipsoid_least_square_fit(self):
        X = self.mesh_points
        #print(self.mesh_points)

        x = array(X, 0)
        y = array(X, 1)
        z = array(X, 2)
        
        """ 
        fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx +
        2Hy + 2Iz + J = 0 and A + B + C = 3 constraint removing one extra parameter
        """
        
        D = np.array([x * x + y * y - 2 * z * z,
                    x * x + z * z - 2 * y * y,
                    2 * x * y,
                    2 * x * z,
                    2 * y * z,
                    2 * x,
                    2 * y,
                    2 * z,
                    1 - 0 * x])
        
        
        d2 = np.array(x * x + y * y + z * z).T # rhs for LLSQ
        
    
        u = np.linalg.solve(D.dot(D.T), D.dot(d2))
        
        a = np.array([u)0] + 1 * u)1] - 1])
        b = np.array([u)0] - 2 * u)1] - 1])
        c = np.array([u)1] - 2 * u)0] - 1])
        v = np.concatenate([a, b, c, u)2:]], axis=0).flatten()
        A = np.array([[v[0], v[3], v[4], v[6]],
                    [v[3], v[1], v[5], v[7]],
                    [v[4], v[5], v[2], v[8]],
                    [v[6], v[7], v[8], v[9]]])

        center = np.linalg.solve(- A[:3, :3], v[6:9])

        translation_matrix = np.eye(4)
        translation_matrix[3, :3] = center.T

        R = translation_matrix.dot(A).dot(translation_matrix.T)
    
        
        W = R[:3, :3] / (- R[3, 3])
        
        evals, evecs = np.linalg.eig(W)
    
        evecs = evecs.T

        radii = np.sqrt(1. / np.abs(evals))
        radii *= np.sign(evals)

        return center, evecs, radii, v

    '''
    '''
    def base_for_tangent_plane(self,u,v):  # not principal curvatures
        
        n = self.normal(u,v)
        x = normalize_vector(np.cross(n,self.evecs[:,0]))
        if is_zero_vector(x):
            x = normalize_vector(np.cross(n,self.evecs[:,1]))
        y =  normalize_vector(np.cross(n,x))
        return x, y
        
    
    '''
''''''
    
class Unit_Ellipsoid(Surface):
    
    def __init__(self):
        self.center = np.zeros(3)
        self.evecs = np.identity(3)
        self.radii = np.array([1,2,3])
        self.inv_evecs = np.transpose(self.evecs)

    
    
    def uv_for_surface_point(self, surface_point):
        
        #pp = PyPlotting.Plot()
        #pp.ellipsoid_plot(self.center, self.evecs, self.radii)
        #pp.plot_point(surface_point, 'b')
       
        
        
        
        p_centered =  surface_point - self.center
        
        #print('p_surface', surface_point)
        #print('radii', self.radii)
        
        p_not_rotated = np.dot(p_centered,self.inv_evecs)
        
        circle_point = np.array([p_not_rotated[i]/ self.radii[i] for i in range(3)]) 
        circle_point = normalize_vector(circle_point)
        
        
        v = np.arccos(circle_point[2]) 
        
        value = circle_point[0]/(np.sqrt(circle_point[0]**2 + circle_point[1]**2))
        
        u = np.arctan2(circle_point[1],circle_point[0]) #np.sign(circle_point[1])*np.arccos(value) 
        
    
        
        #print('u, v', u,v)
        
        #print('recalculated', self.F(u,v))
        
        #pp.plot_point( self.F(u,v), 'r')
        #pp.show()
        
        return u,v
    
    

        
        
    def F(self,u,v):
        #write formula for u and v  # tjek op igen om hvad bounderyes has to be.
        # -pi < u < pi , 0 < v < pi
        #https://en.wikipedia.org/wiki/Ellipsoid Parameterisering
        
        
        x = self.radii[0]*np.cos(u)*np.sin(v) 
        y = self.radii[1]*np.sin(u)*np.sin(v)
        z = self.radii[2]*np.cos(v)
        
        surface_point = [x, y, z]
        point = np.dot(surface_point, self.evecs) + self.center
        res_point = np.array([point[0],point[1], point[2], 1])
        
        return res_point
    
    def plot_object(self, pp):
        pp.ellipsoid_plot(self.center, self.evecs, self.radii)

    '''
    def duF(self, u, v):
        x = -self.radii[0]*np.sin(u)*np.sin(v) 
        y = self.radii[1]*np.cos(u)*np.sin(v)
        z = 0
        
        surface_point = [x, y, z]
        point = np.matmul(self.inv_evecs, surface_point)
        return point

    def dvF(self, u, v):
        
        x = self.radii[0]*np.cos(u)*np.cos(v) 
        y = self.radii[1]*np.sin(u)*np.cos(v)
        z = -self.radii[2]*np.sin(v)
        
        
        surface_point = [x, y, z]
        point = np.matmul(self.inv_evecs, surface_point)
        return point

    def duuF(self, u, v):
        x = -self.radii[0]*np.cos(u)*np.sin(v) 
        y = -self.radii[1]*np.sin(u)*np.sin(v)
        z = 0
        
        surface_point = [x, y, z]
        point = np.matmul(self.inv_evecs, surface_point)
        return point

    
    def duvF(self, u, v):
        x = -self.radii[0]*np.sin(u)*np.cos(v) 
        y = self.radii[1]*np.cos(u)*np.cos(v)
        z = 0
        
        surface_point = [x, y, z]
        point = np.matmul(self.inv_evecs, surface_point)
        return point

    
    def dvvF(self, u, v):
        
        x = -self.radii[0]*np.cos(u)*np.sin(v) 
        y = -self.radii[1]*np.sin(u)*np.sin(v)
        z = -self.radii[2]*np.cos(v)
        
        print(x, y, z)
        
        surface_point = [x, y, z]
        print(surface_point)
        point = np.matmul(self.inv_evecs, surface_point)
        print('point', point)
        return point
    '''


class Finger_NURBS(Surface):
    
    #axis = [0,0,1]
    #theta = 0
    #rotation = rotation_from_axises_and_angle(axis, theta)
    #inv_rotation = np.transpose(rotation)
    
    def __init__(self, path_to_json_file):
        print()
        
        return       
    

class Poly_Fit(Surface):
    
    
    c = [1,1,1]
    dispZ = np.array([0, 0, 5])
    
    
    
    def __init__(self):
        return       
    
    def uv_for_surface_point(self, surface_point):
        p = [(surface_point[i]- self.dispZ[i])/self.c[i] for i in range(3)]
        
        # p[2]/p[0] = a1 =>   sin(u)/cos(u) = a1  =>  u = arctan(a1)
        
        u = np.arctan2(p[2],p[0])
        a = np.cos(u)*p[0]+ np.sin(u)*p[2]
        v = np.arctan2(a,  p[1]) 
        
        return u,v
 
    def F(self,u,v):
        #Fit kan ses i Henriks noter
        #print('u, v', u,v)
        fV = self.fV(u,v)
        ellipsoid = np.array([self.c[0]*np.cos(u)*np.sin(v), self.c[1]*np.cos(v), self.c[2]*np.sin(u)*np.sin(v)])
        p = self.dispZ + fV*ellipsoid
        
        res_point = np.array([p[0], p[1], p[2], 1]) 
        
        return res_point
    
    def fV():
        return 1
      

class Thumb_Fit(Poly_Fit):
    
    c = [9., 9., 24.]
    
    def fV(self,u,v):
   
        l1 =-6.02341 + 17.1039* v - 14.3366 *(v**2) + 5.11026 *(v**3) - 0.659165 *(v**4) + 12.9271 *np.sin(u)
        l2 =-29.0689* v *np.sin(u) + 23.8286 *(v**2) *np.sin(u) - 8.39354 *(v**3) *np.sin(u) + 1.07046 *(v**4) *np.sin(u) 
        l3 = +2.72763 *np.sin(3*u) - 6.26097* v *np.sin(3*u) + 5.21315 *(v**2) *np.sin(3*u) - 1.84692 *(v**3) *np.sin(3*u) 
        l4 = 0.235436 *(v**4) *np.sin(3*u) + 4.28567 *np.sin(5*u) - 9.57613 *v *np.sin(5*u) + 7.57806 *(v**2) *np.sin(5*u) 
        l5 = -2.56177 *(v**3) *np.sin(5*u) + 0.314969 *(v**4) *np.sin(5*u) + 3.60394 *np.sin(7*u) - 8.07055* v *np.sin(7*u) 
        l6 = 6.48434 *(v**2) *np.sin(7*u) - 2.23526 *(v**3) *np.sin(7*u) + 0.280608 *(v**4) *np.sin(7*u) + 0.561426 *np.sin(9*u) 
        l7 = -1.36747 *v *np.sin(9*u) + 1.21102 *(v**2) *np.sin(9*u) - 0.459199 *(v**3) *np.sin(9*u) + 0.0629813 *(v**4) *np.sin(9*u)
        
        
        
        res = l1 + l2 + l3+ l4+ l5 +l6 + l7
        
        
        
        return res
    
        
class MF_Fit(Poly_Fit):
    
    
    c = [7., 7., 24.]
  
    
    def fV(self,u,v):
        
        l1 = 10.6684 - 18.786*v + 13.7743 *(v**2) - 4.37255 *(v**3) + 0.505596 *(v**4) - 8.04855 *np.sin(u) 
        l2 = +16.2539  *v *np.sin(u) - 12.1302 *(v**2) *np.sin(u) + 3.92857 *(v**3) *np.sin(u) - 0.466315 *(v**4) *np.sin(u) 
        l3 =-2.22739 *np.sin(3 * u) + 4.66555  *v *np.sin(3  *u) - 3.52982 *(v**2) *np.sin(3  *u) + 1.17245 *(v**3) *np.sin(3 * u) 
        l4 = -0.144808 *(v**4) *np.sin(3 * u) + 0.923762 *np.sin(5 * u) - 2.10432  * v *np.sin(5  *u) + 1.52727 *(v**2) *np.sin(5 * u) 
        l5 = -0.444592 *(v**3) *np.sin(5  *u) + 0.0446604 *(v**4) *np.sin(5  *u) + 1.21264 *np.sin(7  *u) - 2.7453 * v *np.sin(7  *u) 
        l6 = +2.16086 *(v**2) *np.sin(7  *u) - 0.715907 *(v**3) *np.sin(7 * u) + 0.0852741 *(v**4) *np.sin(7  *u)
        
        
        
        res = l1 + l2 + l3+ l4+ l5 +l6 
        return res
    
        






class Interpolation:
    
    blend_tau = 0.3
    
    
    def parabolic_blend(self, t, q0, q1, q2, t0,t1,t2, blend_t):
        #https://typeset.io/pdf/turning-paths-into-trajectories-using-parabolic-blends-3wgd3eo34y.pdf
        # has to be that t0+blend_t < t < t2- blend_t
        
        v0 = (q1-q0)/(t1-t0)
        v1 = (q2-q1)/(t2-t1)
        
        a1 = (v1- v0)/blend_t
        
        if t < t1 - blend_t/2: 
            res = q0 + v0*(t-t0)
        elif t <= t1 + blend_t/2: 
            res = q1 + v0*(t-t1)+ 1/2 *a1*(t-t1+ blend_t/2)**2
        else: 
            res = q1 + v1*(t-t1)
        return res
    
    

    def parabolic_blend_function(self, t, intersection_time, x1, v1, v2):
        # parabolic of two linear functions. Time in intersection for x0 x1 and x2 time for x1 
        delta_t = t-intersection_time
        res = ((v2-v1)/(4*self.blend_tau))* (delta_t + self.blend_tau)**2 + v1*(delta_t) + x1
        return res

    
    
    def parabolic_blend_points(self, t, x0, x1, x2, t0,t1,t2):
        v1 = (x0-x1)/(t0-t1)
        v2 = (x2-x1)/(t2-t1)
        
        #print(t1)
        #min = np.minimum(np.linalg.norm(x1-x0), np.linalg.norm(x2-x1))
        #self.blend_tau = min*4/(10*np.linalg.norm((v2-v1)))
        
        if np.linalg.norm((v2-v1)) < 10**(-9):
            # chosen abtraly see robotics notes
            tau = 1
        else:
            min = np.minimum(np.linalg.norm(x1-x0), np.linalg.norm(x2-x1))
            #print(min)
            #print(np.linalg.norm(v2-v1))
            tau = min*4/(10*np.linalg.norm((v2-v1)))
            #print('tau', tau)
            tau =1 #chose better
        #print(v1, v2, t, t1, x1)
        res = self.parabolic_blend_function(t, t1, x1, v1, v2 )
        return res
        
    
    def parabolic_blend_acceleration(self, t, q0, q1, q2, t0,t1,t2, blend_t):
        a = 0
        if t >= t1 - blend_t/2 and t <= t1 + blend_t/2:
            v0 = (q1-q0)/(t1-t0)
            v1 = (q2-q1)/(t2-t1)
            a = (v1- v0)/blend_t
        
        return a
        
        
    def parabolic_blend_rotation(self):
        #to do wheen needed to create path.
        return
    
    def linear_interpolation(self, t, x0, x1, t0,t1):
        v1 = (x1-x0)/(t1-t0)
        
        
        res = (t-t0)*v1 + x0
        return res