import Math_Utilities as mu
import Mesh_Utilities
import numpy as np
import PyPlotting
import Joint_Grid





class Shadow_hand:
    
    def __init__(self, mj_sim): 
        self.world_from_palm = self.world_from_palm_transformation(mj_sim.mj_model)
        self.palm_from_wold = np.linalg.inv(self.world_from_palm)
        self.fingers = self.create_fingers(mj_sim)
        

    
    def world_from_palm_transformation(self, mj_model):
        #create world from palm transformation
        world_from_forearm =  mu.transformation_matrix_for_body(mj_model, "rh_forearm")
        forearm_from_wrist =  mu.transformation_matrix_for_body(mj_model,"rh_wrist")
        wrist_from_palm =  mu.transformation_matrix_for_body(mj_model, "rh_palm") 
        world_from_palm = np.matmul(np.matmul(world_from_forearm,forearm_from_wrist),wrist_from_palm)
        return world_from_palm
    
    
    def create_fingers(self,mj_sim):
        fingers = []
        
        fingers.append(Thumb(mj_sim, self.world_from_palm))
        fingers.append(F1(mj_sim, self.world_from_palm))
        fingers.append(F2(mj_sim, self.world_from_palm))
        fingers.append(F3(mj_sim, self.world_from_palm))
        
        fingers.append(LF(mj_sim, self.world_from_palm))
       
        return fingers




class Finger: 
    
    SHADOW_HAND_CALCULATION = True
    
    USE_NURBS = False
    BODY_NAMES = []
    JOINT_NAMES = []
    ACTUARTOR_NAMES =[]
    PARTH_2_STL = ""
    MESH_NAME = ""
    UV_limit =[]
    NAME = ""
    PARTH_2_JSON = ""
    
    tourqe_limits = [] #HARDCODED IN EACH CLASS. 



    
    """
        hardcoded for thum by lookup. Find how to acces this value
    """
   
    
    def __init__(self, mj_sim, world_from_palm):
        self.world_from_palm = world_from_palm
        mj_model = mj_sim.mj_model
        mj_data =  mj_sim.mj_data
        total_mesh_points = Mesh_Utilities.mesh_points_from_obj_file(self.PARTH_2_STL)
        front_mesh_points = self.front_mesh_points(total_mesh_points)
        self.tip_length = np.max([p[2] for p in front_mesh_points])
        self.tip_radius = np.sqrt(np.max([p[0]**2 + p[1]**2 for p in front_mesh_points])) + 3 #3 mm for safety
        if self.USE_NURBS:
            eps = 10**(-5)
            self.UV_limit = [(eps, 1-eps),(eps, 1-eps)]
            self.surface_fit = mu.Finger_NURBS(self.PARTH_2_JSON)
        else: 
            self.surface_fit = self.surface_fit()
        
        self.model_ctrl_indexes =  self.get_mj_acturator_id(mj_data)
        self.model_joint_indexes = self.get_mj_joint_id(mj_data)
        self.transformation_matrices = self.get_body_transformation_matrices(mj_model)
        q_limits = self.get_q_limits(mj_model)
        self.min_limits = self.min_limits(q_limits)
        self.max_limits = self.max_limits(q_limits)
        self.rotation_axises = self.get_rotation_axises_for_joint(mj_model)
        self.number_joints= len(self.JOINT_NAMES)
        
        
        #change min and max ??
        
    
        
        
        
    def get_index_in_shadow_hand(self):  
        return 0    
        
    

    def get_mj_acturator_id(self,mj_data):
        act_indices = []
        for name in self.ACTUARTOR_NAMES: 
            id = mj_data.actuator(name).id
            act_indices.append(id)
        return act_indices
    
    def get_mj_joint_id(self, mj_data):
        joint_indices = []
        for name in self.JOINT_NAMES: 
            id = mj_data.joint(name).id
            joint_indices.append(id)
        return joint_indices
    
    def get_tip_body_id(self, mj_data):
        
        id = mj_data.body(self.BODY_NAMES[-1]).id
        return id
 

    def plot_finger_surface(self, pp):
        tip_points = self.tip_points_finger_tip_frame()
        pp.plot_3D_points(tip_points, 0.5)
        
        '''
        u = self.UV_limit[0][1]
        v = self.UV_limit[1][1]
        
        u = 2.5
        v = 2.6
        
        #print('u, v', u,v)
        normal = self.surface_fit.normal(u,v)
        #print(normal)
        
        p = self.surface_fit.F(u,v)
        
        #print('p', p)
        u,v = self.surface_fit.uv_for_surface_point(p)
        #print('u,v', u,v)
        #print('p', self.surface_fit.F(u,v))
        
        pp.plot_vector(p,normal,'r', scale=10)
        x, y = self.surface_fit.base_for_tangent_plane(u,v)
        #print('dot', np.dot(x,y))
        #print(x , y)
        #print(np.linalg.norm(x))
        
        pp.plot_vector(p,x,'g', scale=10)
        pp.plot_vector(p,y,scale=10)
        '''
        pp.set_axes()
        
        return pp
            
        
        
        
    def tip_points(self, q):
        #print('tip_point', type(self), q)
        
        u_list = np.linspace(self.UV_limit[0][0], self.UV_limit[0][1], 30)
        v_list = np.linspace(self.UV_limit[1][0], self.UV_limit[1][1], 30)
        points =[]
        for u in u_list:
            for v in v_list: 
                points.append(self.contact_point(q,u,v))
        
        return points
        
    def tip_points_finger_tip_frame(self):
        u_list = np.linspace(self.UV_limit[0][0], self.UV_limit[0][1], 30)
        v_list = np.linspace(self.UV_limit[1][0], self.UV_limit[1][1], 30)
        points =[]
        for u in u_list:
            for v in v_list: 
                points.append(self.surface_fit.F(u,v))
        
        return points
                 
        
    
    def get_rotation_axises_for_joint(self, mj_model):
        rotation_axises = []
        for joint_name in self.JOINT_NAMES:
            rotation_axises.append(mj_model.joint(joint_name).axis.copy())
        return rotation_axises
        
    '''
    def get_tourqe_limits(self,mj_model):
        limits = []
        for name in self.JOINT_NAMES:
            limits.append(mj_model.joint(name).forcerange.copy())
        return limits
    '''
    
    
    def get_q_limits(self,mj_model):
        q_limits = []
        for joint_name in self.JOINT_NAMES:
            q_limits.append(mj_model.joint(joint_name).range.copy())
        return q_limits
        
        
    def get_body_transformation_matrices(self, mj_model):  
        transformation_matrices = []
        for body_name in self.BODY_NAMES:
            T =  mu.transformation_matrix_for_body(mj_model,body_name)
            transformation_matrices.append(T)
        return transformation_matrices
    
    
    
    
    def world_from_finger_tip_base_transformation(self,q):
        res_transformation = self.world_from_palm   
        for i in range(0,len(q)): 
            T =  mu.rotaion_transformation_matrix_from_axises_and_angle(self.rotation_axises[i],q[i])
            T_res = np.matmul(self.transformation_matrices[i], T)
            res_transformation = np.matmul(res_transformation, T_res)
        return res_transformation
    
    
    def get_pos_of_joint(self,joint_idx, q):
        res_transformation = self.world_from_palm
        
        for i in range(0,joint_idx): 
            T =  mu.rotaion_transformation_matrix_from_axises_and_angle(self.rotation_axises[i],q[i])
            T_res = np.matmul(self.transformation_matrices[i], T)            
            res_transformation = np.matmul(res_transformation, T_res)
        
        res_transformation = np.matmul(res_transformation, self.transformation_matrices[joint_idx])
        
        return res_transformation[:3,3]
    
    
    
    def get_rotation_axis_of_joint(self,joint_idx,q):
        res_transformation = self.world_from_palm
        
        for i in range(0,joint_idx): 
            T =  mu.rotaion_transformation_matrix_from_axises_and_angle(self.rotation_axises[i],q[i])
            T_res = np.matmul(self.transformation_matrices[i], T)            
            res_transformation = np.matmul(res_transformation, T_res)
        
        res_transformation = np.matmul(res_transformation, self.transformation_matrices[joint_idx])    
        rotation_axis = self.rotation_axises[joint_idx]
        res_axis = np.matmul(mu.rotation_from_transformation(res_transformation), rotation_axis)
        #if joint_idx == 1: 
        #    print('rotation_axis', rotation_axis)
        #    print('res_rotation_axis', res_axis)
        #    print('rotation matrix', mu.rotation_from_transformation(res_transformation))
        #    print('q', q[0])
        
        return res_axis
        
         
        
        
    def linear_and_angualar_jacobians(self, q, u, v):
        J_p = np.zeros((3,len(q)))
        J_r = np.zeros((3,len(q)))
        
        p_f = self.contact_point(q, u, v)
        
        res_transformation = self.world_from_palm
        #print(self.transformation_matrices)
        #print('q', q)
        for i in range(0,len(q)): 
            #print('i', i)
            res_transformation  = np.matmul(res_transformation, self.transformation_matrices[i])
            p_fj = res_transformation[:3,3]
            z_fj = np.matmul(mu.rotation_from_transformation(res_transformation), self.rotation_axises[i])
            #print(z_fj)
            
            J_p[:,i] = np.cross(z_fj, p_f-p_fj)
            J_r[:,i] = z_fj
            T =  mu.rotaion_transformation_matrix_from_axises_and_angle(self.rotation_axises[i],q[i])
            res_transformation = np.matmul(res_transformation, T)
           
            
        return J_p, J_r    
    
    

        
        
        
    def jacobian(self, quv):
        J_p, J_r = self.linear_and_angualar_jacobians(quv[:-2], quv[-2], quv[-1])
        J = np.concatenate((J_p, J_r))
        return J
        
    
    def jacobian_acts(self, quv, do_shadow_hand_calculation = False):
        return self.jacobian(quv)
        
        
        
        
        
        
        
        
        
        
        
        
    

    def contact_point(self,q,u,v): # giv et bedre navn
        transformation = self.world_from_finger_tip_base_transformation(q)
        elipse_point = self.surface_fit.F(u,v) 
        #print('point', elipse_point)
        contact_point = np.matmul(transformation, elipse_point)
        return contact_point[:3]
    
    
    def d_contact_point_palm_frame(self,q, u,v, du = True):
        transformation= self.world_from_finger_tip_base_transformation(q) #Laver denne både ovenover skal udregnes 3 gange måske laves om til 1 og gemme?
        #print("transformation", transformation)
        R = mu.rotation_from_transformation(transformation)
        #print("rotation", R)
        if du: 
            p = self.surface_fit.duF(u,v)
            #print("u,v, du", u, v, p)
        else:
            p = self.surface_fit.dvF(u,v)
            #print("u,v, dv", u, v, p)
            
        res = np.matmul(R, p)
        return res
    

    def is_within_configuration_limits(self, quv):
        for i in range(len(self.min_limits)):
            q = quv[i]
            if q < self.min_limits[i] or q > self.max_limits[i]:
                if i < len(self.min_limits)- 2:
                    print('joint not with in limits', i, q, self.min_limits[i], self.max_limits[i])   
                else:  
                    print('u,v not with in limits', i, q, self.min_limits[i], self.max_limits[i])  
                return False
        return True
    
    def is_within_tourqe_limits(self, tourqe):
        for i in range(len(tourqe)):
            tau = tourqe[i]
            if tau < self.tourqe_limits[i][0] or tau > self.tourqe_limits[i][1]:
                
                #print('joint', i, q, self.min_limits[i], self.max_limits[i])    
                return False
        return True
    
 
 
    def is_five_joints(self):
        if len(self.JOINT_NAMES) >= 5: 
            return True
        else:
            return False
        
    
    
    def min_limits(self, q_limits):
        lim = self.q_limits_min(q_limits)
        lim.append(self.UV_limit[0][0])
        lim.append(self.UV_limit[1][0])
        return lim
    
    def max_limits(self,q_limits):
        lim = self.q_limits_max(q_limits)
        lim.append(self.UV_limit[0][1])
        lim.append(self.UV_limit[1][1])
    
        return lim
    
    def q_limits_min(self,q_limits):
        q_min = []
        for l in q_limits: 
            q_min.append(l[0])
        return q_min
    
    def q_limits_max(self,q_limits):
        q_max= []
        #hardcode_value
        for l in q_limits: 
            q_max.append(l[1])
        if self.get_index_in_shadow_hand() > 0: 
            q_max[-1] = 1.5708
        return q_max
    
    def get_uv_from_point(self, q, p):
        T_palm_from_tip = self.world_from_finger_tip_base_transformation(q)
        T_tip_from_palm = np.linalg.inv(T_palm_from_tip)
        p_palm = np.ones(4)
        p_palm[:3] = p
        P_tip = np.matmul(T_tip_from_palm, p_palm)
        u,v = self.surface_fit.uv_for_surface_point(P_tip[:3])
        
        return u,v
        
    def number_joints(self):
        return self.number_joints
    
    
    def is_thumb(self):
        return False
    
    def tip_direction_from_last_joint(self, q):
        direction = np.matmul(self.world_from_finger_tip_base_transformation(q), np.array([0,0,1,1]))[:3] - self.get_pos_of_joint(len(q)-1,q)
        return direction
    
    #dead code
    def scale_triangle(self, triangle):
        res_triangle = []
        for p in triangle: 
            point = p*self.SCALE
            res_triangle.append(point)
        return res_triangle

    
    def principal_curvature(self,u,v):
        return self.surface_fit.principal_curvature(u,v)
        
        
    
    def contact_normal(self, q, u,v):
        R= mu.rotation_from_transformation(self.world_from_finger_tip_base_transformation(q))
        n = self.surface_fit.normal(u,v)
        res = np.matmul(R, n)
        return res[:3]
    
    def base_for_contact_point(self, q, u,v):
        R= mu.rotation_from_transformation(self.world_from_finger_tip_base_transformation(q))
        x, y = self.surface_fit.principal_axis(u,v)
        x = np.matmul(R, x)
        y = np.matmul(R, y)
        return x, y
      
    def principal_axes_for_contact_point(self, q, u,v):
        R= mu.rotation_from_transformation(self.world_from_finger_tip_base_transformation(q))
        x, y = self.surface_fit.principal_axis(u,v)
        x = np.matmul(R, x)
        y = np.matmul(R, y)
        return x, y
    
    
    def principal_axes_and_curvature_for_contact_point(self, q, u,v):
        R= mu.rotation_from_transformation(self.world_from_finger_tip_base_transformation(q))
        axis, curvature = self.surface_fit.principal_axis(u,v, True)
        x = np.matmul(R, axis[0])
        y = np.matmul(R, axis[1])
        rotated_axes = [x,y]
        return rotated_axes, curvature
    
    #def get_surface_distance(self, uv_start, uv_target):
    #    res = self.ellipsoid_fit.get_surface_distance(uv_start, uv_target)
    #    return res
    
    def directional_curvature(self, q, u, v, direction):
        R = mu.rotation_from_transformation(self.world_from_finger_tip_base_transformation(q))
        surface_direction = np.matmul(np.transpose(R), direction)
        curvature = self.surface_fit.directional_curvature(u,v, surface_direction)
        return curvature
        
        
        
        
    
    def is_intersecting(self, quv1, finger2, quv2):
        
        dis = self.distance_between_fingers(quv1, finger2, quv2)
        print('shortes distance', dis)
        for i in range(len(dis)):
            if dis[i] < 0:
                return True
        return False
        #find axse og find længde
        #print('finger', self, finger2)
        #print(quv1, quv2)
        finger1 = self
        q1 = quv1[:-2]
        p1_middel =finger1.get_pos_of_joint(len(q1)-2,q1)
        p1_distal =finger1.get_pos_of_joint(len(q1)-1,q1)
        v1_middel_distal = p1_distal- p1_middel
        unit_v1_tip= finger1.tip_direction_from_last_joint(q1)
        v1_tip = unit_v1_tip* finger1.tip_length
        
        p1_
        
        
        
        q2 = quv2[:-2]
        p2_middel =finger2.get_pos_of_joint(len(q2)-2,q2)
        p2_distal =finger2.get_pos_of_joint(len(q2)-1,q2)
        v2_middel_distal = p2_distal- p2_middel
        unit_v2_tip= finger2.tip_direction_from_last_joint(q2)
        v2_tip = unit_v2_tip* finger2.tip_length
        
        shortes_distance = []

        
        #between tip
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_distal, p2_distal, v1_tip, v2_tip))
        
        #pp = PyPlotting.Plot()
        #hp = PyPlotting.HandAndObjectPlot(pp)
        
        
            
        #hp.plot_finger_tip(finger1, quv1)

        #hp.plot_finger_tip(finger2, quv2)
        
            
        #hp.plot_finger_linesegments(finger1, quv1, 'r')
        #hp.plot_finger_linesegments(finger2, quv2, 'g')
        
        #C =[390.44595375608856, -42.39670777179159, 41.87172422420728]
        #D = [382.060334993409, 17.677885093359663, 42.75024725245387]
        #pp.plot_point(C, 'blue')
        #pp.plot_point(D)
        
        
        #pp.plot_vector(p1_distal,v1_tip)
        #pp.plot_vector(p2_distal,v2_tip)
        #pp.show()
            
        #print(shortes_distance[0])
        
    
        
        
        
        
        #between tip andn other: 
        
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_distal, p2_middel, v1_tip, v2_middel_distal))
        
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_middel, p2_distal, v1_middel_distal, v2_tip))
        
        for s in shortes_distance: 
            if s < finger1.tip_radius + finger2.tip_radius:
                return True
        
        return False
    
    
    
    def distance_between_fingers(self, quv1, finger2, quv2):
        #find axse og find længde
        #print('finger', self, finger2)
        #print(quv1, quv2)
        finger1 = self
        q1 = quv1[:-2]
        
        p1_knuckle = finger1.get_pos_of_joint(len(q1)-3,q1)
        p1_middel =finger1.get_pos_of_joint(len(q1)-2,q1)
        p1_distal =finger1.get_pos_of_joint(len(q1)-1,q1)
        
        v1_knuckle_middel = p1_middel- p1_knuckle
        v1_middel_distal = p1_distal- p1_middel
        unit_v1_tip= finger1.tip_direction_from_last_joint(q1)
        v1_tip = unit_v1_tip* finger1.tip_length
        
        
        
        q2 = quv2[:-2]
        p2_knuckle = finger2.get_pos_of_joint(len(q2)-3,q2)
        p2_middel =finger2.get_pos_of_joint(len(q2)-2,q2)
        p2_distal =finger2.get_pos_of_joint(len(q2)-1,q2)
        v2_knuckle_middel = p2_middel- p2_knuckle
        v2_middel_distal = p2_distal- p2_middel
        unit_v2_tip= finger2.tip_direction_from_last_joint(q2)
        v2_tip = unit_v2_tip* finger2.tip_length
        
        shortes_distance = []

        
        #between tip
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_distal, p2_distal, v1_tip, v2_tip))
    
        #between tip andn other: 
        
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_distal, p2_middel, v1_tip, v2_middel_distal))
        
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_middel, p2_distal, v1_middel_distal, v2_tip))
        
        shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_middel, p2_middel, v1_middel_distal, v2_middel_distal))
        
        #shortes_distance.append( mu.find_shortes_distance_between_vectors(p1_knuckle, p2_knuckle, v1_knuckle_middel, v2_knuckle_middel))
        
        
        
        radius = finger1.tip_radius + finger2.tip_radius
        
        for i in range(len(shortes_distance)): 
            shortes_distance[i] = shortes_distance[i]- radius
            
        
        return np.array(shortes_distance)
    
    
    
        
         
    def is_intersecting_test(finger1, quv1, finger2, quv2, obj, rotation, obj_pos, uv_on_obj):
        #find axse og find længde
        
        q1 = quv1[:-2]
        p1 = finger1.get_pos_of_joint(len(q1)-1,q1)
        unit_v1= finger1.tip_direction_from_last_joint(q1)
        v1 = unit_v1* finger1.tip_length
        
        q2 = quv2[:-2]
        p2 = finger2.get_pos_of_joint(len(q2)-1,q2)
        unit_v2= finger2.tip_direction_from_last_joint(q2)
        v2 = unit_v2* finger2.tip_length
        
        print()
        
        
        print(type(finger1), type(finger2))
        
        print('vector', v1, v2)
        print(mu.find_shortes_distance_between_vectors(p1, p2, v1, v2))
        
        pp = PyPlotting.Plot()
        hand_plot = PyPlotting.HandAndObjectPlot(pp)
        hand_plot.plot_fixed_fingers_and_configurations( obj, rotation, obj_pos, [finger1, finger2], [quv1, quv2] ,uv_on_obj, False)
        pp.plot_point(p1)
        pp.plot_vector(p1, v1)
        pp.plot_point(p2)
        pp.plot_vector(p2, v2)
        pp.show()
        
    
        
        
        
        shortes_distance = mu.find_shortes_distance_between_vectors(p1, p2, v1, v2)
        if shortes_distance < finger1.tip_radius + finger2.tip_radius:
            return True
        else: 
            return False
        
    
    
        
        
        
    def quv_joint_from_acts(self, quv,do_shadow_hand_calculation = False):  
        return quv
        
        
    
    def quv_acts_from_joint(self, quv, do_shadow_hand_calculation = False):
        return quv
        

    def number_acts(self, do_shadow_hand_calculation = False):
        return self.number_joints
        
    def q_joint_from_acts(self, q, do_shadow_hand_calculation = False):
        return q
    
    
    
    def joint_displacement_from_end_effector_p_displacement(self, q, u, v, delta_p, do_shadow_hand_calculation = False):
        
        quv = np.concatenate((q, [u,v]))
    
        J = self.jacobian_acts(quv, do_shadow_hand_calculation)

        J = J[:3]
        
        J_inv = np.linalg.pinv(J)
        delta_q = np.matmul(J_inv, delta_p)
        
        
        quv_ac = self.quv_acts_from_joint(quv, self.SHADOW_HAND_CALCULATION)

        q_ac = quv_ac[:-2] + delta_q
        q_suppose = self.q_joint_from_acts(q_ac, self.SHADOW_HAND_CALCULATION)
        
        

        delta_q = self.q_joint_from_acts(delta_q, do_shadow_hand_calculation)
       
        
        
        return q_suppose#delta_q + quv[:-2]
    
    
    
    def joint_displacement_from_end_effector_displacement(self, q, u, v, delta_p, do_shadow_hand_calculation = False):
        
        quv = np.concatenate((q, [u,v]))
    
        J = self.jacobian_acts(quv, do_shadow_hand_calculation)
 
        
        J_inv = np.linalg.pinv(J)
        delta_q = np.matmul(J_inv, delta_p)
        
        
        quv_ac = self.quv_acts_from_joint(quv, self.SHADOW_HAND_CALCULATION)

        q_ac = quv_ac[:-2] + delta_q
        q_suppose = self.q_joint_from_acts(q_ac, self.SHADOW_HAND_CALCULATION)
        
        

        delta_q = self.q_joint_from_acts(delta_q, do_shadow_hand_calculation)
       
        
        
        return q_suppose#delta_q + quv[:-2]
        
        #q = self.q_joint_from_acts(q_ac, self.SHADOW_HAND_CALCULATION)
        #return q
        
        #delta_q = self.q_joint_from_acts(delta_q, do_shadow_hand_calculation)
        
        #return delta_q + quv[:-2]
        '''
        x, y = self.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
        disp = np.zeros(5)
        disp[:3] = delta_p[:3]
        
        Jac = self.jacobian_acts(quv, self.SHADOW_HAND_CALCULATION)
        J = np.empty((5,len(Jac[0])))
        J[:3,:] = Jac[:3]
    
        for i in range(len(Jac[0])):
            J[3][i] = np.dot(Jac [3:,i],x)
            J[4][i] = np.dot(Jac [3:,i],y)
        
        inv_J = np.linalg.pinv(J)
        delta_q = np.matmul(inv_J, disp)
    
        
        
        q_ac = self.quv_acts_from_joint(quv, self.SHADOW_HAND_CALCULATION)

        q_ac = q_ac[:-2] + delta_q

        
        q = self.q_joint_from_acts(q_ac, self.SHADOW_HAND_CALCULATION)

        return q
        '''
        
        '''
        q_current = q
        change = np.linalg.norm(delta_p)
        esp = 10**(-4)
        delta_p_current = delta_p.copy()
        while change > esp:
            quv = np.concatenate((q_current, [u,v]))
            J = self.jacobian_acts(quv, do_shadow_hand_calculation)
            J_inv = np.linalg.pinv(J)
            delta_q = np.matmul(J_inv, delta_p_current)
            q_current = q_current + delta_q
            print(self, 'change_q', delta_q)
            print(self, 'q_current', q_current)
            delta_p_current = delta_p_current - np.matmul(J, delta_q)
            print(self, 'change_p', delta_p)
            print(self, 'change_p', delta_p_current)
            
            
            change = np.linalg.norm(delta_p_current)
            print(self, 'change', change)
            
        delta_q = q_current - q
        
        change = self.q_joint_from_acts(delta_q, do_shadow_hand_calculation)
            
        return change
        '''
        
        
    
class Thumb(Finger):
  
    BODY_NAMES = ["rh_thbase", "rh_thproximal" , "rh_thhub" , "rh_thmiddle", "rh_thdistal"]
    JOINT_NAMES = ["rh_THJ5", "rh_THJ4", "rh_THJ3" ,  "rh_THJ2", "rh_THJ1" ]
    ACTUARTOR_NAMES = ["rh_A_THJ5", "rh_A_THJ4", "rh_A_THJ3" ,  "rh_A_THJ2", "rh_A_THJ1" ]
    PARTH_2_STL = '/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation_Shadow_Hand/Shadow_Hand_21_03_2024/assets/th_distal_pst.obj'
    PARTH_2_JSON = '/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation_Shadow_Hand/Shadow_Hand_21_03_2024/assets/th_distal_pst.json'
    
    
    MESH_NAME = 'rh_TH1_z'
    MODEL_JOINT_INDEX = [15, 16, 17, 18, 19]  
    
   
    #UV_limit = [[np.pi*13/16, np.pi], [np.pi*3/8,np.pi*5/8]] # hardcoded posible change. to better choice. 
    EPS = 10**(-5) 
    #UV_limit = [(0.5,2.7),(1.20096, 1.386 + 1.25)]  #find hvor updated og sæt anden funktion ind. 
    UB1 = 10*np.pi /180 + 0.2
    VB1 = 60*np.pi /180 + 0.1
    UV_limit = [(UB1, np.pi -UB1 ),(VB1, np.pi- 0.3)]  
    
    
    NAME = "Thumb" 
    
    
    
    tourqe_limits = [(-1.5, 1.5), (-1.5, 1.5),(-1.5, 1.5),(-1.5, 1.5),(-1.5, 1.5)]
    
    #move to mesh utilities
    
    def front_mesh_points(self, total_points):
        # "manually" inspected to find which triangels are part of the front. 
        front_mesh_points = []
        l = len(total_points) 
        for p in total_points:
            if p[1]< 4.5 and p[2] >10:
                if p[1]< -7 and p[2]<11:
                    continue
                front_mesh_points.append(p)
        
        
        return front_mesh_points
    
    def front_mesh_vectors(self, total_mesh):
        # "manually" inspected to find which triangels are part of the front. 
        front_mesh_vectors = []
        l = len(total_mesh.vectors) 
        for i in range(0, l):
            triangle= total_mesh.vectors[i]
            add = True
            for p in triangle:
                
                if p[2] < 10.1 or p[1] > 5.25:
                        add = False
            if add:         
                front_mesh_vectors.append(triangle) #self.scale_triangle(triangle))
        return front_mesh_vectors
    
    def surface_fit(self):
        thumb_fit = mu.Thumb_Fit()
        return thumb_fit

    def get_index_in_shadow_hand(self): # only used for plotting
        return 0
    
    def is_thumb(self):
        return True
    
    def tip_direction_from_last_joint(self, q):
        direction = np.matmul(self.world_from_finger_tip_base_transformation(q), np.array([0,0,1,1]))[:3] - self.get_pos_of_joint(len(q)-1,q)
        return direction
    
  
    
    
    
    
    
            
    
    

class MF(Finger):
    
    PARTH_2_STL = '/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation_Shadow_Hand/Shadow_Hand_21_03_2024/assets/f_distal_pst.obj'
    PARTH_2_JSON = '/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation_Shadow_Hand/Shadow_Hand_21_03_2024/assets/f_distal_pst.json'
    
    
    spring_constant = 10.2 #N/m # to do change at some point
    #UV_limit = [[np.pi*17/20, np.pi*23/20], [np.pi*13/40, np.pi*3/20]]# [23.*np.pi/16. ,2.*np.pi]]  #  v ca [3*np.pi/2, 2*np.pi] 0 < pi   2*np- x 
    
    uC = 1.5708 #1.5708
    a = 1.4456-0.1
    vC =  1.50976
    b = 1.49678
   
    UV_limit = [(uC-a, uC+a),(vC-b+0.075, 1.53744)]
    

    tourqe_limits = [(-1.5, 1.5), (-1.5, 1.5),(-1.5, 1.5),(-1.5, 1.5)]


    def translate_qest_to_q_joint(self,qest):
        res_val = np.append(qest, [0])
        #res_val = [qest[i] for i in range(len(qest))]
        #res_val.append(0)
        #print('value', qest)
        #print('res', res_val)
        q_max = self.max_limits[-4]
        if qest[-1] > q_max:
            res_val[-2] = q_max
            res_val[-1] = q_max
            
        '''
        if qest[-1] > self.grid.min_limits[-1]: 
            q_val = self.grid.estimate_value(qest)
            #print('q_val', q_val)
            res_val[-2] = q_val
            res_val[-1] = qest[-1]-q_val
        '''
        #print('res', res_val)
        return res_val
            
    
    
    
    #def get_index_in_shadow_hand(self):  
    #    return -1
    
    def front_mesh_points(self, total_points):
        # "manually" inspected to find which triangels are part of the front. 
        front_mesh_points = []
        l = len(total_points) 
        for p in total_points:
            if p[1]< 4:
                if p[0] > -8.5 and p[0]< 8.5 and p[1] > -4 and p[2]<10:
                    continue 
                if p[0] > -7.5 and p[0]< 7.5 and p[1] > -6 and p[2]<10:
                    continue 
                front_mesh_points.append(p)
        
        
        return front_mesh_points
    
    
    def front_mesh_vectors(self, total_mesh):
        # "manually" inspected to find which triangels are part of the front. 
        front_mesh_vectors = []
        l = len(total_mesh.vectors) 
        for i in range(0, l):
            triangle= total_mesh.vectors[i]
            add = True
            for p in triangle:
                
                if p[2] < 8.5 or p[1] > 4.4:
                        add = False
            
            if add:         
                front_mesh_vectors.append(triangle) #self.scale_triangle(triangle))
        return front_mesh_vectors
    
    
    def surface_fit(self):
        mf_fit = mu.MF_Fit()
        
        return mf_fit

    #somthing wrong in mujoco....therfor this hack
    
    def q_limits_max(self,q_limits):
        q_max= []
        #hardcode_value
        for i in range(len(q_limits)): 
            if i == 1:
                q_max.append(q_limits[i][1])
            else: 
                q_max.append(q_limits[i][1])
        return q_max

    
    
    def quv_joint_from_acts(self, quv, do_shadow_hand_calculation = False):
        if not do_shadow_hand_calculation:
            return quv
        else:       
            new_conf = self.translate_qest_to_q_joint(quv[:-2])
            new_conf = np.append(new_conf, quv[-2:])
            return new_conf
    
    
    def q_joint_from_acts(self, q, do_shadow_hand_calculation = False):
        if not do_shadow_hand_calculation:
            return q
        else:       
            new_conf = self.translate_qest_to_q_joint(q)
            return new_conf
    
    
    def quv_acts_from_joint(self, quv,  do_shadow_hand_calculation = False):
        if not do_shadow_hand_calculation:
            return quv
        else: 
            new_conf = []
            for q in quv[:-3]:
                new_conf.append(q)
            new_conf[-1] += quv[-3]
            new_conf = np.append(new_conf, quv[-2:])
            return new_conf
    
    
    
    
    def number_acts(self, do_shadow_hand_calculation = False):
        if do_shadow_hand_calculation:
            res = self.number_joints -1
        else: 
            res = self.number_joints  
        return res
    
   
    def jacobian_acts(self, quv, do_shadow_hand_calculation = False):
        J = self.jacobian(quv)
        if do_shadow_hand_calculation:
            J_a_to_q = np.zeros((self.number_joints, self.number_joints-1))
            J_a_to_q[0][0] = 1
            J_a_to_q[1][1] = 1
            #do sigmoid instead sigmoid = 1/ (1 )
            aq3 = quv[-4]+quv[-3]
            
            # new
            
            qest = [quv[i] for i in range(len(quv)-4)]
            qest.append(aq3) 
            #q = self.translate_qest_to_q_joint(qest)
            #ja3 = q[-2] / aq3
            #ja4 = q[-1] / aq3 
            
             #old
            ja3 = 1
            ja4 = 0
            if aq3 >= self.max_limits[-4]:
                ja3 = 0
                ja4 = 1
            
            J_a_to_q[2][2] = ja3
            J_a_to_q[3][2] = ja4

            J = np.matmul(J, J_a_to_q)
            
        
        return J
   
    
       
class F1(MF):
  
    BODY_NAMES = ["rh_ffknuckle", "rh_ffproximal" , "rh_ffmiddle", "rh_ffdistal"]
    JOINT_NAMES = ["rh_FFJ4", "rh_FFJ3", "rh_FFJ2" ,  "rh_FFJ1" ]
    ACTUARTOR_NAMES = ["rh_A_FFJ4", "rh_A_FFJ3", "rh_A_FFJ2" ,  "rh_A_FFJ1" ]

    NAME = "F1"  
    
    def get_index_in_shadow_hand(self):   # only used for plotting
        return 1

class F2(MF):
  
    BODY_NAMES = ["rh_mfknuckle", "rh_mfproximal" , "rh_mfmiddle", "rh_mfdistal"]
    JOINT_NAMES = ["rh_MFJ4", "rh_MFJ3", "rh_MFJ2" ,  "rh_MFJ1" ]
    ACTUARTOR_NAMES =["rh_A_MFJ4", "rh_A_MFJ3", "rh_A_MFJ2" ,  "rh_A_MFJ1" ]

    NAME = "F2"  
    
    def get_index_in_shadow_hand(self): # only used for plotting
        return 2

class F3(MF):
  
    BODY_NAMES = ["rh_rfknuckle", "rh_rfproximal" , "rh_rfmiddle", "rh_rfdistal"]
    JOINT_NAMES = ["rh_RFJ4", "rh_RFJ3", "rh_RFJ2" ,  "rh_RFJ1" ]
    ACTUARTOR_NAMES =["rh_A_RFJ4", "rh_A_RFJ3", "rh_A_RFJ2" ,  "rh_A_RFJ1" ]
 
    NAME = "F3"  

    def get_index_in_shadow_hand(self): # only used for plotting
        return 3

class LF(MF):
  
    BODY_NAMES = ["rh_lfmetacarpal", "rh_lfknuckle", "rh_lfproximal" , "rh_lfmiddle", "rh_lfdistal"]
    JOINT_NAMES = ["rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2" ,  "rh_LFJ1" ]
    ACTUARTOR_NAMES =["rh_A_LFJ5", "rh_A_LFJ4", "rh_A_LFJ3", "rh_A_LFJ2" ,  "rh_A_LFJ1" ]

    NAME = "LF"  
    
    tourqe_limits = [(-1.5, 1.5), (-1.5, 1.5),(-1.5, 1.5),(-1.5, 1.5),(-1.5, 1.5)]

    
    def get_index_in_shadow_hand(self): # only used for plotting
        return 4
