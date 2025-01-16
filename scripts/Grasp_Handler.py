
import numpy as np
import Math_Utilities as mu
import scipy.optimize as so
import Physics_Engine
import String_Handler
import PyPlotting
import Constants


class Grasp_Creator:
    
    '''
        Try to find feaseble grasps. 
        If SHADOW_HAND_CALCULATION == True:
            Calculations are such that two outer most joints are coupled if the figers are not the thumb.
        If SHADOW_HAND_CALCULATION == False:
            All actuators are used.
    ''' 
    
    SHADOW_HAND_CALCULATION = True
    
    
    
   
    def create_init_grasps(self, obj, fingers, external_force=np.zeros(3), external_torque=np.zeros(3)):
        obj_pos, obj_axis_angle = obj.random_configuration()
        max_itt = 1000
        i = 0
        while i< max_itt:
            print('random itt', i)
            obj_pos, obj_axis_angle = obj.random_configuration()
            obj_axis_angle =  np.array([0.00, 0, 1.009, obj_axis_angle[3]])
            fingers_config, obj_pos, obj_axis_angle, obj_uv,  is_grasp_found  = self.find_grasp(obj, fingers, obj_pos, obj_axis_angle)  
            if is_grasp_found: 
                is_grasp_feaseble = self.is_grasp_feaseble(fingers, fingers_config, obj_pos, obj_axis_angle, obj_uv, external_force, external_torque, True)
                if is_grasp_feaseble: 
                    self.write_grasp_to_file(fingers, fingers_config, obj_pos, obj_axis_angle, obj_uv)
            i+=1  
        return
   
   
    def find_grasp(self, obj, fingers, obj_pos, obj_axis_angle, fixed_pos_and_roation = True):
        bounds = self.calculate_bounds(obj, fingers, fixed_pos_and_roation)
        initial_guess = [bounds[i][0] + (bounds[i][1]-bounds[i][0])/2 for i in range(len(bounds))]
        
        args = [fingers, obj, obj_pos, obj_axis_angle, fixed_pos_and_roation ]
        #print('args', args)
        #working 
        res =  so.fmin_slsqp(self.objective_func_4_finding_grasp, initial_guess, args = args, f_eqcons=self.eq_constraints_4_finding_grasp,iter=300, iprint=0, full_output= True) #, iprint=2 ) #f_ieqcons= self.ieq_constraints_4_finding_grasp, iprint=2)  
        #test
        #res =  so.fmin_slsqp(self.objective_distance_to_center, initial_guess, args = args, f_eqcons=self.eq_constraints_4_finding_grasp, f_ieqcons=self.ieq_constraints_distance_to_center, iter=300, iprint=1, full_output= True) #, iprint=2 ) #f_ieqcons= self.ieq_constraints_4_finding_grasp, iprint=2)          
        
        
        x = res[0]
        is_grasp_found = (res[3]==0)
        
        fingers_config, obj_pos, obj_angleaxis, obj_uv = self.result_devided_in_catagories(x, args)

        
        return fingers_config, obj_pos, obj_angleaxis, obj_uv, is_grasp_found 
   

    
    def calculate_bounds(self, obj, fingers, fixed_pos):
        bounds = []
        for i in range(len(fingers)):
            finger = fingers[i]
            
            min_limits = finger.min_limits
            max_limits = finger.max_limits
            n = finger.number_joints
            j = 0
            while j < len(max_limits):
                bound = [min_limits[j], max_limits[j]]
                if not finger.is_thumb() and j == n-2 and self.SHADOW_HAND_CALCULATION: #ensure that shadow hand has coupled joints.
                    bound[0] += min_limits[j+1]
                    bound[1] += max_limits[j+1]
                    j+=1
                bounds.append(bound)
                j+= 1
        
        if not fixed_pos: 
            radian_limits = (0,2*np.pi)
            obj_pos_limits = obj.CENTER_LIMITS
            for i in range(len(obj_pos_limits)):
                bounds.append((obj_pos_limits[i][0], obj_pos_limits[i][1]))
            '''
            obj_rotation_axis_limits = (0,1)
            for i in range(3):
                bounds.append(obj_rotation_axis_limits)
            '''
            bounds.append(radian_limits)
            
        return bounds
    
    
    
    def result_devided_in_catagories(self,x, args):
        fingers, obj, obj_pos, obj_angel_axis, fixed_pos = args[:]
        if fixed_pos:  
            obj_pos = obj_pos
            obj_angleaxis = obj_angel_axis
        else: 
            obj_pos = x[-7: -4]
            obj_angleaxis = x[-4:]
        obj_rotation = mu.rotation_from_axises_and_angle(obj_angleaxis[:3], obj_angleaxis[3])
        
        fingers_config = []
        idx = 0
        obj_uv = []
        for i in range(len(fingers)):           
            finger = fingers[i]
            quv, elements_of_solution_used = self.finger_config_from_solution(finger, x, idx)
            idx += elements_of_solution_used
            fingers_config.append(quv)
            p = finger.contact_point(quv[:-2], quv[-2], quv[-1])
            uv = obj.uv_for_surface_point(p, obj_rotation, obj_pos )
            obj_uv.append(uv)
        return fingers_config, obj_pos, obj_angleaxis, obj_uv
    
    def finger_config_from_solution(self, finger, x, start_idx):
        n = finger.number_joints
        if finger.is_thumb() or not self.SHADOW_HAND_CALCULATION:
            quv = x[start_idx:start_idx + n+2]
            
        else: 
            n -= 1
            q = x[start_idx:start_idx + n]
            joint_max = finger.max_limits[n-1]
            if q[-1] <= joint_max :
                quv = np.append(np.append(q[:], [0]), x[start_idx + n: start_idx + n+2])
                
            else:
                q_3 = joint_max  
                q_4 = q[-1] -  joint_max 
                quv = np.append( np.append( q[:-1], [q_3, q_4]), x[start_idx + n: start_idx + n+2])
        elements_of_solution_used = n+2
        return quv, elements_of_solution_used
     
    
    def objective_distance_to_center(self, x, *args):
        fingers = args[0]
        fingers_config, obj_pos, obj_angleaxis, obj_uv = self.result_devided_in_catagories(x, args)
        finger_pos = fingers[0].contact_point(fingers_config[0][:-2], fingers_config[0][-2], fingers_config[0][-1]) 
        return np.linalg.norm(finger_pos - obj_pos)
     
    def ieq_constraints_distance_to_center(self, x, *args):
        fingers = args[0]
        fingers_config, obj_pos, obj_angleaxis, obj_uv = self.result_devided_in_catagories(x, args)
        res = []
        for i in range(len(fingers)):
            finger = fingers[i]
            min_limits = finger.min_limits
            max_limits = finger.max_limits
            for j in range(len(max_limits)):
                res.append(fingers_config[i][j]-min_limits[j])
                res.append(max_limits[j]-fingers_config[i][j])
        return res
    
     
    def objective_func_4_finding_grasp(self, x, *args):
        fingers = args[0]
        fingers_config, obj_pos, obj_angleaxis, obj_uv = self.result_devided_in_catagories(x, args)
        res = 0
        for i in range(len(fingers)):
            finger = fingers[i]
            min_limits = finger.min_limits
            max_limits = finger.max_limits
            for j in range(len(max_limits)):
                denumerator = (max_limits[j]-min_limits[j])
                min_term = (fingers_config[i][j]-min_limits[j])/denumerator
                max_term = (max_limits[j]-fingers_config[i][j])/denumerator
                res += self.smooth_penalty(min_term) + self.smooth_penalty(max_term)
        
        
        
        
        
        #test:
        '''
        for i in range(len(fingers)): 
            for j in range(i+1, len(fingers)):
                dist = fingers[i].distance_between_fingers(fingers_config[i], fingers[j], fingers_config[j])
                for d in dist:
                    res += np.exp(-d)
                    
                    #res -= d #np.min([d,1])   
        '''
        
        return res
        
        
      
    def eq_constraints_4_finding_grasp(self, x, *args):
        fingers, obj = args[:2]
        fingers_config, obj_pos, obj_angleaxis, obj_uv = self.result_devided_in_catagories(x, args)
        obj_rotation = mu.rotation_from_axises_and_angle(obj_angleaxis[:3], obj_angleaxis[3])
        
        
        l = len(fingers)
        res = np.zeros(l*2)
        
        for i in range(l):
            finger = fingers[i]
            uv = obj_uv[i]
            quv = fingers_config[i]
            #normal allign
            contact_normal = finger.contact_normal(quv[:-2],quv[-2],quv[-1])
            obj_normal = obj.normal(uv[0], uv[1], obj_rotation)
            res[2*i] = np.dot(obj_normal, contact_normal)+1
            
            #position is equal.
            p_o = obj.surface_point(uv[0], uv[1], obj_rotation, obj_pos)
            p_f = finger.contact_point(quv[:-2],quv[-2],quv[-1])
            res[2*i+1]= np.linalg.norm(p_o-p_f)       
        return res
    
    
    def smooth_penalty(self, x):
        a = 20
        res = np.exp(-a*x)
        return res
   
   
   
    def is_grasp_feaseble(self, fingers, fingers_config, obj_pos, obj_rotation, obj_uv,  external_force= np.zeros(3), external_torque= np.zeros(3), is_obj_rotation_axis_angle = False):
        is_posible_config = self.is_configuration_within_bounderies(fingers, fingers_config)
        print('is_configuration_within_bounderies', is_posible_config)
        if is_posible_config:
            is_intersecting = self.is_finger_intersecting(fingers, fingers_config)
            print('fingers_is_intersecting', is_intersecting)
            if not is_intersecting: 
                if is_obj_rotation_axis_angle:
                    obj_rotation = mu.rotation_from_axises_and_angle(obj_rotation[:3], obj_rotation[3])
                om = Physics_Engine.Object_Movement(obj_pos, obj_rotation)
                is_physical_posible = Physics_Engine.Physics_Engine.is_grasp_physical_stable(om, fingers, fingers_config, obj_uv, external_force, external_torque )
                print('is_grasp_physical_stable', is_physical_posible)
                if is_physical_posible: 
                    return True
            
        return False
    
    
    def is_finger_intersecting(self, fingers, fingers_config):
        for i in range(len(fingers)):
            f1 = fingers[i]
            for j in range(i+1, len(fingers)):
                f2 = fingers[j]
                intersecting = f1.is_intersecting(fingers_config[i], f2, fingers_config[j])
                if intersecting:
                    return True
        return False
            
    
    
    
    
    def is_configuration_within_bounderies(self, fingers, config):
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
            if not finger.is_within_configuration_limits(quv):
                return False
        return True
    
    
    
    def write_grasp_to_file(self, fingers, config, obj_pos, obj_axis_angle, obj_uv):
        
       
        s = "obj_axis_angle = " +  String_Handler.array2string(obj_axis_angle, 0) +  "\n"
        s += "pos =" + String_Handler.array2string(obj_pos, 0) +  "\n"
        s += "fingers_in_contact = " + String_Handler.array2string([fingers[i].get_index_in_shadow_hand() for i in range(len(fingers))], 0) +  "\n"
        s += "fingers_configuration = " + String_Handler.array2string( config , 1) +  "\n"
        s += "fingers_uv_on_obj_surface = " + String_Handler.array2string(obj_uv, 1) +  "\n\n"
        
        print('printing\n')
        print(s)
        f = open("start_configurations.txt", "a")
        f.write(s)
        f.close()
    
    
    
    
    
    '''
    
    add fingers to grasp not--- corrected include  SHADOW_HAND_CALCULATION 
    '''
    
    
    
    
    
    
    
    
    

   #def just add finger, no uv is included
    def add_finger_to_grasp(self, finger, uv_o, obj, obj_rotation, obj_pos, fingers_in_grasp, fingers_in_graps_quv):
        self.uv_o = np.array([uv_o[0],uv_o[1]]) 
        
        bounds = self.calculate_bounds(obj, [finger], True)
        initial_guess = [bounds[i][0] + (bounds[i][1]-bounds[i][0])/2 for i in range(len(bounds))]
        
        #initial_guess = [finger.min_limits[i] + ((finger.max_limits[i] - finger.max_limits[i])/2.) for i in range(len(finger.max_limits))]
        
        obj_elements = [obj, obj_rotation, obj_pos]
        func_args  = (finger, obj_elements, fingers_in_grasp, fingers_in_graps_quv)
        
        res = so.fmin_slsqp(self.add_finger_min_function, initial_guess, f_eqcons=self.add_finger_eq_constraints, args = func_args, f_ieqcons=self.add_finger_ieq_constraints, iprint=1, full_output= True) #, iprint=2 ) #f_ieqcons= self.ieq_constraints_4_finding_grasp, iprint=2)        
        quv,i = self.finger_config_from_solution(finger, res[0], 0)
        
        is_finger_found = (res[3]==0)
        
        is_within_boundery = self.is_configuration_within_bounderies([finger], [quv])
        
        if is_finger_found and is_within_boundery: 
            is_finger_feaseble = True
        else: 
            is_finger_feaseble = False
        
        return is_finger_feaseble, quv
    
    

    
    def add_finger_min_function(self, x, *args):
        
        finger = args[0]
        obj_elements= args[1]
        quv, i = self.finger_config_from_solution(finger, x, 0)
        res = 0
        min_limits = finger.min_limits
        max_limits = finger.max_limits
        for j in range(len(max_limits)):
            denumerator = (max_limits[j]-min_limits[j])
            min_term = (quv[j]-min_limits[j])/denumerator
            max_term = (max_limits[j]-quv[j])/denumerator
            res += self.smooth_penalty(min_term) + self.smooth_penalty(max_term)
        return res
     

    
    def add_finger_eq_constraints(self, x, *args):
      
        finger = args[0]
        obj_elements= args[1]
        quv, i = self.finger_config_from_solution(finger, x, 0)
        p_f = finger.contact_point(quv[:-2],quv[-2], quv[-1] )
        u,v = obj_elements[0].uv_for_surface_point(p_f, obj_elements[1], obj_elements[2])
        
        p_o = obj_elements[0].surface_point(u,v, obj_elements[1], obj_elements[2]) 
        n_o = obj_elements[0].normal(u,v, obj_elements[1]) 
        
        #positional constraints: 
        p_dif = np.linalg.norm(p_f - p_o)
        
        #normal have to be opersit: 
        n_f = finger.contact_normal(quv[:-2],quv[-2], quv[-1] )
        n_dif = np.array([np.dot(n_o, n_f)+1])   
        res = [p_dif, n_dif]
        return res
    
    
    
    def add_finger_ieq_constraints(self, x, *args):
        '''
        Ensures fingers are not intersecting and joints are with in limits. 
        '''
        finger = args[0]
        obj_elements= args[1]
        fingers_in_grasp = args[2]
        fingers_in_graps_quv = args[3]
        
        quv, i = self.finger_config_from_solution(finger, x, 0)
        dist_between_fingers = self.distance_between_fingers(finger, quv, fingers_in_grasp, fingers_in_graps_quv)
        dist_from_joint_limits = self.distance_from_joint_limits(finger, quv) 
        dist = np.append(dist_between_fingers, dist_from_joint_limits)
        
        return dist 
    
      
     
    def distance_between_fingers(self, finger, quv, fingers_in_grasp, fingers_in_graps_quv):
        dist_between_fingers = []
        for i in range(len(fingers_in_grasp)):
            dist = finger.distance_between_fingers(quv, fingers_in_grasp[i], fingers_in_graps_quv[i])
            dist_between_fingers.append(dist)
        dist_between_fingers = np.array(dist_between_fingers).flatten()
        
        return dist_between_fingers
     
    
    def distance_from_joint_limits(self, finger, quv):
        l = len(quv)
        dist = np.empty(2*l)
        for i in range(l):
            dist[2*i] = quv[i]- finger.min_limits[i]
            dist[2*i+1] =  finger.max_limits[i] - quv[i]
        return dist
   












