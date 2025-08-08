
import Shadow_Hand
import numpy as np
import Math_Utilities as mu
import PyPlotting
import String_Handler
import scipy.optimize as so



class Arc:
    
    ### Changed the error adjust from 6 eq to 5  for cat models and spin just update that uv is updated from fingers and not directly from curvature. Also we do not take spin into account. rigth now we assume that we do not roll and spin at the same
    
    
    SHADOW_HAND_CALCULATION = True
    
    
    ROTATION_STEP_MAX_STEP = 0.01 # gives 0.1 radians
    POSITION_MAX_STEP_SIZE = 0.1 # give 1 mm chang
    
    ROTATION_DISTANCE_FOR_FINGER_VECTORS = 0.2 #gives the maximum distance canges in one newton step
    MAX_ROLL_DISTANCE = 1 # give roll distance in one newton step. max.  
    
    DISTANCE_TOLERANCE = 0.01 # distance change**2
    ANGLE_TOLERANCE = 0.02 # 
    
    obj = None
    
    
    
    def set_obj(obj):
        Arc.obj = obj
    
    
    def __init__(self):
        self.start_node = None
        self.target_node = None
        self.axis = None
        self.theta = None
        #self.time = None
        self.config_list = [] #quv_estimates, obj_contoints_acts_sorted, obj_pos 
        self.obj_uv_list= []
        self.obj_pos_list = []
        self.obj_rotation_list = []
        
        self.axis = None
    
    
    def test_existens(self, start_node, target_node, time_to_reach_goal=1):
        return self.create_arc(start_node, target_node, time_to_reach_goal, test_existence= True)
        
        
        
        
    
    def create_arc(self, start_node, target_node, time_to_reach_goal=1, test_existence= False):
        
        
        #Print startnode values before and after
        
        #print(self.SHADOW_HAND_CALCULATION)
        
        
        rot_axis, rot_theta = mu.difference_between_rotations(start_node.rotation, target_node.rotation)
        
        
        obj_rotation = start_node.rotation
        obj_pos = start_node.pos
        quv_estimates= start_node.fingers_configuration.copy()
        obj_uv= start_node.fingers_uv_on_obj_surface.copy()
        target_obj_uv = target_node.fingers_uv_on_obj_surface
        
        fingers = start_node.fingers_in_contact
        
        dt_R = (rot_axis*rot_theta)/time_to_reach_goal 
        dt_P = (target_node.pos - obj_pos)/time_to_reach_goal 
        
        
        
        number_steps = self.calculate_number_steps(obj_pos, obj_rotation,  obj_uv, rot_theta, target_node.pos, target_obj_uv, dt_R, dt_P, quv_estimates, fingers, time_to_reach_goal, 1/time_to_reach_goal)
        print('number_steps', number_steps)
        
        h = time_to_reach_goal / number_steps 
        
        
        is_target_reached = False
        rotation_angle = rot_theta / number_steps
        step_rotation = mu.rotation_from_axises_and_angle(rot_axis, rotation_angle)
        

        
        
        if not test_existence:
            self.start_node = start_node
            self.target_node = target_node
            self.axis = rot_axis
            self.theta = rot_theta
            #self.time = time_to_reach_goal
            self.store_step(obj_pos, obj_rotation, quv_estimates, obj_uv) 

        
        number_fingers = len(fingers)

        #test
        #number_steps =1
        roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers = self.roll_direction_and_axises1(obj_pos, obj_rotation,dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv , time_to_reach_goal, 1/time_to_reach_goal, np.zeros((len(fingers),3)))
            
        stable_finger_conf = [quv_estimates[i].copy() for i in range(number_fingers)]
        stable_uv_obj = [obj_uv[i].copy() for i in range(number_fingers)]
        stable_obj_pos = obj_pos.copy()
            
        for j in range(0,number_steps):
            
            #print('number steps', j)
            old_stable_finger_conf = [stable_finger_conf[i].copy() for i in range(number_fingers)]
            old_stable_uv_obj = [stable_uv_obj[i].copy() for i in range(number_fingers)]
            old_stable_obj_pos = stable_obj_pos.copy()
            
            
            stable_finger_conf = [quv_estimates[i].copy() for i in range(number_fingers)]
            stable_uv_obj = [obj_uv[i].copy() for i in range(number_fingers)]
            stable_obj_pos = obj_pos.copy()
            
          
            
            
            
            time_step_scalar = (number_steps / (number_steps-j))/ time_to_reach_goal
            
            roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers = self.roll_direction_and_axises1(obj_pos, obj_rotation,dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv , time_to_reach_goal, time_step_scalar, roll_axes_and_velocity)
            
     
                

            
            
            #roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers = self.roll_direction_and_axises(obj_pos, obj_rotation,dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv, number_few_joint_fingers , time_to_reach_goal, time_step_scalar)
            #print('test2', j, roll_axes_and_velocity)
            #print('test2', roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers)
            
            
            
            #get new values for uv
            #new_obj_uv = self.update_contacts(h,  obj_pos, obj_uv, roll_direction_and_velocity, curvatures_obj, obj_rotation)
            new_finger_uv = self.update_fingers_contacts(h, fingers, quv_estimates,  roll_direction_and_velocity, curvatures_fingers)
            
            
            
            
            #print('test1', j, new_obj_uv, new_finger_uv)
            #euler and update 
            for k in range(len(fingers)):
                finger = fingers[k]
                quv_estimates[k]  = self.euler_step(h, dt_P, dt_R, quv_estimates[k], finger, obj_pos, roll_axes_and_velocity[k]) 
                quv_estimates[k][-2] = new_finger_uv[k][0]
                quv_estimates[k][-1] = new_finger_uv[k][1]
            
            #update
            #new_obj_uv = self.update_contacts(h,  obj_pos, obj_uv, roll_direction_and_velocity, curvatures_obj, obj_rotation)
            
            
            obj_pos= obj_pos + dt_P/number_steps  
            obj_rotation = np.matmul(obj_rotation, step_rotation)
            
            new_obj_uv = self.update_contacts_from_finger(obj_pos, obj_rotation, fingers, quv_estimates)
            
            
            #print('obj_uv_dif', [new_obj_uv[i][0]- new_obj_uv1[i][0] for i in range(3)], [new_obj_uv[i][1]- new_obj_uv1[i][1] for i in range(3)] )
            
            
            
            obj_uv = new_obj_uv
            
            #quv_new, obj_pos = self.error_adjust_new1(quv_estimates, obj_uv, obj_pos, obj_rotation, start_node.fingers_in_contact)
            
            #obj_pos, quv_estimates =  self.solve_error_adjustment(quv_estimates, obj_uv, obj_pos, obj_rotation, number_few_joint_fingers, fingers)
            #dt_P = (target_node.pos - obj_pos)*time_step_scalar
            
            quv_new= self.quv_error_adjust(quv_estimates, obj_uv, obj_pos, obj_rotation, fingers)
            #quv_new = quv_estimates
            #obj_uv = self.update_contacts_from_finger(obj_pos, obj_rotation, fingers, quv_estimates)
            
            quv_estimates = quv_new
            obj_uv = self.update_contacts_from_finger(obj_pos, obj_rotation, fingers, quv_estimates)
            '''
            quv_new1 = self.pos_error_adjust(quv_estimates, obj_uv, obj_pos, obj_rotation, fingers)
            
            for i in range(len(fingers)):
                if fingers[i].number_acts == 3: 
                    quv_new[i] = quv_new1[i]
            '''
            '''
            new_uv = []
            for i in range(len(quv_new)):
                finger = fingers[i]
                quv = quv_new[i] 
                
                
                p_f = finger.contact_point(quv[:-2],quv[-2], quv[-1] )
                u,v = self.obj.uv_for_surface_point(p_f, obj_rotation, obj_pos)
                
                new_uv.append([u,v])

            obj_uv = new_uv
            '''
          
            
            
            
            
            
           
            if test_existence :
                problem_finger_idx = self.step_existence_check(fingers, quv_estimates, obj_uv, obj_rotation, obj_pos, h)                
                if problem_finger_idx >= 0:
                    print('number step reached-----------------------------------------------------------------', j )
                    #self.pp.show()
                    time_reached = j/ number_steps
                    if j > 0:
                        obj_rotation = np.matmul(obj_rotation, np.transpose(step_rotation))
                    #return is_target_reached, time_reached, stable_obj_pos, stable_finger_conf , stable_uv_obj, obj_rotation
                    if j > 1: 
                        obj_rotation = np.matmul(obj_rotation, np.transpose(step_rotation))
                    return is_target_reached, time_reached, old_stable_obj_pos, old_stable_finger_conf , old_stable_uv_obj, obj_rotation
                    
            
            else: 
                self.store_step(obj_pos, obj_rotation, quv_estimates, obj_uv) 
                
                problem_finger_idx = self.step_existence_check(fingers, quv_estimates, obj_uv, obj_rotation, obj_pos, h)
                if problem_finger_idx >= 0:
                   
                    
                    print('huston_we have a problem....::!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', j)
                    print('huston_we have a problem....::!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! problem finger', problem_finger_idx)

                    return
            
            
            
            

            
            
            
        
        
        #self.pp.set_axes()
        #self.pp.show()
        
        
        is_target_reached = True        
        problem_finger_idx = None

        return is_target_reached, 1, stable_obj_pos, quv_estimates, obj_uv, obj_rotation
        
        
        
        
    def calculate_number_steps(self, obj_pos, obj_rotation,  obj_uv, rot_theta, target_pos, target_obj_uv, dt_R, dt_P, quv_estimates, fingers, time_to_reach_goal, time_step_scalar):
        #number steps due to object_change
        number_steps = np.linalg.norm(target_pos-obj_pos)/self.POSITION_MAX_STEP_SIZE

        #number steps due to rotation 
        rotation_angles = []
        for uv in obj_uv:
            p = self.obj.surface_point(uv[0], uv[1], obj_rotation, obj_pos)
            vector_length = np.linalg.norm(p - obj_pos)
            rotation_angle = self.ROTATION_DISTANCE_FOR_FINGER_VECTORS/ vector_length
            rotation_angles.append(rotation_angle)
        step_rotation_angle = np.min(rotation_angles)
        number_steps += rot_theta/ step_rotation_angle
        
        #number steps due to roll
    
        roll_direction =self.roll_direction_and_axises1(obj_pos, obj_rotation, dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv , time_to_reach_goal, time_step_scalar, np.zeros((len(fingers),3)))[0]
        
        max = 0
        itt = 0
        
        for i in range(len(roll_direction)):
            #print(np.linalg.norm(roll_direction[i]))
            #val = np.linalg.norm(roll_direction[i]) / (self.MAX_ROLL_DISTANCE)
            itt =   np.linalg.norm(roll_direction[i]) / (self.MAX_ROLL_DISTANCE) # as we can not roll more than 20 mm pr finger 
            if itt > max:
                max = itt
        
        number_steps += np.min([max, 20]) # as we can not roll more than 20 mm pr finger  
        
        number_steps = np.int_(np.ceil(number_steps))
        return number_steps   
        
        
        
        
        
        
    #ROLL HANDLING
    
    def roll_direction_and_axises(self,obj_pos, obj_rotation,dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv, number_few_joint_fingers , time_to_reach_goal, time_step_scalar):
        #test
        '''
        for i in range(len(fingers)): 
            f= fingers[i]
            quv = quv_estimates[i]
            J_p, J_r = f.linear_and_angualar_jacobians(quv[:-2], quv[-2], quv[-1])
            N = mu.nullspace(J_p)
            print(f,len(N), N)
        '''
        
        roll_direction_five_joints = self.tangent_roll_direction_for_five_joints(obj_rotation, obj_uv, target_obj_uv, number_few_joint_fingers, time_step_scalar )
        
        
        
        for i in range(len(roll_direction_five_joints)):
            roll_direction_five_joints[i] = roll_direction_five_joints[i] / time_to_reach_goal
        
        #print('roll1', roll_direction_five_joints)
            
        roll_axes_five_joints= self.get_roll_axises_and_velocity_five_joints(obj_rotation,quv_estimates, obj_uv, fingers, roll_direction_five_joints, number_few_joint_fingers)
        
        #roll_axes_five_joints= self.test_find_roll_five(obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers)
        
        roll_axes_and_velocity = self.find_roll(obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers)
        #print('roll 2', roll_axes_and_velocity)
            
        for i in range(0, len( roll_axes_five_joints)):
            roll_axes_and_velocity.append(roll_axes_five_joints[i])
        
        
            
        roll_direction_and_velocity,  curvatures_obj, curvatures_fingers  = self.roll_direction_from_axis(roll_axes_and_velocity, quv_estimates, obj_uv, obj_rotation, fingers)
        #roll_direction_and_velocity[number_few_joint_fingers:] = roll_direction_five_joints[:]
        
        #print('roll2', roll_direction_and_velocity[-2:])
        
        
        return roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers
        

    def tangent_roll_direction_for_five_joints(self, obj_rotation, obj_uv, target_obj_uv, number_few_joint_fingers, time_step_scalar):
        
        roll_direction = []
        for i in range(number_few_joint_fingers ,len(obj_uv)):
            current_uv = obj_uv[i]
            target_uv = target_obj_uv[i]
            tangent = self.obj.tangential_direction(current_uv, target_uv, obj_rotation)
            tangent *= time_step_scalar
            
            roll_direction.append(tangent) 
        return roll_direction
    
    
    def get_roll_axises_and_velocity_five_joints(self, obj_rotation,quv_estimates, obj_uv, fingers, roll_direction_five_joints, number_few_joint_fingers ):
       
        roll_velocities = []
        
        for i in range(number_few_joint_fingers ,len(fingers)):
            roll_direction = roll_direction_five_joints[i-number_few_joint_fingers ] 
            roll_velocity = np.linalg.norm(roll_direction)    
            unit_roll_direction = roll_direction / roll_velocity
            finger = fingers[i]
            
            
            quv =quv_estimates[i]
            
            k_f = finger.directional_curvature(quv[:-2], quv[-2], quv[-1], unit_roll_direction)
            uv_o = obj_uv[i] 
            z_f = -self.obj.normal(uv_o[0], uv_o[1], obj_rotation)         
            k_of = self.obj.directional_curvature(uv_o[0], uv_o[1], unit_roll_direction, obj_rotation)
            
            roll_axis = np.cross( unit_roll_direction, z_f)
           
            
            
            roll = roll_velocity*(k_f+ k_of)*roll_axis 
            roll_velocities.append(roll)
            
            
            #test
            '''
            roll1 = roll/(k_f+k_of)
            roll_vel = np.linalg.norm(roll1)
            #print('given dist vs calculated', np.linalg.norm(roll_direction_five_joints[i-number_few_joint_fingers ]), roll_vel)
            
            roll1 = mu.normalize_vector(roll1)
            z_f_matrix  = mu.scew_symetric_matrix_from_vector(-z_f)
            direction_times_curvature = np.matmul(np.linalg.pinv(z_f_matrix), roll1)
            unit_roll_direction = mu.normalize_vector(direction_times_curvature)
            #print('given unit vs calculated unit', mu.normalize_vector(roll_direction_five_joints[i-number_few_joint_fingers ]), unit_roll_direction)
            print('dif', np.linalg.norm(roll_direction_five_joints[i-number_few_joint_fingers ]) - roll_vel, mu.normalize_vector(roll_direction_five_joints[i-number_few_joint_fingers ])- unit_roll_direction)
            '''
        
        return roll_velocities
    
    
    def test_find_roll_five(self, obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers ):
        l = number_few_joint_fingers 
        p = len(fingers)- l
        
        initial_guess = np.array([0.01 for i in range(p*2)]) 
        
        principle_axis =[]
        for i in range(p):
            finger = fingers[l+i]
            quv = quv_estimates[l+i] 
            axes = finger.principal_axes_for_contact_point( quv[:-2], quv[-2],quv[-1])
            principle_axis.append(axes)
        
        args =  (principle_axis, dt_P, dt_R, quv_estimates[l:],obj_pos, fingers[l:])
  
        res =  so.fmin_slsqp(self.test_min_movement_five_test, initial_guess, args= args, iprint = 0)
        
        tangent_vectors = []
        for i in range(p):
            ax1 = principle_axis[i][0]
            ax2 = principle_axis[i][1]
            tangent_vectors.append( (res[i*2]*ax1 + res[i*2+1]*ax2))

        return tangent_vectors


    def test_min_movement_five_test(self, x, *args):
        principle_axis = args[0]
        dt_P = args[1] 
        dt_R = args[2] 
        finger_quv_estimates = args[3] 
        obj_pos = args[4] 
        fingers = args[5]
        res = 0
        l = len(finger_quv_estimates)
        for i in range(l):
            finger = fingers[i]
            ax1 = principle_axis[i][0]
            ax2 = principle_axis[i][1]
            t = (x[i*2]*ax1 + x[i*2+1]*ax2)
            quv_estimates = self.f(dt_P, dt_R, finger_quv_estimates[i], finger, obj_pos, t)
            for j in range(len(quv_estimates)):
                res += quv_estimates[j]**2
        return res
    
    
    
    
    
    def find_roll(self, obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers ):
        
        l = number_few_joint_fingers 
        initial_guess = np.array([0.01 for i in range(l*2)]) 
        
        dRaf_minus_dpz234_values, bf_vectors = self.change_roll(obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers)
        principle_axis =[]
        
        for i in range(l):
            finger = fingers[i]
            quv = quv_estimates[i] 
            axes = finger.principal_axes_for_contact_point( quv[:-2], quv[-2],quv[-1])
            principle_axis.append(axes)
        
        args =  (principle_axis, dRaf_minus_dpz234_values, bf_vectors, dt_P, dt_R, quv_estimates,obj_pos, fingers, number_few_joint_fingers)
  
        res =  so.fmin_slsqp(self.min_movement, initial_guess, args= args, f_eqcons=self.eq_constraint_roll, iprint = 0)
        
        tangent_vectors = []
        for i in range(l):
            ax1 = principle_axis[i][0]
            ax2 = principle_axis[i][1]
            tangent_vectors.append( (res[i*2]*ax1 + res[i*2+1]*ax2))

        return tangent_vectors


    def min_movement(self, x, *args):
        principle_axis = args[0]
        dt_P = args[3] 
        dt_R = args[4] 
        finger_quv_estimates = args[5] 
        obj_pos = args[6] 
        fingers = args[7]
        number_few_joint_fingers = args[8]
        res = 0
        l = number_few_joint_fingers
        for i in range(l):
            finger = fingers[i]
            ax1 = principle_axis[i][0]
            ax2 = principle_axis[i][1]
            t = (x[i*2]*ax1 + x[i*2+1]*ax2)
            quv_estimates = self.f(dt_P, dt_R, finger_quv_estimates[i], finger, obj_pos, t)
            for j in range(len(quv_estimates)):
                res += quv_estimates[j]**2
        return res
    
    
    def eq_constraint_roll(self,x, *args):
        principle_axis = args[0]
        dRaf_minus_dpz234_values = args[1]
        bf_vectors = args[2]
        
        res = []
        
        for i in range(len(principle_axis)):
            ax1 = principle_axis[i][0]
            ax2 = principle_axis[i][1]
            tangent_vector = x[2*i]*ax1 + x[2*i+1]*ax2

            
            eq = dRaf_minus_dpz234_values[i] + np.dot(tangent_vector, bf_vectors[i])
            
            res.append(eq)
        
        return res
    
    
    def change_roll(self, obj_pos, dt_R, dt_P, quv_estimates, fingers, number_few_joint_fingers ): 
        #equation from notes
        l = number_few_joint_fingers
        
        dRaf_minus_dpz234_values = np.empty((l))
        bf_vectors = []
        for i in range(l):
            finger =  fingers[i]
            quv = quv_estimates[i] 
            
            
            z_1 = finger.get_rotation_axis_of_joint(0, quv[:-2])
            z_234 = finger.get_rotation_axis_of_joint(1, quv[:-2])
            #print('quv', quv)
            p_f = finger.contact_point(quv[:-2], quv[-2], quv[-1])
            p_to_object = obj_pos-p_f
            p_f1 = finger.get_pos_of_joint(0,quv[:-2])
            z_f = finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            
            scalar = np.dot(np.cross(z_234,z_1),(p_f-p_f1)) 
                        
            numinator = np.cross(z_234,z_f)*scalar   
            denuminator = np.dot(z_1, np.cross(z_234, z_f))
            bf =  numinator/denuminator
            
            af = np.cross(p_to_object, z_234) + bf
            dRaf = np.dot(dt_R,af)
            
            dRaf_minus_dpz234_values[i] = dRaf- np.dot(dt_P, z_234)
            bf_vectors.append(bf)
        
        return dRaf_minus_dpz234_values, bf_vectors
    
    
    def roll_direction_from_axis(self, roll_axes, quv_estimates, obj_uv, obj_rotation, fingers):
        n_fingers = len(fingers,)
        
        tangent_vectors = []
        cuvatures_object =[]
        cuvatures_fingers = []
        
        for i in range(0,n_fingers):
            finger = fingers[i]
            quv =quv_estimates[i]
            
            roll_ax = roll_axes[i] 
            
            size = np.linalg.norm(roll_ax)
            if size > 10**(-12):
                
                uv_o = obj_uv[i] 
                z_f = -self.obj.normal(uv_o[0], uv_o[1], obj_rotation) 
                z_f_matrix  = mu.scew_symetric_matrix_from_vector(-z_f)
                direction_times_curvature = np.matmul(np.linalg.pinv(z_f_matrix), roll_ax)
                
                
                
                unit_roll_direction = mu.normalize_vector(direction_times_curvature)
                
                k_f = finger.directional_curvature(quv[:-2], quv[-2], quv[-1], unit_roll_direction)
                        
                k_of = self.obj.directional_curvature(uv_o[0], uv_o[1], unit_roll_direction, obj_rotation)
                
                
                roll_direction = direction_times_curvature/(k_f+k_of)
            else:
                roll_direction = np.zeros(3)
                k_f = 0
                k_of = 0
           
            cuvatures_object.append(k_of)
            cuvatures_fingers.append(k_f)
            tangent_vectors.append(roll_direction)
            
        return tangent_vectors, cuvatures_object, cuvatures_fingers
    
    
    
    
    
    def roll_direction_and_axises1(self,obj_pos, obj_rotation,dt_R, dt_P, quv_estimates, obj_uv, fingers, target_obj_uv , time_to_reach_goal, time_step_scalar, old_roll_axises):
        #test
        roll_axes_and_velocity = []
        
        for i in range(len(fingers)): 
            f= fingers[i]
            quv = quv_estimates[i]
            J = f.jacobian_acts(quv, self.SHADOW_HAND_CALCULATION)
            J_p =J[:3]
            J_r = J[3:]
            #J_p, J_r = f.linear_and_angualar_jacobians(quv[:-2], quv[-2], quv[-1])
            J_p_null = mu.nullspace(J_p)
            
            l = len(J_p_null)
            #print(f, l)
            
            if l >= 2: 
                #roll_direction = self.tangent_roll_direction_for_five_joints(obj_rotation, [obj_uv[i]], [target_obj_uv[i]], 0, time_step_scalar )
                #roll = roll_direction[0]/ time_to_reach_goal
                #roll_axis = self.get_roll_axises_and_velocity_five_joints(obj_rotation,[quv_estimates[i]], [obj_uv[i]], [fingers[i]], [roll], 0)[0]
                roll_axis = self.roll_null_space_2_dim_test(obj_pos, obj_rotation, dt_R, dt_P, f, obj_uv[i], quv_estimates[i], J_r, J_p, J_p_null[0], old_roll_axises[i] )
                roll_axis /= time_to_reach_goal
               
            elif l == 1:
                roll_axis = self.roll_null_space_1_dim(obj_pos, obj_rotation, dt_R, dt_P, f, obj_uv[i], quv_estimates[i], J_r, J_p, J_p_null[0] )
                #print('roll 1', roll_axis)
                #roll_axis = self.roll_null_space_1_dim_test(obj_pos, obj_rotation, dt_R, dt_P, f, obj_uv[i], quv_estimates[i], J_r, J_p, J_p_null[0] )
                #roll_axis/=10
                #print('roll 2', roll_axis)
            else:
                roll_axis = self.roll_null_space_0_dim(obj_pos, dt_R, dt_P, f, quv, J_r, J_p) #to do
            #print('roll 1', roll_axis)
            roll_axes_and_velocity.append(roll_axis)
        
        
        #print(roll_axes_and_velocity)
            
        roll_direction_and_velocity,  curvatures_obj, curvatures_fingers  = self.roll_direction_from_axis(roll_axes_and_velocity, quv_estimates, obj_uv, obj_rotation, fingers)
        #roll_direction_and_velocity[number_few_joint_fingers:] = roll_direction_five_joints[:]
        
        #print('roll2', roll_direction_and_velocity)
        
        
        return roll_direction_and_velocity,  roll_axes_and_velocity, curvatures_obj, curvatures_fingers
    
    
    
    
    def roll_null_space_0_dim(self, obj_pos, dt_R, dt_P, finger, quv, J_r, J_p):
        ax1, ax2 = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
        p_f = finger.contact_point(quv[:-2], quv[-2], quv[-2])
        p_dif = p_f[:3] - obj_pos
        dt_p_f = dt_P + np.cross(dt_R, p_dif)  
        dt_q_p = np.matmul(np.linalg.pinv(J_p), dt_p_f)
        dR_p = dt_R - np.matmul(J_r, dt_q_p)
        
        x_dot_dR_P = np.dot(ax1, dR_p)
        y_dot_dR_P = np.dot(ax2, dR_p)
        
        #print('print(x_dot_dR_P)', x_dot_dR_P)
        
        a = -x_dot_dR_P / (np.dot(ax1, ax1))
        b = -y_dot_dR_P / (np.dot(ax2, ax2))
        

        
        t_roll = a*ax1 + b*ax2
        
    
         
        return t_roll
    
    
    
    def roll_null_space_1_dim_test(self, obj_pos, obj_rot, dt_R, dt_P, finger, uv_obj, quv, J_r, J_p, J_p_null):
        ax1, ax2 = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
        p_f = finger.contact_point(quv[:-2], quv[-2], quv[-2])
        p_dif = p_f[:3] - obj_pos
        dt_p_f = dt_P + np.cross(dt_R, p_dif)  
        dt_q_p = np.matmul(np.linalg.pinv(J_p), dt_p_f)
        dR_p = dt_R - np.matmul(J_r, dt_q_p)
        
        x_dot_dR_P = np.dot(ax1, dR_p)
        y_dot_dR_P = np.dot(ax2, dR_p)
        
        left1 = np.dot(ax1, np.matmul(J_r, J_p_null) ) -x_dot_dR_P
        left2 = np.dot(ax2, np.matmul(J_r, J_p_null) ) -y_dot_dR_P
        
        a = left1 / (np.dot(ax1, ax1))
        b = left2 / (np.dot(ax2, ax2))
        
        t_roll = a*ax1 + b*ax2
         
        return t_roll
    
    
    
    def roll_null_space_2_dim_test(self, obj_pos, obj_rot, dt_R, dt_P, finger, uv_obj, quv, J_r, J_p, J_p_null, old_roll):
        
    
        
        
        #initial_guess = np.array([0.01 for i in range(2)]) 
        principle_axis = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
        ax = np.array(principle_axis )
        
    
        initial_guess = np.matmul(np.linalg.pinv(np.transpose(ax)), old_roll)
        args =  [principle_axis, dt_P, dt_R, quv,obj_pos, finger]
        
       
    
        
        
  
        res =  so.fmin_slsqp(self.min_movement_config, initial_guess, args= args, iprint = 0)
        #print('res a, b', res)
        t_roll = res[0]*principle_axis[0] + res[1]*principle_axis[1]
        
        
        return t_roll

    
    
    def roll_null_space_1_dim(self, obj_pos, obj_rot, dt_R, dt_P, finger, uv_obj, quv, J_r, J_p, J_p_null):
        
        
        initial_guess = np.array([0.01 for i in range(2)]) 
        principle_axis = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
           
           
        z_f = finger.contact_normal(quv[:-2], quv[-2], quv[-1])
        p_f = finger.contact_point(quv[:-2], quv[-2], quv[-1])#
        
        
        
        
        #principle_axis = self.obj.principal_axes(uv_obj[0],uv_obj[1], obj_rot)
        
        #z_f = -self.obj.normal(uv_obj[0],uv_obj[1], obj_rot)
        #p_f = self.obj.surface_point(uv_obj[0],uv_obj[1], obj_rot, obj_pos)
        
        
        #print('null', J_p_null)
        
        J_rJ_P_null = np.matmul(J_r, J_p_null)
        
        #print('z_f', z_f)
        
        #print('J_rJ_P_null', J_rJ_P_null)
        
        
        z_f_cross_J_rJ_P_null = np.cross(z_f, J_rJ_P_null)
        
        p_dif = p_f[:3] - obj_pos
        
        dt_p_f = dt_P + np.cross(dt_R, p_dif)  
        dt_q_p = np.matmul(np.linalg.pinv(J_p), dt_p_f)
        dR_p = dt_R - np.matmul(J_r, dt_q_p)
        
        z_f_vector_dot_dR_p = np.dot(z_f_cross_J_rJ_P_null, dR_p)
        
        args =  (principle_axis, dt_P, dt_R, quv,obj_pos, finger, z_f_cross_J_rJ_P_null, z_f_vector_dot_dR_p)
  
        res =  so.fmin_slsqp(self.min_movement_config, initial_guess, args= args, f_eqcons=self.eq_constraint_roll_one_finger, iprint = 0)
        #print('res a, b', res)
        t_roll = res[0]*principle_axis[0] + res[1]*principle_axis[1]
        
        
        return t_roll


    def min_movement_config(self, x, *args):
        principle_axis = args[0]
        dt_P = args[1] 
        dt_R = args[2] 
        quv = args[3] 
        obj_pos = args[4] 
        finger = args[5]

        res = 0
        ax1 = principle_axis[0]
        ax2 = principle_axis[1]
        t = (x[0]*ax1 + x[1]*ax2)
        quv_estimates = self.f(dt_P, dt_R, quv, finger, obj_pos, t)
        for j in range(len(quv_estimates)):
            res += quv_estimates[j]**2
        return res
    
    
    def eq_constraint_roll_one_finger(self,x, *args):
        ax1, ax2 = args[0]
        z_f_cross_J_rJ_P_null = args[6]
        z_f_vector_dot_dR_p = args[7]
        
        t_roll = (x[0]*ax1 + x[1]*ax2)
        
        res = z_f_vector_dot_dR_p + np.dot(z_f_cross_J_rJ_P_null, t_roll)        
        return res
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    
    #Euler Step

    def euler_step(self, h, dt_P, dt_R, finger_quv_estimates, finger, obj_pos, roll_axis_and_velocity):
        q_adition = h*self.f(dt_P, dt_R, finger_quv_estimates, finger, obj_pos, roll_axis_and_velocity)
        #print(finger, q_adition)
        
        aquv_estimates = finger.quv_acts_from_joint(finger_quv_estimates, self.SHADOW_HAND_CALCULATION)
        #print(aquv_estimates)
        #print(np.pad(q_adition, [(0, 2)], mode='constant', constant_values=0))
        res = aquv_estimates + np.pad(q_adition, [(0, 2)], mode='constant', constant_values=0)
        
        new_finger_quv_estimates = finger.quv_joint_from_acts(res, self.SHADOW_HAND_CALCULATION)
            
        return new_finger_quv_estimates
    
    
    def f(self, dt_p, dt_R, quv, finger, obj_pos, roll):  #virkelig dårligt navn
        
        p_f = finger.contact_point(quv[:-2], quv[-2],quv[-1])
        x, y = finger.base_for_contact_point(quv[:-2], quv[-2],quv[-1])
        
        p_dif = p_f[:3] - obj_pos
        rigth_side = np.empty((5))  
        rigth_side[:3] = dt_p + np.cross(dt_R, p_dif)        ##ny
        rigth_side[3] = np.dot((dt_R+roll),x)
        rigth_side[4] = np.dot((dt_R+roll),y)  
        
        
        J = self.create_kinamtic_matrix(finger, quv, x, y)
        
        J_pinv = np.linalg.pinv(J)
        
        
        
        res = np.matmul(J_pinv, rigth_side)
        
        #if type(finger) == Shadow_Hand.Thumb:
        #    print(res)
        
        
        return res
 
    
    def create_kinamtic_matrix(self,finger, quv, x,y): ## J
        q = quv[:-2]
        u = quv[-2]
        v = quv[-1]
        #J_p, J_r = finger.linear_and_angualar_jacobians(q, u, v)
        
        #len return as somthing differnet.
        Jac = finger.jacobian_acts(quv, self.SHADOW_HAND_CALCULATION)
        J = np.empty((5,len(Jac[0])))
        J[:3,:] = Jac[:3]
    
        for i in range(len(Jac[0])):
            J[3][i] = np.dot(Jac [3:,i],x)
            J[4][i] = np.dot(Jac [3:,i],y)
        return J  
        
        
    
    
    
    #shadow_hand_functions() 
        
        
        
        
    #UPDATE FUNCTIONS
      
    



 
    def update_obj_pos_and_contacts(self, h, dt_P, obj_pos, obj_uv, number_steps, roll_dircetion_and_velocity, curvature_o, obj_rotation): 
        updated_obj_pos= obj_pos + dt_P/number_steps  
        l = len(obj_uv)
        updated_contacts = []
        for i in range(l):
            uv_o = obj_uv[i]
            z_of = self.obj.normal(uv_o[0], uv_o[1], obj_rotation)
            t_roll = np.linalg.norm(roll_dircetion_and_velocity[i])
            unit_direction = roll_dircetion_and_velocity[i]/t_roll
            obj_contact_point = self.obj.surface_point(uv_o[0], uv_o[1], obj_rotation, obj_pos)
            estimate_pos_after_roll = mu.estimation_end_point_of_tangent_vector_on_surface(obj_contact_point, unit_direction, t_roll*h, curvature_o[i], z_of)
            u,v  = self.obj.uv_for_surface_point(estimate_pos_after_roll, obj_rotation, obj_pos)
            updated_contacts.append([u,v])
        return updated_obj_pos, updated_contacts
    
    
    
    def update_contacts(self, h, obj_pos, obj_uv, roll_dircetion_and_velocity, curvature_o, obj_rotation): 
        l = len(obj_uv)
        updated_contacts = []
        for i in range(l):
            uv_o = obj_uv[i]
            t_roll = np.linalg.norm(roll_dircetion_and_velocity[i])
            if t_roll > 10**(-12):
                z_of = self.obj.normal(uv_o[0], uv_o[1], obj_rotation)
                unit_direction = roll_dircetion_and_velocity[i]/t_roll
                obj_contact_point = self.obj.surface_point(uv_o[0], uv_o[1], obj_rotation, obj_pos)
                estimate_pos_after_roll = mu.estimation_end_point_of_tangent_vector_on_surface(obj_contact_point, unit_direction, t_roll*h, curvature_o[i], z_of)
                u,v  = self.obj.uv_for_surface_point(estimate_pos_after_roll, obj_rotation, obj_pos)
            else: 
                u,v = uv_o
            updated_contacts.append([u,v])
        return updated_contacts

    
    def update_contacts_from_finger(self, obj_pos, obj_rotation, fingers, quv_estimates):
        new_uv = []
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = quv_estimates[i]
            contact_point = finger.contact_point(quv[:-2], quv[-2], quv[-1])
            u,v = self.obj.uv_for_surface_point(contact_point, obj_rotation, obj_pos)
            new_uv.append([u,v])
        return new_uv
    
    
    def update_fingers_contacts(self, h, finger_acts_sorted, quv_estimates, roll_direction_and_velocity, curvatures_fingers):
        new_uv = []
        for i in range(len(finger_acts_sorted)):
            finger = finger_acts_sorted[i]
            q = quv_estimates[i][:-2]
            u =  quv_estimates[i][-2]
            v =  quv_estimates[i][-1]
            t_roll = np.linalg.norm(roll_direction_and_velocity[i])
            if t_roll > 10**(-12):
                unit_direction = roll_direction_and_velocity[i]/t_roll
                
                z_f = finger.contact_normal(q,u,v)
                contact_point = finger.contact_point(q,u,v)
                
                estimate_pos_after_roll = mu.estimation_end_point_of_tangent_vector_on_surface(contact_point, unit_direction, t_roll*h, curvatures_fingers[i], z_f)
                
                u,v =  finger.get_uv_from_point(q, estimate_pos_after_roll)
            new_uv.append([u,v])
        return  new_uv

















    def under_five_act_error_and_pos(self, fingers_in_contact, quv_estimates, obj_uv, obj_pos, obj_rotation):
    
        l = len(fingers_in_contact)
        
        n_j = sum([f.number_acts(self.SHADOW_HAND_CALCULATION) for f in fingers_in_contact])
        
        equation_matrix = np.zeros((5*l,n_j+3))
        right_hand_side = np.zeros(5*l)
        
        column_idx = 0
        
        for i in range(l):
            finger = fingers_in_contact[i]
            
            quv = quv_estimates[i]
            x,y = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
            J = self.create_kinamtic_matrix(finger,quv,x,y)
            j_v = finger.number_acts(self.SHADOW_HAND_CALCULATION)

            #print(J)
            row_idx = 5*i
            equation_matrix[row_idx:row_idx+3, -3:] = -1.*np.identity(3)
            equation_matrix[row_idx:row_idx+5, column_idx:column_idx+j_v] = J
            column_idx += j_v
            uv_o = obj_uv[i]
            
            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(finger, quv, uv_o, obj_pos, obj_rotation)
            
            right_hand_side[row_idx:row_idx+3] = pos_displacement
            right_hand_side[row_idx+3] = np.dot(rotation_displacement, x)                         
            right_hand_side[row_idx+4] = np.dot(rotation_displacement, y)
            
        p_inv = np.linalg.pinv(equation_matrix)
        res = np.matmul(p_inv, right_hand_side)
        idx = 0
        new_quv = []
        for i in range(l):
            finger = fingers_in_contact[i]
            quv = quv_estimates[i]
            j_v = finger.number_acts(self.SHADOW_HAND_CALCULATION)
            quv_adj = res[idx:idx+j_v]
            adjust = finger.q_joint_from_acts(quv_adj, self.SHADOW_HAND_CALCULATION)
            adjust = np.pad(adjust, [(0, 2)], mode='constant', constant_values=0)
            #print(finger, adjust)
            new_quv.append(quv + adjust)
            #print(quv)
            #print(quv + adjust)
            #print()
            
            idx += j_v

        #print('pos adjust', res[-3:])
        new_pos = obj_pos + res[-3:]
       
        return new_quv, new_pos
       
    
    

    def five_act_fingers_quv_adjustment(self, fingers_in_contact, quv_estimates, obj_uv, obj_pos, obj_rotation):
        new_quv = []
        l = len(fingers_in_contact)
        for i in range(l):
            finger = fingers_in_contact[i]
            quv =quv_estimates[i]

            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(finger, quv,obj_uv[i], obj_pos, obj_rotation)
            #print('displacement', displacement)
            displacement = np.concatenate((pos_displacement,rotation_displacement))
            J = finger.jacobian(quv)
            q_displacement = np.matmul(np.linalg.pinv(J), displacement)
            #print('q_displacement', q_displacement)
            q_displacement = np.pad(q_displacement, [(0, 2)], mode='constant', constant_values=0)
            new_quv.append(quv +  q_displacement)
        return new_quv
    


    def error_adjust_new(self, quv_estimates, obj_uv, obj_pos, obj_rotation, fingers_in_contact):
        
        fingers_in_contact_34 = []
        quv_estimates_34 = []
        obj_uv_34 = []
        
        fingers_in_contact_5 = []
        quv_estimates_5= []
        obj_uv_5 = []
        for i in range(len(fingers_in_contact)): 
            f = fingers_in_contact[i]
            if f.number_acts() < 5: 
                fingers_in_contact_34.append(f)
                quv_estimates_34.append(quv_estimates[i])
                obj_uv_34.append(obj_uv[i])
            else: 
                fingers_in_contact_5.append(f)
                quv_estimates_5.append(quv_estimates[i])
                obj_uv_5.append(obj_uv[i])
                
        
        new_quv_34, new_obj_pos = self.under_five_act_error_and_pos(fingers_in_contact_34, quv_estimates_34, obj_uv_34, obj_pos, obj_rotation)
        
        #print('new_quv_34', new_quv_34)
        #print('new_pos', new_obj_pos)
        
        new_quv_5 = self.quv_error_adjust(quv_estimates_5,obj_uv,obj_pos, obj_rotation, fingers_in_contact_5)  

        #print('new_quv_5', new_quv_5)

        i_34 = 0
        i_5 = 0
        new_quv = []
        for i in range(len(fingers_in_contact)): 
            f = fingers_in_contact[i]
            if f.number_acts() < 5: 
                new_quv.append(new_quv_34[i_34])
                i_34 += 1
            else: 
                new_quv.append(new_quv_5[i_5])
                i_5 +=1
       
        #print('quv_dif', [new_quv[i]-quv_estimates[i] for i in range(len(quv_estimates))])
        #print('pos_dif', obj_pos-new_obj_pos)
       
        return new_quv, new_obj_pos
       
       
    def error_adjust_new1(self, quv_estimates, obj_uv, obj_pos, obj_rotation, fingers_in_contact):
        
        number_max_act = 6
        
        fingers_in_contact_34 = []
        quv_estimates_34 = []
        obj_uv_34 = []
        
        fingers_in_contact_5 = []
        quv_estimates_5= []
        obj_uv_5 = []
        for i in range(len(fingers_in_contact)): 
            f = fingers_in_contact[i]
            if f.number_acts() < number_max_act: 
                fingers_in_contact_34.append(f)
                quv_estimates_34.append(quv_estimates[i])
                obj_uv_34.append(obj_uv[i])
            else: 
                fingers_in_contact_5.append(f)
                quv_estimates_5.append(quv_estimates[i])
                obj_uv_5.append(obj_uv[i])
                
        
        new_quv_34, new_obj_pos = self.under_five_act_error_and_pos(fingers_in_contact_34, quv_estimates_34, obj_uv_34, obj_pos, obj_rotation)
        
        new_quv_5 = self.five_act_fingers_quv_adjustment(fingers_in_contact_5, quv_estimates_5, obj_uv_5, obj_pos, obj_rotation)       


        i_34 = 0
        i_5 = 0
        new_quv = []
        for i in range(len(fingers_in_contact)): 
            f = fingers_in_contact[i]
            if f.number_acts() < number_max_act: 
                new_quv.append(new_quv_34[i_34])
                i_34 += 1
            else: 
                new_quv.append(new_quv_5[i_5])
                i_5 +=1
       
        return new_quv, new_obj_pos
       
       
       
       
            
        return new_quv, new_pos



    #ERROR FUCTIONS: 
    def solve_error_adjustment(self, quv_estimates, obj_uv, obj_pos, obj_rotation, number_few_joint_fingers, finger_acts_sorted):
        '''ATTENTION, has to be updated if less than four joints fingers are added'''
        l = number_few_joint_fingers
        quv_under_five_finger_and_obj_error_adjustment = self.four_joint_fingers_error_adjustment(quv_estimates, obj_uv, obj_pos, obj_rotation,number_few_joint_fingers, finger_acts_sorted)             ### synes error adjustment er of            
        obj_pos_change = quv_under_five_finger_and_obj_error_adjustment[-3:]
        new_obj_pos = obj_pos + obj_pos_change
        

        new_quv = []
        for k in range(l):
            new_quv.append(quv_estimates[k] + np.pad(quv_under_five_finger_and_obj_error_adjustment[k*4:k*4+4], [(0, 2)], mode='constant', constant_values=0))


        
        
        
        quv_adj = self.five_joint_fingers_quv_adjustment(quv_estimates, obj_uv, new_obj_pos, obj_rotation, number_few_joint_fingers,finger_acts_sorted) 
   
        for k in range(len(quv_adj)):
            new_quv.append(quv_estimates[l+k] + quv_adj[k])
        

        return new_obj_pos, new_quv
    
    

        
    
    
    

    def four_joint_fingers_error_adjustment(self, quv_estimates, obj_uv, obj_pos, obj_rotation,number_few_joint_fingers, finger_acts_sorted):
        equation_matrix, right_hand_side = self.four_joint_fingers_error_eqations(quv_estimates, obj_uv, obj_pos, obj_rotation, number_few_joint_fingers,finger_acts_sorted)
        equation_matrix_pinv = np.linalg.pinv(equation_matrix)
        res = np.matmul(equation_matrix_pinv, right_hand_side)
        return res
    
    
    def four_joint_fingers_error_eqations(self, quv_estimates, obj_uv, obj_pos, obj_rotation, number_few_joint_fingers,finger_acts_sorted):
        #equations from notes
        l = number_few_joint_fingers
        
        equation_matrix = np.zeros((5*l,4*l+3))
        right_hand_side = np.zeros(5*l)
        
        
        for i in range(l):
            finger = finger_acts_sorted[i]
            
            quv = quv_estimates[i]
            x,y = finger.principal_axes_for_contact_point(quv[:-2], quv[-2],quv[-1])
            J = self.create_kinamtic_matrix(finger,quv,x,y)

            #print(J)
            row_idx = 5*i
            column_idx = 4*i
            equation_matrix[row_idx:row_idx+3, -3:] = -1.*np.identity(3)
            equation_matrix[row_idx:row_idx+5, column_idx:column_idx+4] = J
            
            uv_o = obj_uv[i]
            
            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(finger, quv, uv_o, obj_pos, obj_rotation)
            
            #print(finger,  np.linalg.norm(obj_contact_point - p_f))
            #print(finger, obj_contact_points_acts_sorted[i]- p_f)
            right_hand_side[row_idx:row_idx+3] = pos_displacement
            right_hand_side[row_idx+3] = np.dot(rotation_displacement, x)                         
            right_hand_side[row_idx+4] = np.dot(rotation_displacement, y)
            
            #test
            #equation_matrix[row_idx+3:row_idx+5, column_idx:column_idx+4] = np.zeros((2,4))
            #right_hand_side[row_idx+3] = 0#np.dot(cross_product, x)                         
            #right_hand_side[row_idx+4] = 0 #np.dot(cross_product, y)
            
       
            
        return equation_matrix, right_hand_side
    
    
    
    def five_joint_fingers_quv_adjustment(self, quv_estimates, obj_uv, obj_pos, obj_rotation,  number_few_joint_fingers,finger_acts_sorted):
        quv_adjustment = []
        l = number_few_joint_fingers
        for i in range(l, len( finger_acts_sorted)):
            finger = finger_acts_sorted[i]
            quv =quv_estimates[i]

            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(finger, quv,obj_uv[i], obj_pos, obj_rotation)
            #print('displacement', displacement)
            displacement = np.concatenate((pos_displacement,rotation_displacement))
            J = finger.jacobian(quv)
            q_displacement = np.matmul(np.linalg.pinv(J), displacement)
            #print('q_displacement', q_displacement)
            quv_adjustment.append(np.pad(q_displacement, [(0, 2)], mode='constant', constant_values=0))
        return quv_adjustment
    

    def pos_and_rotational_displacement(self, finger, quv, uv_o, obj_pos, obj_rotation):
        
        finger_position = finger.contact_point(quv[:-2],quv[-2],quv[-1])
        correct_position = self.obj.surface_point(uv_o[0], uv_o[1], obj_rotation, obj_pos)
        
        pos_displacement = correct_position - finger_position
        z_f = finger.contact_normal(quv[:-2],quv[-2], quv[-1])
        z_of = self.obj.normal(uv_o[0], uv_o[1], obj_rotation)
        rotation_displacement = np.cross(z_f, -z_of)
        return  pos_displacement, rotation_displacement
    
    
    
    
    
    #NEW ERROR AS NOT ENOUGHT VARIABLES ABOVE (UNLESS LITTEL FINGER PART OF IT as WELL)
    
    def pos_error_adjust(self, quv_estimates, obj_uv, obj_pos, obj_rotation, finger_acts_sorted):
        adjusted_pos = []
        for i in range(len(finger_acts_sorted)):
            f = finger_acts_sorted[i]
            quv = quv_estimates[i]
            J = f.jacobian_acts(quv,self.SHADOW_HAND_CALCULATION)[:3]
            inv_J = np.linalg.pinv(J)
            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(f, quv,obj_uv[i], obj_pos, obj_rotation)
            q_displacement = np.matmul(inv_J, pos_displacement)
            q_displacement = f.quv_joint_from_acts(q_displacement, self.SHADOW_HAND_CALCULATION)
            quv = quv + np.pad(q_displacement, [(0, 2)], mode='constant', constant_values=0)
            '''
            x, y = f.principal_axes_for_contact_point(quv[:-2], quv[-2], quv[-1])
            pos_displacement, rotation_displacement = self.pos_and_rotational_displacement(f, quv,obj_uv[i], obj_pos, obj_rotation)
            J_p = f.jacobian_acts(quv,self.SHADOW_HAND_CALCULATION)[:3]
            J_r = f.jacobian_acts(quv,self.SHADOW_HAND_CALCULATION)[3:]
            J = np.empty((3,len(J_p[0])))
            print(J_p)
            
            z = f.contact_normal(quv[:-2], quv[-2], quv[-1])
            for i in range(f.number_acts(self.SHADOW_HAND_CALCULATION)):
                J[0][i] = np.dot(J_p[:,i],z)
                J[1][i] = np.dot(J_r[:,i],x)
                J[2][i] = np.dot(J_r[:,i],y)
            
            disp = [np.dot(pos_displacement, z), np.dot(rotation_displacement, x), np.dot(rotation_displacement, y)]
            q_displacement = np.matmul(np.linalg.pinv(J), disp)
            
            q_displacement = f.quv_joint_from_acts(q_displacement, self.SHADOW_HAND_CALCULATION)
            new_quv = quv + np.pad(q_displacement, [(0, 2)], mode='constant', constant_values=0)
            '''
            new_quv = quv
            adjusted_pos.append(new_quv)
        return adjusted_pos
    
    
    
    def quv_error_adjust(self, quv_estimates, obj_uv, obj_pos, obj_rotation, finger_acts_sorted):
        
        adjusted_quv= []
        for i in range(len(finger_acts_sorted)):
            f = finger_acts_sorted[i]
            quv = quv_estimates[i]
            #print(quv)
            #J = f.jacobian_acts(quv,self.SHADOW_HAND_CALCULATION)
            x, y = f.principal_axes_for_contact_point(quv[:-2], quv[-2], quv[-1])
            J = self.create_kinamtic_matrix(f, quv, x, y)
            #print(f)
            #print(J)
            
            #rank = np.linalg.matrix_rank(J)
            
            #print('rank',  rank)
            
            
            inv_J = np.linalg.pinv(J)
            #pos_and_rot_displ = self.pos_and_rot_displacement(f, quv, obj_uv[i], obj_pos, obj_rotation)#, x, y)
    
            #J = f.jacobian_acts(quv,self.SHADOW_HAND_CALCULATION)
            
        
            #inv_J = np.linalg.pinv(J)
        
        
            pos_and_rot_displ = self.pos_and_rot_displacement1(f, quv, obj_uv[i], obj_pos, obj_rotation, x, y)
            
            #print('pos_and_rot_displ', pos_and_rot_displ)
            q_displ = np.matmul(inv_J, pos_and_rot_displ) 
            
            
            #print(f,  q_displ )
            
            aquv_estimates = f.quv_acts_from_joint(quv, self.SHADOW_HAND_CALCULATION)

        
            
            res = aquv_estimates + np.pad( q_displ, [(0, 2)], mode='constant', constant_values=0)
        
            new_quv = f.quv_joint_from_acts(res, self.SHADOW_HAND_CALCULATION)
        
            adjusted_quv.append(new_quv)
            
        return adjusted_quv
            
        
    
    
    def pos_and_rot_displacement(self, finger, quv, uv_o, obj_pos, obj_rotation) : #, x, y):
        finger_position = finger.contact_point(quv[:-2],quv[-2],quv[-1])
        correct_position = self.obj.surface_point(uv_o[0], uv_o[1], obj_rotation, obj_pos)
        
        pos_displacement = correct_position - finger_position
        z_f = finger.contact_normal(quv[:-2],quv[-2], quv[-1])
        z_of = self.obj.normal(uv_o[0], uv_o[1], obj_rotation)
        rot_displacement = np.cross(z_f, -z_of)
        
        displacement = np.append(pos_displacement, rot_displacement)#[np.dot(rot_displacement, x),  np.dot(rot_displacement, y)])
        return displacement
    
    def pos_and_rot_displacement1(self, finger, quv, uv_o, obj_pos, obj_rotation, x, y):
        finger_position = finger.contact_point(quv[:-2],quv[-2],quv[-1])
        correct_position = self.obj.surface_point(uv_o[0], uv_o[1], obj_rotation, obj_pos)
        
        pos_displacement = correct_position - finger_position
        z_f = finger.contact_normal(quv[:-2],quv[-2], quv[-1])
        z_of = self.obj.normal(uv_o[0], uv_o[1], obj_rotation)
        rot_displacement = np.cross(z_f, -z_of)
        
        displacement = np.append(pos_displacement, [np.dot(rot_displacement, x),  np.dot(rot_displacement, y)])
        return displacement
    
    
    
    
    
    def quv_adjust_using_optimizer(self, finger, quv, obj_rot, obj_pos):
        
        
        quv_acts=  finger.quv_acts_from_joint(quv, self.SHADOW_HAND_CALCULATION)
        args = [finger, obj_rot, obj_pos, quv_acts]
        initial_guess = quv_acts.copy()
        
        res = so.fmin_slsqp(self.min_displacement, initial_guess, f_eqcons=self.displcaement_constraints, args = args, iprint=0)
        
        new_quv = finger.quv_joint_from_acts(quv_acts + res, self.SHADOW_HAND_CALCULATION)  
        #print('differnece ', finger.quv_joint_from_acts(quv_acts) - new_quv)
        
        
        
        
        
        return new_quv
    
    
    def min_displacement(self, x, *args):

        res = 0
        for i in range(len(x)):
            res += x[i]**2
        return res
    
    
    
    def displcaement_constraints(self, x,*args):

        finger = args[0]
        obj_rot = args[1]
        obj_pos = args[2]
        quv = finger.quv_joint_from_acts(args[3] + x, self.SHADOW_HAND_CALCULATION)  
        
        
        
        
        p_f = finger.contact_point(quv[:-2],quv[-2], quv[-1] )
        u,v = self.obj.uv_for_surface_point(p_f, obj_rot, obj_pos)
        
        
        p_o = self.obj.surface_point(u,v, obj_rot, obj_pos) 
        n_o = self.obj.normal(u,v, obj_rot) 
        
        #positional constraints: 
        p_dif = np.linalg.norm(p_f - p_o)
        
        #normal have to be opersit: 
        n_f = finger.contact_normal(quv[:-2],quv[-2], quv[-1] )
        n_dif = np.array([np.dot(n_o, n_f)+1])   
        res = [p_dif, n_dif]
        return res
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #STEP FUCTION
    

    
    def step_existence_check(self,finger_acts_sorted, quv_estimates, obj_uv, obj_rotation, obj_pos, h):
        l = len(quv_estimates)
        for i in range(l):
            finger = finger_acts_sorted[i]
            quv =  quv_estimates[i]
            within_limits = finger.is_within_configuration_limits(quv)
            if not within_limits: 
                print('finger', type(finger),'is not with in joint limits')
                return finger.get_index_in_shadow_hand()
            #print(finger, quv, obj_uv[i], obj_rotation, obj_pos)
            within_tolarance = self.is_within_position_tolarance(finger, quv, obj_uv[i], obj_rotation, obj_pos, h)
            
            if not within_tolarance: 
                print('finger', type(finger), 'is not within tolerance dif : ' )
                return finger.get_index_in_shadow_hand()
            within_angle_tolarance = self.is_within_angle_tolarance(finger, quv, obj_uv[i], obj_rotation, obj_pos)
            if not within_angle_tolarance:
                print('finger', type(finger), 'is not within angle tolarance')
                return finger.get_index_in_shadow_hand()
        # is finger within bounderies: 
        # differnece between contact point and real point. 
        return -1
    
    def is_within_position_tolarance(self,finger, quv, obj_uv, obj_rotation, obj_pos, h):
        finger_contact = finger.contact_point(quv[:-2], quv[-2], quv[-1])
        obj_contact_point = self.obj.surface_point(obj_uv[0], obj_uv[1], obj_rotation, obj_pos)
        #print(finger, np.linalg.norm(finger_contact-obj_contact_point))
        #print(3*h**2)
        if np.linalg.norm(finger_contact-obj_contact_point) >  self.DISTANCE_TOLERANCE:
            return False
        else: 
            return True
    
    
    def is_within_angle_tolarance(self, finger, quv, obj_uv, obj_rotation, obj_pos):
        f_normal = finger.contact_normal(quv[:-2], quv[-2], quv[-1])
        o_normal = self.obj.normal(obj_uv[0], obj_uv[1], obj_rotation)
        angle = np.arccos(np.dot(f_normal, -o_normal))
        if angle > self.ANGLE_TOLERANCE:
            return False
        else:
            return True
    
    
    
        
        
    def store_step(self, obj_pos, obj_rotation, quv_estimates, obj_uv): 
        self.config_list.append(quv_estimates.copy())
        self.obj_uv_list.append(obj_uv.copy())
        self.obj_pos_list.append(obj_pos.copy())
        self.obj_rotation_list.append(obj_rotation.copy())
        

    
    
    
    
    #UTILITY FUNKTIONS:
        
    
    
    
    
    def reverse_arc(self):
        
        new_start_node = self.start_node
        self.start_node = self.target_node
        self.target_node = new_start_node
    
        self.axis, self.theta = mu.difference_between_rotations(self.start_node.rotation, self.target_node.rotation)
    
        self.config_list = Arc.reverse_list(self.config_list)
        self.obj_uv_list= Arc.reverse_list(self.obj_uv_list)
        
        
        self.obj_pos_list = Arc.reverse_list(self.obj_pos_list)
   
        self.obj_rotation_list = Arc.reverse_list(self.obj_rotation_list)
     
    
    
    def reverse_list(list_to_reverse):
        res = []
        for i in range(len(list_to_reverse)-1, -1, -1):
            res.append(list_to_reverse[i])

        return res
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def obj_contact_points_acts_sorted(self, uv_acts_sorted, obj_rotation, obj_pos):
        obj_contact_points = []
        for i in range(len(uv_acts_sorted)):
            p = self.obj.surface_point(uv_acts_sorted[i][0], uv_acts_sorted[i][1], obj_rotation, obj_pos )
            obj_contact_points.append(p)
        return obj_contact_points    
    #after arc has been created they can be used to find configurations: 
    
    def get_obj_rotation(self, t, duration_of_arc):
        #print( self.theta*t/duration_of_arc)
        
        rotation_change = mu.rotation_from_axises_and_angle(self.axis, self.theta*t/duration_of_arc)
        rotation = np.matmul(self.start_node.rotation, rotation_change)
        return rotation
    
    
    
    
    def get_obj_pos(self, t, duration_of_arc):
        x, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_pos_list)
        #print(x)
        #print(t)
        #print(time_list)
       
        res = mu.Interpolation().parabolic_blend(t, x[0], x[1], x[2], time_list[0], time_list[1], time_list[2], blend_t)
        return res
    
    
    
    
    
    
    def get_obj_acceleration(self, t, duration_of_arc):
        x, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_pos_list)
        res = mu.Interpolation().parabolic_blend_acceleration(t, x[0], x[1], x[2], time_list[0], time_list[1], time_list[2], blend_t)
        #res = np.zeros(3)
        return res
    
    
    def get_obj_rotation_velocity(self, t, duration_of_arc):
        #print( self.theta*t/duration_of_arc)
        rotation_change = self.axis* self.theta/(duration_of_arc*(len(self.obj_pos_list)-1))
        #rotation_change  = np.zeros(3)
        return rotation_change
    
    
    def get_obj_rotation_acceleration(self, t, duration_of_arc):
        '''
        cheading say we reach velocity instentatly.
        '''
        #print( self.theta*t/duration_of_arc)
        if t < duration_of_arc:
            x, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_pos_list)
            max_velocity = self.axis* self.theta/(duration_of_arc*(len(self.obj_pos_list)-1))
            accelration  = mu.Interpolation().parabolic_blend_acceleration(t, np.zeros(3), max_velocity/2, max_velocity, 0, duration_of_arc/2, duration_of_arc, blend_t)
            
            
            
            #accelration =  self.axis* self.theta/(duration_of_arc*(len(self.obj_pos_list)-1))
        elif t > (len( self.obj_pos_list)-2)*duration_of_arc:
            accelration =  - self.axis* (self.theta/(duration_of_arc*(len(self.obj_pos_list)-1)))
        else: 
            accelration = np.zeros(3)
        #print('acceleration', accelration)
        accelration = np.zeros(3)
        return accelration
    
        
    
  
    def get_obj_contact_pos(self, t, duration_of_arc):
        uv_elements, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_uv_list)
        rotation, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_rotation_list)
        pos, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.obj_pos_list)
        
        res = []
        points = self.get_points_from_uv(uv_elements, rotation, pos)
        for i in range(len(points[0])):
            p = mu.Interpolation().parabolic_blend(t,points[0][i], points[1][i], points[2][i], time_list[0], time_list[1], time_list[2], blend_t )
            res.append(p)
        return res

    
    def get_points_from_uv(self, uv_list, rotation_list, pos_list):
        array_of_points = []
        for i in range(len(uv_list)):
            points = []
            for j in range(len(uv_list[i])):
                p = self.obj.surface_point(uv_list[i][j][0], uv_list[i][j][1], rotation_list[i], pos_list[i])
                points.append(p)
            array_of_points.append(points)
        return array_of_points 
    
    def get_configuration(self, t, duration_of_arc):
        
        x, time_list, blend_t = self.get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.config_list)
        
        res = []
        #print('x', x)
        #print('idx')
        #print(len(self.config_list))
        #print(self.config_list)
        #print('number_conf', len(x[0]), x[0])
        #print('t', t)
        #print('time_list', time_list)
        #print('number config', len(x))
        for i in range(len(x[0])):
            finger_res = []
            for j in range(len(x[0][i])):
               
                q = mu.Interpolation().parabolic_blend(t,x[0][i][j], x[1][i][j], x[2][i][j], time_list[0], time_list[1], time_list[2], blend_t)
                #q = x[0][i][j]
                finger_res.append(q)
                
            res.append(finger_res)
            
        #print('res', res[1])
        return res
    
    
   
    
    
    '''
    def get_configuration_start_node_finger_sorted(self, t, duration_of_arc):
        conf = self.get_configuration(t, duration_of_arc)
        res_conf =  self.get_one_conf_start_node_finger_sorted(conf)
        return res_conf
    
    def get_one_conf_start_node_finger_sorted(self, conf):
        res_conf = [[] for i in range(len(conf))]
        conf_idx = self.start_node.finger_idx_joint_sorted()
        #print('start_node fingers', self.start_node.fingers_in_contact)
        #print(conf_idx)
        #print('conf', conf)
        for i in range(len(conf)):
            idx = conf_idx[i]
            res_conf[idx] = conf[i]
        #print(res_conf)
        return res_conf
        
    
    
    def get_configurations_start_node_finger_sorted(self):
        res_conf = []
        for i in range(len(self.config_list)):
            conf = self.config_list[i]
            res_conf.append(self.get_one_conf_start_node_finger_sorted(conf))
        return res_conf
    '''

    
    
    
        
    def get_conf_and_time_for_parablic_blend(self, t, duration_of_arc, array):
        
        number_config = len(array)
        time_interval = duration_of_arc/ (number_config-1)
        blend = time_interval/20
        idx_middel = np.int_(np.floor(t/time_interval ))
        
        if t >= (idx_middel+1)*time_interval - blend/2:
            idx_middel += 1
        
        elements = []
        
        if idx_middel == 0:
            elements.append(array[0])
            elements.append(array[0])
            elements.append(array[1])
        
        elif idx_middel >= len(array)-1:
            elements.append(array[-2])
            elements.append(array[-1])
            elements.append(array[-1])
            
        else:  
            elements.append(array[idx_middel-1])
            elements.append(array[idx_middel])
            elements.append(array[idx_middel+1])

            
        idx_list = [idx_middel-1, idx_middel, idx_middel+1]
        time_list = []
        for i in range(3):
            time_list.append(idx_list[i]*time_interval)  
        
        return elements, time_list, blend
    

    
        
    def get_three_idexes_and_time(self, t, duration_of_arc):
        number_config = len(self.obj_pos_list)
   
        
        #for i in range(number_config):
        #    print(self.config_list[i][0])
        time_interval = duration_of_arc/ (number_config-1)

       
        idx_middel = np.int_(np.floor(t/time_interval ))
        if t >= duration_of_arc: 
            idx_middel = number_config -2
        
        
        #print(t)
        #print(time_interval)
        #print(t/time_interval)
        #print(idx_middel)
    
        
        
        if idx_middel == 0: 
            idx_list = [0,1,2]
        else: 
            idx_list = [idx_middel-1, idx_middel, idx_middel+1]
        
        time_list = []
        for i in range(3):
            time_list.append(idx_list[i]*time_interval)  
            
        #print('idx', idx_list)
        #print(len(self.obj_pos_list))
        #print(len(self.config_list))
        #print('time', time_list)      
        return idx_list, time_list
    
    
    
    
    
    
    def euler_step_test(self, h, dt_P, dt_R, finger_quv_estimates, finger, obj_pos, roll_axis_and_velocity, obj_rotation,  obj_uv):
        q_step_adition = h*self.f_test(dt_P, dt_R, finger_quv_estimates, finger, obj_pos, roll_axis_and_velocity)
        
        print(["{:.4f}".format(finger_quv_estimates[k]) for k in range(len(finger_quv_estimates))])
        
        pp = PyPlotting.Plot()
        hand_plot = PyPlotting.HandAndObjectPlot(pp)

        
          
        
        hand_plot.plot_fixed_fingers_and_configurations( self.obj, obj_rotation, obj_pos, [finger], [finger_quv_estimates], [obj_uv], True)
        
       
        
        
        print('estimate', finger_quv_estimates)
        print('add', q_step_adition)
        new_finger_quv_estimates= finger_quv_estimates + np.pad(q_step_adition, [(0, 2)], mode='constant', constant_values=0)
        #print(new_finger_quv_estimates)
        #print(finger.contact_point(finger_quv_estimates[:-2],finger_quv_estimates[-2], finger_quv_estimates[-1]))
        #print(finger.contact_point(new_finger_quv_estimates[:-2],new_finger_quv_estimates[-2], new_finger_quv_estimates[-1]))
        return new_finger_quv_estimates
    
    
    def f_test(self, dt_p, dt_R, quv, finger, obj_pos, roll):  # virkelig dårligt navn
        
        #print(quv)
        
        p_f = finger.contact_point(quv[:-2], quv[-2],quv[-1])
        
        #print(finger, p_f)
       
        
        
        x, y = finger.base_for_contact_point(quv[:-2], quv[-2],quv[-1])
        J = self.create_kinamtic_matrix_test(finger, quv, x, y)
        
        
        J_pinv = np.linalg.inv(J)
        p_dif = p_f[:3] - obj_pos
        rigth_side = np.empty((5))  
        rigth_side[:3] = dt_p + np.cross(dt_R, p_dif)        ##ny
        rigth_side[3] = np.dot((dt_R+roll),x)
        rigth_side[4] = np.dot((dt_R+roll),y)  
        res = np.matmul(J_pinv, rigth_side)
        
        #print(res)
        return res
    
    
    
    def create_kinamtic_matrix_test(self,finger, quv, x,y): ## J
        q = quv[:-2]
        u = quv[-2]
        v = quv[-1]
        J_p, J_r = finger.linear_and_angualar_jacobians(q, u, v)
        J = np.empty((5,len(q)))
        J[:3,:] = J_p 
        for i in range(len(q)):
            J[3][i] = np.dot(J_r[:,i],x)
            J[4][i] = np.dot(J_r[:,i],y)
        return J



    def to_string(self):
        

        s = "config_acts_sorted = " + String_Handler.array2string(self.config_list, 2) + "\n"
        s += "obj_uv = " + String_Handler.array2string(self.obj_uv_list, 2) + "\n"
        s += "obj_pos = " + String_Handler.array2string(self.obj_pos_list, 1) + "\n"
        s += "obj_rotation = " + String_Handler.array2string(self.obj_rotation_list, 2) + "\n"
        #s += "time = " + String_Handler.array2string([self.time],0) + "\n"
        
        
        return s
        
    
    def arc_from_string(string):
        
        arc = Arc()
        #arc.start_node = start_node
        
        arrays = String_Handler.multiple_arrays_from_string(string)
        
        arc.config_list = arrays[0] #quv_estimates, obj_contoints_acts_sorted, obj_pos 
        arc.obj_uv_list= arrays[1]
        new_array = []
        for v in arrays[2]:
            new_array.append(np.array(v))
        arc.obj_pos_list = new_array
        
        
        arc.obj_rotation_list = arrays[3]
        #arc.time = arrays[4][0]
        
        return arc
        
    
        
        
    
    
    
    
    
    
    
    
    
  
    
    def richardson_test_of_existens(self, obj_start_rotation, obj_pos, finger_idx_acts_sorted, config_acts_sorted, uv_obj_acts_sorted, number_steps):
       
        res = []
        for i in range(3):
            #cal_res = is_target_reached, obj_rotation, obj_pos, quv_estimates, obj_uv, problem_finger_idx
            cal_res = self.create_configurations1(obj_start_rotation, obj_pos, finger_idx_acts_sorted, config_acts_sorted, uv_obj_acts_sorted, number_steps)
            #print('conf', cal_res[3])
            res.append(cal_res)
            number_steps = number_steps*2
    
        dt_R = (self.axis*self.theta)
        dt_P = self.target_node.pos - self.start_node.pos
        
        print('dt_p', dt_P)
        for k in range(len(finger_idx_acts_sorted)):
            finger = self.start_node.fingers_in_contact[finger_idx_acts_sorted[k]]
            function_res = []
            for i in range(3):
                quv = res[i][3]
                print(i, quv)
                function_result = self.f(dt_P, dt_R, quv[k], finger, obj_pos, [np.zeros(3)])
                function_res.append(function_result)
            a1 = function_res[0]
            a2 = function_res[1]
            a3 = function_res[2]
                    
            richardson = np.linalg.norm(a1-a2) / np.linalg.norm(a2-a3)
            richardson_pos = np.linalg.norm(a1[:3]-a2[:3]) / np.linalg.norm(a2[:3]-a3[:3])
            richardson_angle = np.linalg.norm(a1[3:]-a2[3:]) / np.linalg.norm(a2[3:]-a3[3:])
            print('finger', type(finger))
            print('richardson............................................................................', richardson) 
            print('richardson.pos........................................................................', richardson_pos) 
            print('richardson.angle......................................................................', richardson_angle) 
            print('dif', np.linalg.norm(a2-a3))
            
        return True
    
    
    
    
    
    
    
    
    
    
    
    
    


