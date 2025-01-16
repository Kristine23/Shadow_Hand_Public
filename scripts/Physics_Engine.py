
import numpy as np
import PyPlotting 
import Math_Utilities as mu
import scipy.optimize as so
import Constants


class Object_Movement:
    
    # lige nu bruges den i  simulation. 
    def __init__(self,  position, rotation, acceleration= np.zeros(3), rotation_velocity= np.zeros(3), rotation_acceleration= np.zeros(3)):
       self.pos = position 
       self.acc = acceleration
       self.rot = rotation
       self.rot_vel = rotation_velocity
       self.rot_acc = rotation_acceleration
    
        

class Physics_Engine:
    sliding_friction_coefficient = 0.2
    spin_friction_coefficient = 0.005
    gravity_on_obj = np.zeros(3)
    gravity= np.array([0, 0, -9.81]) #np.array([0, 0, -9.81])
    obj = None
    
    def set_obj(obj):
        Physics_Engine.obj = obj
        Physics_Engine.gravity_on_obj = obj.mass * Physics_Engine.gravity
    
    
    
    
    def is_grasp_physical_stable(  object_movement, fingers, fingers_config, obj_uv, external_force =np.zeros(3), external_torque = np.zeros(3)):
        #print('gravity',Physics_Engine.gravity_on_obj, external_force)
        
        
        x =Physics_Engine.force_solver_new_equations(external_force, external_torque, object_movement, fingers, fingers_config, obj_uv)
        normal_forces, tangential_forces = Physics_Engine.normal_and_tangential_forces_from_solution(x, fingers, fingers_config, obj_uv, object_movement)
        
        total_force = sum(normal_forces) + sum(tangential_forces)
        print('total force', total_force)
        print('expected_force:', Physics_Engine.gravity_on_obj)
        eps= 10**(-6)
        if np.linalg.norm(total_force + Physics_Engine.gravity_on_obj) > eps:
            print('grasp can not be calcultated')
            return False
        
        l = len(fingers)
        for i in range(len(fingers)):
            finger = fingers[i]
            print('constrain (has to be positive)', ( Physics_Engine.sliding_friction_coefficient*x[3*l+i])**2-  np.dot(tangential_forces[i], tangential_forces[i]))
        
        weigth =Physics_Engine.contraint_weight(x)    #rename to sliding constraints        
        is_not_sliding = mu.vector_does_not_contain_negative_values(weigth)
        if is_not_sliding:
            print('grasp is not sliding')
            force_closure =Physics_Engine.does_grasp_have_force_closure( fingers, fingers_config)
            if force_closure: 
                normal_forces, tangent_forces = Physics_Engine.normal_and_tangential_forces_from_solution(x, fingers, fingers_config, obj_uv, object_movement)
                is_possible_to_obtain_forces =Physics_Engine.is_possible_to_obtain_forces(fingers,fingers_config, normal_forces)
                if is_possible_to_obtain_forces: 
                    print('grasp is stable', True)
                    return True 
        print('grasp is stable', False)           
        return False
    
    def is_possible_to_obtain_forces(  fingers, fingers_config, end_effector_forces):
        
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = fingers_config[i]
            force = end_effector_forces[i]
            joint_torque = Rigid_Finger_Physics.joint_torque(finger, quv, force)
            print(finger, joint_torque)
            if not finger.is_within_tourqe_limits(joint_torque):
                return False
            
            
        return True
    
    
    
    def end_effector_force_from_solution(  x, fingers, config, obj_uv, object_movement):
        normal_forces, tangent_forces =Physics_Engine.normal_and_tangential_forces_from_solution(x, fingers, config, obj_uv, object_movement)
        end_effector_forces =Physics_Engine.end_effector_force_from_normal_and_tangent_force(normal_forces, tangent_forces)
        return end_effector_forces
    
    
    def end_effector_force_from_normal_and_tangent_force(  normal_force, tangent_force):
        end_effector_force = []
        for i in range(len(normal_force)):
            force = normal_force[i] + tangent_force[i]
            end_effector_force.append(force)
        return force
    
    
    
    def normal_and_tangential_forces_from_solution(  x, fingers, config, obj_uv, object_movement):
        fingers_normal_forces = [np.zeros(3) for i in range(len(fingers))]
        l = len(fingers)
    
        #tangential_forces = x[:-l]
        
        tangential_forces = []
        for i in range(l):
            force = np.array(x[i*3:i*3+3])
            tangential_forces.append(force)
        #print(tangential_forces)
        
        
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
            uv_o = obj_uv[i]
            fingers_normal_forces[i] = x[-(l-i)]*(- Physics_Engine.obj.normal(uv_o[0], uv_o[1], object_movement.rot)) #finger.contact_normal(quv[:-2], quv[-2], quv[-1])
        
        return fingers_normal_forces, tangential_forces
            
            
    
    
    
    
    

    
    def create_static_object_movement(  position, rotation):
        zeros = np.zeros(3)
        om = Object_Movement()
        om.set_constancts(position, zeros, rotation, zeros, zeros)
        return om
    
    
    def weight_for_system(  object_movement, fingers, fingers_configuration, fingers_uv_on_obj, external_force =np.zeros(3), external_torque = np.zeros(3)):
        #print('gravity',Physics_Engine.gravity_on_obj, external_force)
        
        
        x =Physics_Engine.force_solver_new_equations(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj)
        weigth =Physics_Engine.contraint_weight(x)    
        
        
        
        return weigth
    
    
    
    
    
    
    def fingers_normal_and_tangential_forces(  object_movement, fingers, fingers_configuration, fingers_uv_on_obj, external_force =np.zeros(3), external_torque = np.zeros(3), normal_and_tangential_forces_guess = None):
        if normal_and_tangential_forces_guess == None:
            inital_guess = normal_and_tangential_forces_guess
        else: 
            inital_guess = []
            normals =  normal_and_tangential_forces_guess[0]
            tangents =  normal_and_tangential_forces_guess[1]
            for t in tangents:
                for j in range(3):
                    inital_guess.append(round(t[j],12))
            for n in normals:
                inital_guess.append(round(np.linalg.norm(n),12))
            
        x =Physics_Engine.force_solver_new_equations(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj, inital_guess)
        
        
        
        
        #print('solver x', x)
        #print('solver1 x', x)
        #print()
        #print()
        #x =Physics_Engine.force_solver1(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj)
        #print('solver 1 x', x)
        
        
        '''
        normals_h = [[-0.544762, 0.0405293, -0.0916855], [-0.0590592, -0.04704, 0.0234075], [0.861508, -0.0327959, 0.197128]]
        #print('noramlas')
        #print('rotation', object_movement.rot)
        
        for i in range(3):
            print( normals_h[i]/ np.linalg.norm(normals_h[i]))
            print(-Physics_Engine.obj.normal(fingers_uv_on_obj[i][0], fingers_uv_on_obj[i][1], object_movement.rot))
            #print( Physics_Engine.obj.surface_point(fingers_uv_on_obj[i][0], fingers_uv_on_obj[i][1], object_movement.rot, object_movement.pos)*Constants.SI_SCALE)
            
            
            #print(fingers_uv_on_obj[i])
       
        '''
    
      
        obj_principal_axes_for_contact_points= []
        for uv in fingers_uv_on_obj:
            principal_axes =Physics_Engine.obj.principal_axes(uv[0], uv[1], object_movement.rot)
            obj_principal_axes_for_contact_points.append(principal_axes)
            
        #A, b =Physics_Engine.equations(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj)
        
        #A, b =Physics_Engine.equations(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj)
        #print('res Ax', np.matmul(A, x_henrik))
        
        
        #print('res Ax', np.matmul(A, x))
        
        #x_henrik_test = [-0.0218688, 0.400164, -0.148458, .299962,  -0.426278, 0.250822, 0.500951, 0.594039, 0.905273] 
        
       
       
        fingers_normal_forces = [np.zeros(3) for i in range(len(fingers))]
        l = len(fingers)
    
        #tangential_forces = x[:-l]
        
        tangential_forces = []
        for i in range(l):
            force = np.array(x[i*3:i*3+3])
            tangential_forces.append(force)
        #print(tangential_forces)
        
        
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = fingers_configuration[i]
            fingers_normal_forces[i] = x[-(l-i)]*(- Physics_Engine.obj.normal(fingers_uv_on_obj[i][0], fingers_uv_on_obj[i][1], object_movement.rot)) #finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            
            #print('constrain (has to be positive)', ( Physics_Engine.sliding_friction_coefficient*x[3*l+i])**2-  np.dot(tangential_forces[i], tangential_forces[i]))
            
            
           
            
            
            #print('tangetial', 1/2*np.dot(tangential_forces[i], tangential_forces[i]), '<', ( Physics_Engine.sliding_friction_coefficient*x[-(l-i)])**2)
            
            #print('normal_force', np.linalg.norm(fingers_normal_forces[i]))
            #print('tangential_force', np.linalg.norm(tangential_forces[i]))
            
            
            #print('force finger', i, x[-(l-i)])
       
        
        '''
        for i in range(3):
            ax1, ax2 = obj_principal_axes_for_contact_points[i]
            
            print('axise')
            print(ax1, ax2, normals[i])
            n1 = -Physics_Engine.obj.normal(fingers_uv_on_obj[i][0], fingers_uv_on_obj[i][1], object_movement.rot )
            print(fingers_normal_forces[i])
            print(normals[i])
            print(np.dot(normals[i], c_of[i]))
            print(np.dot(fingers_normal_forces[i], c_of[i]))
            #c = np.linalg.solve([ax1, ax2], normals[i] )
            #print(c)
        
        '''
        
        
        
        
        '''
        obj_principal_axes_for_contact_points= []
        for uv in fingers_uv_on_obj:
            principal_axes =Physics_Engine.obj.principal_axes(uv[0], uv[1], object_movement.rot)
            obj_principal_axes_for_contact_points.append(principal_axes)
            
       Physics_Engine.obj_principal_axes_for_contact_points = obj_principal_axes_for_contact_points
        
        A, b =Physics_Engine.equations1(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj, obj_principal_axes_for_contact_points)
        
        #translate
        #for i in range(l):
            #c = tangential_forces[i]
            #c1 = 
            
        '''
        
        
        
        
        
        #x =Physics_Engine.force_solver2(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj)
        
        
        r = sum(fingers_normal_forces) + sum(tangential_forces)
        
        #print('res', r)
        
        
        
        
        
        return  fingers_normal_forces, tangential_forces
    
    
    def is_forces_correct(object_movement, fingers_normal_forces, tangential_forces):
        force = sum(fingers_normal_forces) + sum(tangential_forces)
        res = force + Physics_Engine.gravity_on_obj - object_movement.acc*Physics_Engine.obj.mass
        if np.linalg.norm(res) > 10**(-6):
            return False
        return True
        
    
    
    def force_solver_new_equations(  external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj,  initial_guess= None ):
        
            
        external_force =Physics_Engine.gravity_on_obj + external_force
        
        obj_principal_axes_for_contact_points= []
        for uv in fingers_uv_on_obj:
            principal_axes =Physics_Engine.obj.principal_axes(uv[0], uv[1], object_movement.rot)
            obj_principal_axes_for_contact_points.append(principal_axes)
            
        
        if initial_guess == None: 
            initial_guess = np.zeros(3*len(fingers))
        else: 
            initial_guess =Physics_Engine.old_to_new_result(initial_guess, obj_principal_axes_for_contact_points)
        
        
        
       
        
       
        
        left_side,right_hand_side =Physics_Engine.equations_new(external_force, external_torque, object_movement, fingers, fingers_configuration, fingers_uv_on_obj, obj_principal_axes_for_contact_points)

  
        l = len(fingers)
        
        
        args = (fingers, fingers_configuration, obj_principal_axes_for_contact_points, left_side,right_hand_side)
        
        res =  so.fmin_slsqp( Physics_Engine.min_function_1, initial_guess, args = args, iter=1000, f_eqcons= Physics_Engine.eq_constraints, f_ieqcons= Physics_Engine.ieq_constrains_new_equations, iprint = 0)
        
        #res = so.fmin_cobyla( Physics_Engine.min_function_1, initial_guess, [ Physics_Engine.ieq_constrains_new_equations,Physics_Engine.eq_constraints ], rhoend=0.00000001)

       
        result = np.empty(4*l) # levn fra gamle solver.
        #res = np.matmul(np.linalg.pinv(A), b)
            
        for i in range(l):
            
            result[3*l + i] = res[2*l+i]
            x, y = obj_principal_axes_for_contact_points[i]
            c_f = res[i*2]*x + res[2*i+1]*y
            
            
            
            for j in range(3):
                result[i*3+j] = c_f[j]
              
              
              
        
        
        return result
    
    

    def old_to_new_result(  res, obj_principal_axes_for_contact_points):
        l = len(obj_principal_axes_for_contact_points)
        result = np.empty(3*l)
        for i in range(l): 
            ax1, ax2 = obj_principal_axes_for_contact_points[i]
            c_of = np.array(res[3*i:3*i+3])
            c1 = np.dot(c_of,ax1)
            c2 = np.dot(c_of,ax1)
            result[2*i] = c1
            result[2*i+1] = c2
            result[2*l+i] = res[3*l+i] 
        return result
        
        
    
    

    
    def ieq_constrains_new_equations(x, *args):
        fingers, fingers_configuration, obj_principal_axes_for_contact_points, left_side,right_hand_side = args
        res = []
        l = len( fingers)
        for i in range(l):
            res.append(x[-(i+1)])
            
        
        l = len( fingers)
        result = np.empty(4*l) # levn fra gamle solver.   
        for i in range(l):
                result[3*l + i] = x[2*l+i]
                ax1, ax2 =obj_principal_axes_for_contact_points[i]
                c_f = x[i*2]*ax1 + x[2*i+1]*ax2
                for j in range(3):
                    result[i*3+j] = c_f[j]
        
        weigth =Physics_Engine.contraint_weight(result) 
        
        eps = 10**(-6)
        
        for i in range(len(res)):
            weigth[i] = weigth[i]- eps
       
        #print(weigth)
        
        res = np.append(res, weigth) 
        
        
        
        #print('positive', res)
            
        return res
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def min_function_1( x, *args):
        fingers = args[0]
        res = 0
        for i in range(len( fingers)):
            res += x[-(1+i)]**2
        return res

    
    
    def eq_constraints(  x, *args):
        fingers, fingers_configuration, obj_principal_axes_for_contact_points, left_side,right_hand_side = args
        
        res = np.matmul( left_side, x)-right_hand_side
        return res
    

    
    
    
    
    
    def contraint_weight(  x):
        l = int(len(x) / 4)
        res = []
        eps = 10**(-14)
        for i in range(l):
            force = x[-(l-i)]
            c = x[3*i:3*i+3]
            #print('force', force)
            #print('c', c)
            dot = np.dot(c,c)
            weigth = (force* Physics_Engine.sliding_friction_coefficient)**2 - dot
            if abs(weigth) < eps:
                weigth = 0
            res.append(weigth)
                
        return res
        
        
    
    
    
    
    
    def equations(  external_force, external_torque, object_movement, fingers, fingers_configuration,  uv_on_object):
        # obs antager altid at normal force er større en 0 og at thumb er finger på index 0. 
    
        l = len(fingers)

        #equation 6
        
        #print('sum', sum_of_normal_forces)
        right_hand_side = np.zeros((l+6))
        right_hand_side[:3] =Physics_Engine.obj.mass*object_movement.acc*Constants.SI_SCALE - external_force 
        
        
        left_side = np.zeros((6+l, 3*l+l))
        #print('right_hand_side', right_hand_side)
        #print('left side', left_side)
        
        
        #print(fingers)
        #print(fingers_configuration)
        
        for j in range(l):
            finger = fingers[j]
            quv = fingers_configuration[j]
            #print(finger, quv)
            #print(quv[2])
            #print(quv[:2])
            
            z_of = -Physics_Engine.obj.normal(uv_on_object[j][0], uv_on_object[j][1], object_movement.rot) #finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            
            for i in range(3):
                left_side[i][i+3*j]= 1
                left_side[i][3*l + j] = z_of[i]
                
            
        #equation 7: z
        
        
        I =Physics_Engine.obj.inertia
        ddtR= object_movement.rot_acc
        dtR = object_movement.rot_vel
        I_mul_dtR = np.matmul(I,dtR)
        result_2_equation = np.matmul(I,ddtR)  + np.cross(dtR, I_mul_dtR)- external_torque
        right_hand_side[3:6] = result_2_equation     
        

        
        for i in range(l):
            finger = fingers[i]
            r_of =Physics_Engine.r_of(uv_on_object[i], object_movement)
            quv = fingers_configuration[i]
            z_of = -Physics_Engine.obj.normal(uv_on_object[i][0], uv_on_object[i][1], object_movement.rot) #finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            
            spin_sign = 0
            '''
            if not mu.is_zero_vector(dtR):
                eps= 10**(-6)
                roation_axis_dif =  np.cross(mu.normalize_vector(dtR), z_of[i])
                if np.linalg.norm(roation_axis_dif) < eps:
                    spin_sign = np.sign(np.dot(dtR, z_of[i]))
            '''
            force_seqond_equation = np.cross(r_of,z_of) - spin_sign* Physics_Engine.spin_friction_coefficient*z_of
            for j in range(3):
                left_side[3+j][3*l+ i] = force_seqond_equation[j]
        
        
        
        for i in range(l):
            finger = fingers[i] 
            # left_side # second equation https://www.whitman.edu/mathematics/calculus_late_online/section14.04.html#:~:text=The%20cross%20product%20is%20A,0%2C%20the%20vector%20points%20down. for cross procuct
            #A×B=⟨a2b3−b2a3,b1a3−a1b3,a1b2−b1a2⟩. A = r_of
            #A×B=⟨a1b2−b1a2,b0a2−a0b2,a0b1−b0a1⟩. A = r_of

            quv = fingers_configuration[i]
            z_of = -Physics_Engine.obj.normal(uv_on_object[i][0], uv_on_object[i][1], object_movement.rot) 
            
            r_of =Physics_Engine.r_of(uv_on_object[i], object_movement)
            
            left_side[3][3*i+2] = r_of[1]
            left_side[3][3*i+1] = -r_of[2]
            
            left_side[4][3*i] = r_of[2]
            left_side[4][3*i+2] = -r_of[0]
            
            left_side[5][3*i+1] = r_of[0]
            left_side[5][3*i] = -r_of[1]
            
            
          
          
            # equation 8
            for k in range(3):
                left_side[6+i][3*i+k] = z_of[k] 
        #print('result_2_equation', result_2_equation)
          
                    
        return left_side, right_hand_side
    
    
    
    
    
    
     
    def equations_new(  external_force, external_torque, object_movement, fingers, fingers_configuration,  uv_on_object, obj_principal_axes_for_contact_points):
        # obs antager altid at normal force er større en 0 og at thumb er finger på index 0. 
    
        l = len(fingers)

        #equation 6
        
        #obj_principal_axes_for_contact_points = [[[0.0741931, 0.997244, 0],[-0.165069, 0.0122808, 0.986206]], 
        #                                         [[-0.623019, 0.782207, 0], [0.231624, 0.184486, 0.955152]], 
        #                                         [[-0.0380404, -0.999276, 0], [-0.222738, 0.0084792, 0.974841]]]
        
        #print('\n\n\n\n')
        #print('equation!!')
        #print(np.linalg.norm(mu.normalize_vector(obj_principal_axes_for_contact_points[0][0])))
        #print(np.linalg.norm(obj_principal_axes_for_contact_points[0][1]))
        
        #print('sum', sum_of_normal_forces)
        right_hand_side = np.zeros((6))
        right_hand_side[:3] =Physics_Engine.obj.mass* object_movement.acc*Constants.SI_SCALE - external_force 
        
       
        
        left_side = np.zeros((6, 3*l))
        #print('right_hand_side', right_hand_side)
        
                
        #print(fingers)
        #print(fingers_configuration)
        
        for j in range(l):
            finger = fingers[j]
            quv = fingers_configuration[j]
            #print(finger, quv)
            #print(quv[2])
            #print(quv[:2])
            x , y = obj_principal_axes_for_contact_points[j]
            
            z_of = -Physics_Engine.obj.normal(uv_on_object[j][0], uv_on_object[j][1], object_movement.rot)    #z_of = finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            for i in range(3):
                left_side[i][2*j] = x[i] 
                left_side[i][2*j+ 1] = y[i]
                left_side[i][2*l + j] = z_of[i]
                
            
        #equation 7: z
        
       
        
        I =Physics_Engine.obj.inertia
        ddtR= object_movement.rot_acc
        dtR = object_movement.rot_vel
        I_mul_dtR = np.matmul(I,dtR)
        result_2_equation = np.matmul(I,ddtR)  + np.cross(dtR, I_mul_dtR)- external_torque
        right_hand_side[3:6] = result_2_equation     
        

        for i in range(l):
            finger = fingers[i]
            r_of =Physics_Engine.r_of(uv_on_object[i], object_movement)
            
            quv = fingers_configuration[i]
            z_of = -Physics_Engine.obj.normal(uv_on_object[i][0], uv_on_object[i][1], object_movement.rot)   #finger.contact_normal(quv[:-2], quv[-2], quv[-1])
            
            spin_sign = 0
            '''
            if not mu.is_zero_vector(dtR):
                eps= 10**(-6)
                roation_axis_dif =  np.cross(mu.normalize_vector(dtR), z_of[i])
                if np.linalg.norm(roation_axis_dif) < eps:
                    spin_sign = np.sign(np.dot(dtR, z_of[i]))
            '''
            
            x , y = obj_principal_axes_for_contact_points[i]
            
            cross_r_of_z_of = np.cross(r_of,z_of) - spin_sign* Physics_Engine.spin_friction_coefficient*z_of
            cross_r_of_x = np.cross(r_of,x) 
            cross_r_of_y = np.cross(r_of,y) 
            
            for j in range(3):
                left_side[3+j][2*l+ i] = cross_r_of_z_of[j]
                left_side[3+j][2*i] = cross_r_of_x[j]
                left_side[3+j][2*i+ 1] = cross_r_of_y[j]
        '''
        for i in range(6):
            s = ''
            for j in range(9):
                s+= "%.5f" % left_side[i][j] + ','
            print(s, '\n')
        
        '''
        #print('right_hand_side', right_hand_side)
        
        
        
        return left_side, right_hand_side
    
    
    
    def r_of(  uv, object_movement ):
        vector = ( Physics_Engine.obj.surface_point(uv[0],uv[1], object_movement.rot,object_movement.pos)-object_movement.pos)*Constants.SI_SCALE
        return vector
        
        
    
    
    
    def does_grasp_have_force_closure(  fingers, configurations):
        l = len(fingers)

        wrenches_transpose = np.empty((4*l, 6))
        for i in range(l):
            f = fingers[i]
            quv = configurations[i]
            F =Physics_Engine.cone_edges_for_finger(f, quv)
            for j in range(4):
                wrenches_transpose[4*i+j] = F[j]
        wrenches = np.transpose(wrenches_transpose)
        
        rank = np.linalg.matrix_rank(wrenches)
        #print('rank', rank)
        if rank < 6: 
            return False
        
        force_clousure_exists =Physics_Engine.does_positive_contain_positive_coeficient_for_form_closure(wrenches)
        
        print('force_clousure_exists', force_clousure_exists)
        
        return force_clousure_exists
        
        
        
        #return True
    
    
    def does_positive_contain_positive_coeficient_for_form_closure(  wrenches):
        
        j = len(wrenches[0])
        c = np.ones(j)
        b = [(1, None) for i in range(j)]
        res = so.linprog(c, A_eq= wrenches, b_eq= np.zeros(6), bounds=b)
        #print(res.x)
        return res.success
    
    
    
    
    
    def cone_edges_for_finger(  finger, quv):
        
        
        p = finger.contact_point(quv[:-2],quv[-2],quv[-1])
        x_of, y_of = finger.base_for_contact_point(quv[:-2],quv[-2],quv[-1]) 
        z_of = finger.contact_normal(quv[:-2], quv[-2], quv[-1])
        
        rotation = np.transpose(np.array([x_of, y_of, z_of]))
    
        u =Physics_Engine.sliding_friction_coefficient  
        cone_edges = [[u, 0, 1], [-u, 0, 1], [0, u, 1], [0, -u, 1]]
        
        F = []
        for i in range(len(cone_edges)):
            edge_for_finger = np.matmul(rotation, mu.normalize_vector(cone_edges[i]))
            moment = np.cross(p,edge_for_finger)
            Fi = np.append(edge_for_finger, moment)
            F.append(Fi)
        
        
        
        
        '''
        pp = PyPlotting.Plot()
        #pp.plot_vector(np.zeros(3), x_of, 'b')
        #pp.plot_vector(np.zeros(3), y_of, 'g')
        pp.plot_vector(np.zeros(3), z_of, 'r')
        
        for i in range(len(cone_edges)):
            pp.plot_vector(np.zeros(3), F[i][:3])
        
        pp.show()
        '''
        
        return F
    
    
class Rigid_Finger_Physics:
    
    def gravity_model(  finger, quv, gravity):
        gravity_force_on_finger = np.zeros(3)
        return gravity_force_on_finger
    
    
    def joint_torque( finger, quv, end_effector_force):
        #print(end_effector_force)
        #print('finger', type(finger), 'tourque', end_effector_force)
        # antager at der allerede er for at opretholde gravity i fingren. 
        #gravity_force_on_finger =Physics_Engine.gravity_model(finger, quv)
        
        end_effector_tourque = np.zeros(3)
        
        #print('force, tourque', end_effector_force, end_effector_tourque)
        
        end_effector_force_tourque = np.append(end_effector_force, end_effector_tourque)
        
        #J =Physics_Engine.finger_jacobian_for_simulation(finger, quv)
        
        #J_v, J_w = finger.linear_and_angualar_jacobians(quv[:-2], quv[-2], quv[-1])
        #J = np.concatenate((J_v, J_w))
        J = finger.jacobian(quv)
        #J =Physics_Engine.finger_jacobian_for_simulation(finger, quv)
        
        J_transpose = np.transpose(J)
        #total_torque_to_withstand = end_effector_force #- gravity_force_on_finger
        joint_tourque =  np.matmul(J_transpose, end_effector_force_tourque)
        #print('end_ecffector_force', end_effector_force_tourque)
        #print('joint, tourque1', joint_tourque * Constants.MM_TO_METERS)
        #print('joint, tourque', joint_tourque*Constants.SI_SCALE)
        #print('joint force', np.matmul(np.linalg.pinv(J_transpose), joint_tourque*Constants.SI_SCALE*Constants.METERS_TO_MM))
        #print(end_effector_force)
        
        
        return joint_tourque *Constants.SI_SCALE
        
    
    def finger_jacobian_for_simulation(  finger, quv):
        J_v, J_w = finger.linear_and_angualar_jacobians(quv[:-2], quv[-2], quv[-1])
        for i in range(len(J_v)):
            for j in range(len(J_v[i])):
                J_v[i][j] = J_v[i][j]
                
        J = np.concatenate((J_v, J_w))
        return J
    
    
    
        
    

class Spring_Finger_Physics:
    
    spring_constant = 2.2 #N/m # to do change at some point
    
    # bruger kun yderste led, intet andet... skal måske overveje at bruge fysik nede i hånden. 
    
    def normal_force(  finger, quv):
        q = quv[:-2]
        u = quv[-2]
        v = quv[-1]
        diffrence = abs(q[-2]-q[-1])
        eps = 10**(-8)
        if diffrence < eps: 
            return np.zeros(3)
        z_f = finger.contact_normal(q,u,v)
        force, compliance_direction =Physics_Engine.compliant_force_and_direction(finger, q,u,v)
        force = force / np.dot(z_f,compliance_direction)
        return force * z_f
    
    
    def compliant_force_and_direction(  finger, q, u,v):
        angle_differnce = q[-2]-q[-1]
        #print('angle', angle_differnce)
        force = np.sin(angle_differnce) *Physics_Engine.spring_constant
        r_f = finger.contact_point(q,u,v) - finger.get_pos_of_joint(len(q)-1,q)
        tip_rotation_axis= finger.get_rotation_axis_of_joint(len(q)-1, q)
        compliance_direction = np.cross(tip_rotation_axis, r_f)       
        compliance_direction = mu.normalize_vector(compliance_direction)  
        return force, compliance_direction
        
