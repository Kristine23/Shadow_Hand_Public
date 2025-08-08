

import mujoco
import mujoco.viewer
import time
import numpy as np
import Physics_Engine
import Constants
import Math_Utilities as mu
import Object_Handler
import Arc
import Shadow_Hand
import PyPlotting
import Node
import String_Handler


class Mujoco_Simulator_New: 
    
    
    MAX_FORCE_DIFF = -0.0
    MIN_FORCE_DIFF = -0.1
    
    
    def __init__(self):   
        
        self.mj_model = mujoco.MjModel.from_xml_path('/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation/Shadow_Hand_21_03_2024/world.xml')    
        
        self.gravity = self.mj_model.opt.gravity[:]
        print('gravity ', self.gravity)
        self.mj_data = mujoco.MjData(self.mj_model)  
        self.joint_change_pr_sec = 1.57*0.1
        self.joint_tolerance = 0.02
    
    
    def set_shadow_hand_and_obj(self, shadow_hand, obj):
        self.shadow_hand = shadow_hand
        self.obj = obj
        self.item_id, self.finger_id = self.item_and_finger_id()
    
    
    
    def item_and_finger_id(self):
        l = len(self.mj_data.contact)
        item_id = self.mj_model.body(self.obj.name).id
        finger_id = []
        for f in self.shadow_hand.fingers:
            finger_id.append(self.mj_model.body(f.BODY_NAMES[-1]).id)
        
        return item_id, finger_id
    
    
    def init_configurations(self, obj, init_node):        
        obj_pos = init_node.pos
        obj_rotation = init_node.rotation
            
        
        self.init_fingers_config(obj_pos, init_node)
        #self.set_initial_obj_pos_and_rotation(obj_pos, obj_rotation)
        self.set_obj_pos_and_rotation(obj, obj_pos, obj_rotation)
        
        #hand sinking ISSUE and
        #mass ISSUE!!
        #print(self.mj_model)
        for i in range(len(self.mj_model.body_mass)):
            if i == obj.body_id: 
                continue
            c = 0
            
            self.mj_model.body_mass[i] *= c
            self.mj_model.body_inertia[i, :] *= c
        
        #no contact issue
        
        val = 0.00#2
        vec = np.array([val, val,val])
        self.mj_model.geom(obj.name ).size = self.mj_model.geom(obj.name ).size + vec
        names = ["item:site1", "item:site2", "item:site3", "item:site4"]
        for n in names: 
            self.mj_model.site(n).size = self.mj_model.site(n ).size+ vec
        
    def set_initial_obj_pos_and_rotation(self, obj_pos, rotation):
        body_id = self.mj_model.body('item1').id
        mj_quat = mu.rotation_to_mujoco_quat(rotation)
        pos= [obj_pos[i]*Constants.SI_SCALE for i in range(3)]
        #print('pos', pos)
        #pos = [0,0,0]
        self.mj_model.body_pos[body_id]= pos 
        self.mj_model.body_quat[body_id] = mj_quat
        
        
        
    
    def set_obj_pos_and_rotation(self, obj, obj_pos, rotation):
        mj_quat = mu.rotation_to_mujoco_quat(rotation)
        pos= [obj_pos[i]*Constants.SI_SCALE for i in range(3)]
        #print('pos', pos)
        #pos = [0,0,0]
        self.mj_model.body_pos[obj.body_id]= pos 
        self.mj_model.body_quat[obj.body_id] = mj_quat
        self.mj_data.joint(obj.name).qpos[:3] = pos
        self.mj_data.joint(obj.name).qpos[3:] = mj_quat
        
       
        
    
    def init_fingers_config(self, obj_pos, init_node: Node.Node):
        om = Physics_Engine.Object_Movement(obj_pos, init_node.rotation)
        config = self.get_fingers_config(init_node.fingers_in_contact, init_node.fingers_configuration, init_node.fingers_uv_on_obj_surface, False, om)
        for i in range(len(self.shadow_hand.fingers)): 
            finger = self.shadow_hand.fingers[i]
            quv = config[i]
            for j in range(0,len(finger.model_ctrl_indexes)):
                self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= quv[j]
                self.mj_data.qpos[finger.model_joint_indexes[j]]= quv[j]
                

    def get_fingers_config(self, fingers, fingers_config, fingers_uv_on_obj_surface, only_config = True, om = None):        
        sh_fingers = self.shadow_hand.fingers
        config = []
        for i in range(0, len(sh_fingers)):
            
            finger = sh_fingers[i]
            if finger in fingers: 
                idx = fingers.index(finger)
                quv =fingers_config[idx]
            else: 
                quv = np.zeros(finger.number_joints)

                match i:
                    case 1:
                        quv[0] = finger.min_limits[0]
                    case 2: 
                        quv[0] = config[1][0]
                    case 3:
                        quv[0] = -config[2][0]
                    case 4: 
                        quv[1] = finger.min_limits[1]
            config.append(quv)    
        return config





    def simulate_path(self, path, obj):
        
        
        duration_of_arc = 2 # can be better
        duration_of_finger_change = 4
        
        start_delay = 50
        end_delay = 50
        
        self.testnode = path[0]
        
        gravity_val = 0.0
        #self.mj_model.opt.gravity[:] = [0., 0., -9.81*gravity_val] # disable gravity
        
        
        element_handled = False
        
        
        pos_displacement = [np.zeros(3) for i in range(len(self.shadow_hand.fingers))]
        
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            viewer.sync()
           

            
            self.init_forces(viewer, path[0], pos_displacement)
            
            force = self.get_forces(path[0])
            
            
            
            diff = self.force_difference(force, path[0])
            print('dif1', diff)
            
            self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
            self.mujoc_update(viewer)
            diff = self.force_difference(force, path[0])
            print('dif2', diff)
            #self.simulation_before_movement(4, viewer, path[0], pos_displacement)
            self.idle_simulation(2, viewer)
    
            diff = self.force_difference(force, path[0])
            print('dif3', diff)
            print('idel')
            #self.simulation_before_movement(200, viewer, path[0], pos_displacement)
            self.idle_simulation(20, viewer)
            return
            print('set_free')
            self.set_initial_obj_pos_and_rotation(np.array([10,10,10]), np.identity(3))
            
            self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
            diff = self.force_difference(force, path[0])
            print('dif', diff)
            start = time.time()
            cout = 0
            while viewer.is_running() and time.time() - start < 2:
                self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                self.mujoc_update(viewer)
                diff = self.force_difference(force, path[0])
                print('dif', diff)
            self.mujoc_update(viewer)
            diff = self.force_difference(force, path[0])
            print('dif', diff)
            
            diff = self.force_difference(force, path[0])
            print('dif', diff)
            self.idle_simulation(2, viewer)
            
            
            
            #print('set_free')
            
            #self.simulation_before_movement(2, viewer, path[0])
            
            '''
            self.simulation_before_movement(start_delay, viewer, path[0])


            for j in range(1,len(path)): 
                privios_element = path[j-1]
                current_element = path[j]
                
                if type(privios_element) == Node.Node:
                    self.update_touque_on_fingers(self.obj, Physics_Engine.Object_Movement(privios_element.pos, privios_element.rotation), privios_element.fingers_in_contact, privios_element.fingers_configuration)
                    self.simulation_before_movement(1, viewer, privios_element)
                    if type(current_element) == Node.Node:
                        self.simulation_before_movement(10, viewer, privios_element)
                        self.change_node(privios_element, current_element, viewer, duration_of_finger_change)
                        
                    else: 
                        self.simulate_arc(current_element, viewer)
            
            
            self.simulation_before_movement(end_delay, viewer, path[0])
            '''

    def idle_simulation(self, start_delay, viewer):
        start = time.time()
        cout = 0
        while viewer.is_running() and time.time() - start < start_delay:
            #print('idle')
            #print('idle', cout)
            cout += 1
            self.mujoc_update(viewer)
            
    
    
    def init_forces(self, viewer, start_node, pos_displacement):
        start = time.time()
        force_within_range = False
        
        force = self.get_forces(start_node)
        count = 0
        eps = 0.01
        
        
        pp= PyPlotting.Plot()
        self.obj.plot(pp, start_node.rotation, start_node.pos)
        
        points = []
        
            
    
        while viewer.is_running() and not force_within_range and count < 1000:
            
             
            self.set_obj_pos_and_rotation(self.obj, start_node.pos, start_node.rotation) 
            self.mujoc_update(viewer)
            
            diff = self.force_difference(force, start_node)

            print('count', count)
            print('diff', diff)
            
            force_within_range = True
            for d in diff:
                if d < self.MIN_FORCE_DIFF or  d > self.MAX_FORCE_DIFF:
                    force_within_range = False

            #self.set_obj_pos_and_rotation(self.obj, start_node.pos, start_node.rotation) 
               
            if not force_within_range:
                pos_displacement = self.pos_displacement_based_on_force(diff, start_node, eps, pos_displacement)
                for j in range(0,len(start_node.fingers_in_contact)):
                    finger = start_node.fingers_in_contact[j]
                    new_q = self.mj_data.ctrl[finger.model_ctrl_indexes[0: len(finger.model_ctrl_indexes)]]
                    u, v = start_node.fingers_configuration[j][-2:]
                    new_p = finger.contact_point(new_q, u, v)
                    points.append(new_p)
            
            count += 1
                
            self.set_obj_pos_and_rotation(self.obj, start_node.pos, start_node.rotation)    
            
            self.mujoc_update(viewer)
            diff = self.force_difference(force, start_node)
            #self.test_print(start_node)
            #print('count')
            #print('diff', diff)
            
            force_within_range = True
            for d in diff:
                if d < self.MIN_FORCE_DIFF or  d > self.MAX_FORCE_DIFF:
                    force_within_range = False
                    
        pp.plot_point(points[0], 'g')
        pp.plot_3D_points(points[1:], 1, 'r')
        pp.set_axes()
        #pp.show()

        
        print('diff', diff)
        print('count', count)
        return pos_displacement
    
    def pos_displacement_based_on_force(self, force_dif, node, eps, pos_displacement):
        
        #force_dif[0] = 0.1
        
        disp = np.zeros(6)
        val = 0.005 #this value is in mm. 
        for i in range(len(self.shadow_hand.fingers)):
            finger = self.shadow_hand.fingers[i]
            if (force_dif[i] >  self.MAX_FORCE_DIFF or force_dif[i] < self.MIN_FORCE_DIFF) and finger in node.fingers_in_contact:
                
                finger = self.shadow_hand.fingers[i]
                
                f_node_idx = node.fingers_in_contact.index(finger)
                
                u, v = node.fingers_configuration[f_node_idx][-2:]
                
                
                q = self.mj_data.ctrl[finger.model_ctrl_indexes[0: len(finger.model_ctrl_indexes)]]
                
                #q = node.fingers_configuration[f_node_idx][:-2]
                
                
                n = finger.contact_normal(node.fingers_configuration[f_node_idx][:-2], u,v)
                
                #pos_displacement[i] = pos_displacement[i] + n * val * np.sign(force_dif[i])
                p_disp =  n * val * np.sign(force_dif[i])
                
                disp[:3] = p_disp
                q_n = finger.joint_displacement_from_end_effector_displacement(q, u, v, disp, True) 
                
                
                if finger == self.shadow_hand.fingers[1]:
                    print()
                    #print('pos_displacement', pos_displacement[i])
                    #print(np.linalg.norm(pos_displacement[i][:3]))
                   
                    #print([q_n[p]-q[p] for p in range(len(q_n))])
                    for j in range(0,len(finger.model_ctrl_indexes)):
                        diff = q_n[j] - q[j] 
                        #diff = np.sign(diff)*min(0.08, abs(diff))
                        #print('diff', diff)
                    #print()
                    
                
                if np.dot((pos_displacement[i]),n) > 5:
                    continue
                
               
                
                
                #print('finger', finger, 'disp', disp[:3])
                #print(finger.contact_normal(q, u,v))
                
                    
                in_range = True
    
                for j in range(0,len(finger.model_ctrl_indexes)):
                    if q_n[j] > finger.max_limits[j] or q_n[j] < finger.min_limits[j] or abs(q_n[j]-node.fingers_configuration[f_node_idx][j]) > 0.1:
                        #print('joint out of range', finger)
                        #print('joint', j, 'val', q_n, 'max', finger.max_limits[j], 'min', finger.min_limits[j]
                        in_range = False
                #in_range = True     
                if in_range:
                    pos_displacement[i] += p_disp
                    for j in range(0,len(finger.model_ctrl_indexes)):
                        
                       
                        #diff = np.sign(diff)*min(0.08, abs(diff))
                        self.mj_data.ctrl[finger.model_ctrl_indexes[j]] = q_n[j]
                        #self.mj_data.ctrl[finger.model_ctrl_indexes[j]] = q_n[j]   
        
        return pos_displacement
        
        
    
    

    def force_difference(self, force_targets, node, init = False):
        
        fingers = node.fingers_in_contact 
        
        
        l = len(self.mj_data.contact)
        print('number of contacts', l)
        
        diff = []
        for i in range(len(self.shadow_hand.fingers)):
            if self.shadow_hand.fingers[i] in fingers:
                diff.append(100)
            else:
                diff.append(0)
            
        for i in range(l):
            #print(self.mj_data.contact[i])
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            id1 = self.mj_model.geom(id1).bodyid
            id2 = self.mj_model.geom(id2).bodyid
            #print('contact:',i, ':', self.mj_model.body(id1).name, self.mj_model.body(id2).name,  id1, id2) 
            
            idx = -1
            if init: 
                item_id =  self.mj_model.body('item1').id
            else: 
                item_id = self.item_id
            
            if id1 == item_id and id2 in self.finger_id: 
                idx = self.finger_id.index(id2)
            elif id2 == item_id and id1 in self.finger_id:
                idx = self.finger_id.index(id1)
            
            if idx != -1:
                result = np.zeros(6)
                mujoco.mj_contactForce(self.mj_model, self.mj_data, i,result)
                contact_frame = self.mj_data.contact.frame[i]
                contact_force = np.abs(np.dot(contact_frame[:3], result[:3]))
                diff[idx] = force_targets[idx] - contact_force
                '''
                print('contact between', self.mj_model.body(id1).name, self.mj_model.body(id2).name)
                print('pos:', self.mj_data.contact.pos[i])
                quv = node.fingers_configuration[node.fingers_in_contact.index(self.shadow_hand.fingers[idx])]
                q = quv[:-2]
                u = quv[-2]
                v = quv[-1]
                print('cal pos', self.shadow_hand.fingers[idx].contact_point(q, u, v)*Constants.MM_TO_METERS)
                print('force in system', contact_force)
                print('force target', force_targets[idx])
                '''
                
                
                
                
        return diff
            
            
    
    
    
    
    
    def get_forces(self, node):
        object_movement = Physics_Engine.Object_Movement(node.pos, node.rotation)
        fingers = node.fingers_in_contact 
        config = node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, node.fingers_uv_on_obj_surface)
        forces =np.zeros(len(self.shadow_hand.fingers))
        for i in range(len(normal_forces)):
            n = normal_forces[i]
            f = fingers[i]
            forces[f.get_index_in_shadow_hand()] = np.linalg.norm(n)
        
        return forces
    
    
    
    
    
            
    def mujoc_update(self, viewer):
        mujoco.mj_step(self.mj_model, self.mj_data)
        viewer.sync()
                

            #change_node
            
    
    
    def simulation_before_movement(self, start_delay, viewer, start_node, pos_displacement):
        start = time.time()
        current_element = start_node
        printing = -1
        count = 2000
        while viewer.is_running() and time.time() - start < start_delay: #and counter < 1:
            count -= 1
            if printing < 0:
                diff = self.force_difference(self.get_forces(current_element), current_element)
                print('diff', diff)    
                '''
                force_within_range = True
                eps = 0.1
                for d in diff:
                    if d < self.MIN_FORCE_DIFF or  d > self.MAX_FORCE_DIFF:
                        print('force not within range')
                        force_within_range = False
        
                if not force_within_range:
                    pos_displacement = self.pos_displacement_based_on_force(diff, start_node, eps, pos_displacement)
                print('diff', diff)
                '''
                self.test_print(current_element)
                #printing+= 1
            self.mujoc_update(viewer)

    
    
    
    
    
    
    
    
    
    
    

    def test_print(self, current_element ):
        #print('pos1', self.mj_data.qpos[0], self.mj_data.ctrl[0])
        #print('pos2', self.mj_data.qpos[1], self.mj_data.ctrl[1])
        
        l = len(self.mj_data.contact)
        print('number of contacts', l)
        print('contact fiction', self.mj_data.contact.friction)
        #print('contact', (self.mj_data.contact))
        #print(self.mj_data.contact)
        object_movement = Physics_Engine.Object_Movement(current_element.pos, current_element.rotation)
        
        fingers = current_element.fingers_in_contact 
        config = current_element.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, current_element.fingers_uv_on_obj_surface)
        total_force = np.array([0,0,0])
        for i in range(l):
            #print(self.mj_data.contact[i])
            
            
            
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            body_id1 = self.mj_model.geom(id1).bodyid
            body_id2 = self.mj_model.geom(id2).bodyid
            print('contact:',i, ':', self.mj_model.body( body_id1).name, self.mj_model.body( body_id2).name,  id1, id2) 
            print('pos:', self.mj_data.contact.pos[i]) 
            
            
            result = np.zeros(6)
            mujoco.mj_contactForce(self.mj_model, self.mj_data, i,result)
            contact_frame = self.mj_data.contact.frame[i]
            
            result = np.zeros(6)
            mujoco.mj_contactForce(self.mj_model, self.mj_data, i,result)
            contact_frame = self.mj_data.contact.frame[i]
            contact_force = np.abs(np.dot(contact_frame[:3], result[:3]))
            
            print('normal_force', contact_force)
            
            #print('tangential_force',result[1]+ result[2])            
            
            
            total_force = total_force + contact_force
        print('total force', total_force)
            
        for i in range(len(current_element.fingers_in_contact)):
            f = current_element.fingers_in_contact[i]
            quv = current_element.fingers_configuration[i]
            p =f.contact_point(quv[:-2],quv[-2],quv[-1])
            print('calculated contact', type(f), p*Constants.SI_SCALE)
            print('calculated force', np.linalg.norm(normal_forces[i]))

        
        self.finger_print(current_element)
        
    
    
    
    
    
    
    
    
    
    
    
    
    def test_physics(self, init_node):
        
        object_movement = Physics_Engine.Object_Movement(init_node.pos, init_node.rotation)
        
        fingers = init_node.fingers_in_contact 
        config = init_node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, init_node.fingers_uv_on_obj_surface)
        print('forces', normal_forces)
        print(fingers)
        i = 0
        j = 0
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
                
            body_id = finger.get_tip_body_id(self.mj_data)      
            joint = Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, (normal_forces[i]))     
            print(finger, joint) 
            for i in range(len(finger.model_joint_indexes)):
                print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[i]])
        #tourqe =  self.PF.joint_torque(finger, quv, normal_forces[i])
        
        #self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= 0.1
       
        #self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] = -0.1
        #print(self.mj_data.qpos)
    
    
    
    
    def finger_print(self, current_element):
        
        #fingers = self.shadow_hand.fingers
        fingers = current_element.fingers_in_contact
        
        om = Physics_Engine.Object_Movement(current_element.pos,  current_element.rotation)
        
        #object_movement = self.PE.create_static_object_movement( current_element.pos,  current_element.rotation)
           
        
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, fingers,  current_element.fingers_configuration,  current_element.fingers_uv_on_obj_surface)
        
        for i in range(3):
            finger=fingers[i]
            quv = current_element.fingers_configuration[i]
            print('finger', type(finger))
            tourqe =  Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, normal_forces[i])
            for j in range(0, len(finger.model_joint_indexes)):
                    #eps = 0.2
                    #if val > 0:
                    #    val += 0.0001
                    #if val < 0:
                    #    val -= 0.0001
                    #print('ctrl - real', self.mj_data.ctrl[finger.model_ctrl_indexes[j]]-self.mj_data.qpos[finger.model_joint_indexes[j]])
                    #print('real', )
                    
                    print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[j]]) 
                    #print('joint calculated:', tourqe[j])
                    #print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] -  tourqe[j]) 
        
        


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

class Mujoco_Simulator:
    
    def __init__(self):   
        
        self.mj_model = mujoco.MjModel.from_xml_path('/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation/Shadow_Hand_21_03_2024/world.xml')    
        self.disable_gravity = False
        if self.disable_gravity:
            self.gravity_constant = 0.0
            self.mj_model.opt.gravity[:] = [0., 0., -9.81*self.gravity_constant] # disable gravity
        self.gravity = self.mj_model.opt.gravity[:]
        print('gravity ', self.gravity)
        self.mj_data = mujoco.MjData(self.mj_model)  
        self.joint_change_pr_sec = 1.57*0.1
        self.joint_tolerance = 0.02
    
    
    def set_shadow_hand_and_obj(self, shadow_hand, obj):
        self.shadow_hand = shadow_hand
        self.obj = obj
    
    
    
    def init_configurations(self, obj, init_node):        
        obj_pos = init_node.pos
        obj_rotation = init_node.rotation
            
        
        self.init_fingers_config(obj_pos, init_node)
        self.set_obj_pos_and_rotation(obj, obj_pos, obj_rotation)
        
        #hand sinking ISSUE and
        #mass ISSUE!!
        print(self.mj_model)
        for i in range(len(self.mj_model.body_mass)):
            if i == obj.body_id: 
                continue
            c = 0
            
            self.mj_model.body_mass[i] *= c
            self.mj_model.body_inertia[i, :] *= c
        
        #no contact issue
        
        val = 0.002 #2
        vec = np.array([val, val,val])
        self.mj_model.geom(obj.name ).size = self.mj_model.geom(obj.name ).size + vec
        names = ["item:site1", "item:site2", "item:site3", "item:site4"]
        for n in names: 
            self.mj_model.site(n).size = self.mj_model.site(n ).size+ vec
        
    
    
    def set_obj_pos_and_rotation(self, obj, obj_pos, rotation):
        mj_quat = mu.rotation_to_mujoco_quat(rotation)
        pos= [obj_pos[i]*Constants.SI_SCALE for i in range(3)]
        #print('pos', pos)
        #pos = [0,0,0]
        self.mj_model.body_pos[obj.body_id]= pos 
        self.mj_model.body_quat[obj.body_id] = mj_quat
        self.mj_data.joint(obj.name).qpos[:3] = pos
        self.mj_data.joint(obj.name).qpos[3:] = mj_quat
        
       
        
    
    def init_fingers_config(self, obj_pos, init_node: Node.Node):
        om = Physics_Engine.Object_Movement(obj_pos, init_node.rotation)
        config, torque = self.get_targets_for_fingers(init_node.fingers_in_contact, init_node.fingers_configuration, init_node.fingers_uv_on_obj_surface, False, om)
        if self.disable_gravity:
            torque =[np.zeros(len(torque[i])) for i in range(len(torque))]
        #print(init_node.fingers_in_contact)
        #print('conf', config)
        #print('torque', torque)
         
        for i in range(len(self.shadow_hand.fingers)): 
            finger = self.shadow_hand.fingers[i]
            quv = config[i]
            tau = torque[i]
            for j in range(0,len(finger.model_ctrl_indexes)):
                self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= quv[j]
                self.mj_data.qpos[finger.model_joint_indexes[j]]= quv[j]
                self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] = tau[j]
                print(finger, self.mj_data.qfrc_applied[finger.model_joint_indexes[j]])
            
            #self.mj_data.xfrc_applied[finger.model_joint_indexes[-1]] = normal_forces[i]
                

        
        
        
    
    def get_targets_for_fingers(self, fingers, fingers_config, fingers_uv_on_obj_surface, only_config = True, om = None):

      
        if not only_config:
            normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, fingers, fingers_config , fingers_uv_on_obj_surface)
            self.current_forces = [normal_forces, tangential_forces]
        
        
        sh_fingers = self.shadow_hand.fingers
        config = []
        joint_torque = []
        for i in range(0, len(sh_fingers)):
            
            finger = sh_fingers[i]
            if finger in fingers: 
                idx = fingers.index(finger)
                quv =fingers_config[idx]
                #quv = np.zeros(finger.number_joints+2)
                if not only_config:    
                    tourqe =  Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, (normal_forces[idx]))
                    #tourqe = np.zeros(finger.number_joints)
            else: 
                quv = np.zeros(finger.number_joints)
                if not only_config:
                    tourqe = np.zeros(finger.number_joints)

                
                ## set fingers not in contact, there can be isues with fingers not in use being in contact!!!
                match i:
                    case 1:
                        quv[0] = finger.min_limits[0]
                    case 2: 
                        quv[0] = config[1][0]
                    case 3:
                        quv[0] = -config[2][0]
                    case 4: 
                        quv[1] = finger.min_limits[1]
            config.append(quv)
            if not only_config:
                joint_torque.append(tourqe)
        if only_config: 
            return config
        else: 
            return config, joint_torque
      
    
        
        
        
        
    def simulate_show(self): 
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            viewer.sync()
            while viewer.is_running():
                self.mujoc_update(viewer)
        
        
        
        
        
        
        
        
    def simulate_path(self, path, obj):
        
        
        duration_of_arc = 2 # can be better
        duration_of_finger_change = 4
        
        start_delay = 20
        end_delay = 50
        
        self.testnode = path[0]
        
        gravity_val = 0.0
        #self.mj_model.opt.gravity[:] = [0., 0., -9.81*gravity_val] # disable gravity
        
        
        element_handled = False
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            viewer.sync()
            self.simulation_before_movement(start_delay, viewer, path[0])


            for j in range(1,len(path)): 
                privios_element = path[j-1]
                current_element = path[j]
                
                if type(privios_element) == Node.Node:
                    self.update_touque_on_fingers(self.obj, Physics_Engine.Object_Movement(privios_element.pos, privios_element.rotation), privios_element.fingers_in_contact, privios_element.fingers_configuration)
                    self.simulation_before_movement(1, viewer, privios_element)
                    if type(current_element) == Node.Node:
                        self.simulation_before_movement(10, viewer, privios_element)
                        self.change_node(privios_element, current_element, viewer, duration_of_finger_change)
                        
                    else: 
                        self.simulate_arc(current_element, viewer)
            
            
            self.simulation_before_movement(end_delay, viewer, path[0])



    def simulate_arc(self, arc: Arc.Arc, viewer):
        
        current_element = arc
        arc_step_time = 0.1
        duration_of_element = len(current_element.config_list)*arc_step_time   
        last_update = time.time()
        while viewer.is_running(): # and not sim_has_finished:
           
            t = time.time()- last_update
            print('arc_is running', t, duration_of_element)
            if t > duration_of_element: 
                break

            conf = current_element.get_configuration(t, duration_of_element)
            #has_reached_conf = self.has_reached_conf(current_element.start_node.fingers_in_contact, conf)
            #print(current_element.start_node.fingers_configuration)
            #print()
            print(t, '/',  duration_of_element)
            #print(conf)
            
            self.update_control_of_fingers(current_element.start_node.fingers_in_contact, conf, 100) 
            
            obj_pos = current_element.get_obj_pos(t, duration_of_element)
            obj_acc = current_element.get_obj_acceleration(t, duration_of_element)
            rotation =  current_element.get_obj_rotation(t, duration_of_element)
            rotation_vel = current_element.get_obj_rotation_velocity(t, duration_of_element)
            rotation_acc = current_element.get_obj_rotation_acceleration(t, duration_of_element)
            

            om = Physics_Engine.Object_Movement(obj_pos, rotation, obj_acc, rotation_vel, rotation_acc)
            self.update_touque_on_fingers(self.obj ,om,  current_element.start_node.fingers_in_contact, conf )   
            self.mujoc_update(viewer)

    def update_touque_on_fingers(self, obj, object_movement, fingers, config):
       
        obj_rotation = object_movement.rot
        obj_pos =object_movement.pos
        
        
        fingers_uv_on_obj = []
        for i in range(len(fingers)):
            f = fingers[i]
          
            quv = config[i]
            p = f.contact_point(quv[:-2],quv[-2],quv[-1])
            uv = obj.uv_for_surface_point(p, obj_rotation, obj_pos)
            fingers_uv_on_obj.append(uv)
        
        
        
        
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, fingers_uv_on_obj, normal_and_tangential_forces_guess=self.current_forces)
        
        total_force = sum(normal_forces) + sum(tangential_forces)
        print('total force', total_force)
        print('expected_force:',  Physics_Engine.Physics_Engine.gravity_on_obj)
        eps= 10**(-6)
        if np.linalg.norm(total_force +  Physics_Engine.Physics_Engine.gravity_on_obj) > eps:
            return
        
           
            #print('w', PE.weight_for_system(object_movement, fingers, config, fingers_uv_on_obj, external_force, external_torque))
        
        self.current_forces = [normal_forces, tangential_forces]
        
        for i in range( len(fingers)):  
            finger = fingers[i]
            
            quv = config[i]
            #print('force ', normal_forces[i], tangential_forces[i])
            #print(normal_forces[i]+ tangential_forces[i])
            tourqe =  Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, normal_forces[i] )
            #print('finger', finger)
            #print('tourqe', tourqe)
            for j in range(0, len(finger.model_joint_indexes)):
                #eps = 0.2
                #val = quv[j]
                #self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= val
                #self.mj_data.qpos[finger.model_joint_indexes[j]]= val
                self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] = tourqe[j]
                    #print(self.mj_data.qfrc_applied[finger.model_joint_indexes[j]])
    
    
    def get_uv_on_obj(self, fingers, configuration, rotation, pos):
        conf = configuration
        res = []
        for i in range(len(conf)):
            finger = fingers[i]
            finger_pos = finger.contact_point(conf[i][:-2], conf[i][-2], conf[i][-1])
            uv = self.obj.uv_for_surface_point(finger_pos, rotation, pos)
            res.append(uv)
        return res
    
    def set_arc_and_get_config(self, arc: Arc.Arc, t, duration_of_arc): 
        obj_pos = arc.get_obj_pos(t, duration_of_arc)
        obj_acc = arc.get_obj_acceleration(t, duration_of_arc)
        rotation =  arc.get_obj_rotation(t, duration_of_arc)
        rotation_vel = arc.get_obj_rotation_velocity(t, duration_of_arc)
        rotation_acc = arc.get_obj_rotation_acceleration(t, duration_of_arc)
            
        om = Physics_Engine.Object_Movement(obj_pos, rotation, obj_acc, rotation_vel, rotation_acc)
        
        fingers_config = arc.get_configuration(t, duration_of_arc)
        uv = self.get_uv_on_obj(arc.start_node.fingers_in_contact, fingers_config, rotation, obj_pos)
        
        
        config = self.get_targets_for_fingers(arc.start_node.fingers_in_contact,fingers_config, uv, True, om)
        
        #config, torque = self.get_targets_for_fingers(arc.start_node.fingers_in_contact,fingers_config, uv, false, om)
        
        #self.update_tourqe_of_fingers(self.shadow_hand.fingers, torque)
        return config





        
    def mujoc_update(self, viewer):
        mujoco.mj_step(self.mj_model, self.mj_data)
        viewer.sync()
                

            #change_node
            
    
    
    def simulation_before_movement(self, start_delay, viewer, start_node):
        start = time.time()
        current_element = start_node
        printing = -50
        while viewer.is_running() and time.time() - start < start_delay: #and counter < 1:
                if printing < 0:
                    self.test_print(current_element)
                    printing+= 1
                self.mujoc_update(viewer)
                
    
    def change_node(self, current_node: Node.Node, new_node: Node.Node, viewer, duration_of_finger_change):
        print('change')
        change_is_needed = False
        for f in new_node.fingers_in_contact:
            if f not in current_node.fingers_in_contact:
                change_is_needed = True
                break
            
        for f in current_node.fingers_in_contact:
            if f not in new_node.fingers_in_contact:
                change_is_needed = True
                break
        if not change_is_needed:
            return
                
        
        self.add_fingers_to_object(current_node, new_node, viewer)
        self.remove_fingers_from_object(current_node, new_node, viewer)
        

    def update_finger_configuration(self, fingers, fingers_config, viewer, arc = None):
        
        print('fingers', fingers, fingers_config)
        
        if arc != None:
            arc_step_time = 0.05
            duration_of_element = len(arc.config_list)*arc_step_time
        
        time_to_change = 0.2
        
        config_is_obtained = False
        current_time = time.time()
        last_update = current_time
        while viewer.is_running() and not config_is_obtained:
            update_time = time.time()
            
            
            t = update_time - current_time
            if arc != None:
                print('arc')
            else: 
                print('node', fingers, fingers_config)
            
            if update_time - last_update > time_to_change:
                last_update = update_time
                max_change = self.joint_change_pr_sec*time_to_change
                if arc != None:
                    fingers_config = self.set_arc_and_get_config(arc, t, duration_of_element)    
                    print('t', 'fingers_config', t, duration_of_element)
                self.test_print(self.testnode)
                config_is_obtained = self.update_control_of_fingers(fingers, fingers_config, max_change)
            self.mujoc_update(viewer)      
            
            if arc != None:
                if t> duration_of_element:
                    config_is_obtained = True
                else: 
                    config_is_obtained = False
        
            self.mujoc_update(viewer)
        
        
        
        
    
    
    def add_fingers_to_object(self, current_node: Node.Node, new_node: Node.Node, viewer):
        print('adding')
        fingers_to_add = []
        fingers_config_first_change = []
        fingers_to_add_config =  []
        
        for i in range(len(new_node.fingers_in_contact)):
            f =  new_node.fingers_in_contact[i]
            if f not in current_node.fingers_in_contact:
                fingers_to_add.append(f)
                fingers_to_add_config.append(new_node.fingers_configuration[i])
                idx = f.get_index_in_shadow_hand()
                
                
            
                config = np.zeros(len(new_node.fingers_configuration[i]))
            
                if idx > 0 and idx < 4:
                    config[:2] = new_node.fingers_configuration[i][:2]
                elif idx == 4: 
                    config[1:3] = new_node.fingers_configuration[i][1:3]
                else: 
                    config = new_node.fingers_configuration[i]  
                fingers_config_first_change.append(config)
        print(fingers_to_add, fingers_config_first_change)        
        #self.update_finger_configuration(fingers_to_add, fingers_config_first_change, viewer) 
        
        
        self.update_finger_configuration(fingers_to_add, fingers_to_add_config, viewer)        
        
    
    def remove_fingers_from_object(self, current_node: Node.Node, new_node: Node.Node, viewer):
        fingers_to_remove = []
        fingers_config = []
        om = Physics_Engine.Object_Movement(new_node.pos, new_node.rotation)
        config, torque = self.get_targets_for_fingers(new_node.fingers_in_contact, new_node.fingers_configuration, new_node.fingers_uv_on_obj_surface, False, om)
        print(torque)
        
        for i in range(len(current_node.fingers_in_contact)):
            f =  current_node.fingers_in_contact[i]
            if f not in new_node.fingers_in_contact:
                fingers_to_remove.append(f)
                fingers_config.append(config[f.get_index_in_shadow_hand()])

        #self.update_tourqe_of_fingers(self.shadow_hand.fingers, torque)
        om = Physics_Engine.Object_Movement(new_node.pos, new_node.rotation)
        self.update_touque_on_fingers(self.obj, om, new_node.fingers_in_contact, new_node.fingers_configuration)
        time_to_change_tourqe = 1
        self.simulation_before_movement(time_to_change_tourqe, viewer, current_node)
        self.update_tourqe_of_fingers(self.shadow_hand.fingers, torque)
        self.simulation_before_movement(time_to_change_tourqe, viewer, current_node)
        #self.update_finger_configuration(fingers_to_remove, fingers_config, viewer)
        
        
        

    def update_tourqe_of_fingers(self, fingers, joint_tourqe): 
        
        for i in range(len(fingers)):
            finger = fingers[i]
            tourqe = joint_tourqe[i]
            for j in range(0, len(finger.model_joint_indexes)):
                    #eps = 0.2
                    #val = quv[j]
                    #self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= val
                    #self.mj_data.qpos[finger.model_joint_indexes[j]]= val
                    self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] = tourqe[j]
        
    
    
    def update_control_of_fingers(self, fingers, config, max_change):
        
        config_obtained = True
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
            for j in range(0, len(finger.model_ctrl_indexes)):
                   
                    val = quv[j]
                    dif = val- self.mj_data.qpos[finger.model_joint_indexes[j]] 
                    if abs(dif) > max_change:
                        val = self.mj_data.qpos[finger.model_joint_indexes[j]] + np.sign(dif)* max_change
                    '''
                    if finger.get_index_in_shadow_hand() != 0 and j == len(finger.model_ctrl_indexes)-1:
                        if self.mj_data.ctrl[finger.model_ctrl_indexes[j-1]] < finger.max_limits[-4]:
                            val = 0    
                    '''
                    self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= val
                    if abs(self.mj_data.qpos[finger.model_joint_indexes[j]] - quv[j]) > self.joint_tolerance:
                        config_obtained = False
        return config_obtained
    
    
    
    
    
    
    
    
    
    
    
    

    def test_print(self, current_element ):
        #print('pos1', self.mj_data.qpos[0], self.mj_data.ctrl[0])
        #print('pos2', self.mj_data.qpos[1], self.mj_data.ctrl[1])
        
        l = len(self.mj_data.contact)
        print('number of contacts', l)
        print('contact fiction', self.mj_data.contact.friction)
        #print('contact', (self.mj_data.contact))
        #print(self.mj_data.contact)
        object_movement = Physics_Engine.Object_Movement(current_element.pos, current_element.rotation)
        
        fingers = current_element.fingers_in_contact 
        config = current_element.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, current_element.fingers_uv_on_obj_surface)
        total_force = np.array([0,0,0])
        for i in range(l):
            #print(self.mj_data.contact[i])
            
            
            
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            body_id1 = self.mj_model.geom(id1).bodyid
            body_id2 = self.mj_model.geom(id2).bodyid
            print('contact:',i, ':', self.mj_model.body( body_id1).name, self.mj_model.body( body_id2).name,  id1, id2) 
            print('pos:', self.mj_data.contact.pos[i]) 
            
            
            result = np.zeros(6)
            mujoco.mj_contactForce(self.mj_model, self.mj_data, i,result)
            contact_frame = self.mj_data.contact.frame[i]
            
            print('normal_force', result[0])
            
            print('tangential_force',result[1]+ result[2])            
            
            
            total_force = total_force + np.dot(contact_frame[:3], result[:3])
        print('total force', total_force)
            
        for i in range(len(current_element.fingers_in_contact)):
            f = current_element.fingers_in_contact[i]
            quv = current_element.fingers_configuration[i]
            p =f.contact_point(quv[:-2],quv[-2],quv[-1])
            print('calculated contact', type(f), p*Constants.SI_SCALE)
            print('calculated force', np.linalg.norm(normal_forces[i]))

        
        self.finger_print(current_element)
        
    
    
    
    
    
    
    
    
    
    
    
    
    def test_physics(self, init_node):
        
        object_movement = Physics_Engine.Object_Movement(init_node.pos, init_node.rotation)
        
        fingers = init_node.fingers_in_contact 
        config = init_node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, init_node.fingers_uv_on_obj_surface)
        print('forces', normal_forces)
        print(fingers)
        i = 0
        j = 0
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
                
            body_id = finger.get_tip_body_id(self.mj_data)      
            joint = Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, (normal_forces[i]))     
            print(finger, joint) 
            for i in range(len(finger.model_joint_indexes)):
                print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[i]])
        #tourqe =  self.PF.joint_torque(finger, quv, normal_forces[i])
        
        #self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= 0.1
       
        #self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] = -0.1
        #print(self.mj_data.qpos)
    
    
    
    
    def finger_print(self, current_element):
        
        #fingers = self.shadow_hand.fingers
        fingers = current_element.fingers_in_contact
        
        om = Physics_Engine.Object_Movement(current_element.pos,  current_element.rotation)
        
        #object_movement = self.PE.create_static_object_movement( current_element.pos,  current_element.rotation)
           
        
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, fingers,  current_element.fingers_configuration,  current_element.fingers_uv_on_obj_surface)
        
        for i in range(3):
            finger=fingers[i]
            quv = current_element.fingers_configuration[i]
            print('finger', type(finger))
            tourqe =  Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, normal_forces[i])
            for j in range(0, len(finger.model_joint_indexes)):
                    #eps = 0.2
                    #if val > 0:
                    #    val += 0.0001
                    #if val < 0:
                    #    val -= 0.0001
                    #print('ctrl - real', self.mj_data.ctrl[finger.model_ctrl_indexes[j]]-self.mj_data.qpos[finger.model_joint_indexes[j]])
                    #print('real', )
                    
                    print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[j]]) 
                    #print('joint calculated:', tourqe[j])
                    #print('joint', self.mj_data.qfrc_applied[finger.model_joint_indexes[j]] -  tourqe[j]) 
        
        


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def simulate_from_file(self, file_name, obj):
        self.remove_object(obj)

        with open(file_name) as f:
            lines = [line.rstrip() for line in f]
        
        start_delay = 15
        end_delay = 5
        
        update_time = 0.04
        idx = 0
        current_t = time.time()
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            while viewer.is_running():
                print(time.time()- current_t) 
                if time.time()- current_t >start_delay:
                    
                    break 
            current_t = time.time()
            while viewer.is_running():
                
                if time.time()- current_t > update_time and idx < len(lines):
                    
                    line = lines[idx]
                    idx +=1
                    current_t = time.time()
                    
                    l = len(self.mj_data.contact)

                    for i in range(l):
                        id1 = self.mj_data.contact.geom1[i]
                        id2 = self.mj_data.contact.geom2[i]
                        body_id1 = self.mj_model.geom(id1).bodyid
                        body_id2 = self.mj_model.geom(id2).bodyid
                        print('contact:',i, ':', self.mj_model.body( body_id1).name, self.mj_model.body( body_id2).name,  id1, id2) 
          

                    config = String_Handler.array_from_sting(line.strip())
                    for i in range(5):
                        c = config[i]
                        f = self.shadow_hand.fingers[i]
                        res_ctrl = c
                        if len(config[i]) < f.number_joints:
                            res_ctrl = f.q_joint_from_acts(c, True)
                        
                        #print(f, res_ctrl)
                        for j in range(len(f.model_ctrl_indexes)):
                            self.mj_data.ctrl[f.model_ctrl_indexes[j]]= res_ctrl[j]
                if idx >= len(lines):
                    break

                self.mujoc_update(viewer)
                
            while viewer.is_running():
                print(time.time()- current_t) 
                if time.time()- current_t >end_delay:
                    
                    break 
    
    
    
    def remove_object(self, obj):
        
        pos = [100,100,100]
        self.mj_model.body_pos[obj.body_id]= pos 
        self.mj_data.joint(obj.name).qpos[:3] = pos
        self.mj_model.body_mass[obj.body_id]= 0
    

        
    def simulate_from_files(self, start_file, movement_file,  obj, show_obj = False, obj_pos = None, obj_rotation = None):
        self.remove_object(obj)

        with open(start_file) as f:
            lines_start = [line.rstrip() for line in f]
        
        with open(movement_file) as f:
            lines_movement = [line.rstrip() for line in f]
        
        
        start_delay = 2
        end_delay = 5
        
        update_time = 0.04
        idx = 0
        
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            self.delay_change(viewer, start_delay)
            
            
            self.file_update(viewer, lines_start, update_time)
            if show_obj: 
                self.set_obj_pos_and_rotation(obj, obj_pos, obj_rotation)
            self.delay_change(viewer, 5)
            self.file_update(viewer, lines_movement, update_time)
                
                
                
            self.delay_change(viewer, end_delay)
    
    def file_update(self, viewer, lines, update_time):
        current_t = time.time()
        idx = 0
        while viewer.is_running():
                self.test_contacts()
                if time.time()- current_t > update_time and idx < len(lines):
                    line = lines[idx]
                    self.set_line_to_config(line)
                    
                    idx +=1
                    current_t = time.time()
                if idx >= len(lines):
                    break
                self.mujoc_update(viewer)
    
    
    def delay_change(self, viewer, delay):
        current_t = time.time()
        while viewer.is_running():
            print(time.time()- current_t) 
            if time.time()- current_t >delay:
                    break 
            self.mujoc_update(viewer)
    
    def test_contacts(self):
        l = len(self.mj_data.contact)
        for i in range(l):
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            body_id1 = self.mj_model.geom(id1).bodyid
            body_id2 = self.mj_model.geom(id2).bodyid
            print('contact:',i, ':', self.mj_model.body( body_id1).name, self.mj_model.body( body_id2).name,  id1, id2) 
    
    
    def set_line_to_config(self, line):
        config = String_Handler.array_from_sting(line.strip())
        for i in range(5):
            c = config[i]
            f = self.shadow_hand.fingers[i]
            res_ctrl = c
            if len(config[i]) < f.number_joints:
                res_ctrl = f.q_joint_from_acts(c, True)
            
            #print(f, res_ctrl)
            for j in range(len(f.model_ctrl_indexes)):
                self.mj_data.ctrl[f.model_ctrl_indexes[j]]= res_ctrl[j]

    
        
        #insert_obj
        #move_fingers 
        
        