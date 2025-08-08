

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


class Mujoco_Simulator: 
    
    
    MAX_FORCE_DIFF = -0.0
    MIN_FORCE_DIFF = -0.1
    
    
    def __init__(self):   
        
        self.mj_model = mujoco.MjModel.from_xml_path('/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation/Shadow_Hand_21_03_2024/world.xml')    
        
        self.gravity = self.mj_model.opt.gravity[:]
        
        print('gravity ', self.gravity)
        self.mj_data = mujoco.MjData(self.mj_model)  
        self.joint_change_pr_sec = 1.57*0.1
        self.joint_tolerance = 0.02
        #self.mj_model.jnt_actgravcomp = [1 for i in range(len(self.mj_model.jnt_actgravcomp))]
        
        #self.mj_model.actuator_biastype = [0 for i in range(len(self.mj_model.actuator_biastype))]
    
    
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
        self.set_forces(init_node)
        
        force = self.get_normal_forces(init_node)
        calc_force = [np.zeros(6) for i in range(len(self.shadow_hand.fingers))]
        for i in range(len(self.shadow_hand.fingers)):
            f = self.shadow_hand.fingers[i]
            calc_force[i][:3] = force[i]
        
        #self.set_body_forces(self.shadow_hand.fingers, calc_force)
        
        #f = self.shadow_hand.fingers[0]
        #print(self.mj_data.qfrc_applied[f.model_ctrl_indexes[0]:f.model_ctrl_indexes[-1]+1])
        self.set_obj_pos_and_rotation(obj, obj_pos, obj_rotation)
        
        #hand sinking ISSUE and
        #mass ISSUE!!
        #print(self.mj_model)
        for i in range(len(self.mj_model.body_mass)):
            if i == obj.body_id: 
                c = 1
            
                self.mj_model.body_mass[i] *= c
                self.mj_model.body_inertia[i, :] *= c
                continue
            c = 0
            
            self.mj_model.body_mass[i] *= c
            self.mj_model.body_inertia[i, :] *= c
        
        #no contact issue
        
        
        val = 0.0005#5#2
        vec = np.array([val, val,val])
        
        self.mj_model.geom(obj.name ).size = self.mj_model.geom(obj.name ).size + vec
        print(self.mj_model.geom(obj.name ).size )
        
        names = ["item:site1", "item:site2", "item:site3", "item:site4"]
        for n in names: 
            self.mj_model.site(n).size = self.mj_model.site(n ).size+ vec
        
        
        
        
    
    def set_obj_pos_and_rotation(self, obj, obj_pos, rotation):
        mj_quat = mu.rotation_to_mujoco_quat(rotation)
        pos= [obj_pos[i]*Constants.SI_SCALE for i in range(3)]
        #print('pos', pos)
        #pos = [1000,1000,1000]
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
                joint_value = quv[j] #+ np.sign(quv[j])*0.01
                self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= joint_value
                self.mj_data.qpos[finger.model_joint_indexes[j]]= joint_value
                

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



    
        
    def test(self, node):
        
        object_movement = Physics_Engine.Object_Movement(node.pos, node.rotation)
        fingers = node.fingers_in_contact 
        config = node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, node.fingers_uv_on_obj_surface)

        
        
        print('obj mass', self.mj_model.body_mass[self.item_id])
        
        pp= PyPlotting.Plot()
        self.obj.plot(pp, node.rotation, node.pos)
        
        fingers = node.fingers_in_contact 
        
        
        l = len(self.mj_data.contact)
        print('number of contacts', l)
        
        force = np.zeros(3)
        
        result_mujoco = np.zeros(3)
        
        for i in range(l):
            #print(self.mj_data.contact[i])
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            id1 = self.mj_model.geom(id1).bodyid
            id2 = self.mj_model.geom(id2).bodyid
        

            item_id = self.item_id
            
            if id1 == item_id and id2 in self.finger_id: 
                idx = self.finger_id.index(id2)
            elif id2 == item_id and id1 in self.finger_id:
                idx = self.finger_id.index(id1)
            
            if idx != -1:
                result = np.zeros(6)
                mujoco.mj_contactForce(self.mj_model, self.mj_data, i,result)
                contact_frame = self.mj_data.contact.frame[i]
                contact_pos = self.mj_data.contact.pos[i] * Constants.METERS_TO_MM
                pp.plot_vector(contact_pos, -contact_frame[:3], 'g', 10)
                
                #print('normal force', np.linalg.norm(result[:3]))
                print('calculated normal', np.linalg.norm(normal_forces[node.fingers_in_contact.index(self.shadow_hand.fingers[idx])]))
                
                
                
                f = self.shadow_hand.fingers[idx]
                quv = node.fingers_configuration[node.fingers_in_contact.index(f)]
                
                
                pp.plot_vector(contact_pos, f.contact_normal(quv[:-2], quv[-2], quv[-1]), 'r', 10)
                
                contact_force = np.abs(np.dot(contact_frame[:3], result[:3]))
                print('mujoco nomal', contact_force)
                force = force + np.matmul(np.reshape(contact_frame,(3,3)), result[:3]) 
                
                print('calculated total', np.linalg.norm(normal_forces[node.fingers_in_contact.index(self.shadow_hand.fingers[idx])]+ tangential_forces[node.fingers_in_contact.index(self.shadow_hand.fingers[idx])]))
                print('total mujoco', np.linalg.norm(result[:3]))
                print('')
                
                print()
        print('force_ mujoco', force)
        pp.set_axes()
        #pp.show()
                
                

    def smooth_adjust(self, node, viewer, forces, scalar):
        print()
        for i in range(len(node.fingers_in_contact)):
            f = node.fingers_in_contact[i]
            #qfrc = self.mj_data.qfrc_actuator[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1] + self.mj_data.qfrc_applied[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1]
            #qfrc = self.mj_data.qfrc_inverse[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1]
            qfrc = self.mj_data.qfrc_smooth[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1] + self.mj_data.qfrc_constraint[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1]
            #print(qfrc)
            J = f.jacobian(node.fingers_configuration[i])
            J[:3] = J[:3]*Constants.MM_TO_METERS
            J_T = np.transpose(J)
            res = np.matmul(np.linalg.pinv(J_T), qfrc)
            
            n = f.contact_normal(node.fingers_configuration[i][:-2], node.fingers_configuration[i][-2], node.fingers_configuration[i][-1])
            mujoco_n = np.dot(n, res[:3])
            calc_n = np.linalg.norm(forces[self.shadow_hand.fingers.index(f)][:3])
            diff = calc_n- mujoco_n
            diff_force = np.zeros(6)
            diff_force[:3] =  diff*n*scalar #diff*n
            jont_torque = np.matmul(J_T, diff_force)
            
            for j in range(0,len(f.model_joint_indexes)):
                self.mj_data.qfrc_applied[f.model_joint_indexes[j]] += jont_torque[j]
            
            
            
            #print(f)
            #print('mujoco_n', mujoco_n)
            #print('calc_n', np.linalg.norm(forces[self.shadow_hand.fingers.index(f)][:3]))
            #print('diff', diff)
    
    print()
    
    
    

    
    
    def simulate_path_new(self, path, obj): 
        
        
        
        
        arc_step_time = 0.1    
        sim_has_finished = False
        
        arc_is_handled = False
        
        element_handled = False
        
        
        
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            
            viewer.sync()
            while viewer.is_running() and not sim_has_finished:
                            
                
                if element_handled: #does not depend on time as we might want to change it. 
                    
                    element_handled = False
                    
                    if arc_is_handled: 
                        #springer node efter arc over, da den allerede er opnået via arc
                        idx +=1
                    
                    if idx >= len(path):
                        print('idx', idx, 'length', len(path))
                        break
                    
                    privios_element = path[idx-1]
                    current_element = path[idx]
                    #new_element = path[idx]
                    idx += 1
                    
                    print('.................updating A to B', type(privios_element),  type(current_element))
                

                    if type(current_element)  == Arc.Arc:
                        arc_is_handled = True        
                        duration_of_element = len(current_element.config_list)*arc_step_time         
                    else:  
                        arc_is_handled = False
                        
                        duration_of_element = 1
                        fingers_to_add = []
                        fingers_to_add_config = []
                        fingers_to_add_old_config = []
                        fingers_to_remove = []
                        fingers_to_remove_config = []
                        fingers_to_remove_new_config = []
                        new_targets = self.get_targets_for_fingers(current_element) 
                        for i in range(len(current_element.fingers_in_contact)):
                            f = current_element.fingers_in_contact[i]
                            conf = current_element.fingers_configuration[i][:-2]
                            if f not in privios_element.fingers_in_contact:
                                fingers_to_add.append(f)
                                fingers_to_add_config.append(conf)
                                fingers_to_add_old_config.append([self.mj_data.ctrl[f.model_ctrl_indexes[j]] for j in range(len(f.model_ctrl_indexes)) ])
                        for i in range(len(privios_element.fingers_in_contact)):
                            f = privios_element.fingers_in_contact[i]
                            if f not in current_element.fingers_in_contact:
                                fingers_to_remove.append(f)
                                fingers_to_remove_config.append(privios_element.fingers_configuration[i])
                                fingers_to_remove_new_config.append(new_targets[self.shadow_hand.fingers.index(f)])
                        om = Physics_Engine.Object_Movement(current_element.pos, current_element.rotation)
                        
                    last_update = time.time()
                    t = time.time()- last_update
                        
                        
                else: 
                    
                    
                    if arc_is_handled:
                        #print(t, '/',  duration_of_element)
                        
                        #self.plot_conf( current_element.start_node.fingers_in_contact, plot, t)
                        #self.plot_touqe( current_element.start_node.fingers_in_contact, plot, t)
                        
                        #arc = path[1]
                        #conf = arc.get_configuration_start_node_finger_sorted( t, duration_of_arc)
                        #finger =  arc.start_node.fingers_in_contact[0]
                        #print('finger', finger, finger.model_joint_indexes)
                        #print('val_to set', conf[0][:-2])
                        #print('old_controle_ value', self.mj_data.ctrl[finger.model_ctrl_indexes[0]:finger.model_ctrl_indexes[-1]+1])
                        #print('current', self.mj_data.qpos[finger.model_joint_indexes[0]:finger.model_joint_indexes[-1]+1])
                        #print(current_element.start_node.fingers_in_contact)
                        
                        conf = current_element.get_configuration(t, duration_of_element)
                        #has_reached_conf = self.has_reached_conf(current_element.start_node.fingers_in_contact, conf)
                        #print(current_element.start_node.fingers_configuration)
                        #print()
                        print(t, '/',  duration_of_element)
                        #print(conf)
                        
                        self.update_control_of_fingers(current_element.start_node.fingers_in_contact, conf)
                        
                        obj_pos = current_element.get_obj_pos(t, duration_of_element)
                        obj_acc = current_element.get_obj_acceleration(t, duration_of_element)
                        rotation =  current_element.get_obj_rotation(t, duration_of_element)
                        rotation_vel = current_element.get_obj_rotation_velocity(t, duration_of_element)
                        rotation_acc = current_element.get_obj_rotation_acceleration(t, duration_of_element)
                        
        
                        om = Physics_Engine.Object_Movement(obj_pos, rotation, obj_acc, rotation_vel, rotation_acc)
                        self.update_touque_on_fingers(obj,om,  current_element.start_node.fingers_in_contact, conf )   
                        #self.plot_touqe( current_element.start_node.fingers_in_contact, plot, t)
                        
                    
                    elif not arc_is_handled: 
                        
                        print('change_finger', t, duration_of_element)
                        #print('fingers to add', fingers_to_add)
                        #print('fingers to remove', fingers_to_remove)
                        print('fingers to add', fingers_to_add, t)
                        self.change_finger(fingers_to_add, fingers_to_add_config, fingers_to_add_old_config,  fingers_to_remove, fingers_to_remove_config, fingers_to_remove_new_config, t, duration_of_element)
                        self.update_touque_on_fingers(obj, om, current_element.fingers_in_contact, current_element.fingers_configuration)
                
                
                    t = round(time.time()- last_update, 12)
                    
                    if t > duration_of_element: 
                        element_handled = True      
                
                
                self.mujoc_update(viewer)
                
        
            
            print('simulation has finshed', path)
            print(path[-1])
            obj_pos = path[-1].pos
            rotation = path[-1].rotation
            om = Physics_Engine.Object_Movement(obj_pos, rotation)
            start = time.time()
            while viewer.is_running() and time.time() - start < 5:
                #time.sleep(1)
                print('simulation has finshed')
                print(time.time() - start)
                #self.test_print(current_element)
                #self.plot_conf( current_element.start_node.fingers_in_contact, plot, t)
                #self.plot_touqe( current_element.start_node.fingers_in_contact, plot, t)
                #self.update_touque_on_fingers(obj,om, path[-1].fingers_in_contact, path[-1].fingers_configuration)
                self.mujoc_update(viewer)
            
        
        
        
    
    


    def simulate_path(self, path, obj):
        
        
        duration_of_arc = 2 # can be better
        duration_of_finger_change = 4
        
        start_delay = 50
        end_delay = 50
        
        self.testnode = path[0]
        
        gravity_val = 0.0
        #self.mj_model.opt.gravity[:] = [0., 0., -9.81*gravity_val] # disable gravity
        
        element_handled = False
        
        start = time.time()
        
        forc = self.get_normal_forces(path[0])
        forces = [np.zeros(6) for i in range(len(self.shadow_hand.fingers))]
        for i in range(len(self.shadow_hand.fingers)):
            forces[i][:3] = forc[i]
        
        
        
        pos_displacement = [np.zeros(3) for i in range(len(self.shadow_hand.fingers))]
        
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            viewer.sync()
            c = 0
            test = False
            self.test(path[0])
            #self.smooth_adjust(path[0], viewer, forces)
            
            
            
            while viewer.is_running() and c < 20000 :#time.time() - start < 500 and c < 444:
                self.step_start = time.time()
                c += 1
                #self.set_obj_pos_and_rotation(obj, path[0].pos, path[0].rotation)
                #print(time.time() - start)
                self.set_forces(self.testnode)
                if c % 100 == 0:
                    #
                    print('n/n/')
                    print(c)
                    self.test_print(self.testnode)
                    
                    
                    number_of_contacts = len(self.mj_data.contact)
                    print(number_of_contacts)
                    for i in range(number_of_contacts):
                    #print(self.mj_data.contact[i])
                        id1 = self.mj_data.contact.geom1[i]
                        id2 = self.mj_data.contact.geom2[i]
                        id1 = self.mj_model.geom(id1).bodyid
                        id2 = self.mj_model.geom(id2).bodyid
                        print('contact:',i, ':', self.mj_model.body(id1).name, self.mj_model.body(id2).name,  id1, id2) 
                        print('pos:', self.mj_data.contact.pos[i]) 
                         
                        #    self.smooth_adjust(path[0], viewer, forces, 0.9)
                #else: 
                #    self.smooth_adjust(path[0], viewer, forces, 0.0001)
                self.mujoc_update(viewer)
            
    
            '''
            
            print('const', self.mj_data.qfrc_constraint)
            print('smooth',self.mj_data.qfrc_smooth)
            #self.set_body_forces(self.shadow_hand.fingers, forces)
            
            number_of_contacts = len(self.mj_data.contact)
            c = 0
            test = False
            print(number_of_contacts)
        
            while viewer.is_running() and time.time() - start < 500: # and number_of_contacts < 3:
                c += 1
                print(c)
                number_of_contacts = len(self.mj_data.contact)
                #time.sleep(0.01)
                #self.set_body_forces(self.shadow_hand.fingers, forces)
                self.mujoc_update(viewer)
            print('const', self.mj_data.qfrc_constraint)
            
            
            
                
            f = self.shadow_hand.fingers[0]
            print('before', self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)])
            
            
            self.set_body_forces(self.shadow_hand.fingers, forces)
            
            f = self.shadow_hand.fingers[0]
            print(self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)])
            force = self.get_forces(path[0])
            print('force', force)
            self.test(path[0])
            c= 0
            while viewer.is_running() and time.time() - start < 10:
                self.set_body_forces(self.shadow_hand.fingers, forces)
                #self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                
                #self.adjust_body_forces(force, path[0])
                self.mujoc_update(viewer)
                c += 1
                f = self.shadow_hand.fingers[0]
            self.test(path[0])
            return
            c = 0
            '''
            '''
            while viewer.is_running() and time.time() - start < 10:
                #self.test(path[0])
                self.set_body_forces(self.shadow_hand.fingers, forces)
                c += 1
                f = self.shadow_hand.fingers[0]
                #print(self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)])
                #print(forces)
                
                self.mujoc_update(viewer)
            '''
            '''    
            self.test(path[0])
            #self.idle_simulation(20, viewer)
            
            while viewer.is_running() and time.time() - start < 10:
                self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)

                self.adjust_forces(force, path[0])
                self.mujoc_update(viewer)
                time.sleep(0.1)
            
            '''
            self.step_start = time.time()
            print(self.step_start)
            
            #self.idle_simulation(20, viewer)
            
            return
            
            self.idle_simulation(10, viewer)
            
            print(self.mj_data.qfrc_applied[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])

            
            
            
            self.mujoc_update(viewer)
            self.test(path[0])
            
            print('const', self.mj_data.qfrc_constraint)
            print('smooth',self.mj_data.qfrc_smooth)
            
            print('aplied',self.mj_data.qfrc_applied)
            #print('act',self.mj_data.qfrc_actuator)
            #print('imnve',self.mj_data.qfrc_inverse)
            #print('act',self.mj_data.actuator_force)
            
            
            f = self.shadow_hand.fingers[0]
            print(f.model_joint_indexes[0])
            print(self.mj_data.qfrc_applied[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])
            print(self.mj_data.qfrc_constraint[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])
            print(self.mj_data.qfrc_smooth[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])
            print(self.mj_data.qfrc_inverse[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])
            print(self.mj_data.actuator_force[f.model_joint_indexes[0]:f.model_joint_indexes[-1]+1])    
            print('her')
            thm = []
            for i in range(0,len(f.model_joint_indexes)):
                thm.append(self.mj_data.actuator_force[f.model_joint_indexes[i]]) # + self.mj_data.qfrc_applied[i]) # + self.mj_data.qfrc_actuator[i])
                #thm.append(self.mj_data.qfrc_smooth[f.model_joint_indexes[i]] + self.mj_data.qfrc_constraint[f.model_joint_indexes[i]]) # + self.mj_data.qfrc_actuator[i])
            
            thm = np.array(thm)
            
            print(thm)
            quv = path[0].fingers_configuration[0]
            J = f.jacobian(quv)
            J[:3] = J[:3]*Constants.MM_TO_METERS
            J_transpose = np.transpose(J)
            
            res = np.matmul(np.linalg.pinv(J_transpose), thm)
            print(np.linalg.norm(res))
            
            #print(self.mj_model.jnt_actgravcomp)
            #print(self.mj_model.actuator_forcelimited)
            #print(self.mj_model.actuator_forcerange)
            
            return
            
            #self.set_forces(path[0])
          
            
            while viewer.is_running() and time.time() - start < 5:
                print('total', self.mj_data.actuator_force)
                self.mujoc_update(viewer)
            
            tau = self.mj_data.qfrc_applied.copy() 
            print('tau', tau)
            print(self.mj_data.qfrc_actuator)
            
            print('netto', self.mj_data.qfrc_smooth)
            print('contraint', self.mj_data.qfrc_constraint)
            
            print('total', self.mj_data.qfrc_applied[2]+self.mj_data.qfrc_actuator[2])
            
            #print(self.test)
            print('force', self.mj_data.qfrc_actuator)   
            
            '''
            for i in range(len(self.mj_model.body_mass)):
                if i == obj.body_id: 
                    c = 0
                    mass = self.mj_model.body_mass[i]
                    inertia = self.mj_model.body_inertia[i, :]
                    self.mj_model.body_mass[i] *= c
                    self.mj_model.body_inertia[i, :] *= c
            '''
            tau = self.mj_data.qfrc_applied.copy() 
            print('tau', tau)
            c = 0
            while viewer.is_running() and c < 500:
                c+= 1
                self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                for i in range(len(path[0].fingers_in_contact)):
                    f = path[0].fingers_in_contact[i]
                    for j in range(0,len(f.model_ctrl_indexes)):
                        #print(self.mj_data.qfrc_actuator[f.model_joint_indexes[j]])
                        self.mj_data.qfrc_applied[f.model_joint_indexes[j]] =tau[f.model_joint_indexes[j]] #- self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
                
                self.mujoc_update(viewer)    
            
            print(self.mj_data.qfrc_actuator)
            for i in range(len(self.mj_data.qfrc_applied)):
                print(i)
                print()
                print(self.mj_data.qfrc_applied[i])
                print(self.mj_data.qfrc_actuator[i])
                print()
            
            
            print('total', self.mj_data.actuator_force)
            print(len(self.mj_data.actuator_force))
            print(len(self.mj_data.qfrc_actuator))
            
            #print('total' , self.mj_data.qfrc_inverse)   
            #print(self.test(path[0]))
            self.idle_simulation(200, viewer)
            return 
            
            #self.mj_data.xfrc_applied[f.model_joint_indexes[-1]] = force
            #print('force', n)
           
            for j in range(0,len(f.model_ctrl_indexes)):
                #print(self.mj_data.qfrc_actuator[f.model_joint_indexes[j]])
                self.mj_data.qfrc_applied[f.model_joint_indexes[j]] = tau[j] - self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
                
                #self.set_forces(path[0])
                #self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                self.mujoc_update(viewer)
                print(c)
            c = 0
            
            while viewer.is_running() and c < 100:
                c+= 1
                self.adjust_forces(self.get_forces(path[0]), path[0])
                self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                self.mujoc_update(viewer)
                print(c)
            
            for i in range(len(self.mj_model.body_mass)):
                if i == obj.body_id: 
                    self.mj_model.body_mass[i] = mass
                    self.mj_model.body_inertia[i, :] = inertia
            
            self.mujoc_update(viewer)
            #self.set_obj_pos_and_rotation(obj, path[0].pos, path[0].rotation)
            diff = self.force_difference(self.get_forces(path[0]), path[0])
            
            self.test(path[0])
            print('diff', diff)
            self.idle_simulation(20, viewer)
            return
            
            force = self.get_forces(path[0])
            diff = self.force_difference(force, path[0])
            
            
            
            print('dif', diff)
            
            #self.idle_simulation(20, viewer)
            
            
            self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
            self.adjust_forces(force, path[0])
            diff = self.force_difference(force, path[0])
            #print('dif', diff)
            #self.idle_simulation(20, viewer)

            adjust_complete = False
            c = 0
            while viewer.is_running() and not adjust_complete:
                c+= 1
                self.set_obj_pos_and_rotation(self.obj, path[0].pos, path[0].rotation)
                self.mujoc_update(viewer)
                #
                if c%100 == 0: 
                    
                    adjust_complete = self.adjust_forces(force, path[0])
                    
                    diff = self.force_difference(force, path[0])
                    print('dif1', diff)
                    print(time.time()- start)
                self.mujoc_update(viewer)
            diff = self.force_difference(force, path[0])
            print('dif1', diff)
            print('idel')
            self.idle_simulation(20, viewer)
            
            diff = self.force_difference(force, path[0])
            print('dif2', diff)
            
    
    
    def set_body_forces(self, fingers, forces):
        for i in range(len(fingers)):
            f = fingers[i]
            xfrc = forces[i]
            self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)] = xfrc*1
            #for j in range(0,len(f.model_joint_indexes)):
            #    print(self.mj_data.qfrc_actuator[f.model_joint_indexes[j]])
            #    self.mj_data.qfrc_applied[f.model_joint_indexes[j]] = - self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
    
    
    def set_forces(self, node):
        object_movement = Physics_Engine.Object_Movement(node.pos, node.rotation)
        fingers = node.fingers_in_contact 
        config = node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, node.fingers_uv_on_obj_surface)
        for i in range(len(normal_forces)):
            n = normal_forces[i] #+ tangential_forces[i]
            tau = Physics_Engine.Rigid_Finger_Physics.joint_torque(fingers[i], config[i], n)
            f = fingers[i]
            force = np.zeros(6)
            force[:3] = n #+ tangential_forces[i]
        
            #self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)] = force*1
            #print('force', n)
            #print(f, self.mj_model.body(f.get_tip_body_id(self.mj_data)).name)
            for j in range(0,len(f.model_joint_indexes)):
                #print(self.mj_data.qfrc_actuator[f.model_joint_indexes[j]])
                self.mj_data.qfrc_applied[f.model_joint_indexes[j]] = tau[j] #- self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
            #print(self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)])
    
    def adjust_body_forces(self,  goal_forces, node):
        dif = self.force_difference(goal_forces, node)
        fingers = node.fingers_in_contact
        adjust_complete = True
        #print('adjust', len)
        print('dif', dif)
        for i in range(len(fingers)):
            f = fingers[i]
            #print(f, i)
            idx = f.get_index_in_shadow_hand()
            if dif[idx] > 0.05: # or dif[idx]< -0.05:
                adjust_complete = False
                f_dif = dif[idx]*100 #np.sign(dif[idx])*0.01 #min(0.001, abs(dif[idx]))
                
                
                force = np.zeros(6)
                force[:3] = f_dif*f.contact_normal(node.fingers_configuration[i][:-2], node.fingers_configuration[i][-2], node.fingers_configuration[i][-1])
      
                self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)] += force
                
                #tau = Physics_Engine.Rigid_Finger_Physics.joint_torque(f, node.fingers_configuration[i], force[:3])
                #print('force', force)
                #print(f, tau)
                #force = goal_forces[idx]*f.contact_normal(node.fingers_configuration[i][:-2], node.fingers_configuration[i][-2], node.fingers_configuration[i][-1])
                #tau1 = Physics_Engine.Rigid_Finger_Physics.joint_torque(f, node.fingers_configuration[i], force)
                #for j in range(0,len(f.model_ctrl_indexes)):
                    #self.mj_data.qfrc_applied[f.model_joint_indexes[j]] += tau[j]
                #    self.mj_data.qfrc_applied[f.model_joint_indexes[j]] =- self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
                #print( [self.mj_data.qfrc_applied[f.model_joint_indexes[j]] for j in range(0,len(f.model_ctrl_indexes))])
                #print(goal_forces)
        return adjust_complete
        
    
    def adjust_forces(self, goal_forces, node):
        
        dif = self.force_difference(goal_forces, node)
        fingers = node.fingers_in_contact
        adjust_complete = True
        #print('adjust', len)
        print('dif', dif)
        for i in range(len(fingers)):
            f = fingers[i]
            #print(f, i)
            idx = f.get_index_in_shadow_hand()
            if dif[idx] > 0.05 or dif[idx]< -0.05:
                adjust_complete = False
                f_dif = np.sign(dif[idx])*0.01 #min(0.001, abs(dif[idx]))
                
                
                force = np.zeros(6)
                force[:3] = f_dif*f.contact_normal(node.fingers_configuration[i][:-2], node.fingers_configuration[i][-2], node.fingers_configuration[i][-1])
      
                #self.mj_data.xfrc_applied[f.get_tip_body_id(self.mj_data)] += force
                
                #tau = Physics_Engine.Rigid_Finger_Physics.joint_torque(f, node.fingers_configuration[i], force[:3])
                #print('force', force)
                #print(f, tau)
                #force = goal_forces[idx]*f.contact_normal(node.fingers_configuration[i][:-2], node.fingers_configuration[i][-2], node.fingers_configuration[i][-1])
                #tau1 = Physics_Engine.Rigid_Finger_Physics.joint_torque(f, node.fingers_configuration[i], force)
                #for j in range(0,len(f.model_ctrl_indexes)):
                    #self.mj_data.qfrc_applied[f.model_joint_indexes[j]] += tau[j]
                #    self.mj_data.qfrc_applied[f.model_joint_indexes[j]] =- self.mj_data.qfrc_actuator[f.model_joint_indexes[j]]
                #print( [self.mj_data.qfrc_applied[f.model_joint_indexes[j]] for j in range(0,len(f.model_ctrl_indexes))])
                #print(goal_forces)
        return adjust_complete
    

    def idle_simulation(self, start_delay, viewer):
        start = time.time()
        cout = 0
        while viewer.is_running() and time.time() - start < start_delay:
            #print('idle')
            print('idle',  time.time() - start )
            #print(time.time() - start)
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
                print('contact force', contact_force)
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
            
            
    
    
    def get_normal_forces(self, node):
        object_movement = Physics_Engine.Object_Movement(node.pos, node.rotation)
        fingers = node.fingers_in_contact 
        config = node.fingers_configuration
        normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(object_movement, fingers, config, node.fingers_uv_on_obj_surface)
        forces =np.zeros((len(self.shadow_hand.fingers), 3))
        for i in range(len(normal_forces)):
            n = normal_forces[i]
            f = fingers[i]
            forces[f.get_index_in_shadow_hand()] = n
        
        return forces
    
    
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
        viewer.sync()
        mujoco.mj_step(self.mj_model, self.mj_data)
        time_until_next_step = self.mj_model.opt.timestep - (time.time() - self.step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
       
                

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
        #print('calculated normal forces', fingers, normal_forces)
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
            #print(result)
            
            #print('tangential_force',result[1]+ result[2])            
            
            
            total_force = total_force + result[:3]
        print('total force', total_force)
            
        for i in range(len(current_element.fingers_in_contact)):
            f = current_element.fingers_in_contact[i]
            quv = current_element.fingers_configuration[i]
            p =f.contact_point(quv[:-2],quv[-2],quv[-1])
            print('calculated contact', type(f), p*Constants.SI_SCALE)
            print('calculated force', np.linalg.norm(normal_forces[i]))

            for j in range(0,len(f.model_joint_indexes)):
                print('actuater', self.mj_data.qfrc_actuator[f.model_joint_indexes[j]])
                print('calculated', self.mj_data.qfrc_applied[f.model_joint_indexes[j]])
            
        print('actual',  self.mj_data.qfrc_inverse)
        print('constraint', self.mj_data.qfrc_constraint)
        print(self.mj_model.actuator_biastype)
                
            
        #self.finger_print(current_element)
        
    
    
    
    
    
    
    
    
    
    
    
    
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
        
        


    
    
    
    
    
    
    
    
    
    
    
    