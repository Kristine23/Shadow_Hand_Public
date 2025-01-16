

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



class Mujoco_Simulator: 
    
    def __init__(self):   
        
        self.mj_model = mujoco.MjModel.from_xml_path('/home/klinkby/Documents/Mujoco_Py_Workspace/Finger_Tip_Manipulation/Shadow_Hand_21_03_2024/world.xml')    
        self.disable_gravity = False
        if self.disable_gravity:
            self.gravity_constant = 0.0
            #self.mj_model.opt.gravity[:] = [0., 0., -9.81*self.gravity_constant] # disable gravity
        self.gravity = self.mj_model.opt.gravity[:]
        print('gravity ', self.gravity)
        self.mj_data = mujoco.MjData(self.mj_model)  
    
    
    def set_shadow_hand(self, shadow_hand):
        self.shadow_hand = shadow_hand
    
    
    def init_configurations(self, obj, init_node):        
        obj_pos = init_node.pos
        obj_rotation = init_node.rotation
            
        self.set_obj_pos_and_rotation(obj, obj_pos, obj_rotation)
        self.init_fingers_config(obj_pos, init_node)
        
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
        
        val = 0.005 #2
        vec = np.array([val, val,val])
        self.mj_model.geom(obj.name ).size = self.mj_model.geom(obj.name ).size + vec
        names = ["item:site1", "item:site2", "item:site3", "item:site4"]
        for n in names: 
            self.mj_model.site(n).size = self.mj_model.site(n ).size+ vec
        
    
    
    def set_obj_pos_and_rotation(self, obj, obj_pos, rotation):
        mj_quat = mu.rotation_to_mujoco_quat(rotation)
        pos= [obj_pos[i]*Constants.SI_SCALE for i in range(3)]
        self.mj_model.body_pos[obj.body_id]= pos 
        self.mj_model.body_quat[obj.body_id] = mj_quat
        self.mj_data.joint(obj.name).qpos[:3] = pos
        self.mj_data.joint(obj.name).qpos[3:] = mj_quat
        
       
        
    
    def init_fingers_config(self, obj_pos, init_node):
        om = Physics_Engine.Object_Movement(obj_pos, init_node.rotation)
        config, torque = self.get_targets_for_fingers(init_node, False, om)
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
                #print(finger, self.mj_data.qfrc_applied[finger.model_joint_indexes[j]])
                

        
        
        
    
    def get_targets_for_fingers(self, node, only_config = True, om = None):
        fingers = node.fingers_in_contact 
        finger_config = node.fingers_configuration
        if not only_config:
            normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, fingers, finger_config , node.fingers_uv_on_obj_surface)
            self.current_forces = [normal_forces, tangential_forces]
        #print('normal_forces', normal_forces)
        sh_fingers = self.shadow_hand.fingers
        config = []
        joint_torque = []
        for i in range(0, len(sh_fingers)):
            
            finger = sh_fingers[i]
            if finger in fingers: 
                idx = fingers.index(finger)
                quv =finger_config[idx]
                #quv = np.zeros(finger.number_joints+2)
                if not only_config:
                    tourqe =  Physics_Engine.Rigid_Finger_Physics.joint_torque(finger, quv, (normal_forces[idx]))
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
                        quv[0] = - config[2][0]
                    case 4: 
                        quv[1] = finger.min_limits[1]
            config.append(quv)
            if not only_config:
                joint_torque.append(tourqe)
        if only_config: 
            return config
        else: 
            return config, joint_torque
      
            
        
    
    
    
    
    
    def run(self, path, obj):
        
        #plot  = PyPlotting.Finger_Configuration_Track(start_node.fingers_in_contact, start_node.fingers_configuration)
        
        start_node = path[0]
   
    
    
        duration_of_arc = 2
        duration_of_finger_change = 8
       
        
        
        
        
        duration_of_element = 0
    
        start = time.time()
        element_handled = True
        arc_is_handled= False
        current_element = path[0]
        new_element = current_element
        idx = 1
        
        

        
        start_delay = 20 #2
        time_after_simulation = 20
        
        arc_step_time = 0.005
        duration_of_finger_change = 20 # 4
        
        
        counter = 0
       
        gravity_val = 0.8
        #self.mj_model.opt.gravity[:] = [0., 0., -9.81*gravity_val] # disable gravity
        #self.mj_model.opt.gravity[:] = np.zeros(3)
        first_print = 10
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            viewer.sync()
            while viewer.is_running() and time.time() - start < start_delay: #and counter < 1:
                #print('hej')
                print(time.time() - start)
                #self.test_physics(path[0])
                #self.set_obj_pos_and_rotation(obj, current_element.pos, current_element.rotation)
                if first_print< 0:
                
                    self.test_print(current_element)
                    #first_print -= 1
                time.sleep(0.1)
                
                
                #self.finger_print(current_element)
                self.mujoc_update(viewer)
                
                counter += 1
                #self.finger_print(current_element)
                #self.test_print(current_element)
               
            '''
            start = time.time()
            last_update =  start
            
            #self.mj_model.opt.gravity[:] = [0., 0., -9.81] # disable gravity
            
            while viewer.is_running() and time.time() - start < start_delay*2000:
                self.test_print(current_element)
                
                self.mujoc_update(viewer)
                self.test_print(current_element)
               
            '''
            start = time.time()
            last_update =  start
           
            om = None
            

            sim_has_finished = False
            while viewer.is_running(): # and not sim_has_finished:
                
                #self.test_print(current_element)
                    
                
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
                        
                        duration_of_element = duration_of_finger_change
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
            while viewer.is_running() and time.time() - start < time_after_simulation:
                #time.sleep(1)
                print('simulation has finshed')
                print(time.time() - start)
                #self.test_print(current_element)
                #self.plot_conf( current_element.start_node.fingers_in_contact, plot, t)
                #self.plot_touqe( current_element.start_node.fingers_in_contact, plot, t)
                #self.update_touque_on_fingers(obj,om, path[-1].fingers_in_contact, path[-1].fingers_configuration)
                self.mujoc_update(viewer)
            
            #plot.plot()
                


    
    
    
    
    
    
    
    

    def update_body_pos_and_rotation(self, body_name, pos, rotation):
        return
        #print(self.mj_data.mocap_pos)
        #self.mj_data.mocap_pos(body_name) = pos
        
        #self.mj_data.body(self.name).quat = mu.rotation_to_quat(self.world_from_center_rotation)
        

    def is_stady_state_reached(self, old_config, new_config):
        return 
    
    def has_reached_conf(self, fingers, config):
        has_reached_control = True
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
        return
    
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

        
        
    
    
    def update_control_of_fingers(self, fingers, config):
        for i in range(len(fingers)):
            finger = fingers[i]
            quv = config[i]
            for j in range(0, len(finger.model_ctrl_indexes)):
                   
                    val = quv[j]
                    self.mj_data.ctrl[finger.model_ctrl_indexes[j]]= val
        
        #finger = self.shadow_hand.fingers[2]
        #self.mj_data.ctrl[finger.model_ctrl_indexes[0]] = (config[1][0] + config[2][0])/2
        
                   
            
    
    def change_finger(self,fingers_to_add, fingers_to_add_config, fingers_to_add_old_config,  fingers_to_remove, fingers_to_remove_config, fingers_to_remove_new_config, t, duration):
        if t < duration/3:
            finish_time = duration/3    
            devide = t/finish_time
            new_conf = []
            print('finger-----------------------------------------------------------------------', fingers_to_add)
            print(fingers_to_add_config)
            for i in range(len(fingers_to_add_config)):
                conf = fingers_to_add_config[i] #(np.array(fingers_to_add_config[i])-np.array(fingers_to_add_old_config[i]))*devide + np.array(fingers_to_add_old_config[i])
                new_conf.append(conf)
            print('new_conf', new_conf)
            #print('add', fingers_to_add, fingers_to_add_config)
            self.update_control_of_fingers(fingers_to_add, new_conf)
        elif t < 2* duration/3 :
            print('has both fingers', t)
            #finish_time = duration/3    
            #devide = t/finish_time
            #new_conf = []
            #for i in range(len(fingers_to_add_config)):
            #    conf = np.array(fingers_to_add_config[i])
            #    new_conf.append(conf)
                
            #print('add', fingers_to_add, fingers_to_add_config)
            self.update_control_of_fingers(fingers_to_add, fingers_to_add_config)
            
            
        else:
            self.update_control_of_fingers(fingers_to_add, fingers_to_add_config)
            finish_time = duration/3    
            devide = 3-t/finish_time
            new_conf = []
            for i in range(len(fingers_to_remove_config)):
                conf = fingers_to_remove_new_config[i] #np.array(fingers_to_remove_config[i].copy())*devide
                new_conf.append(conf)
            
            #print('removing finger', fingers_to_remove,  new_conf )
            self.update_control_of_fingers(fingers_to_remove, new_conf)
        
        '''
        if t < duration/2:
            finish_time = duration/2    
            devide = t/finish_time
            new_conf = []
            for i in range(len(fingers_to_add_config)):
                conf = np.array(fingers_to_add_config[i].copy())*devide
                new_conf.append(conf)
                
            print('add', fingers_to_add, fingers_to_add_config)
            self.update_control_of_fingers(fingers_to_add, new_conf)
        
        else:
            self.update_control_of_fingers(fingers_to_add, fingers_to_add_config)
            finish_time = duration/2    
            devide = 2-t/finish_time
            new_conf = []
            for i in range(len(fingers_to_remove_config)):
                conf = np.array(fingers_to_remove_config[i].copy())*devide
                new_conf.append(conf)
            
            print('removing finger', fingers_to_remove,  new_conf )
            self.update_control_of_fingers(fingers_to_remove, new_conf)
         '''


              
    
    
    def mujoc_update(self, viewer):
        mujoco.mj_step(self.mj_model, self.mj_data)
        viewer.sync()
        
    
    
    
    def plot_conf(self, fingers, plot, t):
        conf_cal = []
        conf_mujo = []
        for i in range(0, len(fingers)):  
            finger = fingers[i]
            quv_cal = []
            quv_mujo = []
            for j in range(0, len(finger.model_joint_indexes)):
                quv_cal.append(self.mj_data.ctrl[finger.model_ctrl_indexes[j]])
                quv_mujo.append(self.mj_data.qpos[finger.model_joint_indexes[j]])

            quv_cal.append(0)
            quv_cal.append(0)
            quv_mujo.append(0)
            quv_mujo.append(0)
            conf_cal.append(quv_cal)
            conf_mujo.append(quv_mujo)
        
            
        plot.add_points(t, conf_cal, conf_mujo)
        
    
    def plot_touqe(self, fingers, plot, t):
        torqe_cal_fin = []
        torqe_applied_fin =  []
        total_torqe_mujo_fin = []
        passiv_torqe_fin =[]
        for i in range(0, len(fingers)):  
            finger = fingers[i]
            torqe_cal = []
            torqe_applied =  []
            total_torqe_mujo = []
            passiv_torqe =[]
            for j in range(0, len(finger.model_joint_indexes)):
                a = self.mj_data.qfrc_actuator[finger.model_joint_indexes[j]] #touqe udregnet for hvert joint ud fra position og velocity.
                b= self.mj_data.qfrc_applied[finger.model_joint_indexes[j]]  # user tourqe
                c = self.mj_data.qfrc_inverse[finger.model_joint_indexes[j]]  # net external force; should equal qfrc_applied + qfrc_actuator
                d = self.mj_data.mjData.qfrc_passive[finger.model_joint_indexes[j]] # torqe der pasivly er med
                
                    
                torqe_cal.append(a)
                torqe_applied.append(b)
                total_torqe_mujo(c)
                passiv_torqe.append(d)
            
            for arrray in [torqe_cal, torqe_applied, total_torqe_mujo, passiv_torqe]:    
                arrray.append(0)
                arrray.append(0)
            
            torqe_applied_fin.append(torqe_applied)
            torqe_cal_fin.append(torqe_cal)
            total_torqe_mujo_fin.append(total_torqe_mujo)
            passiv_torqe_fin.append(passiv_torqe)
            
            
        
            
        plot.add_points(t, torqe_applied_fin, torqe_cal_fin)
    
    
    
    
    def test_physics(self, init_node):
        
        object_movement = self.PE.create_static_object_movement(init_node.pos, init_node.rotation)
        
        fingers = init_node.fingers_in_contact 
        config = init_node.fingers_configuration
        normal_forces, tangential_forces = self.PE.fingers_normal_and_tangential_forces(object_movement, fingers, config, init_node.fingers_uv_on_obj_surface)
        
        i = 0
        j = 0
        
        finger = fingers[i]
        quv = config[i]
            
        body_id = finger.get_tip_body_id(self.mj_data)            
        tourqe =  self.PF.joint_torque(finger, quv, normal_forces[i])
        
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
        
        



    def test_print(self, current_element ):
        #print('pos1', self.mj_data.qpos[0], self.mj_data.ctrl[0])
        #print('pos2', self.mj_data.qpos[1], self.mj_data.ctrl[1])
        
        l = len(self.mj_data.contact)
        print('number of contacts', l)
        print('contact fiction', self.mj_data.contact.friction)
        #print('contact', (self.mj_data.contact))
        #print(self.mj_data.contact)
        for i in range(l):
            #print(self.mj_data.contact[i])
            
            
            
            id1 = self.mj_data.contact.geom1[i]
            id2 = self.mj_data.contact.geom2[i]
            body_id1 = self.mj_model.geom(id1).bodyid
            body_id2 = self.mj_model.geom(id2).bodyid
            print('contact:',i, ':', self.mj_model.body( body_id1).name, self.mj_model.body( body_id2).name,  id1, id2) 
            print('pos:', self.mj_data.contact.pos[i]) 
            
        for i in range(len(current_element.fingers_in_contact)):
            f = current_element.fingers_in_contact[i]
            quv = current_element.fingers_configuration[i]
            p =f.contact_point(quv[:-2],quv[-2],quv[-1])
            print('calculated contact', type(f), p*Constants.SI_SCALE)

        
        self.finger_print(current_element)
        
        
        
        print('\n \n')