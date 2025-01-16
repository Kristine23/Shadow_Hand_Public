import Node
import Arc
import Physics_Engine
import numpy as np
import Config_Deliver
import Constants
import Math_Utilities as mu
import String_Handler

class Force_Node:

    def __init__(self, normal_forces):
        self.normal_force = normal_forces

    def get_force(self): 
        return self.normal_force
    

class Force_Arc:

    def __init__(self, normal_forces):
        self.normal_force = normal_forces
        
    
    def get_force(self, t, duration_of_arc):
        x, time_list, blend_t =  Arc.Arc().get_conf_and_time_for_parablic_blend(t, duration_of_arc, self.normal_force)
        
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
            q = mu.Interpolation().parabolic_blend(t,x[0][i], x[1][i], x[2][i], time_list[0], time_list[1], time_list[2], blend_t)
                #q = x[0][i][j]
            res.append(q)
                
            
        #print('res', res)
        return res

class Force_Path_Calculator:
    
    def __init__(self, path, shadow_hand, writing_interval):
        fingers = [shadow_hand.fingers[i] for i in range(1,5)]
        fingers.append(shadow_hand.fingers[0])
        
        deliver = Config_Deliver.Config_Deliver(shadow_hand, writing_interval)
        
        self.force_path = []
        for i in range(len(path)):
            finger_normal_forces = []
            
            path_elm = path[i]
            if type(path[i]) == Node.Node:
                om = Physics_Engine.Object_Movement(path_elm.pos, path_elm.rotation)
                normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, path_elm.fingers_in_contact, path_elm.fingers_configuration , path_elm.fingers_uv_on_obj_surface)
                for i in range(5):
                    f = fingers[i]
                    if f in path_elm.fingers_in_contact:
                        idx = path_elm.fingers_in_contact.index(f)
                        force = np.linalg.norm(normal_forces[idx])
                        finger_normal_forces.append(force)
                    else:
                        finger_normal_forces.append(0)
                force_elm = Force_Node(finger_normal_forces)
            else: # type is Arc
                
                start_q = deliver.q_sh_from_quv(path_elm.start_node.fingers_in_contact, path_elm.start_node.fingers_configuration)
                end_q= deliver.q_sh_from_quv( path_elm.target_node.fingers_in_contact, path_elm.target_node.fingers_configuration)
                #print('start_q', start_q)
                #print('end_q', end_q)
                max_diff = deliver.max_joint_distance(start_q, end_q)
                duration_of_element = (max_diff / deliver.radian_change_pr_sec + deliver.writing_interval)
                
                time_interval = duration_of_element / (len(path_elm.config_list)-1)
                
                for i in range(len(path_elm.config_list)):
                    config = path_elm.config_list[i]
                    
                    obj_pos = path_elm.obj_pos_list[i]
                    
                    uv = path_elm.obj_uv_list[i]
                    rotation = path_elm.obj_rotation_list[i]
                    
                    t = i * time_interval
                    obj_acc = path_elm.get_obj_acceleration(t, duration_of_element)
                    
                    rotation_vel = path_elm.get_obj_rotation_velocity(t, duration_of_element) * Constants.SI_SCALE
                    rotation_acc = path_elm.get_obj_rotation_acceleration(t, duration_of_element)
                    
                    
                    
                    #print('rotation_vel', rotation_vel)
                    

                    om = Physics_Engine.Object_Movement(obj_pos, rotation, obj_acc, rotation_vel, rotation_acc)    
                    normal_forces, tangential_forces = Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, path_elm.start_node.fingers_in_contact, config , uv)
                    correct_forces = Physics_Engine.Physics_Engine.is_forces_correct(om, normal_forces, tangential_forces)
                    if correct_forces:
                        forces_in_itt = []
                        for i in range(5):
                            f = fingers[i]
                            if f in path_elm.start_node.fingers_in_contact:
                                idx =  path_elm.start_node.fingers_in_contact.index(f)
                                force = np.linalg.norm(normal_forces[idx])
                                forces_in_itt.append(force)
                            else:
                                forces_in_itt.append(0)
                    else: 
                        forces_in_itt = None
                    finger_normal_forces.append(forces_in_itt)
                force_elm = Force_Arc(finger_normal_forces)   
                print(finger_normal_forces)
                 
            self.force_path.append(force_elm)
            
        self.adjust_path()
        
        
        for elm in self.force_path:
            print(elm.normal_force)
            
    def adjust_path(self):
        for i in range(len(self.force_path)):
            elm = self.force_path[i]
            if type(elm) == Force_Arc:
                elm.normal_force[0] = self.force_path[i-1].normal_force
                elm.normal_force[-1] = self.force_path[i+1].normal_force
                stable_idx = 0
                idx = 0
                while idx < len(elm.normal_force):
                    if elm.normal_force[idx] != None:
                        if stable_idx < idx-1:
                            for j in range(stable_idx+1, idx):
                                new_force = []
                                for h in range(5):
                                    
                                    force =  mu.Interpolation().linear_interpolation(j, elm.normal_force[stable_idx][h], elm.normal_force[idx][h], stable_idx, idx)
                                    new_force.append(force)
                                elm.normal_force[j] = new_force
                             #do stuff
                            print('do stuff', stable_idx, idx)
                        
                        stable_idx = idx
                        
                    idx += 1
                        

class File_Handler:

    def write_path(path, file_name = 'force_path.txt'):
        open(file_name, 'w').close() #remove content
        f = open(file_name, "a")
        for e in path:
            if type(e)== Force_Arc:
                f.write("Arc")
                s = String_Handler.array2string(e.normal_force, 1)
            else:
                f.write("Node")
                s = String_Handler.array2string(e.normal_force, 0)
            f.write(s)
        f.close()
        
    def read_path( file_name = 'force_path.txt'):
    
        
        f = open(file_name, "r")
        s = f.read()
        f.close()
        node_arc_strings = s.split("Node")[1:]
        path = []
        print(len(node_arc_strings))
        
        #print('\n\n\n her')
        for i in range(len(node_arc_strings)):
            #print(i)
            string = node_arc_strings[i]
            node_arc_str = string.split("Arc")
            node_str = node_arc_str[0]
            
            #print(node_str)
            
            normal_force = String_Handler.array_from_sting(node_str)

            node = Force_Node(normal_force)
            print('node', normal_force)
            
            path.append(node)
            
            if len(node_arc_str) > 1: 
                arc_str = node_arc_str[1]
                normal_force = String_Handler.array_from_sting(arc_str)
                arc = Force_Arc(normal_force)
                path.append(arc)
                print('arc', normal_force)

        return path
        