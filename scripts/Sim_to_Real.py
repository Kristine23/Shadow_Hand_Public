import Physics_Engine
import numpy as np
import Node
import Arc
import String_Handler

class Sim_to_Real:
    
    #to do function to read, and remeber to change target so it fits the shadowhand.
    WRITE_COPLED_AS_ONE = False
    
    def __init__(self, shadow_hand):
        self.shadow_hand = shadow_hand
 
    
    def store_finger_config_for_path(self, path, file_name = 'q_for_path.txt', zero_start = True):
        radian_change_pr_sec = 1.57
        writing_interval = 0.01 # 10 ms
        max_radian_change = radian_change_pr_sec * writing_interval
        
        f = open(file_name, 'w')
        if zero_start:
            current_config = [np.zeros(f.number_acts(self.WRITE_COPLED_AS_ONE)) for f in self.shadow_hand.fingers]
        else: 
            current_config = self.get_config_targets(path[0].fingers_in_contact, path[0].fingers_configuration)
            
        start_config = current_config
        
        
        #element = path[0]
        #current_config = self.get_config_targets(element.fingers_in_contact, element.fingers_configuration)
        
        array = String_Handler.array2string(current_config, 1)
        f.write(array + '\n')
        
        #write current config to file. 
        for i in range(len(path)):
            element = path[i]

            
            if type(element)== Node.Node:                
                fingers_in_contact = element.fingers_in_contact
                fingers_config = element.fingers_configuration
             
                
                target_configurations = self.get_config_targets(fingers_in_contact, fingers_config)
                self.write_configurations_to_file(current_config, target_configurations, f, max_radian_change)
                current_config = target_configurations
                
                
            if type(element) == Arc.Arc:
                print('arc')
                fingers_in_contact = element.start_node.fingers_in_contact
                start_config = self.get_config_targets(fingers_in_contact, element.start_node.fingers_configuration)
                end_config = self.get_config_targets(fingers_in_contact, element.target_node.fingers_configuration)
                max_diff = self.max_joint_distance(start_config, end_config)
                duration = (max_diff / radian_change_pr_sec + writing_interval)
                
                
                current_time = 0
                while current_time < duration:            
                    target_config = self.get_config_targets(fingers_in_contact, element.get_configuration(current_time, duration))
                    self.write_configurations_to_file(current_config, target_config, f, max_radian_change)
                    current_config = target_config
                    current_time += writing_interval
                    
                    
                    
                    
                self.write_configurations_to_file(current_config, end_config, f, max_radian_change)
                current_config = end_config
                
                
                
        f.close()
    
    def write_configurations_to_file(self, current_config, target_config, file, max_radian_change):
        is_target_reached = False
        while not is_target_reached:
            new_config, is_target_reached = self.new_config(current_config, target_config, max_radian_change)
            current_config = new_config
            
            array = String_Handler.array2string(current_config, 1)
            file.write(array + '\n')
            if len(array[1]) ==3:
                print(array)
            
    
    def new_config(self, current_config, target_config, max_radian_change):
        max_distance = self.max_joint_distance(current_config, target_config)
        is_target_reached = False
        if max_distance <= max_radian_change:
            
            is_target_reached = True
            new_config= target_config.copy()
        else:
            new_config = []
            for i in range(0, len(current_config)):
                new_finger_config = []
                for j in range(0, len(current_config[i])):
                    distance = abs(current_config[i][j] - target_config[i][j])
                    if distance <= max_radian_change:
                        new_finger_config.append(target_config[i][j])
                    else:
                        if current_config[i][j] < target_config[i][j]:
                            new_finger_config.append(current_config[i][j] + max_radian_change)
                        else:
                            new_finger_config.append(current_config[i][j] - max_radian_change)
                new_config.append(new_finger_config)
        
        return new_config, is_target_reached
        
        
    
    def max_joint_distance(self, current_config, target_config):
        max_distance = 0
        for i in range(0, len(current_config)): 
            for j in range(0, len(current_config[i])):
                distance = abs(current_config[i][j] - target_config[i][j])
                if distance > max_distance:
                    max_distance = distance
        return max_distance
    
    
    
    def get_config_targets(self, fingers_in_contact, fingers_config):
        config = []
        sh_fingers = self.shadow_hand.fingers
        for i in range(0, len(sh_fingers)):
            finger = sh_fingers[i]
            if finger in fingers_in_contact: 
                idx = fingers_in_contact.index(finger)
                q =fingers_config[idx][:-2]
            else: 
                q = np.zeros(finger.number_joints)
                match i:
                    case 1:
                        q[0] = finger.min_limits[0]
                    case 2: 
                        q[0] = config[1][0]
                    case 3:
                        q[0] = config[2][0]
                    case 4: 
                        q[1] = finger.max_limits[1]
            res_q = q
            #print(finger, q)
            if i != 0 and self.WRITE_COPLED_AS_ONE: 
                res_q = q[:-1]
                res_q[-1] += q[-1] 
           
            config.append(res_q)
        return config
    
    

    
    
    
    
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
                        quv[0] = config[2][0]
                    case 4: 
                        quv[1] = finger.max_limits[1]
            config.append(quv)
            if not only_config:
                joint_torque.append(tourqe)
        if only_config: 
            return config
        else: 
            return config, joint_torque
        
    
    
    
    
    
    def create_file_subset_of_fingers_moving(self, from_file_name, file_name, fingers_idx, init_other_fingers = False):
        with open(from_file_name) as f:
            lines = [line.rstrip() for line in f]
        f.close()
        f = open(file_name, 'w')
        for line in lines:
            #print('line', line)
            array = String_Handler.array_from_sting(line.strip())
            #print(array)
            new_array = []
            for i in range(5):
                if i in fingers_idx:
                    new_array.append(array[i])
                else: 
                    new_array.append([0.0 for j in range(len(array[i]))])
                    if init_other_fingers:
                        finger = self.shadow_hand.fingers[i]
                        match i:
                            case 1:
                                new_array[i][0] = finger.min_limits[0]
                            case 2: 
                                new_array[i][0] = new_array[1][0]
                            case 3:
                                new_array[i][0] = new_array[2][0]
                            case 4: 
                                new_array[i][1] = finger.max_limits[1]
            #print(new_array)
            
            f.write(String_Handler.array2string(new_array, 1) + '\n')
        f.close()
        
     