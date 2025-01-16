import Physics_Engine
import numpy as np
import Node
import Arc
import String_Handler

class Config_Deliver:
    
    WRITE_COPLED_AS_ONE = False
    
    def __init__(self, shadow_hand, writing_interval):
        self.shadow_hand = shadow_hand
        self.radian_change_pr_sec = 1.57/3 # pi/2 pr sec
        self.writing_interval = writing_interval
        self.max_radian_change = self.radian_change_pr_sec * writing_interval
        
    
    def get_config(self, current_config, target_config, adjust_writing_interval= None):
        if adjust_writing_interval != None: 
            self.max_radian_change *= adjust_writing_interval
            
        
        
        max_distance = self.max_joint_distance(current_config, target_config)
        is_target_reached = False
        if max_distance <= self.max_radian_change:
            
            is_target_reached = True
            new_config= target_config.copy()
        else:
            new_config = []
            for i in range(0, len(current_config)):
                new_finger_config = []
                for j in range(0, len(current_config[i])):
                    distance = abs(current_config[i][j] - target_config[i][j])
                    if distance <= self.max_radian_change:
                        new_finger_config.append(target_config[i][j])
                    else:
                        if current_config[i][j] < target_config[i][j]:
                            new_finger_config.append(current_config[i][j] + self.max_radian_change)
                        else:
                            new_finger_config.append(current_config[i][j] - self.max_radian_change)
                new_config.append(new_finger_config)
                
                
        if adjust_writing_interval != None:
             self.max_radian_change /= adjust_writing_interval
        
        return new_config, is_target_reached
    
    
    
    def max_joint_distance(self, current_config, target_config):
        max_distance = 0
        for i in range(0, len(current_config)): 
            for j in range(0, len(current_config[i])):
                distance = abs(current_config[i][j] - target_config[i][j])
                if distance > max_distance:
                    max_distance = distance
        return max_distance
    
    
    
    
    
    
    def q_sh_from_quv(self, fingers_in_contact, fingers_config):
 
        config = []
        sh_fingers = self.shadow_hand.fingers
        for i in range(0, len(sh_fingers)):
            finger = sh_fingers[i]
            
            if finger in fingers_in_contact: 
                idx = fingers_in_contact.index(finger)
                q = fingers_config[idx][:-2]
            else: 
                q = np.zeros(finger.number_joints)
                if i == 1: 
                        q[0] = finger.min_limits[0]
                elif i == 2: 
                        q[0] = config[1][0]
                elif i == 3:
                        q[0] = -config[2][0]
                elif i == 4: 
                        q[1] = finger.min_limits[1]
            res_q = q 
            config.append(res_q)
        

        finger = sh_fingers[1]
            
        if finger not in fingers_in_contact:
            config[1][0] = config[2][0]


        config = self.config_to_real(config)
            
        return config
    
    
    def config_to_real(self, config):
        real_config = [config[i] for i in range(1, len(config))]
        real_config.append(config[0])
        return real_config
        
    
    
    
    
    
    
    
    
    
    
    
    
    