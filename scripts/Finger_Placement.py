import numpy as np


class Finger_Placement:
    
    def place_finger(self, finger_to_place, obj, obj_pos, obj_rotation, fingers_in_contact, configurations, include_physics):
        self.finger = finger_to_place
        
    
    
    
    def function_to_minimize(self, x):
      
        finger = self.fingers
        min_limits = finger.min_limits
        max_limits = finger.max_limits
        res = 0
        for j in range(len(max_limits)):
            denumerator = (max_limits[j]-min_limits[j])
            min_term = (x[j]-min_limits[j])/denumerator
            max_term = (max_limits[j]-x[j])/denumerator
            res += self.smooth_penalty(min_term) + self.smooth_penalty(max_term)
            
        return res
    
    def smooth_penalty(self, x):
        a = 20
        res = np.exp(-a*x)
        return res
   