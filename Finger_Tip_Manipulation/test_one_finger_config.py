import Sim_to_Real
import String_Handler



class Test_Coupled: 
    
    def __init__(self, shadow_hand, finger):
        self.shadow_hand = shadow_hand
        self.finger = finger
        self.has_limits_been_reached = False
        self.max_limits = finger.max_limits[:-4]
        self.config = [finger.min_limits[:-2] for i in range(len(finger.min_limits[:-2]))]
        self.max_coupled = 1.57*2
        coupled_min = 1.50
        self.config[-2] = coupled_min
        

        
    
    def write_all_config(self, finger_idx):
        finger = self.shadow_hand.fingers[finger_idx]
        min_limits = finger.min_limits[:-4]
        max_limits = finger.max_limits[:-4]
        change = 0.01
        config = [min_limits[i] for i in range(len(min_limits))]
        coupled_min = 1.50
        coupled_max = 1.57*2
        max_joint = 1.57
        max_reached = 0
        file_name =  'robot_config_finger_' + str(finger_idx)+ '_.txt'
        f = open(file_name, 'w')
        c = 0
        sim_to_real = Sim_to_Real.Sim_to_Real(self.shadow_hand)
        while max_reached < len(min_limits):
            max_reached = 0
            c+=1
            print(c)
            for i in range(len(min_limits)):
                for j in range(150, 157*2,1):
                    val = j/100.
                    configuration_to_reverse = [config[r] for r in range(len(config))] + [min(val, max_joint), max(0, val-max_joint)]
                    self.write_config(f, configuration_to_reverse)
            
                if config[i] + change  < max_limits[i]:
                    config[i] += change
                else: 
                    max_reached += 1
                    config[i] = max_limits[i]
    
    
    
    def run_trough_all_config(self, finger_idx):
        finger = self.shadow_hand.fingers[finger_idx]
        min_limits = finger.min_limits[:-4]
        max_limits = finger.max_limits[:-4]
        change = 0.01
        config = [min_limits[i] for i in range(len(min_limits))]
        coupled_min = 1.50
        coupled_max = 1.57*2
        max_joint = 1.57
        max_reached = 0
        file_name =  'results_finger_' + str(finger_idx)+ '_.txt'
        f = open(file_name, 'w')
        c = 0
        sim_to_real = Sim_to_Real.Sim_to_Real(self.shadow_hand)
        while max_reached < len(min_limits):
            max_reached = 0
            c+=1
            print(c)
            for i in range(len(min_limits)):
                for j in range(150, 157*2,1):
                    val = j/100.
                    configuration_to_reverse = [config[r] for r in range(len(config))] + [min(val, max_joint), max(0, val-max_joint)]
                    self.write_calculated_config(f, configuration_to_reverse)
                    configuration_to_reverse += [0,0]
                    
                    config_to_reverse_and_send = sim_to_real.get_config_targets([finger], [configuration_to_reverse])
                    
                    send_to_robot = [c[::-1] for c in config_to_reverse_and_send[1:]] + [config_to_reverse_and_send[0][::-1] ]
                    #print(send_to_robot)
                    
                    resived_config =  send_to_robot # change to real robot.
                    config_to_revers = resived_config[finger_idx-1]                       
                
                    from_robot = config_to_revers[::-1]
                    
                    self.write_robot_config(f, from_robot)
                    
                    # format: 
                    #file_name: type(finger).text
                    # s: configuration
                    # r: resived
                
        
                    #revers configuration and sendt to robot. 
                    #store configuration
                    #feed configuration to robot
                    #read configuration
                    #store configuration
                
            
                if config[i] + change  < max_limits[i]:
                    config[i] += change
                else: 
                    max_reached += 1
                    config[i] = max_limits[i]
    

    
    def wirte_config(self, file_manipulater, config):
        str_array = String_Handler.array2string(config)
        file_manipulater.write(str_array + '\n')
        
    
    
    def write_calculated_config(self, file_manipulater, config):
        #print(config)
        str_array = String_Handler.array2string(config)
        file_manipulater.write('s:' + str_array + '\n')
        
    def write_robot_config(self, file_manipulater, config):
        str_array = String_Handler.array2string(config, 0)
        file_manipulater.write('r:' + str_array + '\n')
        