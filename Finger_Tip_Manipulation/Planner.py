import Arc
import Node
import PyPlotting
import Math_Utilities as mu
import numpy as np
import random
import Grasp_Handler
import Constants
import Physics_Engine
import Graph_Utilities
import Shadow_Hand
import gc



class Planner:
    
   
    
    ROLL_ANGLE = 0.01 # used for roll bør måske fjernes. 
    
    #object values given pr. random step.
    #values given in mm
    max_pos_movement = 30
    max_roll_distance = 10 #movement of finger roll, used to find random config. ca. max 7 mm pr pi/2 ved kig på fingeren, derfor 3 cm i alt. 
    max_rotation_distance = np.pi/2
    
    
    
    def __init__(self, obj, shadow_hand): #, physics_engine):
        
        self.obj = obj
        self.shadow_hand = shadow_hand
        #self.PE = physics_engine
        self.gh = Grasp_Handler.Grasp_Creator()
        #Arc.Arc.set_obj(obj)
        self.graph_nodes = None
        self.fingers_for_grasp= []
        
        
    def RRT_extension(self,start_node, target_node):
        self.graph_nodes = [start_node] 
        start_node.init_as_graph_node() 
        
        path_exists = False
        max_itt = 5000
        i = 0
        best = target_node.distance_between_nodes(start_node)
        
        idx_to_explore = 0 # index such that every node before that has been searched if it can reach target. 
        while not path_exists and i < max_itt: 
            i += 1
            
            print('itteration of max_it', i, ' /', max_itt)
            print('best', best)
           
            print('number_nodes', len(self.graph_nodes))
            
            if idx_to_explore < len(self.graph_nodes):
                node_to_explore = self.graph_nodes[idx_to_explore]
                path_exists = self.create_arc(node_to_explore, target_node)
                idx_to_explore += 1
            else:
                node = self.graph_nodes[random.randint(0, len(self.graph_nodes)-1)]
                self.extend_with_random_configuration(node)
                
                self.make_new_grasp_for_node(node)
         
            
            if i % 10 == 0: 
                closest_node = Graph_Utilities.closest_node(target_node, self.graph_nodes)
                best = target_node.distance_between_nodes(closest_node)
                
        
        print('total_itt', i)
        #print('best', self.graph_nodes[0][0])
        print('number of nodes', len(self.graph_nodes))
        if path_exists:
            goal_node = Graph_Utilities.closest_node(target_node, self.graph_nodes)
            path = Graph_Utilities.find_shortest_path(self.graph_nodes[0], goal_node, self.graph_nodes)
        else: 
            path = None
        return path, path_exists
        
        
        
        
    def init_algoritms(self, start_node):
        self.graph_nodes = [start_node] 
        start_node.init_as_graph_node() 
        self.add_fingers_grasp(start_node.fingers_in_contact)
        
    def create_road_map(self, start_node, store_map = False, file_name = 'roadmap.txt', extended_file_name = None):
        if extended_file_name == None: 
            self.init_algoritms(start_node)
        else:
            self.graph_nodes = Graph_Utilities.read_graph(self.shadow_hand, extended_file_name)
            for node in self.graph_nodes: 
                self.add_fingers_grasp(node.fingers_in_contact)
        
        max_itt = 6000
        i = 0
        best = 0
        best_rot = 0
        print('number nodes before----------------------------', len(self.graph_nodes))
        if len(self.graph_nodes)>=2: 
            best = Graph_Utilities.find_max_difference(self.graph_nodes)
            best_rot = Graph_Utilities.find_max_rot_diff(self.graph_nodes)
        print('after_best')
        while i < max_itt: 
            print('main_itterations................................................', i, ' / ', max_itt)
            print('number of nodes', len(self.graph_nodes))
            print('max_dif', best)
            print('best_rot', best_rot)
            i += 1
            
            target_rotation, target_pos = self.create_truely_random_config()
            closest_nodes = Graph_Utilities.closest_nodes_to_obj_config(self.graph_nodes, target_rotation, target_pos, self.fingers_for_grasp) 
            print('number_closest_nodes', len(closest_nodes))
            #node = self.graph_nodes[random.randint(0, len(self.graph_nodes)-1)]
            self.extend_with_random_configuration(target_rotation, target_pos, closest_nodes)
             
            node = closest_nodes[random.randint(0, len(closest_nodes)-1)]
            self.make_new_grasp_for_node(node)
            '''
            if i % 3!= 0: 
                self.extend_with_random_configuration(node)
            else: 
                self.make_new_grasp_for_node(node)
            '''
            if i %10 == 0 and len(self.graph_nodes)>=2: 
                best = Graph_Utilities.find_max_difference(self.graph_nodes)
                best_rot = Graph_Utilities.find_max_rot_diff_nodes(self.graph_nodes)
            if i % 50 == 0 and store_map:
                Graph_Utilities.write_graph(self.graph_nodes, file_name)
          
        if store_map: 
            Graph_Utilities.write_graph(self.graph_nodes, file_name)
    
    
    def add_fingers_grasp(self, fingers_in_grasp):
        if fingers_in_grasp not in self.fingers_for_grasp:
            self.fingers_for_grasp.append(fingers_in_grasp)
        
    
          
    def extend_with_random_configuration(self,target_rotation, target_pos, closest_nodes ):
        #target_rotation, target_pos, target_uv_thumb = self.create_random_configuation(node)
        #print('her')
        #closest_nodes = Graph_Utilities.closest_nodes_to_obj_config(self.graph_nodes, target_rotation, target_pos, self.fingers_for_grasp) # target_uv_thumb, self.fingers_for_grasp)
       
        for closest_node in closest_nodes:
            '''
            target_uv = [] #target_uv_thumb]
            for i in range(0, len(node.fingers_in_contact)):
                if type(node.fingers_in_contact[i]) == Shadow_Hand.Thumb:
                    new_uv = self.uv_based_on_node(node.fingers_uv_on_obj_surface[i])
                    target_uv.append(new_uv)
                if type(node.fingers_in_contact[i]) == Shadow_Hand.LF :
                    new_uv = self.uv_based_on_node(node.fingers_uv_on_obj_surface[i])
                    target_uv.append(new_uv)
                else: 
                    target_uv.append(node.fingers_uv_on_obj_surface[i])
            '''
            target_node = Node.Node(target_rotation, target_pos, closest_node.fingers_in_contact) #, fingers_uv_on_obj_surface = target_uv)
            self.create_arc(closest_node, target_node)
            
    
    def create_truely_random_config(self):
        axis = np.empty(3)
        target_pos = np.empty(3)
        for i in range(3):
            axis[i] = random.random() - 0.5 
             
            limits = self.obj.CENTER_LIMITS[i]
            target_pos[i] = limits[0] + random.random()* (limits[1]- limits[0])
        
        target_rotation = mu.rotation_from_axises_and_angle(axis, random.random()*self.max_rotation_distance)
        
        return target_rotation, target_pos  
        
                      
            
    def create_random_configuation(self, node):
        axis = np.empty(3)
        target_pos = np.empty(3)
        for i in range(3):
            axis[i] = random.random() - 0.5 
            #limits = self.obj.CENTER_LIMITS[i]
            target_pos[i]= node.pos[i] + (random.random()*2 - 1)* self.max_pos_movement

        angle = random.random()*2*np.pi
        target_rotation = mu.rotation_from_axises_and_angle(axis, angle)
        axis, theta = mu.difference_between_rotations(node.rotation, target_rotation)
        #print(difference_between_nodes)
        #new_rotation = difference_between_nodes[:3] + [random.random()*self.max_rotation_distance]
        target_rotation = np.matmul(node.rotation, mu.rotation_from_axises_and_angle(axis, random.random()*self.max_rotation_distance))
        
        
        
        target_uv_thumb = self.uv_based_on_node(node.fingers_uv_on_obj_surface[0] ) 
        return target_rotation, target_pos, target_uv_thumb
    
    def uv_based_on_node(self, current_uv):
        guessed_direction = [ (random.random()-0.5), (random.random()-0.5)]    
        distance =  random.random()*self.max_roll_distance
        uv_guess =[current_uv[0] + guessed_direction[0], current_uv[1] + guessed_direction[1]]
        curent_dist = self.obj.surface_distance(current_uv, uv_guess)
        scalar = curent_dist/distance
        new_uv = [current_uv[0] + guessed_direction[0]/scalar, current_uv[1] + guessed_direction[1]/scalar]
        return new_uv
        
    
    def create_arc(self, start_node, target_node):
        arc = Arc.Arc()
        res = arc.test_existens(start_node, target_node, True)
        
        
        is_target_reached, time_reached, stable_obj_pos, stable_quv, stable_uv_obj, obj_rotation= res
        
        
        #obj_rotation, obj_pos, uv_contacts_on_obj = self.create_intermediate_node(start_node, target_node, obj_pos, time_fraction_reached)
        
        
        current_differnece = start_node.graph_distance(obj_rotation, stable_obj_pos) #, stable_uv_obj[0]) 
        
        print('best differnece', current_differnece)
        eps = 10**(-2)
        is_possible_to_create_node = False
        if current_differnece > eps:
            print('---------------------------------------------------------try to add node')
            
            new_node = Node.Node(obj_rotation, stable_obj_pos) #, start_node.fingers_in_contact, stable_quv, fingers_uv_on_obj_surface = stable_uv_obj)
            res = arc.test_existens(start_node, new_node)
            
            is_target_reached, time_reached, stable_obj_pos, stable_quv, stable_uv_obj, obj_rotation= res 
        
            print('test arc', is_target_reached)
            if is_target_reached:
                new_node.fingers_in_contact = start_node.fingers_in_contact
                new_node.fingers_configuration = stable_quv
                new_node.fingers_uv_on_obj_surface = stable_uv_obj
                is_possible_to_create_node = self.try_add_node_and_edge(start_node, new_node)  
           
        return  is_possible_to_create_node
    
        
    def create_intermediate_node(self, start_node, target_node, obj_pos, fraction_distance):
        rotation_dif = mu.difference_between_rotations(start_node.rotation, target_node.rotation)
        obj_rotation = np.matmul(start_node.rotation, mu.rotation_from_axises_and_angle(rotation_dif[0], rotation_dif[1]*fraction_distance))
        #obj_pos = start_node.pos + (target_node.pos-start_node.pos)*fraction_distance
        uv_contacts_on_obj = []
        for i in range(len(start_node.fingers_uv_on_obj_surface)): 
            uv = start_node.fingers_uv_on_obj_surface[i]
            #print('uv')
            #print(uv)
            uv_t = target_node.fingers_uv_on_obj_surface[i]
            #print(uv_t)
            new_uv = [uv[i]+ (uv_t[i]-uv[i])*fraction_distance for i in range(2)]
            uv_contacts_on_obj.append(new_uv)
        return obj_rotation, obj_pos, uv_contacts_on_obj
        
        
    
    def try_add_node_and_edge(self,start_node, new_node):
        is_possible = False
        is_grasp_feasble = self.gh.is_grasp_feaseble(new_node.fingers_in_contact, new_node.fingers_configuration, new_node.pos, new_node.rotation, new_node.fingers_uv_on_obj_surface) #Does not include test of how far the fingers are from each other.

        if is_grasp_feasble: 
            print('adding')
            is_possible = True
            new_node.init_as_graph_node()
            new_node.add_edge(start_node)
            self.graph_nodes.append(new_node)
        return is_possible
             
             
             
    def make_new_grasp_for_node(self, node):
        remove_idx = random.randint(0,len(node.fingers_in_contact)-1) #random.randint(1,len(node.fingers_in_contact)-1)
        self.create_new_grasp(node, remove_idx) 
            
            
    # create random grasp kan forbedres en del
    def create_new_grasp(self, node, finger_idx_to_remove):
        # antager at der kun er 3 fingre paa altid. maybe remove some. 
        fingers = node.fingers_in_contact.copy()
        posible_fingers = []
        for finger in self.shadow_hand.fingers:
            if finger not in fingers:
                posible_fingers.append(finger)
        
        insert_idx = random.randint(0,len(posible_fingers)-1)
        finger = posible_fingers[insert_idx]    
        uv_o_limits = self.obj.uv_limits()
        u = random.uniform(uv_o_limits[0][0],uv_o_limits[0][1])
        v = random.uniform(uv_o_limits[1][0],uv_o_limits[1][1])
        
        
        
        new_grasp_found, quv = self.gh.add_finger_to_grasp(finger,[u,v], self.obj, node.rotation, node.pos, node.fingers_in_contact, node.fingers_configuration)
        
        
        print('grasph_is_found', new_grasp_found)
        
        if new_grasp_found: 
            uv_on_object = node.fingers_uv_on_obj_surface.copy()
            fingers_configuration = node.fingers_configuration.copy()
            
            p_f = finger.contact_point(quv[:-2],quv[-2], quv[-1] )
            u_o,v_o = self.obj.uv_for_surface_point(p_f, node.rotation, node.pos)
            fingers.append(finger)
            uv_on_object.append([u_o,v_o])
            fingers_configuration.append(quv)
            
            is_intersecting = self.gh.is_finger_intersecting(fingers, fingers_configuration)
            print('four_fingers intersecting', is_intersecting)
            if not is_intersecting:
                
                fingers.pop(finger_idx_to_remove)
                uv_on_object.pop(finger_idx_to_remove)
                fingers_configuration.pop(finger_idx_to_remove)
                    
                
                is_new_grasp_feaseble = self.gh.is_grasp_feaseble(fingers, fingers_configuration, node.pos, node.rotation, uv_on_object)
                
                
                if is_new_grasp_feaseble:
                    new_node = Node.Node(node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
                    new_node.init_as_graph_node()
                    new_node.add_edge(node)
                    self.graph_nodes.append(new_node)
                    self.add_fingers_grasp(fingers)
        
        return
    







    def shorten_path(node_path):
        path = node_path
        shortende_path = [path[0]]
        i = 1
        while i < len(path):
            start_node = shortende_path[-1]
            target_node = path[i]
            contain_same_fingers =start_node.contain_same_fingers(target_node)
            j = 0
            
            if contain_same_fingers:
                
                
                new_target_node = target_node
                while i+j+1 < len(path) and start_node.contain_same_fingers(new_target_node):
                    j += 1
                    new_target_node = path[i+j]
                
                if j>0:
                    j-=1
                
                
                #j = 0
                
                print('number of posible short cuts', j)
                
                arc = Arc.Arc()
                
                while j > 0:
                    target_node = path[i+j]
                    #print('target2', target_node.fingers_in_contact)
                    if start_node.id < target_node.id: 
            
                        res = arc.test_existens(start_node, target_node)
                    else: 
                        res = arc.test_existens(target_node, start_node)
                    exists = res[0]
                    if exists:
                        break
                    j-= 1
                    
                
                

                print('real number short cuts', j)
                
                target_node = path[i+j]
                print('-----------------------------------------------------------------------')
                if start_node.id < target_node.id: 
                    #arc.create_arc(target_node, start_node)
                    arc.create_arc(start_node, target_node)
                else: 
                    #arc.create_arc(start_node, target_node)
                    arc.create_arc(target_node, start_node)
                    arc.reverse_arc()
                
                shortende_path.append(arc)
                
            shortende_path.append(target_node)
            start_node = shortende_path[-1]
            i = i+j+1
        
        return shortende_path
        







