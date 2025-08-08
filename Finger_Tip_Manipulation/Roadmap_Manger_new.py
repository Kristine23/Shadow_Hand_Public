import Arc
import Node
import PyPlotting
import Math_Utilities as mu
import numpy as np
import random
import Grasp_Handler
import Constants
import Physics_Engine
import String_Handler
import Shadow_Hand

class Graph_Creator:
    
   
    ROLL_ANGLE = 0.01
    
    #object values given pr. random step.
    #values given in mm
    max_pos_movement = 10
    max_roll_distance = 30 #argument rotation set randomly, ca. max 7 mm pr pi/2 ved kig på fingeren.
    
    #rotation given in radians
    #max_rotation = np.pi/2
    
    
    # Max position of object center, hardcode, change: x , y , z
    OBJ_LIMITS = [[280,380], [-60,60], [20,100]]    
    
    def __init__(self, node, obj, shadow_hand, physics_engine):
        node.init_as_graph_node()
        self.graph_nodes = [node]  
        self.obj = obj
        self.shadow_hand = shadow_hand
        self.PE = physics_engine
        self.gh = Grasp_Handler.Grasp_Creator()
        Arc.Arc.set_obj(obj)
        
        
        
    def create_road_map(self, store_map = False):
        max_itt = 5000
        i = 0
        n = 0
        eps = 10**(-12)
        best = 0
        while i < max_itt: 
            print('main_itterations................................................', i, ' / ', max_itt)
            print('number of nodes', len(self.graph_nodes))
            print('max_dif', best)
            i += 1
            node = self.graph_nodes[random.randint(0, len(self.graph_nodes)-1)]
            #self.create_random_configuation_using_node(node)
            
            #self.make_new_grasp_for_node(node)
            
            if i % 3 != 0: 
                self.create_random_configuation_using_node(node)
            else: 
                self.make_new_grasp_for_node(node)
            
            #self.store_only_differnet_nodes()
            if i %100 == 0: 
                best = self.find_max_difference()
            
            if store_map and i% 101 == 0:
                n = len(self.graph_nodes)
                Graph_Creator.write_graph(self.graph_nodes, 'roadmap.txt')
        
        
        
        #self.create_id_for_nodes()
        
        
        #self.print_differnet_nodes()
        #max_pair = self.find_max_difference()
        #return max_pair
    def create_id_for_nodes(self):
        for i in range(len(self.graph_nodes)):
            n = self.graph_nodes[i]
            n.set_id(i)
        

    def store_only_differnet_nodes(self):
        nodes = []
        for graph_node in self.graph_nodes:
            insert = True
            for node in nodes: 
                if graph_node.is_graph_node_equal(node):
                    insert = False
                    break
            if insert: 
                nodes.append(graph_node)
        self.graph_nodes = nodes
        
    
    
    def print_differnet_nodes(self):
        nodes = []
        for graph_node in self.graph_nodes:
            insert = True
            for node in nodes: 
                if graph_node.is_graph_node_equal(node):
                    insert = False
                    break
            if insert: 
                nodes.append(graph_node)
        print('number calculated nodes', len(self.graph_nodes))
        print('real nodes', len(nodes))
        
        
    def find_max_difference(self):
        #print('nodes', self.graph_nodes)
        n = len(self.graph_nodes)
        max_pair = []
        max_value = 0
        for i in range(n):
            #print('node', i)
            node1 = self.graph_nodes[i]
            for j in range(i+1, n):
                #print('i/j ' , j , '/', i)
                node2 = self.graph_nodes[j]
                dif = node1.difference_between_graph_nodes(node2)
                if dif > max_value:
                    max_value = dif
                    #print('(',i ,',', j,')')
                    #max_pair =[node1, node2]
                    max_pair = [i, j]
        #print(max_pair[0].to_string())
        #print(max_pair[1].to_string())
        print('max_value', max_value)
        return max_value
    
    
    def find_max_difference_nodes(self):
        #print('nodes', self.graph_nodes)
        n = len(self.graph_nodes)
        max_pair = []
        max_value = 0
        for i in range(n):
            #print('node', i)
            node1 = self.graph_nodes[i]
            for j in range(i+1, n):
                #print('i/j ' , j , '/', i)
                node2 = self.graph_nodes[j]
                dif = node1.difference_between_graph_nodes(node2)
                if dif > max_value:
                    max_value = dif
                    #print('(',i ,',', j,')')
                    #max_pair =[node1, node2]
                    max_pair = [i, j]
        #print(max_pair[0].to_string())
        #print(max_pair[1].to_string())
        print('max_value', max_value)
        nodes = [self.graph_nodes[max_pair[0]], self.graph_nodes[max_pair[1]]]
        return nodes

    '''
    function Dijkstra(Graph, source):
 2     
 3      for each vertex v in Graph.Vertices:
 4          dist[v] ← INFINITY
 5          prev[v] ← UNDEFINED
 6          add v to Q
 7      dist[source] ← 0
 8     print('real target')
 9      while Q is not empty:
10          u ← vertex in Q with minimum dist[u]
11          remove u from Q
12         
13          for each neighbor v of u still in Q:
14              alt ← dist[u] + Graph.Edges(u, v)
15              if alt < dist[v]:
16                  dist[v] ← alt
17                  prev[v] ← u
18
19      return dist[], prev[]
    '''
    




    def find_shortest_path(self, start_node, goal_node):
        #digstra dårligt implementeret. Lav evt om så bedre køretid
        que = []
        for i in range(len(self.graph_nodes)):
            n = self.graph_nodes[i].set_id(i)
            que.append(self.graph_nodes[i])
        
            
        
        dist = [1000 for i in range(len(self.graph_nodes))]
        prev = [-1 for i in range(len(self.graph_nodes))]
        
        dist[ start_node.id ] = 0
        
        while len(que) > 0:
            que_idx = self.find_que_idx_for_minimum_dist(que, dist)
            node = que[que_idx]

            for adj_node in node.adj_nodes:
                adj_idx = adj_node.id
                #adj_node = self.graph_nodes[adj_idx]
                d = node.difference_between_graph_nodes(adj_node)
                res = dist[node.id] + d
                if res < dist[adj_idx]:
                    dist[adj_idx] = res
                    prev[adj_idx] = node.id
            
            que.pop(que_idx)
        
        
        idx = goal_node.id
        reversed_path = [goal_node.id]
        while idx != start_node.id:
            p = prev[idx]
            reversed_path.append(p)
            idx = p
        '''
        print('revers_path')
        for i in reversed_path:
            print(i)
            adj = self.graph_nodes[i].adj_nodes
            print([n.id for n in adj])
        '''
        
        path = []
        for i in range(len(reversed_path)-1, -1, -1):
            n = self.graph_nodes[reversed_path[i]]
            path.append(n)
    
        print('path')
        for i in path:
            print(i.id)
            adj = i.adj_nodes
            print([n.id for n in adj])
    
        res_path = Graph_Creator.create_path(path)
    
        
        return res_path
    
    def does_contain_same_fingers(node1, node2):
        if len(node1.fingers_in_contact) != len(node2.fingers_in_contact):
            return False
        for f in node1.fingers_in_contact: 
            if f not in node2.fingers_in_contact:
                return False    
        return True
    
    
    def create_path(node_path):
        path = node_path
        shortende_path = [path[0]]
        i = 1
        while i < len(path):
            start_node = shortende_path[-1]
            target_node = path[i]
            contain_same_fingers = Graph_Creator.does_contain_same_fingers(start_node, target_node)
            
            j = 0
           
            
            if contain_same_fingers:
                
                new_target_node = path[i]
               
                
                while i+j+1 < len(path):
                    j += 1
                    #print(new_target_node.fingers_in_contact, Graph_Creator.does_contain_same_fingers(start_node, new_target_node))
                    if Graph_Creator.does_contain_same_fingers(start_node, new_target_node):
                        target_node = new_target_node
                        new_target_node = path[i+j]
                        continue
                    elif j > 0:
                        j -= 1
                    break
                
                #j = 0
                
                print('number of posible short cuts', j)
                
                
                while j >= 0:
                    target_node = path[i+j]
                    #print('arc from', start_node.id, target_node.id)
                    if start_node.id < target_node.id: 
                        #print('start_ node smalles')
                        arc = Arc.Arc(start_node)
                        res = arc.create_arc(target_node, True)
                        #print(res[0])
                    else: 
                        #print('target_ node smalles')
                        arc = Arc.Arc(target_node)
                        res = arc.create_arc(start_node, True)
                        #print(res[0])
                    exists = res[0]
                    j-= 1
                    if exists:
                        break
                    
                
                j += 1

                print('real number short cuts', j)
                
                target_node = path[i+j]
                
                if start_node.id < target_node.id: 
                    #print('start_ node smalles')
                    arc = Arc.Arc(start_node)
                    res = arc.create_arc(target_node, False)
                else: 
                    #print('target_ node smalles')
                    arc = Arc.Arc(target_node)
                    res = arc.create_arc(start_node, False)
                    arc.reverse_arc()
                
                #print('real number short cuts', j)
                #print('idx', i+j)
            
                
                #print('distance', shortende_path[0].difference_between_graph_nodes(target_node))
                #print('distance', shortende_path[0].difference_between_graph_nodes(path[i+j]))
                #print('ditance', shortende_path[0].difference_between_graph_nodes(reversed_path[0]))
        
                
                shortende_path.append(arc)
                #resulting_path.append(arc)
            
            
            shortende_path.append(target_node)
            start_node = shortende_path[-1]
            i = i+j+1
            #print('i', i)
        #
        
        
        #path_plot = PyPlotting.Path_Plot()
        #print(path)
        #path_plot.video_of_arc_path(path, self.obj, 3)
        
        
        return shortende_path
        
        
    def find_que_idx_for_minimum_dist(self, que, dist):
        min = 300
        min_idx = 0
        for i in range(len(que)):
            n = que[i]
            idx = n.id
            d = dist[idx]
            if d < min:
                min = d
                min_idx = i
        
        return min_idx
        
    
        
        
        
        
        
        
        
        
            
            
    def create_random_configuation_using_node(self, node):
        axis = np.empty(3)
        obj_pos = np.empty(3)
        for i in range(3):
            axis[i] = random.random() - 0.5 
            obj_pos[i]= node.pos[i] + (random.random()*2 - 1)* self.max_pos_movement
        #axis = mu.normalize_vector(axis)
        
        #tjek om vi kan sringe det her over og bruge axis fra random genneret oven over
        angel = random.random()*2*np.pi
        obj_rotation = mu.rotation_from_axises_and_angle(axis, angel)
        #axis, theta = mu.difference_between_rotations(node.rotation, obj_rotation)
        #print('axis', axis)
        
        #theta = random.random()*self.max_rotation
        #calculated_rotation = mu.rotation_from_axises_and_angle(axis,theta) #theta) #np.sign(theta)*self.RANDOM_DIRECTION_DISTANCE)
        
        #new_roation = np.matmul(node.rotation, calculated_rotation)
        
        #do other as, only thumb and lf.
            
        
        
        #theta = (random.random()-0.5)*np.pi*2
        
        #print('theta', axis, theta)
        
        #new_rotation = mu.rotation_from_axises_and_angle(axis, theta)
        #thumb_uv:
        
        uv_thumb = self.uv_from_initial(node.fingers_uv_on_obj_surface[0] ) 
                
        self.append_node_to_graph(obj_rotation, obj_pos, uv_thumb)
    
    def uv_from_initial(self, current_uv):
        guessed_direction = [ (random.random()-0.5), (random.random()-0.5)]    
        distance =  random.random()*self.max_roll_distance
        uv_guess =[current_uv[0] + guessed_direction[0], current_uv[1] + guessed_direction[1]]
        curent_dist = self.obj.surface_distance(current_uv, uv_guess)
        scalar = curent_dist/distance
        new_uv = [current_uv[0] + guessed_direction[0]/scalar, current_uv[1] + guessed_direction[1]/scalar]
        #print('current distance', curent_dist, scalar)
        #print('guessed distance', distance, self.obj.surface_distance(current_uv,new_uv))
        return new_uv
        
        
        
        
    
    def append_node_to_graph(self, obj_rotation, obj_pos, uv_thumb):
        # include uv_thumb
        nodes_to_consider =  [self.find_closes_node(obj_rotation, obj_pos)] # self.graph_nodes
        print('nodes_to consider', nodes_to_consider)
        r = 0 # only for printing: 
        n = len(nodes_to_consider)
        for i in range(n):
            node = nodes_to_consider[i]
            print('new_nodes', r, '/', n-1)
            r+= 1
            
            new_finger_uv = [uv_thumb]
            for i in range(1, len(node.fingers_in_contact)):
                #print(type(node.fingers_in_contact[i]))
                if type(node.fingers_in_contact[i]) == Shadow_Hand.LF :
                    #print('is lf')
                    new_uv = self.uv_from_initial(node.fingers_uv_on_obj_surface[i])
                    new_finger_uv.append(new_uv)
                else: 
                    new_finger_uv.append(node.fingers_uv_on_obj_surface[i])
        
            #print(node.fingers_uv_on_obj_surface)
            #print(fingers_uv_on_obj)
            target = Node.Node(obj_rotation, obj_pos, node.fingers_in_contact, fingers_uv_on_obj_surface= new_finger_uv)
            self.create_arcs(node, target)
        return
    
    def create_intermediate_node(self, start_node, target_node, obj_pos, fraction_distance):
        
        rotation_dif = mu.difference_between_rotations(start_node.rotation, target_node.rotation)
        obj_rotation = np.matmul(start_node.rotation, mu.rotation_from_axises_and_angle(rotation_dif[0], rotation_dif[1]*fraction_distance))
        #obj_pos = start_node.pos + (target_node.pos-start_node.pos)*fraction_distance
        uv_contacts_on_obj = []
        for i in range(len(start_node.fingers_uv_on_obj_surface)): 
            uv = start_node.fingers_uv_on_obj_surface[i]
            uv_t = target_node.fingers_uv_on_obj_surface[i]
            new_uv = [uv[i]+ (uv_t[i]-uv[i])*fraction_distance for i in range(2)]
            uv_contacts_on_obj.append(new_uv)
        return obj_rotation, obj_pos, uv_contacts_on_obj
    
    def create_arcs_with_roll(self, start_node, target_node):
        #has to be updated due to arc return values. 
        # NOT USEFULL due to arc return not the saem. 
        #print('create_arcs', target_node_is_goal)
        max_itt = 15
        best_differnece = 0 
        best_node_difference = 0
        i = 0
        arc = Arc.Arc(start_node)
        res = arc.create_arc(target_node, True)
        is_node_reached, time_fraction_reached, quv_estimates, problem_finger_idx = res
        obj_rotation, obj_pos, uv_contacts_on_obj = self.create_intermediate_node(start_node, target_node, time_fraction_reached)
        
        
        current_differnece = start_node.graph_distance(obj_rotation, obj_pos, uv_contacts_on_obj[0]) 
        
    
        eps = 10**(-2)
        
        is_possible_to_create_node = False
        
        while (current_differnece > best_differnece or i == 0): #and i < max_itt:
            problem_finger = start_node.fingers_in_contact[problem_finger_idx]
            
            print('using roll', i )
            #print('problem finger', problem_finger)
            #print('current_differnece', current_differnece)
            print('best differnece', best_differnece)
            best_differnece = current_differnece
            
            
            if current_differnece > best_node_difference + eps:  
                '''
                
                print('start_node')
                start_node.is_graph_node = False
                print(start_node.to_string())
                
                print('target')
                target_node.is_graph_node = False
                print(target_node.to_string())
                '''
                print('---------------------------------------------------------try to add node')
                #print('start_node')
                #print(start_node.to_string())
                #print('real target')
                #print(target_node.to_string())
                #arc.create_arc(target_node, True)  

                is_possible_to_create_node = self.add_node_and_edge(start_node, obj_rotation, obj_pos, uv_contacts_on_obj)
                if is_possible_to_create_node:
                    best_node_difference = current_differnece
                else: 
                    break
            
            
            
            if is_node_reached: # or not problem_finger.is_five_joints():
                break
           
           
            i += 1
            
            new_target = self.create_new_taget_using_roll(start_node, target_node, arc, problem_finger_idx)
            res = arc.create_arc(new_target, True)            
            is_node_reached, time_fraction_reached,  quv_estimates, problem_finger_idx = res
            obj_rotation, obj_pos, uv_contacts_on_obj = self.create_intermediate_node(start_node, new_target, time_fraction_reached)
            
            current_differnece = start_node.graph_distance(obj_rotation, obj_pos, uv_contacts_on_obj[0])
            print('current_differnece', current_differnece) 
            print('best_difference', best_differnece)          
            target_node = new_target
            
        
            
            
        return is_node_reached and is_possible_to_create_node
    
    
    def create_arcs(self, start_node, target_node):
        #print('create_arcs', target_node_is_goal)
        max_itt = 15
        best_differnece = 0 
        best_node_difference = 0
        i = 0
        arc = Arc.Arc(start_node)
        res = arc.create_arc(target_node, True)
        is_node_reached, time_fraction_reached, obj_pos, quv_estimate = res
        obj_rotation, obj_pos, uv_contacts_on_obj = self.create_intermediate_node(start_node, target_node, obj_pos, time_fraction_reached)
        
        
        current_differnece = start_node.graph_distance(obj_rotation, obj_pos, uv_contacts_on_obj[0]) 
        
        print('best differnece', current_differnece)
        
        eps = 10**(-2)
        
        is_possible_to_create_node = False
        
        if current_differnece > eps:
            print('---------------------------------------------------------try to add node')
            is_possible_to_create_node = self.add_node_and_edge(start_node, obj_rotation, obj_pos, uv_contacts_on_obj)
        
            
        return is_node_reached and is_possible_to_create_node
   
            
    def find_closes_node(self, obj_rotaiton, obj_pos):
        number_closest_nodes = 1
        closest_node = self.graph_nodes[0]
        best_score = closest_node.graph_distance(obj_rotaiton, obj_pos, closest_node.thumb_uv)
        for node in self.graph_nodes:
            score = node.graph_distance(obj_rotaiton, obj_pos, node.thumb_uv)
            if score < best_score: 
                best_score = score
                closest_node = node
        return closest_node
    
    
    def add_node_and_edge(self, start_node, obj_rotation, obj_pos, uv_contacts_on_obj):
        new_node = Node.Node(obj_rotation, obj_pos, start_node.fingers_in_contact, fingers_uv_on_obj_surface = uv_contacts_on_obj)
        a = Arc.Arc(start_node)
        test_arc = a.create_arc(new_node, True)
        arc_exists, time_reached, new_obj_pos, quv_estimates = test_arc
        # sammenlign nye objct_pos
        
        is_possible = False
        print('test arc', arc_exists)
        if arc_exists:
            print('current, reached pos', obj_pos, new_obj_pos)
            test1_node = Node.Node(obj_rotation, new_obj_pos, start_node.fingers_in_contact, fingers_uv_on_obj_surface = uv_contacts_on_obj)
            test1_a = Arc.Arc(start_node)
            test1_res = a.create_arc(test1_node, True)
            print(test1_res)
            
            new_node.fingers_configuration = quv_estimates

        
            is_physical_possible = self.is_physical_posible(obj_rotation, obj_pos, start_node.fingers_in_contact, quv_estimates, uv_contacts_on_obj)
            print('is physical possible', is_physical_possible)
            if is_physical_possible: 
                print('adding')
                is_possible = True
                
                
                #as the estimate of uv is not correct, update if not possible,
                new_node.init_as_graph_node()
                
                
                new_node.add_edge(start_node)
                #print('new node', new_node)
                #print('start node', start_node)
                #print(self.graph_nodes)
                self.graph_nodes.append(new_node)
            
            
        
            
        return is_possible
            
    
    
    def is_physical_posible(self, obj_rotation, obj_pos, fingers, fingers_configuration, uv_contacts_on_obj):
        object_movement = self.PE.create_static_object_movement(obj_pos, obj_rotation)
        #print(object_movement)
        
        #print(object_movement.rotation, object_movement.pos, object_movement.acceleration)
        weight =  self.PE.weight_for_system(object_movement, fingers, fingers_configuration, uv_contacts_on_obj)
        return mu.vector_does_not_contain_negative_values(weight)
            
            
        

    
    def create_new_taget_using_roll(self, start_node, target_node, arc, problem_finger_idx):
       
        fingers_uv_on_obj_surface = target_node.fingers_uv_on_obj_surface.copy()
        
        uv_for_problem_finger = fingers_uv_on_obj_surface[problem_finger_idx]
        if problem_finger_idx == None:
            print('best..................................................', self.graph_nodes[0][0])
            print('target node', type(target_node.fingers_uv_on_obj_surface), target_node.fingers_uv_on_obj_surface)
            print('copy', type(fingers_uv_on_obj_surface), fingers_uv_on_obj_surface)
            print('problem_finger_idx', problem_finger_idx)
            print(('uv_for_problem_finger', uv_for_problem_finger))
            print( fingers_uv_on_obj_surface[problem_finger_idx])
            
        negativ_rotation = np.matmul(start_node.rotation, mu.rotation_from_axises_and_angle(arc.axis, -self.ROLL_ANGLE))
        p_end = self.obj.surface_point(uv_for_problem_finger[0], uv_for_problem_finger[1], negativ_rotation, start_node.pos)
        new_uv_for_problem_finger = self.obj.uv_for_surface_point(p_end, start_node.rotation, start_node.pos)
        fingers_uv_on_obj_surface[problem_finger_idx] = new_uv_for_problem_finger
        new_target = Node.Node(target_node.rotation, target_node.pos, target_node.fingers_in_contact, fingers_uv_on_obj_surface=fingers_uv_on_obj_surface)
        return new_target
    
            
        
    
    
    
    def make_new_grasp_for_node(self, node):
        remove_idx = random.randint(1,len(node.fingers_in_contact)-1)
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
            
            fingers.pop(finger_idx_to_remove)
            uv_on_object.pop(finger_idx_to_remove)
            fingers_configuration.pop(finger_idx_to_remove)
                
            
            p_f = finger.contact_point(quv[:-2],quv[-2], quv[-1] )
            u_o,v_o = self.obj.uv_for_surface_point(p_f, node.rotation, node.pos)
            fingers.append(finger)
            uv_on_object.append((u_o,v_o))
            fingers_configuration.append(quv)
            
            
            #pp = PyPlotting.Plot()
            #hp = PyPlotting.HandAndObjectPlot(pp).plot_fixed_fingers_and_configurations(self.obj, node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
            #pp.show()
            
            
            is_possible = self.is_physical_posible(node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
            print('is_physical_posible', is_possible)
            if is_possible:
                new_node = Node.Node(node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
                new_node.init_as_graph_node()
                new_node.add_edge(node)
                self.graph_nodes.append(new_node)
        
        return
        
        '''old: 
        while len(posible_fingers) > 0: 
            idx = random.randint(0,len(posible_fingers)-1)
            finger = posible_fingers.pop(idx)      
            new_grasp_found, quv = self.gh.add_finger_to_grasp(finger,self.obj, node.rotation, node.pos, node.fingers_in_contact, node.fingers_configuration)
            if new_grasp_found: 
                uv_on_object = node.fingers_uv_on_obj_surface.copy()
                fingers_configuration = node.fingers_configuration.copy()
                
                fingers.pop(finger_idx_to_remove)
                uv_on_object.pop(finger_idx_to_remove)
                fingers_configuration.pop(finger_idx_to_remove)
                
                
                fingers.append(finger)
                uv_on_object.append(uv_o)
                fingers_configuration.append(quv)
                
                
                break
        
        uv_on_object = node.fingers_uv_on_obj_surface.copy()
        fingers_configuration = node.fingers_configuration.copy()
        fingers.pop(finger_idx_to_remove)
        uv_on_object.pop(finger_idx_to_remove)
        fingers_configuration.pop(finger_idx_to_remove)
        new_grasp_found = False
        while len(posible_fingers) > 0: 
            idx = random.randint(0,len(posible_fingers)-1)
            finger = posible_fingers.pop(idx) ### tager random finger
            quv, uv_o, new_grasp_found = self.gh.get_initial_point(finger, self.obj,  node.rotation, node.pos,  True, uv_on_object)
            print('create_new_grasp,', new_grasp_found)
            if new_grasp_found:
                new_grasp_found = not self.is_fingers_intersecting(finger, quv, node.fingers_in_contact, node.fingers_configuration) #move to grasp_handler-
                if new_grasp_found:
                    break
        print('new_grasp_found', new_grasp_found)
        
        if new_grasp_found:
            
            fingers.append(finger)
            uv_on_object.append(uv_o)
            fingers_configuration.append(quv)
            is_possible = self.is_physical_posible(node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
            print('is possible_to _add', is_possible)
            if is_possible:
                new_node = Node.Node(node.rotation, node.pos, fingers, fingers_configuration, uv_on_object)
                new_node.init_as_graph_node()
                new_node.add_edge(node)
                self.graph_nodes.append(new_node)
        '''
    
    
    
    # burde komme i en anden klasse
    def is_fingers_intersecting(self, finger, quv, other_fingers, other_quv):
        for i in range(len(other_fingers)):
            
            finger2= other_fingers[i]
            quv2 = other_quv[i]
            finger.is_intersecting(quv, finger2, quv2)
            intersecting = finger.is_intersecting(quv, finger2, quv2)
            #print('intersecting', finger, finger2, intersecting)
            #print(finger.is_intersecting(quv, finger2, quv2))
            #print(quv, quv2)
            if intersecting: 
                return True
        return False
    

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    def read_graph(shadow_hand, file_name = "roadmap.txt"):
        f = open(file_name, "r")
        string = f.read()
        f.close()
        graph = []
        nodes = string.split(" Node ")        
        for j in range(1, len(nodes)):
            #print('node to read:', nodes[j])
            nodes[j]
            node = Node.Node.from_string(nodes[j], shadow_hand)   
            graph.append(node)

        
        #print(graph)

        for i in range(len(graph)):
            node = graph[i]
            node.set_id(i)
            ids = node.adj_nodes.copy()
            adj = []
            for id in ids:
                #print(id, type(id))
                #print(graph[id])
                adj.append(graph[id])

            node.adj_nodes = adj
            
        return graph
        
    
    
            
            
        
    
    def write_graph(graph_nodes, file_name="roadmap.txt"):
        
        print(graph_nodes)
        for i in range(len(graph_nodes)):
            n = graph_nodes[i]
            n.set_id(i)

        
        
        open(file_name, 'w').close() #remove content
        f = open(file_name, "a")
        for n in graph_nodes:
            f.write(" Node ")
            s = n.to_string()
            f.write(s)
        f.close()
        
        
        
        
        
