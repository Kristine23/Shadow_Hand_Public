import numpy as np
import Math_Utilities as mu
import Constants
import String_Handler

class Node:
    
    def __init__(self, obj_rotation, obj_pos, fingers_in_contact = [], fingers_configuration= [], fingers_uv_on_obj_surface= []):
        self.is_graph_node = False
        self.rotation = obj_rotation
        self.pos = obj_pos
        self.fingers_in_contact = fingers_in_contact
        self.fingers_configuration = fingers_configuration
        self.fingers_uv_on_obj_surface = fingers_uv_on_obj_surface
    
    '''
    def under_five_joints_fingers_idx_and_five_joint_fingers_idx(self):
        idx_five_joints_fingers = []
        idx_under_five_joints_fingers = []
        for i in range(len(self.fingers_in_contact)):
            finger = self.fingers_in_contact[i]
            if finger.is_five_joints():
                idx_five_joints_fingers.append(i)
            else:
                idx_under_five_joints_fingers.append(i)
        return idx_under_five_joints_fingers, idx_five_joints_fingers
    
    def finger_idx_joint_sorted(self):
        idx_under_five_joints_fingers, idx_five_joints_fingers = self.under_five_joints_fingers_idx_and_five_joint_fingers_idx()
        idx_list = idx_under_five_joints_fingers + idx_five_joints_fingers
        return idx_list
    
    def fingers_sorted_by_number_joints(self):
        idx_list = self.finger_idx_joint_sorted()
        fingers = []
        for idx in idx_list:
            fingers.append(self.fingers_in_contact[idx])
        return fingers
    '''
    
        
    
    
    # as graph node
    
    def init_as_graph_node(self):
        self.is_graph_node = True
        self.adj_nodes = []
        for i in range(len(self.fingers_in_contact)):
            finger = self.fingers_in_contact[i]
            if finger.get_index_in_shadow_hand() == 0: 
                self.thumb_uv = self.fingers_uv_on_obj_surface[i]
                return
        
    def difference_between_graph_nodes(self, node):
        res = self.graph_distance(node.rotation, node.pos) #, node.thumb_uv)
        return res
    
    def distance_between_nodes(self, node):
        res = self.graph_distance(node.rotation, node.pos) #, node.thumb_uv)
        return res
        
        
    def graph_distance(self, rotation, pos): #, thumb_uv= np.zeros):
        vector, theta = mu.difference_between_rotations(self.rotation, rotation)
        dist_pos = np.linalg.norm((self.pos - pos))*Constants.SI_SCALE
        #dist_uv = sum([abs(self.thumb_uv[i]- thumb_uv[i]) for i in range(2)])
        res = abs(theta) + dist_pos #+ dist_uv
        return res
        
    
    
    def is_graph_node_equal(self, node):
        eps = 10**(-7)
        diff = self.difference_between_graph_nodes(node)
        if diff < eps:
            return True
        else: 
            return False
    
    def add_edge(self, node):
        self.add_arc(node)
        node.add_arc(self)
    
    def add_arc(self, node):
        self.adj_nodes.append(node)
        
    
    def contain_same_fingers(self, node):
    
        if len(self.fingers_in_contact) != len(node.fingers_in_contact):
            return False
        for f in self.fingers_in_contact: 
            if f not in node.fingers_in_contact:
                return False    
        return True
    
    
    
    
    def set_id(self, id):
        self.id = id
    
    
        

    
    def to_string(self):
        fingers = []
        
        for i in range(len(self.fingers_in_contact)):
            f = self.fingers_in_contact[i]
            fingers.append(f.get_index_in_shadow_hand())
        
        s = ""
        
        
        s = "rotation = " +  String_Handler.array2string(self.rotation, 1) +  "\n"
        s += "pos =" + String_Handler.array2string(self.pos, 0) +  "\n"
        s += "fingers_in_contact = " + String_Handler.array2string(fingers, 0) +  "\n"
        s += "fingers_configuration = " + String_Handler.array2string( self.fingers_configuration , 1) +  "\n"
        s += "fingers_uv_on_obj_surface = " + String_Handler.array2string( self.fingers_uv_on_obj_surface, 1) +  "\n"
        
        if self.is_graph_node:
            adj_id = [self.adj_nodes[i].id for i in range(len(self.adj_nodes))]
            s += "adjacent_nodes = " + String_Handler.array2string(adj_id, 0) + '\n'
        
        return s
    
 
    
    
    
    def from_string(string, shadowhand):

        arrays = String_Handler.multiple_arrays_from_string(string)

        rotation = arrays[0]
        pos = np.array(arrays[1])
        fingers_id = arrays[2]
        fingers = []
        for i in range(len(fingers_id)):
            idx = int(fingers_id[i])
            fingers.append(shadowhand.fingers[idx])
        
        fingers_configuration = arrays[3]
        fingers_uv_on_obj_surface = arrays[4]
        
        node = Node(rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        if len(arrays) > 5: 
            node.init_as_graph_node()
            int_array = [int(id) for id in arrays[5]]
            node.adj_nodes = int_array
            
        
        return node
        
        

    
    