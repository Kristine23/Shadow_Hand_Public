import Arc
import Node
import PyPlotting
import Math_Utilities as mu
import numpy as np
import random
import Grasp_Handler
import Constants
import time


def find_max_difference(graph_nodes, return_nodes = False):
    n = len(graph_nodes)
    max_pair = []
    max_value = 0
    for i in range(n):
        #print('node', i)
        node1 = graph_nodes[i]
        for j in range(i+1, n):
            node2 = graph_nodes[j]
            dif = node1.difference_between_graph_nodes(node2)
            if dif > max_value:
                max_value = dif
                max_pair = [i, j]
    if return_nodes: 
        nodes = [graph_nodes[max_pair[0]], graph_nodes[max_pair[1]]]
        return nodes
    return max_value

def find_max_difference_nodes(graph_nodes):
    nodes = find_max_difference(graph_nodes, True)
    return nodes


def find_max_dist_from_node(node, graph_nodes):
    n = len(graph_nodes)
    max_idx = 0
    max_value = 0
    node1 = node
    for j in range( n):
        node2 = graph_nodes[j]
        dif = node1.difference_between_graph_nodes(node2)
        if dif > max_value:
            max_value = dif
            max_idx = j
    print('max', max_value)
    return graph_nodes[max_idx]


def find_max_rot_diff_nodes(graph_nodes):
    n = len(graph_nodes)
    max_pair = []
    max_value = 0
    for i in range(n):
        #print('node', i)
        node1 = graph_nodes[i]
        for j in range(i+1, n):
            node2 = graph_nodes[j]
            axis, dif =  mu.difference_between_rotations(node1.rotation, node2.rotation)
            if dif > max_value:
                max_value = dif
                max_pair = [i, j]
    nodes = [graph_nodes[max_pair[0]], graph_nodes[max_pair[1]]]
    return nodes    
    

    
    

def closest_nodes_to_obj_config(graph_nodes, target_rotation, target_pos,fingers_in_grasps): # target_uv_thumb, fingers_in_grasps):
    l = len(fingers_in_grasps)
    best_scores = [None for i in range(l)]
    closest_node = [None for i in range(l)]
    
    
    for i in range(l):
        fingers = fingers_in_grasps[i]
        for node in graph_nodes:
            if node.fingers_in_contact == fingers: 
                score = node.graph_distance(target_rotation, target_pos) #, target_uv_thumb)
                if best_scores[i] == None: 
                    best_scores[i] = score
                    closest_node[i] = node
                elif score < best_scores[i]:
                    best_scores[i] = score
                    closest_node[i] = node
    return closest_node




def node_closest_to_obj_config(graph_nodes, target_rotation, target_pos): #, target_uv_thumb):
    closest_node = graph_nodes[0]
    best_score = closest_node.graph_distance(target_rotation, target_pos) #, target_uv_thumb)
    for graph_node in graph_nodes:
        score = graph_node.graph_distance(target_rotation, target_pos) #, target_uv_thumb)
        if score < best_score: 
            best_score = score
            closest_node =graph_node
    return closest_node
    
def closest_node(node, graph_nodes):
    closest_node = node_closest_to_obj_config(graph_nodes, node.rotation, node.pos, node.fingers_uv_on_obj_surface[0])
    return closest_node


def find_shortest_path(start_node, goal_node, graph_nodes):
    #digstra dårligt implementeret. Lav evt om så bedre køretid
    que = []
    for i in range(len(graph_nodes)):
        n = graph_nodes[i].set_id(i)
        que.append(graph_nodes[i])
    
    dist = [1000 for i in range(len(graph_nodes))]
    prev = [-1 for i in range(len(graph_nodes))]
    
    dist[start_node.id ] = 0
    
    while len(que) > 0:
        que_idx = find_que_idx_for_minimum_dist(que, dist)
        node = que[que_idx]

        for adj_node in node.adj_nodes:
            adj_idx = adj_node.id
            #adj_node = graph_nodes[adj_idx]
            d = node.distance_between_nodes(adj_node)
            res = dist[node.id] + d
            if res < dist[adj_idx]:
                dist[adj_idx] = res
                prev[adj_idx] = node.id
        
        que.pop(que_idx)
    
    
    idx = goal_node.id
    reversed_idx_path = [goal_node.id]
    while idx != start_node.id:
        p = prev[idx]
        reversed_idx_path.append(p)
        idx = p

    path = []
    for i in range(len(reversed_idx_path)-1, -1, -1):
        n = graph_nodes[reversed_idx_path[i]]
        path.append(n)
    
    print('path')
    for i in path:
        print(i.id)
        adj = i.adj_nodes
        print([n.id for n in adj])
    
    return path






def find_que_idx_for_minimum_dist( que, dist):
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
        
        



def read_path(shadow_hand, file_name = "path.txt"):
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

        node = Node.Node.from_string(node_str, shadow_hand)
        
    
        if i > 0: 
            #print(type(path[-1]))
            if type(path[-1]) == Arc.Arc:
                #print('inside')
                arc = path[-1]
                arc.target_node = node
                arc.axis, arc.theta = mu.difference_between_rotations(arc.start_node.rotation, arc.target_node.rotation)
        path.append(node)
        if len(node_arc_str) > 1: 
            arc_str = node_arc_str[1]
            arc = Arc.Arc.arc_from_string(arc_str)
            arc.start_node = node
            path.append(arc)
    #print(path)
    return path
            
        
        
    

def write_path(path, file_name = "path.txt"):
    open(file_name, 'w').close() #remove content
    f = open(file_name, "a")
    for e in path:
        if type(e)== Arc.Arc:
            f.write(" Arc ")
        else:
            f.write(" Node ")
        s = e.to_string()
        f.write(s)
    f.close()
    










