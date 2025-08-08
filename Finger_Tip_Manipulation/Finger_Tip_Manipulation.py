import Mujoco_Simulator
import Shadow_Hand
import Object_Handler
import Grasp_Handler
import PyPlotting
import Node
import Math_Utilities as mu
import numpy as np
import Constants
import Planner
import Arc
import Roadmap_Manger_new
import Physics_Engine
import Graph_Utilities
import time
import Sim_to_Real
import Mujoco_Simulator_old
import force_visualisation_test
import Mesh_Utilities
import String_Handler



class Finger_Tip_Manipulation:
    SHADOW_HAND_CALCULATION =True
    
    def __init__(self):
        Grasp_Handler.Grasp_Creator.SHADOW_HAND_CALCULATION = True
        Arc.Arc.SHADOW_HAND_CALCULATION = True
        
        
        
        self.mj_sim = Mujoco_Simulator.Mujoco_Simulator()
        print(self.mj_sim)
        self.shadow_hand = Shadow_Hand.Shadow_hand(self.mj_sim)
       
        self.obj = Object_Handler.Obj(self.mj_sim.mj_model)
        
        self.mj_sim.set_shadow_hand_and_obj(self.shadow_hand, self.obj)
        Arc.Arc.set_obj(self.obj)
        Physics_Engine.Physics_Engine.set_obj(self.obj)
        self.planner = Planner.Planner(self.obj, self.shadow_hand)
        
        
        #self.mj_sim_old = Mujoco_Simulator_old.Mujoco_Simulator()
        #self.mj_sim_old.set_shadow_hand(self.shadow_hand)
        
        #self.mj_sim_new = Mujoco_Simulator.Mujoco_Simulator_New()
        #self.mj_sim_new.set_shadow_hand_and_obj(self.shadow_hand, self.obj)
        
        
        
        
    
    def create_init_grasps_file(self):
        
        for i in range(4):
            for j in range(i+1,4):
                for r in range(j+1,4):
                    fingers = [self.shadow_hand.fingers[i], self.shadow_hand.fingers[j], self.shadow_hand.fingers[r+1]]
                    print(fingers)
                    Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
        
        #fingers = [self.shadow_hand.fingers[0], self.shadow_hand.fingers[1], self.shadow_hand.fingers[3]]
        #Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
        
        #fingers = [self.shadow_hand.fingers[0], self.shadow_hand.fingers[1], self.shadow_hand.fingers[4]]
        #Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
        return
        '''
        for k in range(0,  len(self.shadow_hand.fingers)):
            f0 = self.shadow_hand.fingers[k]
            for i in range(k+1, len(self.shadow_hand.fingers)):
                f1 = self.shadow_hand.fingers[i]
                for j in range(i+1, len(self.shadow_hand.fingers)):
                    if j ==2:
                        continue
                    f2 = self.shadow_hand.fingers[j]
                    fingers = [f0, f1, f2]
                    Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
        '''
    
    def create_road_map(self,road_map_name = 'sh_roadmap.txt'):
        
        
        obj_axis_angle = [0.0,0.0,1.009,1.0533043693016086]
        pos =[348.613106432645,-26.677983935558643,91.55489208067999]
        fingers_in_contact = [0,2,3]
        fingers_configuration = [[0.174358261916914,0.8335486791378002,0.0425022575971488,0.16526013018572824,0.6363396380769559,1.148938948349853,1.760122698936911],[0.11222568014985145,0.8475734351406403,0.9492955240627459,0.0,0.672452776694484,1.1824042511025714],[-0.2317496177264179,1.2596610388046003,0.9795280615841697,0.0,0.505366655675056,1.0796315849215607]]
        fingers_uv_on_obj_surface = [[-2.981924274064981,1.8662227785653023],[-0.14111305161921,1.6469930987811017],[0.8165198533944444,1.5693132516789003]]


        
        
        a =  fingers_in_contact[0]
        b = fingers_in_contact[1]
        c = fingers_in_contact[2]
        
                
        fingers = self.shadow_hand.fingers[a:a+1] +self.shadow_hand.fingers[b:b+1] +self.shadow_hand.fingers[c:c+1]
     
        rotation =mu.rotation_from_axises_and_angle(obj_axis_angle[:3], obj_axis_angle[-1])
        
        pos = np.array(pos)
        node = Node.Node(rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        '''
        pp = PyPlotting.Plot()
        hp = PyPlotting.HandAndObjectPlot(pp)
        hp.plot_fixed_fingers_and_configurations(self.obj, rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        pp.show()

        path = Graph_Utilities.read_path(self.shadow_hand, 'path2.txt')
        node = path[8]
        pp = PyPlotting.Plot()
        hp = PyPlotting.HandAndObjectPlot(pp)
        hp.plot_fixed_fingers_and_configurations(self.obj, node.rotation, node.pos, node.fingers_in_contact, node.fingers_configuration, node.fingers_uv_on_obj_surface)
        
        pp.show()s
        
        #self.planner.create_road_map(node, True, 'roadmap.txt', 'roadmap.txt')
        
        om = Physics_Engine.Object_Movement(node.pos, node.rotation)
        
        stable = Physics_Engine.Physics_Engine.is_grasp_physical_stable(om, fingers, fingers_configuration, fingers_uv_on_obj_surface)        
        print('grasp is stable', stable)
        #self.mujoco_simulate( [node])
        
        
        for i in range(3):
            finger = fingers[i]
            quv =  fingers_configuration[i]
            within_limits = finger.is_within_configuration_limits(quv)
            for j in range(i+1, 3):
                print(finger.is_intersecting(quv, fingers[j], fingers_configuration[j]))
            
            print('finger', type(finger), within_limits)
        '''
        
        #self.planner.create_road_map(node, True, road_map_name, road_map_name) #, 'roadmap_with_no_tourqe_limits.txt')
        
        self.planner.create_road_map(node, True, road_map_name, road_map_name) #, 'roadmap_with_no_tourqe_limits.txt')
    
    def create_path_from_road_map(self, file_name_road_map = 'roadmap.txt' ):
        
  
        graph_nodes = Graph_Utilities.read_graph(self.shadow_hand, file_name_road_map)
        node1, node2 = Graph_Utilities.find_max_difference_nodes(graph_nodes)
        
        
    
        
        #node1, node2 = Graph_Utilities.find_max_rot_diff_nodes(graph_nodes)
        path = Graph_Utilities.find_shortest_path(node1, node2, graph_nodes)
        
  
        res_path =  Planner.Planner.shorten_path(path)
        return res_path
        
    
    def create_path_between_nodes(self, node1, node2):
        self.planner.RRT_extension(node1, node2)
    
    
    def plot_path(self, path, filename = 'path.mp4'):
        path_plot = PyPlotting.Path_Plot()
        path_plot.video_of_path(path, self.obj, file_name= filename)
        
    
    def mujoco_simulate(self, path):
        self.mj_sim.init_configurations(self.obj, path[0])
        
    
        
        self.mj_sim.simulate_path(path, self.obj)
        
        #self.mj_sim_old.init_configurations(self.obj, path[0])
        #self.mj_sim_old.run(path, self.obj)
        
        
    def test(self):
        

        
        obj_axis_angle = [0.0,0.0,1.009,1.0533043693016086]
        pos =[348.613106432645,-26.677983935558643,91.55489208067999]
        fingers_in_contact = [0,2,3]
        fingers_configuration = [[0.174358261916914,0.8335486791378002,0.0425022575971488,0.16526013018572824,0.6363396380769559,1.148938948349853,1.760122698936911],[0.11222568014985145,0.8475734351406403,0.9492955240627459,0.0,0.672452776694484,1.1824042511025714],[-0.2317496177264179,1.2596610388046003,0.9795280615841697,0.0,0.505366655675056,1.0796315849215607]]
        fingers_uv_on_obj_surface = [[-2.981924274064981,1.8662227785653023],[-0.14111305161921,1.6469930987811017],[0.8165198533944444,1.5693132516789003]]

        
        obj_axis_angle = [-0.2508152682038627,-0.39566720117911713,-0.11067770504769237,5.3177489982479065]
        pos =[315.53208906053555,-62.977993839440074,74.3768660766864]
        fingers_in_contact = [0,1,2]
        fingers_configuration = [[-0.6645005762848972,0.4058404722774093,-0.006004409432892607,-0.41300040623694095,0.8276208532809765,0.8913560574268543,2.462614518611903],[-0.06616282980573365,1.0730049873159817,1.130217327252042,0.0,0.9210896850182835,0.7007445240748656],[0.1456915251704635,1.460409418731588,1.039689775733351,0.0,0.30411412605006055,0.9265741048783496]]
        fingers_uv_on_obj_surface = [[-1.5137476103067373,1.696356769896128],[0.8807529008205979,1.1095048618490067],[1.304093889323797,1.9744362646293798]]


        obj_axis_angle = [0.0,0.0,1.009,1.3135153351209035]
        pos =[334.44604501435936,-41.71930119689458,81.05786894564717]
        fingers_in_contact = [0,1,2]
        fingers_configuration = [[-0.1479407948970016,0.7267075442043712,0.009459720559117011,0.14030831733061327,0.8447291058882996,1.2446051441941404,2.1332807561484346],[-0.18738658283328918,0.4418978490068979,1.13352931335076,0.0,0.7235039435980223,1.167893693021622],[0.1817379175038831,1.2047130584952337,1.2124328542153942,0.20633625115953902,0.4381168971882882,1.0250102191943014]]
        fingers_uv_on_obj_surface = [[-3.0352135263079787,1.6221298730667388],[-1.1780980172105142,1.7247785593020635],[0.38255416447967205,1.5646336316281646]]

        

        a =  fingers_in_contact[0]
        b = fingers_in_contact[1]
        c = fingers_in_contact[2]
        
                
        fingers = self.shadow_hand.fingers[a:a+1] +self.shadow_hand.fingers[b:b+1] +self.shadow_hand.fingers[c:c+1]
        
        
                
        fingers = self.shadow_hand.fingers[a:a+1] +self.shadow_hand.fingers[b:b+1] +self.shadow_hand.fingers[c:c+1]
     
        rotation =mu.rotation_from_axises_and_angle(obj_axis_angle[:3], obj_axis_angle[-1])
        
        pos = np.array(pos)
        node = Node.Node(rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        pp= PyPlotting.Plot()
        hp = PyPlotting.HandAndObjectPlot(pp)
        
        hp.plot_fixed_fingers_and_configurations(self.obj, rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        pp.show()
        
        
        return
        
        
        
    def point_to_str(self, p):
        s = '{' + str(p[0]) + ',' + str(p[1]) + ',' + str(p[2]) + '}' 
        return s
    
    def test2(self):
        
        f = self.shadow_hand.fingers[1]
        
        #f = self.shadow_hand.fingers[1]
        
        pp = PyPlotting.Plot()
        f.plot_finger_surface(pp)
        
        #pp.show()
        
        
    
        mesh = Mesh_Utilities.read_Stl_file('f_distal_mstXL.stl')
        #mesh = Mesh_Utilities.read_Stl_file('th_distal_mst.stl')
    
        
        front_vectors = f.front_mesh_vectors(mesh)
        
        front_mesh = Mesh_Utilities.new_mesh_from_vectors(front_vectors)
        
        points = Mesh_Utilities.points_in_mesh(front_mesh)
        
        '''
        #file_name = 'f_distal_mstXL.txt'
        file_name = 'th_distal_mst.txt'
        open(file_name, 'w').close() #remove content
        f = open(file_name, "a")
        
            
        f.write('points\n')
        f.write('{' )
        for p in points: 
            s = self.point_to_str(p) + ' , '
            f.write(s)
        f.write('}')
        f.write('\n\n\n')
        f.write('triangles\n')
        f.write('{')
        l = len(front_mesh.vectors) 
        for i in range(0, l):
            triangle= front_mesh.vectors[i]
            s = '{'
            s += self.point_to_str(triangle[0])
            s += ','
            s += self.point_to_str(triangle[1])
            s += ','
            s += self.point_to_str(triangle[2])
            s += '},'
            f.write(s)
        f.write('}')
        
        f.close()
        '''
        
        
        
        
        
        
        
        
        #mesh1 = Mesh_Utilities.read_Stl_file('th1_distal_mst.stl')
        
        
        #th_mesh_points = Mesh_Utilities.mesh_points_from_obj_file(Shadow_Hand.F1.PARTH_2_STL)
        
        
        #mesh = Mesh_Utilities.read_Stl_file('f1_distal_mst.stl')
        
        
        
        #mesh= Mesh_Utilities.read_Stl_file('th_distal_mst.stl')
        #th = self.shadow_hand.fingers[0]
        #front_vectors = th.front_mesh_vectors(mesh)
        #front_mesh = Mesh_Utilities.new_mesh_from_vectors(front_vectors)
        #points1 = Mesh_Utilities.points_in_mesh(front_mesh)
        
        #pp = PyPlotting.Plot()
        
        #pp.plot_3D_points(points1, 0.3, 'g')
        
        u = 2.8
        v = 1.5
        
        #print('u, v', u,v)
        normal = f.surface_fit.normal(u,v)
        p1 = f.surface_fit.F(u,v)
        pp.plot_vector(p1, normal, 'b', 10)
        print('p1', p1)
        print(normal)
        
        pp.plot_3D_points(points, 0.3, 'r')
        #pp.plot_3D_points(points1, 0.3, 'b')
        pp.set_axes()
        pp.show()
        
        return
        
        
    
    
    
    def store_q_values(self, path):
        sr = Sim_to_Real.Sim_to_Real(self.shadow_hand)
      
      
        file_name_full = 'sh_small_movements_full.txt'
        #sr.store_finger_config_for_path(path, file_name_full, True)
        
        
        file_name_start = 'sh_small_movements_start.txt'
        
        sr.store_finger_config_for_path(path[:1], file_name_start)
        
        #self.mj_sim.simulate_from_file(file_name_start, self.obj)
        
        file_name_move = 'sh_small_movements_movement.txt'
        sr.store_finger_config_for_path(path, file_name_move, False)
        '''
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f0.txt', [0])
        
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f1.txt', [1])
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f2.txt', [2], True)
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f3.txt', [3], True)
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f4.txt', [4])
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f01.txt', [0, 1])
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f014.txt', [0, 1, 4])
        sr.create_file_subset_of_fingers_moving(file_name_full, 'f0134.txt', [0, 1, 3, 4])
        '''
        
        #self.mj_sim.simulate_from_file(f1, self.obj)
        #self.mj_sim.simulate_from_file(f4, self.obj)
        
        #self.mj_sim.simulate_from_file(f2, self.obj)
        #self.mj_sim.simulate_from_file(f3, self.obj)
        #self.mj_sim.simulate_from_file(f01, self.obj)
        #self.mj_sim.simulate_from_file(f014, self.obj)
        #self.mj_sim.simulate_from_file(f0134, self.obj)        
        
        
        #self.mj_sim.simulate_from_file(f0, self.obj)
       
        
        
        #self.mj_sim.simulate_from_file(f4, self.obj)
        
        #self.mj_sim.simulate_from_file(f2, self.obj)
        #self.mj_sim.simulate_from_file(f3, self.obj)
        
        #self.mj_sim.simulate_from_file(f01, self.obj)
        #self.mj_sim.simulate_from_file(f012, self.obj)
        #self.mj_sim.simulate_from_file(f0123, self.obj)
        #self.mj_sim.simulate_from_file(file_name_full, self.obj)
        
        
        
        
        
        #self.mj_sim.simulate_from_file(file_name_move, self.obj)
        #self.mj_sim.simulate_from_files(file_name_start, file_name_move, self.obj, False)
        #self.mj_sim.simulate_from_files(file_name_start, file_name_move, self.obj, False)
        #self.mj_sim.simulate_from_files(file_name_start, file_name_move, self.obj, True, path[0].pos, path[0].rotation)
            
    

        

def main():
    manipulation = Finger_Tip_Manipulation()

    create_init_grasps_file = True
    create_road_map = False
    crete_simulation = False
    
    #manipulation.run_trough_all_config(1)
    
    
    #manipulation.test()
    #return
    
    #kenematic_plot.DataCreater(manipulation.shadow_hand)
    
    
    
    #manipulation.test2()
    
    #manipulation.test()
    
    
    if create_init_grasps_file:
        manipulation.create_init_grasps_file()

    road_map_name = 'roadmap.txt'
    path_name = 'path.txt' 
    plot_name = 'path.mp4'
    
    if create_road_map:
    
        manipulation.create_road_map(road_map_name)
        path = manipulation.create_path_from_road_map(road_map_name)
        Graph_Utilities.write_path(path, path_name)
        manipulation.plot_path(path,plot_name)
    
    if crete_simulation:
        path = Graph_Utilities.read_path(manipulation.shadow_hand,  path_name)
        node = path[0]
        print(node.fingers_configuration[-1])
        manipulation.mujoco_simulate(path)

    path_name = 'path_ny1.txt' 

    path = Graph_Utilities.read_path(manipulation.shadow_hand,  path_name)   
    
    
    node = path[0]
    fingers = node.fingers_in_contact
    
    quv = node.fingers_configuration[0]
    q= quv[:-2]
    u = quv[-2]
    v = quv[-1]
    
    
 
    pp = PyPlotting.Plot()
    #print('finger contact', fingers[-1].contact_point(q, u, v))
    u_th, v_th = node.fingers_uv_on_obj_surface[0]
    for i in range(3):
        print(fingers[i])
        u_o, v_o= node.fingers_uv_on_obj_surface[i]
        print('object', np.linalg.norm(manipulation.obj.surface_point(u_o, v_o, np.identity(3), np.zeros(3))-manipulation.obj.surface_point(u_th, v_th, np.identity(3), np.zeros(3))))
        quv = node.fingers_configuration[i]
        print('point', fingers[i].contact_point(quv[:-2], quv[-2], quv[-1]))
        #pp.plot_point(manipulation.obj.surface_point(u_o, v_o, np.identity(3), np.zeros(3)))
    #
    # 
    
    quv_th = node.fingers_configuration[0]
    
    j_th = fingers[0].get_pos_of_joint(4, quv_th[:-2])
    
    quv_mf = node.fingers_configuration[1]
    j_mf = fingers[1].get_pos_of_joint(3, quv_mf[:-2])
    print(j_mf-j_th)
    print('Dif between joints,', np.linalg.norm((j_mf-j_th)))
    
    
    hp = PyPlotting.HandAndObjectPlot(pp)
    #manipulation.obj.plot(pp)
    #pp.set_axes()
    hp.plot_fixed_fingers_and_configurations(manipulation.obj, node.rotation, node.pos, fingers, node.fingers_configuration, node.fingers_uv_on_obj_surface)
    
    pp.show()
    
    
    print(manipulation.shadow_hand.world_from_palm)
    
    
    for f in manipulation.shadow_hand.fingers: 
        print(f)
        print(f.tip_length)
    
    
     
    
    
    u_th, v_th = node.fingers_uv_on_obj_surface[0]
    for i in range(3):
        print(fingers[i])
        u_o, v_o= node.fingers_uv_on_obj_surface[i]
        #print('object', np.linalg.norm(manipulation.obj.surface_point(u_o, v_o, np.identity(3), np.zeros(3))-manipulation.obj.surface_point(u_th, v_th, np.identity(3), np.zeros(3))))
        quv = node.fingers_configuration[i]
        if i == 1: 
            quv1 = [0.334, 0.738, 1.123, 0.020]
            quv1.append(quv[-2])
            quv1.append(quv[-1])
            quv = quv1
        if i == 0: 
            quv1 = [-0.132, 0.897, -0.193, 0.197, 1.172]
            quv1.append(quv[-2])
            quv1.append(quv[-1])
            quv =quv1
        
        
        print('point', fingers[i].contact_point(quv[:-2], quv[-2], quv[-1]))
    
    pt = [338.06047684, -45.73450166,  80.54124203]
    pm = [355.97900506, -23.08673902,  85.69376064]
    pr = [325.66643531,   3.66066631 , 86.44868941]
    
    pt = [338.79497887, -46.25876467,  80.14011263]
    pm = [363.28383086, -13.56223217,  85.65720315]
    
    p = []
    for i in range(3): 
        p.append(pm[i]-pr[i])
        
    print(np.linalg.norm(p))
    
    '''
    
    change = False
    for i in range(len(path)):
        n = path[i]
        if type(path[i]) == Node.Node: 
            if change == True:
                print('index', i)
            else: 
                change = True
        else: 
            change = False
    
    
    
    node = path[0]
    
    fingers = node.fingers_in_contact
    
    
    pp = PyPlotting.Plot()
    hp = PyPlotting.HandAndObjectPlot(pp)
    #hp.plot_fingers(fingers, node.fingers_configuration)
    
    hp.plot_fixed_fingers_and_configurations(manipulation.obj, node.rotation, node.pos, fingers, node.fingers_configuration, node.fingers_uv_on_obj_surface)
    #manipulation.obj.plot(pp, np.identity(3), np.zeros(3))  
    pp.set_axes()
    
    
    for i in range(len(fingers)):
        f = fingers[i]
        u, v = node.fingers_uv_on_obj_surface[i]
        print(f, manipulation.obj.surface_point(u,v, np.identity(3), np.zeros(3)))
        #pp.plot_point(manipulation.obj.surface_point(u,v, np.identity(3), np.zeros(3)))
    
    '''
    '''
    for i in range(len(fingers)):
        f = fingers[i]
        l = len(f.JOINT_NAMES)
    
        p = f.get_pos_of_joint(l-1, node.fingers_configuration[i][:-2])
        
        print(f) 
        print('joint', p)
        
        unit_v1_tip= f.tip_direction_from_last_joint(node.fingers_configuration[i][:-2])
        v1_tip = unit_v1_tip* f.tip_length
        
        print('tip', v1_tip)
        print()
        
    
    '''
    #pp.show()
        
    
    #manipulation.plot_path(path[:14],'path_used_on_robot_to_13.mp4')
    #manipulation.plot_path(path[:28],'path_used_on_robot_to_13.mp4')
   
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

if __name__ == "__main__":
    main()
    
    
    
    
    
    
 