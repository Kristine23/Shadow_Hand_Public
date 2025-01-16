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



class Finger_Tip_Manipulation:
    SHADOW_HAND_CALCULATION =True
    
    def __init__(self):
        Grasp_Handler.Grasp_Creator.SHADOW_HAND_CALCULATION = True
        Arc.Arc.SHADOW_HAND_CALCULATION = True
        
        
        
        self.mj_sim = Mujoco_Simulator.Mujoco_Simulator()
        self.shadow_hand = Shadow_Hand.Shadow_hand(self.mj_sim)
       
        self.obj = Object_Handler.Obj(self.mj_sim.mj_model)
        
        self.mj_sim.set_shadow_hand_and_obj(self.shadow_hand, self.obj)
        Arc.Arc.set_obj(self.obj)
        Physics_Engine.Physics_Engine.set_obj(self.obj)
        self.planner = Planner.Planner(self.obj, self.shadow_hand)
        
        
        self.mj_sim_old = Mujoco_Simulator_old.Mujoco_Simulator()
        self.mj_sim_old.set_shadow_hand(self.shadow_hand)
        
        #self.mj_sim_new = Mujoco_Simulator.Mujoco_Simulator_New()
        #self.mj_sim_new.set_shadow_hand_and_obj(self.shadow_hand, self.obj)
        
        
        
        
    
    def create_init_grasps_file(self):
        #
        fingers = [self.shadow_hand.fingers[0], self.shadow_hand.fingers[1], self.shadow_hand.fingers[3]]
        Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
        
        fingers = [self.shadow_hand.fingers[0], self.shadow_hand.fingers[1], self.shadow_hand.fingers[2]]
        Grasp_Handler.Grasp_Creator().create_init_grasps(self.obj, fingers) 
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
        
        
        #ok: 
        
        obj_axis_angle = [0.0,0.0,1.009,0.9255246947425946]
        pos =[352.6304841417851,-28.820720527279477,76.30556461974349]
        fingers_in_contact = [0,1,3]
        fingers_configuration = [[0.20068611627887228,0.7943757841088611,0.05264592041120182,0.1348934922527655,0.7389144817080673,1.3181064544873202,1.8120933143497602],[0.17022062202671667,0.019176296741574724,1.571,0.3494868336819956,1.091900305373079,1.7772968582373971],[-0.24154563155785286,0.919894857932856,1.571,0.21279181031551841,0.8248629348904774,1.467475038708935]]
        fingers_uv_on_obj_surface = [[-3.0142150375031593,1.4954581318018763],[-0.5251862342145274,2.1691998610655823],[1.0568613868973262,1.677188939196393]]

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
        '''
        
    
        
        #path = Graph_Utilities.read_path(self.shadow_hand, 'from_max_difference_easy_start.txt')
        #node = path[-1]
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
        
        
    def test2(self):
        rotation = [[0.5782907880021831,-0.8158299836985555,0.0010964534782494956],[0.8158110969253176,0.5782670342079792,-0.007713059173861231],[0.005658502039062848,0.005354889982377978,0.9999696527934998]]
        pos =[353.06920672674886,-28.9045995602952,76.2164826435153]
        fingers_in_contact = [0,1,3]
        fingers_configuration = [[0.18900297982103204,0.7193672463594303,0.16579584206489456,0.19191481014391812,0.6541603198026074,1.2758719467843747,1.8299096434827966],[0.18672832828886687,0.0126712055989638,1.571,0.34308867174459534,1.0876163666382503,1.7831750464887501],[-0.2800744529287509,0.9367553693794435,1.571,0.19421167564847974,0.8245439926385308,1.4697864358998074]]
        fingers_uv_on_obj_surface = [[-3.0292433664639637,1.4981320879117592],[-0.5269121218437333,2.1733387600694183],[1.0568745883691029,1.6782443148964492]]
        
        
        rotation1 = [[0.6221789757970478,-0.7827362441227468,-0.014740902711703295],[0.7826945678198083,0.6223295476338784,-0.009754365544600933],[0.01680879476647057,-0.005468663313120626,0.9998437668656388]]
        pos1 =[352.51189588440417,-30.08049434945221,77.8893826809949]
        fingers_in_contact1 = [0,1,3]
        fingers_configuration1 = [[0.18036935705323867,0.867081953573887,-0.07398755211585746,0.14260191668099353,0.7146465531785535,1.3417639734954783,1.814972740545792],[0.11708422686318967,0.052967383807938175,1.571,0.2856400303083375,1.0814320995813,1.7837357447142923],[-0.27984144499530383,0.929113403153073,1.571,0.14737224178923958,0.8470091046168813,1.4957790961057447]]
        fingers_uv_on_obj_surface1 = [[-3.011764726408607,1.5072656707444803],[-0.5294411478929311,2.1761620481681496],[1.077028041033928,1.680027351816791]]

        rotation2 = [[0.6335575534902776,-0.773574653131158,-0.013677808612199766],[0.7732885489010222,0.6336986576159315,-0.021232792428826315],[0.02509275899481368,0.0028753032511002192,0.9996809921556242]]
        pos2 =[351.74090391016426,-31.953182368789612,79.48101601951397]
        fingers_in_contact2 = [0,1,3]
        fingers_configuration2 = [[0.16868486891035578,0.8682514038714544,-0.07456812407455006,0.11720655727440528,0.7095388949159765,1.3483054250317312,1.8240320147459106],[0.05443374649238078,0.08649097954956665,1.571,0.24544856695992756,1.0628474545445423,1.7786739220597965],[-0.3469311182415548,0.9727816002988413,1.571,0.034514579695347436,0.8688177172601353,1.5365611383094042]]
        fingers_uv_on_obj_surface2 = [[-3.0123291877479765,1.513346818404016],[-0.5363273642979051,2.180631244451078],[1.0988789402271315,1.6877590822088508]]
        
        
        rotation3 = [[0.6946359927656413,-0.7190480181612053,-0.02123170113185848],[0.7192415811272546,0.6947564062000979,0.0022547774850104614],[0.013129567093893145,-0.01683697188886498,0.9997720394397693]]
        pos3 =[353.8459181661042,-32.87447332156403,78.11166255766719]
        fingers_in_contact = [0,1,3]
        fingers_configuration3 = [[0.17147621149543482,0.8874692157691509,-0.13291566920296247,0.1259958997386057,0.6991841980037697,1.437402737191489,1.7957196382948892],[-0.14075271510271017,-0.01831382360667492,1.571,0.37334836184140086,1.0893758738033468,1.681657043327908],[-0.3482358926363774,0.8749050904155057,1.571,0.25256417452330227,0.8797591247430082,1.452625700776299]]
        fingers_uv_on_obj_surface3 = [[-2.9886126463156026,1.527610025897214],[-0.7151237918047726,2.3524673482966056],[1.1720377991015638,1.7190411674905768]]
                
        a =  fingers_in_contact[0]
        b = fingers_in_contact[1]
        c = fingers_in_contact[2]
        
                
        fingers = self.shadow_hand.fingers[a:a+1] +self.shadow_hand.fingers[b:b+1] +self.shadow_hand.fingers[c:c+1]
        
        rotation =np.array([np.array(rotation[i]) for i in range(3)])
        
        pos = np.array(pos)
        i = 0
        j = 3
        node = Node.Node(rotation, pos, fingers[i:j], fingers_configuration[i:j], fingers_uv_on_obj_surface[i:j])
        
        
        #pp= PyPlotting.Plot()
        #hp = PyPlotting.HandAndObjectPlot(pp)
        
        #hp.plot_fixed_fingers_and_configurations(self.obj, rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        
        #pp.show()
        pos1 = np.array(pos1)
        rotation1 =np.array([np.array(rotation1[i]) for i in range(3)])
        
        pos2 = np.array(pos2)
        rotation2 =np.array([np.array(rotation2[i]) for i in range(3)])
        
        pos3 = np.array(pos3)
        rotation3 =np.array([np.array(rotation3[i]) for i in range(3)])
    
        node1 = Node.Node(rotation1, pos1 , fingers, fingers_configuration1, fingers_uv_on_obj_surface1)
        node2 = Node.Node(rotation2, pos2 , fingers, fingers_configuration2, fingers_uv_on_obj_surface2)
        node3 = Node.Node(rotation3, pos3 , fingers, fingers_configuration3, fingers_uv_on_obj_surface3)
        
        arc = Arc.Arc()
        
        #arc.step_existence_check(fingers, fingers_configuration, fingers_uv_on_obj_surface, rotation, pos, 0)
        #arc.test_existens(node2, node1)
        
        
        
        
    
    def test(self):
        path = Graph_Utilities.read_path(self.shadow_hand,  'path.txt') 
        
        node = path[0] 
        #pp.show()
        
        
        #pp= PyPlotting.Plot()
        #hp = PyPlotting.HandAndObjectPlot(pp)
        
        #hp.plot_fixed_fingers_and_configurations(self.obj, node.rotation, node.pos, node.fingers_in_contact, node.fingers_configuration, node.fingers_uv_on_obj_surface)
        
        #pp.show()
        
        
        
        #self.mj_sim.init_configurations(self.obj, path[0])
        #self.mj_sim.simulate_path(path, self.obj)
        '''
        print('fingers configuration', node.fingers_configuration[0])
       
        print(self.shadow_hand.fingers[0].max_limits)
        print(self.shadow_hand.fingers[0].min_limits)
            
        for i in range(len(path)):
            if type(path[i]) == Node.Node:
                print(i)
                print(path[i].fingers_in_contact)
        
        
        '''
        #return
        i = 0
        j = 3
        r = 0
        
        #<Shadow_Hand.Thumb object at 0x73bc3da42ad0> [-0.00533303  0.91371047 -0.18680871  0.21648195  0.97086914]
        #<Shadow_Hand.F1 object at 0x73bc3dbc4670> [-0.26305059  0.05900286  1.571       0.66382098]
        #<Shadow_Hand.F2 object at 0x73bc3dbc44c0> [0.15892135 0.45674141 1.571      0.4705149 ]
        
        #config = [[-0.00533303,  0.91371047, -0.18680871,  0.21648195,  0.97086914, path[i].fingers_configuration[0][-2], path[i].fingers_configuration[0][-1]], [-0.26305059,  0.05900286,  1.571,  0.66382098, path[i].fingers_configuration[1][-2], path[i].fingers_configuration[1][-1]], [0.15892135, 0.45674141, 1.571, 0.4705149, path[i].fingers_configuration[2][-2], path[i].fingers_configuration[2][-1]]]
        
        
        #config =  [[-0.003925359536275102, 0.9308867807961124, -0.20738309469251448, 0.11132225419281616, 1.0856421051295546, path[i].fingers_configuration[0][-2], path[i].fingers_configuration[0][-1]], [-0.23479396349708773, 0.033286367086248915, 1.571, 0.6836010685658718, path[i].fingers_configuration[1][-2], path[i].fingers_configuration[1][-1]], [0.2667787505383686, 0.44409723810185087, 1.571, 0.479438311908466, path[i].fingers_configuration[2][-2], path[i].fingers_configuration[2][-1]]]
        #node = Node.Node(path[i].rotation, path[i].pos, path[i].fingers_in_contact, config, path[i].fingers_uv_on_obj_surface)
        #self.mj_sim_new.init_configurations(self.obj, node)
        #self.mj_sim_new.simulate_path([node], self.obj)
        
        
        #force_visualisation_test.test(path[0], self.obj)
        '''
        obj_axis_angle = [0.0,0.0,1.009,0.9255246947425946]
        pos =[352.6304841417851,-28.820720527279477,76.30556461974349]
        fingers_in_contact = [0,1,3]
        fingers_configuration = [[0.20068611627887228,0.7943757841088611,0.05264592041120182,0.1348934922527655,0.7389144817080673,1.3181064544873202,1.8120933143497602],[0.17022062202671667,0.019176296741574724,1.571,0.3494868336819956,1.091900305373079,1.7772968582373971],[-0.24154563155785286,0.919894857932856,1.571,0.21279181031551841,0.8248629348904774,1.467475038708935]]
        fingers_uv_on_obj_surface = [[-3.0142150375031593,1.4954581318018763],[-0.5251862342145274,2.1691998610655823],[1.0568613868973262,1.677188939196393]]

        a =  fingers_in_contact[0]
        b = fingers_in_contact[1]
        c = fingers_in_contact[2]
        
                
        fingers = self.shadow_hand.fingers[a:a+1] +self.shadow_hand.fingers[b:b+1] +self.shadow_hand.fingers[c:c+1]
     
        rotation =mu.rotation_from_axises_and_angle(obj_axis_angle[:3], obj_axis_angle[-1])
        
        pos = np.array(pos)
        node = Node.Node(rotation, pos, fingers, fingers_configuration, fingers_uv_on_obj_surface)
        '''
        node = path[0]
        
        self.mj_sim.init_configurations(self.obj, node)
        self.mj_sim.simulate_path([node], self.obj)
        
        #self.mj_sim_old.init_configurations(self.obj, node)
        #self.mj_sim_old.run([node], self.obj)
        
        #self.mujoco_simulate([path[-1]])
        
        
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

    create_init_grasps_file = False
    create_road_map = False
    crete_simulation = False
    
    
    manipulation.test()
    
    
    
    
    if create_init_grasps_file:
        manipulation.create_init_grasps_file()

    road_map_name = 'roadmap.txt'
    path_name = 'path.txt' 
    plot_name = 'plot.mp4'
    
    if create_road_map:
    
        #manipulation.create_road_map(road_map_name)
        path = manipulation.create_path_from_road_map(road_map_name)
        Graph_Utilities.write_path(path, path_name)
        manipulation.plot_path(path,plot_name)
    
    if crete_simulation:
        path = Graph_Utilities.read_path(manipulation.shadow_hand,  path_name)        
        i = 0
        path = path[i:i+1]
        manipulation.mujoco_simulate(path)
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #path = path[5:14]
    #manipulation.mujoco_simulate(path)
    return
    for i in range(len(path)):
        if type(path[i]) == Node.Node:
            path[i].is_graph_node = False
            print(path[i].fingers_in_contact)
    
    #path = path[5:6]
    
    #f = path[0].fingers_in_contact[1]
    #f2 = path[0].fingers_in_contact[2]
    #intersection = f.is_intersecting(path[0].fingers_configuration[1], f2, path[0].fingers_configuration[2])
    #dist = f.distance_between_fingers(path[0].fingers_configuration[1], f2, path[0].fingers_configuration[2])
    #print(dist)
    #return
    
    
    
    '''
    new_path = []
    for i in range(len(path)-1, len(path)-6, -1):
    
        if type(path[i]) == Arc.Arc:
            path[i].reverse_arc()
        else: 
            path[i].is_graph_node = False
        new_path.append(path[i])
    '''
    
    path = path[-5:]
    
    #print(path[0].fingers_in_contact)
    #print(path[0].fingers_configuration)
    for i in range(3):
        print(manipulation.obj.surface_point(path[0].fingers_uv_on_obj_surface[i][0], path[0].fingers_uv_on_obj_surface[i][1], np.identity(3), np.zeros(3)))
    p1 =manipulation.obj.surface_point(path[0].fingers_uv_on_obj_surface[0][0], path[0].fingers_uv_on_obj_surface[0][1], np.identity(3), np.zeros(3))
    p2 =manipulation.obj.surface_point(path[0].fingers_uv_on_obj_surface[1][0], path[0].fingers_uv_on_obj_surface[1][1], np.identity(3), np.zeros(3))
    p3 = manipulation.obj.surface_point(path[0].fingers_uv_on_obj_surface[2][0], path[0].fingers_uv_on_obj_surface[2][1], np.identity(3), np.zeros(3))
    
    
    #pp = PyPlotting.Plot()
    #hp = PyPlotting.HandAndObjectPlot(pp)
    #hp.plot_fixed_fingers_and_configurations(manipulation.obj, path[0].rotation, path[0].pos, path[0].fingers_in_contact, path[-1].fingers_configuration, path[-1].fingers_uv_on_obj_surface)

    #Graph_Utilities.write_path(new_path, 'sh_small_movement.txt')
    
    #path = Graph_Utilities.read_path(manipulation.shadow_hand,  'sh_small_movement.txt')

    
    #print(new_path)
    
    #manipulation.plot_path(new_path,'sh_small_movement.mp4')
    
    #manipulation.mujoco_simulate(path)
   
    #manipulation.store_q_values(path)
    
    #manipulation.mj_sim.simulate_from_file('sh_small_movements_start.txt', manipulation.obj)
    
    #manipulation.mj_sim.simulate_from_file('sh_small_movements_movement.txt', manipulation.obj)
    pp = PyPlotting.Plot()
    manipulation.obj.plot(pp, np.identity(3), np.zeros(3))
    pp.plot_point(p1, 'red')
    pp.plot_point(p2, 'green')
    pp.plot_point(p3, 'blue')
    pp.set_axes()
    pp.show()
    '''
    arc = path[1]
    
    node0 = path[0]
    idx = 50
    
    node0.fingers_configuration = arc.config_list[idx]
    node0.fingers_uv_on_obj_surface = arc.obj_uv_list[idx]
    node0.pos = arc.obj_pos_list[idx]
    node0.rotation = arc.obj_rotation_list[idx]
    
    arc = Arc.Arc()
    arc.create_arc(path[2], node0)
    arc.reverse_arc()
    path = [path[0], arc] + path[2:]
    
    Graph_Utilities.write_path(path, 'sh_test_path.txt')
    '''
    #path = Graph_Utilities.read_path(manipulation.shadow_hand,  'sh_test.txt')
    
    #print(path)
    
    #for f in manipulation.shadow_hand.fingers:
    #    print(f, f.max_limits)
    
    #manipulation.store_q_values(path)
    
   
    
    #print(len(arc.config_list))
    return
    
    node = path[7]
    
    
    
    manipulation.mujoco_simulate(path[-1:])
    
    #manipulation.mujoco_simulate(path[3:4])
    #print(manipulation.obj.mass*np.array([0, 0, -9.81]))
    #print(manipulation.obj.mass)
    
    #node = path[3] 
    om = Physics_Engine.Object_Movement(node.pos, node.rotation)
    
    Physics_Engine.Physics_Engine.fingers_normal_and_tangential_forces(om, node.fingers_in_contact, node.fingers_configuration, node.fingers_uv_on_obj_surface)
    
    is_grasp_stable = Physics_Engine.Physics_Engine.is_grasp_physical_stable(om, node.fingers_in_contact, node.fingers_configuration, node.fingers_uv_on_obj_surface)
    print(is_grasp_stable)
    #Graph_Utilities.write_path(path, 'test.txt')
    #path = Graph_Utilities.read_path(manipulation.shadow_hand,  'test.txt')
    #manipulation.plot_path(path,'test1.mp4')
    #manipulation.test()
    return
    
    # 
    # J = f1.jacobian_acts(quv1, True)
    #print(J)
    
    

if __name__ == "__main__":
    main()
    
    
    
    
    
    
 