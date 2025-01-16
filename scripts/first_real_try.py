#!/usr/bin/env python3

import os
from threading import Lock, Thread
from tkinter.tix import Tree

import Main
import numpy as np
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import MSTAll  # This is the tactile sensor message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray



import time
from enum import Enum

import Arc
import Config_Deliver
import Constants
import Force_Path_Calculator
import Graph_Utilities
import Grasp_Handler
# Sim
import mujoco
import mujoco.viewer
import Node
import numpy as np
import Object_Handler
import Physics_Engine
import PyPlotting
import Shadow_Hand

mutex1 = Lock()
mutex2 = Lock()


class State(Enum):
    INIT_ROBOT = 0
    INSERT_OBJ = 1
    RUN_PATH = 2
    PATH_FINISHED = 3
    ADJUST_FORCE = 4


class Execute_State(Enum):
    NODE_TO_NODE = 0
    ARC = 1
    FINISHED = 2


class Node_State(Enum):
    ADD_FINGER = 0
    GRASP_HOLD = 1
    REMOVE_FINGER = 2


class Arc_Element_Values():
    def __init__(self, arc, q_deliver: Config_Deliver.Config_Deliver):
        self.fingers_in_contact = arc.start_node.fingers_in_contact
        start_q = q_deliver.q_sh_from_quv(
            self.fingers_in_contact, arc.start_node.fingers_configuration)
        end_q = q_deliver.q_sh_from_quv(
            self.fingers_in_contact, arc.target_node.fingers_configuration)
        # print('start_q', start_q)
        # print('end_q', end_q)
        max_diff = q_deliver.max_joint_distance(start_q, end_q)
        self.duration = (
            max_diff / q_deliver.radian_change_pr_sec + q_deliver.writing_interval)
        self.current_time = 0  # -q_deliver.writing_interval # due to set_new_exc_state

    def set_current_time(self, time):
        self.current_time = time


class Node_To_Node_Element_Values():
    def __init__(self, node1, node2, q_deliver: Config_Deliver.Config_Deliver):
        self.node1_target = q_deliver.q_sh_from_quv(
            node1.fingers_in_contact, node1.fingers_configuration)
        self.node2_target = q_deliver.q_sh_from_quv(
            node2.fingers_in_contact, node2.fingers_configuration)

        fingers = [f for f in node1.fingers_in_contact]
        configurations = [c for c in node1.fingers_configuration]
        for i in range(len(node2.fingers_in_contact)):
            f = node2.fingers_in_contact[i]
            if f not in fingers:
                fingers.append(f)
                configurations.append(node2.fingers_configuration[i])
        # print('fingers', fingers)
        # print('configurations', configurations)

        self.sharing_config = q_deliver.q_sh_from_quv(fingers, configurations)

        self.state = Node_State.ADD_FINGER

        self.holding_time = 0.5
        self.current_time = 0


class Path_Runner:

    def __init__(self):
        # other class dependent
        dir_path = os.path.dirname(os.path.realpath(__file__))
        xml_path = dir_path+ '/Shadow_Hand_21_03_2024/world.xml'
        mj_model = mujoco.MjModel.from_xml_path(xml_path)
        mj_data = mujoco.MjData(mj_model)

        Grasp_Handler.Grasp_Creator.SHADOW_HAND_CALCULATION = True
        Arc.Arc.SHADOW_HAND_CALCULATION = True

        self.shadow_hand = Shadow_Hand.Shadow_hand(mj_model, mj_data)
        self.fingers = [self.shadow_hand.fingers[i]
                        for i in range(1, len(self.shadow_hand.fingers))]
        self.fingers.append(self.shadow_hand.fingers[0])

        obj = Object_Handler.Obj(mj_model)
        Arc.Arc.set_obj(obj)
        Physics_Engine.Physics_Engine.set_obj(obj)

        # path_execute_ dependen
        path_name = dir_path + '/path.txt'

        self.include_force = False
        self.MAX_FORCE_DIFF = 0.1
        self.MIN_FORCE_DIFF = -0.1
        self.MAX_PENETRATION = 1  # mm
        self.POS_ADJUST_ONE_STEP = 0.01  # mm

        writing_interval = 0.01
        self.deliver = Config_Deliver.Config_Deliver(
            self.shadow_hand, writing_interval)

        self.path = Graph_Utilities.read_path(self.shadow_hand, path_name)

        # force_path_cal = Force_Path_Calculator.Force_Path_Calculator(self.path, self.shadow_hand, writing_interval)
        # force_path = force_path_cal.force_path
        # Force_Path_Calculator.File_Handler.write_path(force_path, 'force_path.txt')

        self.force_path = Force_Path_Calculator.File_Handler.read_path(
            dir_path +'/force_path.txt')
        path_idx = 0
        end_idx = 3
        self.path = self.path[path_idx:]  # self.path[3:]
        self.force_path = self.force_path[path_idx:]  # self.force_path[3:]
        # print(self.force_path[0])

        self.state = State.INIT_ROBOT

        self.idx = 0
        self.is_hadeling_element = False
        self.element = self.path[self.idx]

        self.q_current = self.deliver.q_sh_from_quv(self.shadow_hand.fingers, [
                                                    [0 for i in range(f.number_joints + 2)] for f in self.shadow_hand.fingers])
        self.q_target = self.deliver.q_sh_from_quv(
            self.path[0].fingers_in_contact, self.path[0].fingers_configuration)

        self.exc_state = Execute_State.NODE_TO_NODE
        self.arc_val = None

        '''
        print('test')
        for i in range(len(force_path)):
            e = force_path[i]
            e2 = self.force_path[i]
            if type(e) == Force_Path_Calculator.Force_Arc:
                for j in range(len(e.normal_force)):
                    for p in range(len(e.normal_force[j])):
                        if e.normal_force[j][p] != e2.normal_force[j][p]:
                            print('error')
            else: 
                for j in range(len(e.normal_force)):
                    if e.normal_force[j] != e2.normal_force[j]:
                        print('error')
        '''
        # self.force_path =  Force_Path_Calculator.File_Handler.read_path('force_path.txt')

        # np.array([3,3,3,3,3]) #np.zeros(5)
        self.pos_displacement = np.zeros(5)
        self.fingers_in_contact = self.element.fingers_in_contact
        self.fingers_configuration = self.element.fingers_configuration
        self.force_target = self.force_path[self.idx].normal_force

        # test
        # print(self.force_target)
        # self.state = State.ADJUST_FORCE
        # self.q_current = self.deliver.q_sh_from_quv(self.fingers_in_contact, self.fingers_configuration)

        # print(self.path)

    def set_new_exc_state(self):
        if self.exc_state == Execute_State.ARC:
            self.idx += 1
            self.element = self.path[self.idx]

        self.idx += 1
        if self.idx >= len(self.path):
            self.exc_state = Execute_State.FINISHED
        else:
            self.element = self.path[self.idx]
            if type(self.element) == Arc.Arc:
                self.exc_state = Execute_State.ARC
                self.arc_val = Arc_Element_Values(self.element, self.deliver)
                self.q_target = self.q_current
            else:
                self.exc_state = Execute_State.NODE_TO_NODE
                self.node_to_node_val = Node_To_Node_Element_Values(
                    self.path[self.idx-1], self.element, self.deliver)
                self.q_target = self.node_to_node_val.sharing_config

    def get_current_q(self):
        return self.q_current

    def execute_step(self):
        if self.include_force:
            delta_q = [[0 for i in range(len(self.q_current[j]))]
                       for j in range(5)]

        if self.state == State.INIT_ROBOT:
            continue_to_new_state = self.INIT_ROBOT_function()
            if continue_to_new_state:
                self.state = State.INSERT_OBJ

        elif self.state == State.INSERT_OBJ:
            continue_to_new_state = self.INSERT_OBJ_function()
            if continue_to_new_state:
                self.set_new_exc_state()
                if self.include_force:
                    self.state = State.ADJUST_FORCE
                else:
                    self.state = State.RUN_PATH

        elif self.state == State.RUN_PATH:
            # print('running path')
            continue_to_new_state = self.RUN_PATH_function()

        elif self.state == State.ADJUST_FORCE:
            print('adjusting force')
            force = self.get_force_from_robot()
            print('force', force)
            print('target', self.force_target)
            print('dif', [force[i] - self.force_target[i] for i in range(len(force))])
            
            pos_delta = self.pos_displacement.copy()

            delta_q = self.adjust_force()

            change = abs(
                sum([pos_delta[i] - self.pos_displacement[i] for i in range(5)]))
        
            if change < 10**-6:
                ind = input('continue to path_run: c')
                if ind == 'c':
                    self.state = State.RUN_PATH
    
        q_current = self.q_current

        if self.include_force and (self.state == State.RUN_PATH):
            delta_q = self.adjust_force()
            q_current = [[q_current[j][i] + delta_q[j][i]
                          for i in range(len(q_current[j]))] for j in range(5)]
        if self.include_force and (self.state == State.ADJUST_FORCE):
            
            q_current = [[q_current[j][i] + delta_q[j][i]
                          for i in range(len(q_current[j]))] for j in range(5)]

        return q_current

    def RUN_PATH_function(self):
        path_executed = False

        if self.exc_state == Execute_State.ARC:
            # print('Arc', self.arc_val.current_time, self.arc_val.duration)
            self.q_current, is_target_reached = self.deliver.get_config(
                self.q_current, self.q_target)
            if self.arc_val.current_time >= self.arc_val.duration and is_target_reached:
                self.set_new_exc_state()
            else:
                if is_target_reached:
                    current_time = self.arc_val.current_time + self.deliver.writing_interval
                    current_time = min(current_time, self.arc_val.duration)
                    self.arc_val.set_current_time(current_time)
                    duration = self.arc_val.duration
                    config = self.element.get_configuration(
                        current_time, duration)

                    self.q_target = self.deliver.q_sh_from_quv(
                        self.arc_val.fingers_in_contact, config)

                    if self.include_force:
                        self.force_target = self.force_path[self.idx].get_force(
                            current_time, duration)
                        self.fingers_configuration = config

        elif self.exc_state == Execute_State.NODE_TO_NODE:
            if self.node_to_node_val.state == Node_State.ADD_FINGER:
                # node1 force
                # print('add finger')
                self.q_current, is_target_reached = self.deliver.get_config(
                    self.q_current, self.q_target)
                if is_target_reached:
                    self.node_to_node_val.state = Node_State.GRASP_HOLD
            elif self.node_to_node_val.state == Node_State.GRASP_HOLD:

                if self.include_force:
                    self.fingers_in_contact = self.element.fingers_in_contact
                    self.fingers_configuration = self.element.fingers_configuration
                    self.force_target = self.force_path[self.idx].normal_force
                # new force
                # print('hold finger')
                if self.node_to_node_val.current_time >= self.node_to_node_val.holding_time:
                    self.node_to_node_val.state = Node_State.REMOVE_FINGER
                else:
                    self.node_to_node_val.current_time += self.deliver.writing_interval
                    self.q_target = self.node_to_node_val.node2_target
            else:

                # node2 force
                # print('remove finger')
                self.q_current, is_target_reached = self.deliver.get_config(
                    self.q_current, self.q_target)
                if is_target_reached:
                    self.set_new_exc_state()
        else:
            print('path has finished')
            path_executed = True

        return path_executed

    def adjust_force(self):
        # print('adjust_force0')
        q_delta = self.delta_q_from_pos_displacement()
        # print('force_adjust1', q_delta)
        pos_delta, q_delta = self.force_adjust(q_delta)
        # print('asjusting2 force', pos_delta)
        # time.sleep(10)

        self.pos_displacement = np.array(pos_delta)

        # time.sleep(10)
        return q_delta

    def delta_q_from_pos_displacement(self):
        q_delta = [[0 for i in range(len(self.q_current[j]))]
                   for j in range(5)]

        old_pos_delta = self.pos_displacement.copy()
        pos_delta = np.zeros(5)

        change = abs(sum([old_pos_delta[i] - pos_delta[i] for i in range(5)]))

        # print('gating pos_adjust', change)
        c = 0
        while change > 10**-6:
            c += 1
            # print(c)
            old_pos_delta = pos_delta.copy()

            pos_delta, q_delta = self.pos_adjust(q_delta, pos_delta)

            change = abs(sum([old_pos_delta[i] - pos_delta[i]
                         for i in range(5)]))
            # print('change', change, pos_delta, old_pos_delta, sum([old_pos_delta[i] - pos_delta[i] for i in range(5)] ))
            # print('q_delta', q_delta)
        # print('pos_delta', pos_delta)
        return q_delta

    def get_force_from_robot(self):
        with mutex1:
            force = CallBack_Functions.robot_force.copy()
        # to do
        return force

    def get_pos_from_robot(self):
        with mutex2:
            posistions = CallBack_Functions.robot_pos.copy()

        return posistions  # def get p from robot

    def pos_adjust(self, delta_q, delta_pos):
        # print('pos_adjust', delta_q)
        # print(self.fingers_configuration)

        fingers_pos_displacement = []
        fingers = []
        config = []
        fingers = self.fingers
        for i in range(len(self.fingers_in_contact)):
            f = self.fingers_in_contact[i]
            idx = fingers.index(f)
            quv = [self.fingers_configuration[i][j] + delta_q[idx][j]
                   for j in range(len(delta_q[idx]))]
            quv.append(self.fingers_configuration[i][-2])
            quv.append(self.fingers_configuration[i][-1])
            config.append(quv)
            fingers_pos_displacement.append(
                self.pos_displacement[idx]-delta_pos[idx])

        # print('force_adjust2', config)
        # print('remaining_pos_displacement', fingers_pos_displacement)
        delta_pos_new, q_disp = self.pos_displacement_function(
            self.fingers_in_contact, config, fingers_pos_displacement)
        # print('new_q_disp', q_disp)
        # print('new_pos_disp', delta_pos_new)
        # print('old_pos_disp', delta_pos)
        # print('old_q_disp', delta_q)

        res_delta_q = delta_q.copy()
        res_delta_pos = delta_pos.copy()

        for i in range(len(self.fingers_in_contact)):
            f = self.fingers_in_contact[i]
            idx = self.fingers.index(f)
            res_delta_q[idx] = [res_delta_q[idx][j] + q_disp[i][j]
                                for j in range(len(q_disp[i]))]
            res_delta_pos[idx] += delta_pos_new[i]
        # print('res')
        # print(res_delta_q)

        return res_delta_pos, res_delta_q

    def force_adjust(self, delta_q):
        # print('force_adjust1', delta_q, self.fingers_configuration)
        force_from_robot = self.get_force_from_robot()
        print('force_from_robot', force_from_robot)
        diff = np.array(self.force_target) - force_from_robot
        print('diff', diff)

        force_dif_for_fingers = []
        fingers_pos_displacement = []
        fingers = []
        config = []
        fingers = [f for f in self.shadow_hand.fingers[1:]]
        fingers.append(self.shadow_hand.fingers[0])
        for i in range(len(self.fingers_in_contact)):
            f = self.fingers_in_contact[i]
            idx = fingers.index(f)
            quv = [self.fingers_configuration[i][j] + delta_q[idx][j]
                   for j in range(len(delta_q[idx]))]
            quv.append(self.fingers_configuration[i][-2])
            quv.append(self.fingers_configuration[i][-1])
            config.append(quv)
            force_dif_for_fingers.append(diff[idx])
            fingers_pos_displacement.append(self.pos_displacement[idx])

        # print('force_adjust2', config)

        delta_pos, q_disp = self.pos_displacement_based_on_force(
            force_dif_for_fingers, self.fingers_in_contact, config, fingers_pos_displacement)

        res_delta_q = delta_q.copy()
        res_delta_pos = self.pos_displacement.copy()

        # print(delta_q)
        # print(q_disp)

        for i in range(len(self.fingers_in_contact)):
            f = self.fingers_in_contact[i]
            idx = f.get_index_in_shadow_hand()-1
            res_delta_q[idx] = [res_delta_q[idx][j] + q_disp[i][j]
                                for j in range(len(q_disp[i]))]
            res_delta_pos[idx] += delta_pos[i]

        # print(res_delta_pos, res_delta_q)

        return res_delta_pos, res_delta_q

    def pos_displacement_function(self, fingers_in_contact, fingers_configuration, remaining_pos_displacement):

        # print('pos_displacement_function', remaining_pos_displacement)
        # force_dif[0] = 0.1

        disp = np.zeros(6)
        val = self.POS_ADJUST_ONE_STEP  # this value is in mm.
        q_adjust = []
        displacement = np.zeros(len(remaining_pos_displacement))
        for i in range(len(fingers_in_contact)):
            finger = fingers_in_contact[i]
            if abs(remaining_pos_displacement[i]) >= val:
                q = fingers_configuration[i][:-2]
                u, v = fingers_configuration[i][-2:]
                # print(finger, q, u, v)
                # print('contact_point', finger.contact_point(q, u, v))
                n = finger.contact_normal(q, u, v)

                p_disp = n * val * np.sign(remaining_pos_displacement[i])

                disp[:3] = p_disp
                # print(finger, q)
                # print('disp', disp)

                # pp = PyPlotting.Plot()
                # hp = PyPlotting.HandAndObjectPlot(pp)
                # hp.plot_fixed_fingers_and_configurations(Arc.Arc.obj, self.element.rotation, self.element.pos, self.fingers_in_contact, self.fingers_configuration, self.element.fingers_uv_on_obj_surface, False)

                # pp.plot_vector(finger.contact_point(q, u, v), n, 'r',)

                # pp.show()

                q_n = finger.joint_displacement_from_end_effector_displacement(
                    q, u, v, disp, True)

                in_range = True

                for j in range(0, len(q_n)):
                    if q_n[j] > finger.max_limits[j] or q_n[j] < finger.min_limits[j] or abs(q_n[j]-fingers_configuration[i][j]) > 0.1:
                        in_range = False

                if in_range:
                    q_adjust.append(np.array(q_n)-np.array(q))
                    displacement[i] += val * \
                        np.sign(remaining_pos_displacement[i])
                else:
                    q_adjust.append([0 for i in range(len(q_n))])

            else:
                q_adjust.append(
                    [0 for i in range(len(fingers_configuration[i][:-2]))])
        return displacement, q_adjust

    def pos_displacement_based_on_force(self, force_dif_for_fingers, fingers_in_contact, fingers_configuration, fingers_pos_displacement):

        force_dif = force_dif_for_fingers
        pos_displacement = fingers_pos_displacement.copy()
        # force_dif[0] = 0.1

        disp = np.zeros(6)
        val = self.POS_ADJUST_ONE_STEP  # this value is in mm.
        q_adjust = []
        displacement = np.zeros(len(fingers_pos_displacement))
        for i in range(len(fingers_in_contact)):
            finger = fingers_in_contact[i]
            if (force_dif[i] > self.MAX_FORCE_DIFF or force_dif[i] < self.MIN_FORCE_DIFF):

                q = fingers_configuration[i][:-2]
                u, v = fingers_configuration[i][-2:]

                n = finger.contact_normal(q, u, v)

                p_disp = n * val * np.sign(force_dif[i])

                disp[:3] = p_disp
                # print(finger, q)

                q_n = finger.joint_displacement_from_end_effector_displacement(
                    q, u, v, disp, True)

                in_range = True

                if abs(pos_displacement[i]) > self.MAX_PENETRATION:
                    in_range = False

                else:
                    for j in range(0, len(q_n)):
                        if q_n[j] > finger.max_limits[j] or q_n[j] < finger.min_limits[j] or abs(q_n[j]-fingers_configuration[i][j]) > 0.1:
                            in_range = False

                if in_range:
                    displacement[i] += val * np.sign(force_dif[i])
                    q_adjust.append(np.array(q)-np.array(q_n))
                else:
                    q_adjust.append([0 for i in range(len(q_n))])
            else:
                q_adjust.append(
                    [0 for i in range(len(fingers_configuration[i][:-2]))])
        return displacement, q_adjust

    def INIT_ROBOT_function(self):
        self.q_current, is_target_reached = self.deliver.get_config(
            self.q_current, self.q_target)
        return is_target_reached

    def INSERT_OBJ_function(self):
        node = self.element
        self.print_dif_pos(node)
        ind = input("b: backwards, f: forwards, adjusted and conticue: c  ")
        # ind = 'c'
        q_back = self.deliver.q_sh_from_quv([], [])
        q_forward = self.deliver.q_sh_from_quv(
            node.fingers_in_contact, node.fingers_configuration)
        go_to_new_state = False
        if ind == 'b':
            print('going back')
            self.q_current, is_target_reached = self.deliver.get_config(
                self.q_current, q_back)
        elif ind == 'f':
            print('going forwards')
            self.q_current, is_target_reached = self.deliver.get_config(
                self.q_current, q_forward)
            if is_target_reached:
                print('config target is reached')
        elif ind == 'c':
            print('going to execute path')
            go_to_new_state = True
            # if force with in limits write as well.
        return go_to_new_state

    def print_dif_pos(self, node):
        p_calc = self.get_contact_pos(
            node.fingers_in_contact, node.fingers_configuration)

        p_from_robot = self.get_pos_from_robot()

        world_from_robot = []
        T = self.shadow_hand.world_from_palm
        for p in p_from_robot:
            p_t = np.ones(4)
            p_t[:3] = p[:3]
            world_from_robot.append(np.matmul(T, p_t)[:3])

        for i in range(5):
            f = self.fingers[i]
            dif = (p_calc[i]-world_from_robot[i])*Constants.MM_TO_METERS
            print(f.NAME, dif,  '\n')

    def get_contact_pos(self, fingers_in_contact, fingers_config):
        pos = [np.zeros(3) for i in range(5)]
        for i in range(len(fingers_in_contact)):
            f = fingers_in_contact[i]
            quv = fingers_config[i]
            p = f.contact_point(quv[:-2], quv[-2], quv[-1])
            idx = f.get_index_in_shadow_hand()-1
            pos[idx] = p
        return pos






class CallBack_Functions():

    robot_force =[0 for i in range(5)]
    robot_pos = [np.zeros(3) for i in range(5)]


    def tactile_callbackff(self, msg):
        #p1 = msg.markers[0].points[0]
        magnitude = 0

        for i in range(len(msg.markers)):
            p = msg.markers[i].points[1]
            magnitude += np.linalg.norm([p.x,p.y,p.z])
        magnitude /= len(msg.markers)

        force = 803.072182237894*magnitude
        with mutex1: 
            self.robot_force[0] = force

    
    def tactile_callbackmf(self, msg):
        #p1 = msg.markers[0].points[0]
        magnitude = 0

        for i in range(len(msg.markers)):
            p = msg.markers[i].points[1]
            magnitude += np.linalg.norm([p.x,p.y,p.z])
        magnitude /= len(msg.markers)

        force = 803.072182237894*magnitude
        with mutex1: 
            self.robot_force[1] = force

    
    def tactile_callbackrf(self, msg):
        #p1 = msg.markers[0].points[0]
        magnitude = 0

        for i in range(len(msg.markers)):
            p = msg.markers[i].points[1]
            magnitude += np.linalg.norm([p.x,p.y,p.z])
        magnitude /= len(msg.markers)

        force = 803.072182237894*magnitude
        with mutex1: 
            self.robot_force[2] = force


    def tactile_callbacklf(self, msg):
        #p1 = msg.markers[0].points[0]
        magnitude = 0

        for i in range(len(msg.markers)):
            p = msg.markers[i].points[1]
            magnitude += np.linalg.norm([p.x,p.y,p.z])
        magnitude /= len(msg.markers)

        force = 803.072182237894*magnitude
        with mutex1: 
            self.robot_force[3] = force

    
    def tactile_callbackth(self, msg):
        #p1 = msg.markers[0].points[0]
        magnitude = 0

        for i in range(len(msg.markers)):
            p = msg.markers[i].points[1]
            magnitude += np.linalg.norm([p.x,p.y,p.z])
        magnitude /= len(msg.markers)

        force = 803.072182237894*magnitude
        with mutex1: 
            self.robot_force[4] = force
    


    def pose_callbackff(self, msg):
        pose = msg.pose
        xyz = pose.position
        new_pos = np.array([xyz.x, xyz.y, xyz.z])
        with mutex2:
            self.robot_pos[0] = new_pos
            #print('ff', self.robot_pos)
    
    def pose_callbackmf(self, msg):
        pose = msg.pose
        xyz = pose.position
        new_pos = np.array([xyz.x, xyz.y, xyz.z])
        with mutex2:
            self.robot_pos[1] = new_pos
            #print('mf')

    def pose_callbackrf(self, msg):
        pose = msg.pose
        xyz = pose.position
        new_pos = np.array([xyz.x, xyz.y, xyz.z])
        with mutex2:
            self.robot_pos[2] = new_pos
            #print('rf')

    def pose_callbacklf(self, msg):
        pose = msg.pose
        xyz = pose.position
        new_pos = np.array([xyz.x, xyz.y, xyz.z])
        with mutex2:
            self.robot_pos[3] = new_pos
            #print('lf')


    def pose_callbackth(self, msg):
        #print(msg)
        pose = msg.pose
        xyz = pose.position
        new_pos = np.array([xyz.x, xyz.y, xyz.z])
        with mutex2:
            self.robot_pos[4] = new_pos
            #print('th')
  


if __name__ == '__main__':
    rospy.init_node('obj_controller')

    # hand_commander = SrHandCommander("right_hand")

    # joints_position = hand_commander.get_joints_position()
    # joints_velocity = hand_commander.get_joints_velocity()

    # print(joints_position['rh_FFJ1'])

    call_back = CallBack_Functions()

    sub = rospy.Subscriber("/rh/publish_hand_mst_markers/rh_ff_arrows", MarkerArray,
                           callback=call_back.tactile_callbackff)


    sub = rospy.Subscriber("/rh/publish_hand_mst_markers/rh_mf_arrows", MarkerArray,
                           callback=call_back.tactile_callbackmf)

    
    sub = rospy.Subscriber("/rh/publish_hand_mst_markers/rh_rf_arrows", MarkerArray,
                           callback=call_back.tactile_callbackrf)
    

    sub = rospy.Subscriber("/rh/publish_hand_mst_markers/rh_lf_arrows", MarkerArray,
                           callback=call_back.tactile_callbacklf)

    
    sub = rospy.Subscriber("/rh/publish_hand_mst_markers/rh_th_arrows", MarkerArray,
                           callback=call_back.tactile_callbackth)















    sub = rospy.Subscriber("/sdu/tactile/ff_cop", Marker,
                           callback=call_back.pose_callbackff)
    
    sub = rospy.Subscriber("/sdu/tactile/mf_cop", Marker,
                           callback=call_back.pose_callbackmf)


    sub = rospy.Subscriber("/sdu/tactile/rf_cop", Marker,
                           callback=call_back.pose_callbackrf)

    sub = rospy.Subscriber("/sdu/tactile/lf_cop", Marker,
                           callback=call_back.pose_callbacklf)
    
    sub = rospy.Subscriber("/sdu/tactile/th_cop", Marker,
                           callback=call_back.pose_callbackth)

    pup = rospy.Publisher("/rh_trajectory_controller/command",
                          JointTrajectory, queue_size=10)

    rate = rospy.Rate(10)

    dir_path = os.path.dirname(os.path.realpath(__file__))

    rospy.loginfo('obj_controller has started')

    path_runner = Path_Runner()
    q_current = path_runner.get_current_q()

    while not rospy.is_shutdown():

        #ind = input("press enter to continue")

        msg = JointTrajectory()

        msg_joint_names = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2',
                           'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']
        msg.joint_names = msg_joint_names
        point = JointTrajectoryPoint()

        point.accelerations = []  #
        point.effort = []  #
        point.time_from_start = rospy.Duration(0.1)

        
        q_current = path_runner.execute_step()
        #print(q_current)
        msg_joint_positions = []
        for i in range(len(q_current)):
            quv =  q_current[i][::-1]
            for j in range(len(q_current[i])):
                msg_joint_positions.append(quv[j])
        
        #print(msg_joint_positions)
        #msg_joint_positions= np.zeros(len(msg_joint_names))
        point.positions = msg_joint_positions
        msg.points = [point]
        pup.publish(msg)
        rate.sleep()

        # rospy.spinOnce()
