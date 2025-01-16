import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import Arc

import Math_Utilities as mu
import Node
 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

class Plot:
    
    def __init__(self): 
        self.fig = plt.figure()
        self.axes = self.fig.add_subplot(111, projection='3d')
        self.axes.set_xlabel("X")
        self.axes.set_ylabel("Y")
        self.axes.set_zlabel("Z")

        
    def plot_3D_points(self, points, point_alpha = 0.01, point_color = 'g'):
        xx = [points[i][0] for i in range(len(points))]
        yy = [points[i][1] for i in range(len(points))]
        zz = [points[i][2] for i in range(len(points))]
        self.axes.scatter(xx, yy, zz, marker='o', color=point_color, alpha=point_alpha)
        #self.axes.scatter(points[0], points[:,1], points[:,2], marker='o', color='g', alpha=0.1)

    def plot_mesh(self, res_mesh):
        self.axes.add_collection3d(mplot3d.art3d.Poly3DCollection(res_mesh.vectors))
        scale = res_mesh.vectors.flatten()
        self.axes.auto_scale_xyz(scale, scale, scale)
        
    def plot_point(self, point, point_color = 'red'):
        self.axes.scatter(point[0], point[1], point[2], marker='o', color=point_color, s=20)
    
    
    def plot_vector(self, start, vector, vector_color = 'b', scale = 1):
        self.axes.quiver(start[0],start[1],start[2], vector[0]*scale, vector[1]*scale, vector[2]*scale, color=vector_color)  

    def plot_vectors(self, start_vectors, vectors, vector_color = 'b', scale = 1):
        for i in range(len(start_vectors)):
            self.plot_vector(start_vectors[i],vectors[i], vector_color, scale)
    
    def show(self):
        plt.show()
        
        
    def plot_linesegment(self, p1, p2, color):
        values = []
        for i in range(3):
            values.append([p1[i], p2[i]])
        self.axes.plot(values[0], values[1], values[2], color = color, linewidth = 2)
        
            
    def ellipsoid_plot(self, center, evecs, radii, cage_alpha=0.3):
        """Plot an ellipsoid"""
        
        
            
        u = np.linspace(0.0, 2.0 * np.pi, 100)
        v = np.linspace(0.0, np.pi, 100)
        
        # cartesian coordinates that correspond to the spherical angles:
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
        # rotate accordingly
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.matmul(evecs, [x[i, j], y[i, j], z[i, j]]) + center

        l = len(x)
        x_val = []
        y_val = []
        z_val = []
        start_id = 0
        for i in range(1,5):
            end_id =  np.int_(np.floor(l/4))*i 
            x_val.append(x[start_id:end_id])
            y_val.append(y[start_id:end_id])
            z_val.append(z[start_id:end_id])
            start_id = end_id -1
            
        #self.axes.plot_wireframe(x, y, z,  rstride=4, cstride=4, color=cage_color, alpha=cage_alpha)
        colors= ['blue', 'red','orange' ,'magenta',  'purple''purple']
        for i in range(len(x_val)):
            self.axes.plot_surface(x_val[i],y_val[i],z_val[i], color=colors[i], alpha=cage_alpha, linewidth=0)
        

    def set_axes(self, aspect = None):
        self.axes.set_aspect('equal', adjustable='box')
        #plt.axis('off') #poster remove
        #self.axes.set_box_aspect([1,1,1])

    def get_axes(self):
        return self.axes.get_aspect()
    


    def plot_sphere(self, radius, center):
    
    

        #radius (r), inclination (theta), and azimuth (phi).
        n = 100
        r = radius
        theta = np.linspace(0, 2.*np.pi, n)
        phi = np.linspace(0, np.pi, n)


        # Convert to Cartesian coordinates
        x = r * np.outer(np.cos(theta), np.sin(phi)) + center[0]
        y = r * np.outer(np.sin(theta), np.sin(phi)) + center[1]
        z = r * np.outer(np.ones(np.size(theta)), np.cos(phi)) + center[2]


        # Plot the sphere
        self.axes.plot_surface(x, y, z, color='gray', alpha=0.3, linewidth=0)
        #ax.plot_surface(x[0], y[0], z[0],  color = 'r' )
        #print(x[0][0],y[0][0],z[0][0])

        #plot points
        


        # Add title and labels
        self.axes.set_title("Sphere")
        self.axes.set_xlabel("X")
        self.axes.set_ylabel("Y")
        self.axes.set_zlabel("Z")

        # Adjust aspect ratio
        self.axes.set_aspect('equal', adjustable='box')
        #self.axes.set_box_aspect([1,1,1])




   


class HandAndObjectPlot: 
    
    #Finger_colors = ['magenta', 'blueviolet', 'deeppink', 'cyan', 'lime']
    
    Finger_colors = ['magenta', 'blue', 'deeppink', 'cyan', 'lime']
    
    def __init__(self, pp):
        self.pp = pp
        
    def plot_fingers_contact_and_goal(self, obj, obj_rotation, obj_pos, fingers, config, current_uv_on_obj, target_uv_on_obj):
        obj.plot(self.pp, obj_rotation, obj_pos)
         
        
        for i in range(0, len(fingers)):  
            
            finger = fingers[i]
            quv= config[i]
            color = self.Finger_colors[finger.get_index_in_shadow_hand()]
            
            self.plot_point_on_object(obj, obj_rotation, obj_pos, current_uv_on_obj[i])
            
            self.plot_point_on_object(obj, obj_rotation, obj_pos, target_uv_on_obj[i], 'green')
            self.plot_finger_tip(finger, quv)
            
            
            self.plot_finger_linesegments(finger, quv, color)
            
            
            
            #self.plot_contact_vectors(finger, obj, obj_rotation, obj_pos, uv_on_object[i], quv ) 
        self.pp.set_axes()
        
        
    
    
    def plot_fixed_fingers_and_configurations(self, obj, obj_rotation, obj_pos, fingers, configurations, uv_on_object, plot_contact_vecoters = True):
        obj.plot(self.pp, obj_rotation, obj_pos)
        
        for i in range(0, len(fingers)):  
            
            finger = fingers[i]
            quv= configurations[i]
            color = self.Finger_colors[finger.get_index_in_shadow_hand()]
            
            self.plot_point_on_object(obj, obj_rotation, obj_pos, uv_on_object[i])
            self.plot_finger_tip(finger, quv)
            self.plot_finger_linesegments(finger, quv, color) 
            if plot_contact_vecoters:
                self.plot_contact_vectors(finger, obj, obj_rotation, obj_pos, uv_on_object[i], quv ) 

        self.pp.set_axes()
    
    def plot_finger_tip(self, finger, quv):
        tip_points = finger.tip_points(quv[:-2])
        contact_point = finger.contact_point(quv[:-2],quv[-2], quv[-1])
        self.pp.plot_3D_points(tip_points, point_alpha = 0.001)
        self.pp.plot_point(contact_point, 'blue')
        #test
        #quv1 = quv.copy()
        #quv1[-2:] =  [3.63490481, 24.39471647]
        #contact_point = finger.contact_point(quv1[:-2],quv1[-2], quv1[-1])
        #self.pp.plot_point(contact_point, 'red')
        
    
    def plot_finger_linesegments(self, finger, quv, color):
        #u = (finger.UV_limit[0][0] + finger.UV_limit[0][1])/2
        #v = (finger.UV_limit[1][0] + finger.UV_limit[1][1])/2
        first_point = finger.get_pos_of_joint(0,quv[:-2])
        for i in range(1,len(finger.JOINT_NAMES)):
            pos = finger.get_pos_of_joint(i,quv[:-2])
            self.pp.plot_linesegment(first_point, pos, color)
            first_point = pos
        tip_direction = finger.tip_direction_from_last_joint(quv[:-2]) * 25 
        end_point = first_point + tip_direction 
        #tip_middel = finger.contact_point(quv[:-2],u, v)
        self.pp.plot_linesegment(first_point, end_point, color)
        tip_points = finger.tip_points(quv[:-2])
        contact_point = finger.contact_point(quv[:-2],quv[-2], quv[-1])
        self.pp.plot_3D_points(tip_points)
        self.pp.plot_point(contact_point, 'blue')  
    
    def plot_point_on_object(self, obj, obj_rotation, obj_pos, uv_on_object, point_color = 'red'):
        p = obj.surface_point(uv_on_object[0], uv_on_object[1], obj_rotation, obj_pos)
        self.pp.plot_point(p, point_color)
            
    
    def plot_obejct_normal(self, obj, obj_rotation, obj_pos, uv_on_object):
        p = obj.surface_point(uv_on_object[0], uv_on_object[1],obj_rotation, obj_pos)
        n =  obj.normal(uv_on_object[0], uv_on_object[1],obj_rotation)
        self.pp.plot_vector(p,n, 'r', scale= 10)
    
    def plot_finger_contact_normal(self, finger, quv):
        n = finger.contact_normal(quv[:-2],quv[-2], quv[-1])
        p = finger.contact_point(quv[:-2],quv[-2], quv[-1])
        self.pp.plot_vector(p,n, scale= 10)
    
    
    def plot_contact_vectors(self, finger, obj, obj_rotation, obj_pos, uv_on_object, quv):
        self.plot_obejct_normal(obj,obj_rotation, obj_pos, uv_on_object)
        self.plot_finger_contact_normal(finger, quv)
        
       




class ArcPlot:
    def __init__(self):
        return
    
    
    
    def plot_finger_and_configurations_and_goal_uv_and_roll(arc, obj_rotation, obj_pos, finger_idx, configurations, current_uv, uv_goals, roll_directions):
        pp = Plot()
        hand_plot = HandAndObjectPlot(pp)
        fingers = []
        for i in finger_idx: 
            fingers.append(arc.start_node.fingers_in_contact[i])
            uv = uv_goals[i]
            pp.plot_point(arc.obj.surface_point(uv[0], uv[1], obj_rotation, obj_pos), 'green')
        print(len(current_uv))
        
        for i in range(len(current_uv)): 
            
            p = arc.obj.surface_point(current_uv[i][0], current_uv[i][1], obj_rotation, obj_pos)
            pp.plot_point(p,'r')
            pp.plot_vector(p,roll_directions[i], scale=10)
        
        
    
        hand_plot.plot_fingers_contact_and_goal(arc.obj, obj_rotation, obj_pos, fingers, configurations, current_uv, uv_goals)
        pp.show()
    
    
    
    
    
    def plot_finger_and_configurations_and_goal_uv(arc, obj_rotation, obj_pos, finger_idx, configurations, obj_contact_points, uv_goals):
        pp = Plot()
        hand_plot = HandAndObjectPlot(pp)
        fingers = []
        for i in finger_idx: 
            fingers.append(arc.start_node.fingers_in_contact[i])
            uv = uv_goals[i]
            pp.plot_point(arc.obj.surface_point(uv[0], uv[1], obj_rotation, obj_pos), 'blue')
        uv_on_object = []
        for p in obj_contact_points: 
            uv = arc.obj.uv_for_surface_point(p,obj_rotation, obj_pos)
            uv_on_object.append(uv)
            
        
    
        hand_plot.plot_fixed_fingers_and_configurations(arc.obj, obj_rotation, obj_pos, fingers, configurations, uv_on_object)
        pp.show()
    
    
    def plot_finger_and_configurations(arc, obj_rotation, obj_pos, finger_idx, configurations, uv_on_object):
        fingers = []
        for i in finger_idx: 
            fingers.append(arc.start_node.fingers_in_contact[i])
        pp = Plot()
        hand_plot = HandAndObjectPlot(pp)
        hand_plot.plot_fixed_fingers_and_configurations(arc.obj, obj_rotation, obj_pos, fingers, configurations, uv_on_object)
        pp.show()

    
    
    def video_of_arc(self, arc, duration):
        self.arc = arc
        self.pp = Plot()
        self.duration = duration
        self.fingers = arc.start_node.fingers_sorted_by_number_joints()
        self.hand_plot = HandAndObjectPlot(self.pp)
        # creating animation
        animation = VideoClip(self.frame_for_arc, duration = duration)
 
        # displaying animation with auto play and looping
        animation.ipython_display(fps = 20, loop = True, autoplay = True)
        
    
    def frame_for_arc(self, t):
        print(self.pp.axes.get_ylim())
        xlim = self.pp.axes.get_xlim()
        ylim = self.pp.axes.get_ylim()
        zlim = self.pp.axes.get_zlim()
        print(xlim)
        self.pp.axes.clear()
        obj_rotation = self.arc.get_obj_rotation(t, self.duration)
        # to do make the following from arcs instead.
        quv_estimates = self.arc.get_configuration(t, self.duration)
        obj_contact_points = self.arc.get_obj_contact_pos(t, self.duration)
        obj_pos = self.arc.get_obj_pos(t, self.duration)
        uv_on_object = []
        for p in obj_contact_points: 
            uv = self.arc.obj.uv_for_surface_point(p,obj_rotation, obj_pos)
            uv_on_object.append(uv)
        
        self.hand_plot.plot_fixed_fingers_and_configurations(self.arc.obj, obj_rotation, obj_pos, self.fingers, quv_estimates, uv_on_object)
        print(xlim)
        self.pp.axes.set_xlim(xlim)
        self.pp.axes.set_ylim(ylim)
        self.pp.axes.set_zlim(zlim)
        self.pp.set_axes()
        
        return mplfig_to_npimage(self.pp.fig)




class Path_Plot:
  
    def video_of_path(self, path, obj, duration_for_node = 0.1, duration_for_arc = 2, file_name = 'path_.mp4'):
        self.obj = obj
        self.path = path
        self.idx = 0
        self.pp = Plot()
        self.duration_of_node = duration_for_node
        self.duration_of_arc = duration_for_arc
        if type(path[0]) == Arc.Arc:
            self.duration_of_element = duration_for_arc
        else:
            self.duration_of_element = duration_for_node 
        duration = 0
        for i in range(len(path)):
            if type(path[i]) == Arc.Arc:
                duration += duration_for_arc
            else:
                add = duration_for_node
                if i < len(path)-1 and i> 0:
                    if type(path[i-1]) == Arc.Arc and type(path[i+1]) == Arc.Arc:
                        add = 0
                duration += add
                
                
                
        self.last_t = 0
        self.hand_plot = HandAndObjectPlot(self.pp)
        self.xlim =None
        self.ylim = None
        self.zlim = None
        # creating animation
        animation = VideoClip(self.frame_for_path, duration = duration)
        animation.write_videofile(file_name,fps = 20)
    
    
    def frame_for_path(self, t):
        element = self.path[self.idx]
        if t - self.last_t > self.duration_of_element:
            self.last_t += self.duration_of_element
            self.idx += 1
            element = self.path[self.idx]
            if type(element) == Arc.Arc:
                self.duration_of_element = self.duration_of_arc
            else: 
                self.duration_of_element = self.duration_of_node
                '''
                if self.idx < len(self.path)-1 and type(self.path[self.idx-1]) == Arc.Arc:
                    if type(self.path[self.idx +1]) == Arc.Arc:
                        self.idx +=1
                        element = self.path[self.idx]
                        self.duration_of_element = self.duration_of_arc
                '''
                
        if type(element) == Arc.Arc:
            return self.frame_for_path_arc(t)
        else: 
            print(type(element))
            return self.frame_for_path_node(t)
    
    
    def frame_for_path_arc(self,t):
        element = self.path[self.idx]
        print('arc', type(element))
        self.pp.axes.clear()
        
       
        t1 = t - self.last_t
        obj_rotation = element.get_obj_rotation(t1, self.duration_of_element)
           
        quv_estimates = element.get_configuration(t1, self.duration_of_element)
        
        
        
        #print('idx', element.get_three_idexes_and_time( t1, self.duration_of_element))
        
        #quv_test = element.get_configuration_start_node_finger_sorted(t1, self.duration_of_element)
        #print('quv', quv_test[0])
        
        obj_contact_points = element.get_obj_contact_pos(t1, self.duration_of_element)
        obj_pos = element.get_obj_pos(t1, self.duration_of_element)
        #print(obj_pos)
        
        uv_on_object = []
        
        for p in obj_contact_points: 
            uv = self.obj.uv_for_surface_point(p,obj_rotation, obj_pos)
            uv_on_object.append(uv)
        fingers_in_contact = element.start_node.fingers_in_contact
        
        self.hand_plot.plot_fixed_fingers_and_configurations(self.obj, obj_rotation, obj_pos, fingers_in_contact , quv_estimates, uv_on_object)
       
        
        if (self.xlim == None):
            self.xlim = self.pp.axes.get_xlim()
            self.ylim = self.pp.axes.get_ylim()
            self.zlim = self.pp.axes.get_zlim()  
        
        self.pp.axes.set_xlim([self.xlim[0]-5, self.xlim[1]+5])
        self.pp.axes.set_ylim([self.ylim[0]-5, self.ylim[1]+5])
        self.pp.axes.set_zlim([self.zlim[0]-5, self.zlim[1]+5])
        
        
        return mplfig_to_npimage(self.pp.fig)
    
    def frame_for_path_node(self, t):
        element = self.path[self.idx]
        self.pp.axes.clear()
        obj_pos = element.pos
        obj_rotation = element.rotation
        uv_on_object = element.fingers_uv_on_obj_surface
        quv_estimates = element.fingers_configuration
        fingers_in_contact = element.fingers_in_contact
        
        
        print(obj_pos)
        self.hand_plot.plot_fixed_fingers_and_configurations(self.obj, obj_rotation, obj_pos,fingers_in_contact , quv_estimates, uv_on_object)
        
        if (self.xlim == None):
            self.xlim = self.pp.axes.get_xlim()
            self.ylim = self.pp.axes.get_ylim()
            self.zlim = self.pp.axes.get_zlim()  
        
        self.pp.axes.set_xlim([self.xlim[0]-5, self.xlim[1]+5])
        self.pp.axes.set_ylim([self.ylim[0]-5, self.ylim[1]+5])
        self.pp.axes.set_zlim([self.zlim[0]-5, self.zlim[1]+5])
        
        
        return mplfig_to_npimage(self.pp.fig)
    
    
    
    
    
    def video_of_arc_path(self, path, obj, duration):
        self.obj = obj
        self.path = path
        self.idx = 0
        self.pp = Plot()
        self.duration_of_element = duration/len(path)
        self.hand_plot = HandAndObjectPlot(self.pp)
        self.xlim =None
        self.ylim = None
        self.zlim = None
        # creating animation
        animation = VideoClip(self.frame_for_arcs, duration = duration)
        animation.write_videofile('arcs_.mp4',fps = 20)
        # displaying animation with auto play and looping
        #animation.ipython_display(fps = 20, loop = True, autoplay = True)
        
    
    def frame_for_arcs(self, t):
        if t > (self.idx+1)*self.duration_of_element:
            self.idx += 1
        return self.frame_for_arc(t)
    
    def frame_for_arc(self,t):
        element = self.path[self.idx]
        print('arc', type(element))
        self.pp.axes.clear()
        
       
        t1 = t - self.idx*self.duration_of_element
        obj_rotation = element.get_obj_rotation(t1, self.duration_of_element)
           
        quv_estimates = element.get_configuration(t1, self.duration_of_element)
        obj_contact_points = element.get_obj_contact_pos(t1, self.duration_of_element)
        obj_pos = element.get_obj_pos(t1, self.duration_of_element)
        uv_on_object = []
        
        for p in obj_contact_points: 
            uv = self.obj.uv_for_surface_point(p,obj_rotation, obj_pos)
            uv_on_object.append(uv)
        fingers_in_contact = element.start_node.fingers_sorted_by_number_joints()
        
        self.hand_plot.plot_fixed_fingers_and_configurations(self.obj, obj_rotation, obj_pos, fingers_in_contact , quv_estimates, uv_on_object)
       
        
        if (self.xlim == None):
            self.xlim = self.pp.axes.get_xlim()
            self.ylim = self.pp.axes.get_ylim()
            self.zlim = self.pp.axes.get_zlim()  
        
        self.pp.axes.set_xlim([self.xlim[0]-5, self.xlim[1]+5])
        self.pp.axes.set_ylim([self.ylim[0]-5, self.ylim[1]+5])
        self.pp.axes.set_zlim([self.zlim[0]-5, self.zlim[1]+5])
        
        
        return mplfig_to_npimage(self.pp.fig)

    
    def video_nodes_in_path(self, path_arcs_and_nodes, obj, duration):
        self.obj = obj

        self.idx = 0
        self.pp = Plot()
        # creating animation
        self.path = []
        for e in path_arcs_and_nodes: 
            if type(e) == Node.Node: 
                self.path.append(e)
        
        self.duration_of_element = duration/len(self.path)
        self.hand_plot = HandAndObjectPlot(self.pp)
        
        
        animation = VideoClip(self.frame_for_nodes, duration = duration)
 
        
        animation.write_videofile('nodes_.mp4',fps = 20)
        
        # displaying animation with auto play and looping
        #animation.ipython_display(fps = 20, loop = True, autoplay = True)



    def frame_for_nodes(self, t):
        if t > (self.idx+1)*self.duration_of_element:
            self.idx += 1
        return self.frame_for_node(t)
            
    def frame_for_node(self, t):
        element = self.path[self.idx]
        self.pp.axes.clear()
        obj_pos = element.pos
        obj_rotation = element.rotation
        uv_on_object = element.fingers_uv_on_obj_surface
        quv_estimates = element.fingers_configuration
        fingers_in_contact = element.fingers_in_contact
        
        self.hand_plot.plot_fixed_fingers_and_configurations(self.obj, obj_rotation, obj_pos,fingers_in_contact , quv_estimates, uv_on_object)
        return mplfig_to_npimage(self.pp.fig)
    
    
    def plot_nodes_in_path(self, path_arcs_and_nodes, obj, duration):
        self.obj = obj

        self.idx = 0
        # creating animation
        self.path = []
        for e in path_arcs_and_nodes: 
            if type(e) == Node.Node: 
                self.path.append(e)
        
        for element in self.path:
            pp = Plot()
            hand_plot = HandAndObjectPlot(pp)
            obj_pos = element.pos
            obj_rotation = element.rotation
            uv_on_object = element.fingers_uv_on_obj_surface
            quv_estimates = element.fingers_configuration
            fingers_in_contact = element.fingers_in_contact 
            print('pos', obj_pos)
            print('object_rotation', obj_rotation)
            print('uv', uv_on_object)
            print('quv', quv_estimates)
            print('fingers', fingers_in_contact)
            
            hand_plot.plot_fixed_fingers_and_configurations(self.obj, obj_rotation, obj_pos,fingers_in_contact , quv_estimates, uv_on_object)
            pp.show()





class Finger_Configuration_Track:
    
    def __init__(self, fingers, first_configuration):
        self.time = [0]
        self.calculated_configurations = [first_configuration]
        self.compairson_configurations = [first_configuration]
        self.fingers = fingers
        number_fingers= len(fingers)
        
        max_val = max([len(first_configuration[i]) for i in range(number_fingers)])
     
        self.fig, self.axs = plt.subplots(number_fingers, max_val)
        for i in range(number_fingers):
            finger = fingers[i]
            names = finger.JOINT_NAMES
            n_names = len(names)
            for j in range(n_names):
                self.axs[i,j].set_title(names[j])
        
            if n_names < len(first_configuration[i]):
                self.axs[i,-2].set_title('u')
                self.axs[i,-1].set_title('v')

        
        
    
    def add_points(self, t, calculated_conf, comperison_conf):
        self.time.append(t)
        self.calculated_configurations.append(calculated_conf)
        self.compairson_configurations.append(comperison_conf)
    
    
    def plot_calculated(self):
        
        n_names = [len(self.fingers[i].JOINT_NAMES) for i in range(len(self.fingers))]
        uv_contained = (n_names[0] < len(self.calculated_configurations[0][0]))
        for i in range(len(self.calculated_configurations[0])):
            for j in range(n_names[i]):
                y1 = [self.calculated_configurations[k][i][j] for k in range(len(self.calculated_configurations))]
                self.axs[i,j].plot(self.time, y1, color ='b')
        plt.show()
    
        
    def plot(self):
        
        n_names = [len(self.fingers[i].JOINT_NAMES) for i in range(len(self.fingers))]
        uv_contained = (n_names[0] < len(self.calculated_configurations[0][0]))
        
        
        
        for i in range(len(self.calculated_configurations[0])):
            for j in range(n_names[i]):
                y1 = [self.calculated_configurations[k][i][j] for k in range(len(self.calculated_configurations))]
                self.axs[i,j].plot(self.time, y1, color ='b')
                y2 = [self.compairson_configurations[k][i][j] for k in range(len(self.compairson_configurations))]
                self.axs[i,j].plot(self.time, y2, color ='r')
            
        '''
        
        for k in range(len(self.time)):
            conf1 = self.calculated_configurations[k]
            conf2 = self.compairson_configurations[k]
            t = self.time[k]
            
            for i in range(len(conf1)):
                for j in range(n_names[i]):
                    self.axs[i,j].plot(conf1[i][j],t, color ='b')
                    self.axs[i,j].plot(conf2[i][j],t, color ='r')
            
                if uv_contained:
                    self.axs[i,-2].plot(conf1[i][-2],t, color ='b')
                    self.axs[i,-1].plot(conf1[i][-1],t, color ='b')
                    
                    self.axs[i,-2].plot(conf2[i][-2],t, color ='r')
                    self.axs[i,-1].plot(conf2[i][-1],t, color ='r')
        '''
        
        
        plt.show()
            
            
        
        
        
        
        
        
        
        