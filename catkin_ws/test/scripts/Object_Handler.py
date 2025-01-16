import mujoco
import numpy as np
import Math_Utilities as mu
import PyPlotting
import Constants
import random


class Obj():
    
    """
    For more compleks models do something nice, add models instead og hardcode them
    """
    
    
    
    #CENTER_LIMITS = [[280,380], [-60,60], [20,100]] 
    
    CENTER_LIMITS = [[290,370], [-70,-10], [50,100]] 
    
    def __init__(self, mj_model): 
        
        
        self.name = "item"
        evecs = np.identity(3)
        center = np.zeros(3) #self.world_from_center[:3,3]
        radii = mj_model.geom(self.name ).size.copy()*Constants.WOLD_SCALE
        print('obj_size', radii)
        radii = radii #- np.array([0.1,0.1,0.1])
        self.fit = mu.Ellipsoid()
        self.fit.direct_fit(center, evecs, radii)
        self.body_id = mj_model.body(self.name).id
        
        
        #world_from_center= mu.transformation_matrix_for_body(mj_model, self.name)
        #self.world_from_center_rotation = mu.rotation_from_transformation(world_from_center)
        #self.world_from_center_translation = world_from_center[:3,3]
        
        
        
        
        ''' To Use for physics'''
    
        self.mass = mj_model.body(self.name).mass
        self.inertia = np.zeros((3,3))
        dia_inertia = mj_model.body(self.name).inertia
        
        #print('mass', self.mass)
        #print('inertia',  dia_inertia )
        
        
        for i in range(3):
            self.inertia[i][i] = dia_inertia[i] * Constants.WOLD_SCALE**2


        
    def center_from_world_rotation_of_vector(self, vector, current_rotation):
        res = np.matmul(np.linalg.pinv(current_rotation), vector)
        return res
    
    
    def center_from_world_pos(self, point, current_rotation, current_translation):
        res = point - current_translation
        res = np.matmul(np.invert(current_rotation), res[:3]) 
        return res
    
    
    
    
    def world_from_center_pos(self, point, current_rotation, current_translation):
        res = np.matmul(current_rotation, point[:3]) + current_translation
        return res
    
    def world_from_center_rotation_of_vector(self, vector, current_rotation):
        res = np.matmul(current_rotation, vector)
        return res
    
    def surface_point(self, u,v, current_rotation, current_translation):
        p = self.fit.F(u,v)
        p_res = self.world_from_center_pos(p, current_rotation, current_translation)
        return p_res
    
    def normal(self, u,v, current_rotation):
        n = self.fit.normal(u,v)
        res_n = self.world_from_center_rotation_of_vector(n, current_rotation)
        return res_n
        
    def uv_for_surface_point(self, point, current_rotation, current_translation):
        p0 = point - current_translation
        p1 = np.matmul(np.transpose(current_rotation), p0)
        
        #print('point_on surface_calculated', p1)
        
        #pp = PyPlotting.Plot()
        #self.plot(pp, current_rotation, current_translation)
        #pp.plot_point(point)
        #pp.show()
        
        
        u,v = self.fit.uv_for_surface_point(p1)
        #print('res', self.fit.F(u,v))
        
        
        return u,v

    def principal_axes_and_curvature(self, u, v, current_rotation):
        axes, curvature = self.fit.principal_axis(u, v, True)
        x = self.world_from_center_rotation_of_vector(axes[0], current_rotation)
        y =  self.world_from_center_rotation_of_vector(axes[1], current_rotation)
        return [x,y], curvature

    def principal_axes(self, u, v, current_rotation):
        axes, curvature = self.principal_axes_and_curvature( u, v, current_rotation)
        return axes
    
    
    def surface_distance(self, current_uv, target_uv):
        
        return self.fit.surface_distance(current_uv, target_uv)
    
    def uv_limits(self):
        return self.fit.uv_limits
    
    def unit_tangential_direction(self, current_uv, target_uv, current_rotation):
        
        target_point = self.fit.F(target_uv[0], target_uv[1])
        tangent_origo = self.fit.F(current_uv[0], current_uv[1]) 
        tangent_normal = self.fit.normal(current_uv[0], current_uv[1])
        
        tangent = self.fit.tangential_direction(current_uv, target_point, tangent_origo, tangent_normal)    
        unit_tangent = mu.normalize_vector(tangent)
        rotated_unit_tangent = self.world_from_center_rotation_of_vector(unit_tangent, current_rotation)
        
        
        return rotated_unit_tangent
        
    def tangential_direction(self, current_uv, target_uv, current_rotation):
        
        target_point = self.fit.F(target_uv[0], target_uv[1])
        tangent_origo = self.fit.F(current_uv[0], current_uv[1]) 
        tangent_normal = self.fit.normal(current_uv[0], current_uv[1])
        
        tangent = self.fit.tangential_direction(current_uv, target_point, tangent_origo, tangent_normal)  
        rotated_tangent = self.world_from_center_rotation_of_vector(tangent, current_rotation)
        return rotated_tangent
    
    
    def directional_curvature(self, u, v, direction,  current_rotation):
        un_rotated_direction = self.center_from_world_rotation_of_vector(direction, current_rotation)
        curvature = self.fit.directional_curvature(u, v, un_rotated_direction)
        return curvature
    
    
    
    def plot(self, pp, current_rotation = np.identity(3), current_translation=np.zeros(3)):
        evecs = current_rotation
        center = current_translation
        pp.ellipsoid_plot(center, evecs, self.fit.radii)
        return pp
        
        
    
    
    
    def random_configuration(self):

        axis = np.empty(3)
        obj_pos = np.empty(3)
        for i in range(3):
            axis[i] = random.random() - 0.5 
            obj_pos [i]= self.CENTER_LIMITS[i][0] + random.random()*(self.CENTER_LIMITS[i][1]-self.CENTER_LIMITS[i][0])
        theta = random.random()*np.pi*2
        
        axis_angle = np.append(axis, theta)
        
        return obj_pos, axis_angle