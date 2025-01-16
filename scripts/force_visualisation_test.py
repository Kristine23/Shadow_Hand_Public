import Node
import PyPlotting
import Object_Handler
import numpy as np


def test(node:Node.Node, obj:Object_Handler.Obj):
    
    
    
    pp= PyPlotting.Plot()
    #hp = PyPlotting.HandAndObjectPlot(pp)
        
    #hp.plot_fixed_fingers_and_configurations(obj, node.rotation, node.pos, node.fingers_in_contact[i:j], node.fingers_configuration[i:j], node.fingers_uv_on_obj_surface[i:j])
    
    obj.plot(pp, node.rotation, node.pos)
    
    print('test')
    points = []
    new_q = [ node.fingers_configuration[j][:-2].copy() for j in range(0,len(node.fingers_in_contact))]

    for i in range(500):
        print(i)
        for j in range(0,len(node.fingers_in_contact)):
            
            finger = node.fingers_in_contact[j]
            quv = node.fingers_configuration[j]
            u, v = node.fingers_configuration[j][-2:]
            q = node.fingers_configuration[j][:-2]
            #n = finger.contact_normal(q, u,v)
            n = -obj.normal(node.fingers_uv_on_obj_surface[j][0], node.fingers_uv_on_obj_surface[j][1], node.rotation)
            disp = np.zeros(6)
           
                
            disp[:3] = n*0.01
            
            #new_q = finger.joint_displacement_from_end_effector_displacement(q, u, v, disp, True)
            new_q[j] = finger.joint_displacement_from_end_effector_displacement(new_q[j], u, v, disp, True)
            
            new_p = finger.contact_point(new_q[j], u, v)

            if i == 0:
                n = obj.normal(node.fingers_uv_on_obj_surface[j][0], node.fingers_uv_on_obj_surface[j][1], node.rotation)
                pp.plot_vector(new_p, n, 'b', 10)
                x, y = obj.principal_axes(node.fingers_uv_on_obj_surface[j][0], node.fingers_uv_on_obj_surface[j][1], node.rotation)
                pp.plot_vector(new_p, x, 'r', 10)
                pp.plot_vector(new_p, y, 'g', 10)
            
            points.append(new_p)
            
    
    pp.plot_point(points[0], 'g')
    pp.plot_3D_points(points[1:],1, 'r')
    
    pp.set_axes()
        
    pp.show()
    
    
    