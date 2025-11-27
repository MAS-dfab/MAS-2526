from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane
from group_one_sticks import Stick
import math



class Planarize:
    def __init__(self, point):
        self.point = point
    
    # def create_polygon(self):
    #     """
    #     Docstring for create_polygon
        
    #     :param self: the input must be list inside list of points
    #     :return: polyline object
    #     """
    #     polylines = []
    #     for pts in self.points:
    #         for p in pts:
    #             pol = Polyline(p)
    #             polylines.append(pol)
    #     return polylines
    
    def initial_face(self, angle_x = 0, angle_z = 0):
        """
        Docstring for initial_face
        
        :param self: initial point, vector angles in two directions, what is the heigth and width 
        :return: four points, 
        """
        face_points = []
        # STEP 1: create rotation planes for both points
        base_frame = Frame(self.point)
        
        frame_x_rotation = base_frame.rotated(math.radians(angle_x), base_frame.zaxis, base_frame.point)
        v_x = frame_x_rotation.xaxis
        v_x.unitize()
        v_p1 = v_x.scaled(3)

        frame_z_rotation = base_frame.rotated(math.radians(angle_z), base_frame.xaxis, base_frame.point)
        v_z = frame_z_rotation.zaxis
        v_z.unitize()
        v_p2 = v_z.scaled(3)
        
        # STEP 2: move root point in v_x direction and v_z direction
        p0 = self.point
        p1 = self.point.translated(v_p1)
        p2 = self.point.translated(v_p2)
        face_points += p0, p1, p2
        
        # STEP 3: construct fourt point from v_z point in v_x direction
        v_p3 = v_x.scaled(5)
        p3 = p2.translated(v_p3)
        face_points.insert(-1, p3)
        
        # STEP 4: return four points and maybe plain? If needed
        return face_points
        
        