from compas.geometry import Line, Frame, Vector, Rotation, Polyline
from Sticks import Stick
import math


class Planarize:
    def __init__(self, points):
        self.points = points
        self.polyline = self.create_polygon()
    
    def create_polygon(self):
        """
        Docstring for create_polygon
        
        :param self: the input must be list inside list of points
        :return: polyline object
        """
        polylines = []
        for pts in self.points:
            for p in pts:
                pol = Polyline(p)
                polylines.append(pol)
        return polylines