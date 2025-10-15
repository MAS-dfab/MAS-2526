from compas.geometry import Point
from compas.geometry import Frame
from compas.geometry import Box
from compas.geometry import Point, Bezier, NurbsCurve, Line, Translation, Vector, Transformation
from compas.geometry import Frame
from compas.geometry import cross_vectors
from compas_rhino.conversions import curve_to_compas
from compas_rhino.geometry import RhinoNurbsCurve

import math as m

#there is a data class called wall_curve
#wall_curve has attributes length, divisions, row_height, row_divisions, nature (convex or concave), and degree (amplitude)

class Wall:
    def __init__(self, length, divisions, row_height, row_count, concave, degree):
        self.length = length
        self.divisions = divisions
        self.row_height = row_height
        self.row_divisions = row_count
        self.wall_height = row_count * row_height
        self.concave = bool(concave)
        self.degree = degree

    def __str__(self):
        return f"Wall of {self.length} length, {self.divisions} divisions, {self.wall_height} height"
    
    def draw(self):
        box = Box(self.length, self.width, self.height)
        box.frame = self.frame.copy()
        return box