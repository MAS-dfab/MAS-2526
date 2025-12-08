# branch.py

from compas.geometry import Vector, Line
from stick_fixed import Stick

EPS = 1e-9

def face_direction(parent_frame, face_index):
    if face_index in (2,3):
        normal = parent_frame.yaxis.copy()
        fam = "Y"
    elif face_index in (4,5):
        normal = parent_frame.zaxis.copy()
        fam = "Z"
    else:
        return None, None

    if face_index % 2 == 1:
        normal *= -1.0

    return normal, fam


class BranchingModule:

    def __init__(self, root_stick, stick_length, width, depth,
                 offset01=0.5, collision_clearance=0.0):

        self.root = root_stick
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.offset01 = offset01
        self.collision_clearance = collision_clearance

        self.sticks = [root_stick]


    def grow_once(self, face_index, stick_angle,
                  existing_sticks=None, collision_safe=False):

        parent = self.sticks[-1]
        pf = parent.frame

        normal, fam = face_direction(pf, face_index)
        if normal is None:
            return None

        x = pf.xaxis.copy()
        y = pf.yaxis.copy()
        z = pf.zaxis.copy()

        # offset
        if fam == "Y":
            start = pf.point + y * (parent.width*0.5 + self.width*0.5)
        else:
            start = pf.point + z * (parent.depth*0.5 + self.depth*0.5)

        # direction
        t = self.offset01
        dir_vec = (1-t)*x + t*normal
        if dir_vec.length < EPS:
            dir_vec = normal
        dir_vec.unitize()

        ang = stick_angle * 3.141592653589793 / 180.0
        dir_vec = dir_vec.rotated(ang, x)

        end = start + dir_vec * self.stick_length
        axis = Line(start, end)

        child = Stick(axis,
                      length=self.stick_length,
                      width=self.width,
                      depth=self.depth,
                      parent_frame=pf)

        child.family = fam
        child.parent = parent
        parent.children.append(child)

        if collision_safe and existing_sticks:
            for other in existing_sticks:
                if child.intersects(other, clearance=self.collision_clearance):
                    return None

        self.sticks.append(child)
        return child
