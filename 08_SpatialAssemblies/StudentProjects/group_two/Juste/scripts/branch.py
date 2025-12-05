# branch.py

import math
from compas.geometry import Line, Frame, Vector
from stick import Stick, stable_perp


class BranchingModule:
    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    def _build_child_from_face(self, parent, face_index, stick_angle):
        # ... your full correct implementation here ...
        pass

    def grow_once(self, face_index=0, stick_angle=0.0):
        """Grow a single child stick from a parent stick."""
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        """Grow multiple sequential sticks."""
        for _ in range(int(steps)):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)

