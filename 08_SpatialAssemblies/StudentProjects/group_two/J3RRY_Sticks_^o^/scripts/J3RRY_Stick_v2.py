from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import Transformation
from Sticks import Stick
import math, random


class JStickAggregation:
    def __init__(self, first_axis):
        self.sticks = []
        self._init_first_stick(first_axis)

    def _init_first_stick(self, first_axis):
        self.sticks.append(Stick(first_axis))


    def spawn_next_stick(self, angle):
        """Create random index and t value for the next stick"""
        # random.seed(0)
        indices = [random.randint(0,3) for _ in range(2)]
        ts = [random.random() for _ in range(2)]
        to_frame = self.sticks[-1].eval_frame(indices[0], ts[0])
        from_frame = self.sticks[-1].eval_frame(indices[1], ts[1])
        from_frame.flip()
        
        """Orient stick using frame to frame"""
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_axis = self.sticks[-1].axis.transformed(T)
        ax = to_frame.zaxis
        pt = to_frame.point
        new_axis.rotate(math.radians(angle), ax, pt)
        self.sticks.append(Stick(new_axis))


    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]