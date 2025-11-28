from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import Transformation
from Sticks import Stick
import math, random


class JStickAggregation:
    def __init__(self, first_frame, length=50, aggregation_type=0, global_seed=None):
        self.sticks = []
        self.axes = []
        self.from_frames = []
        self.to_frames = []
        self.new_frames = []
        self.length = length
        # 0 = regular, 1 = random
        self.aggregation_type = aggregation_type
        self.global_seed = global_seed
        if global_seed is not None:
            random.seed(global_seed)

        self._init_first_stick(first_frame)

    def _init_first_stick(self, first_frame):
        self.sticks.append(Stick(first_frame, self.length))
        self.axes.append(Stick(first_frame, self.length).axis)

    def spawn_next_stick(self, angle=0, from_index=0, from_t=0.5, to_index=1, to_t=0.5):
        current_stick = self.sticks[-1]

        from_frame = current_stick.eval_frame(from_index, from_t)
        to_frame = current_stick.eval_frame(to_index, to_t)

        # Flip frame (fixed X axis)
        from_frame.yaxis = -from_frame.yaxis
        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        """Orient stick frame using frame to frame"""
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_frame = current_stick.frame.transformed(T)
        new_stick = Stick(new_frame, length=self.length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)
        self.new_frames.append(new_frame)
        return [from_frame, to_frame, new_frame]

    def spawn_next_stick_random(self, angle=0, local_seed=None):
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        current_stick = self.sticks[-1]
        """Create random index and t value for the next stick"""
        from_index = random.randint(0, 3)
        from_t = random.random()
        to_index = random.randint(0, 3)
        to_t = random.random()

        from_frame = current_stick.eval_frame(from_index, from_t)
        to_frame = current_stick.eval_frame(to_index, to_t)

        # Flip frame (fixed X axis)
        from_frame.yaxis = -from_frame.yaxis
        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        """Orient stick frame using frame to frame"""
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_frame = current_stick.frame.transformed(T)
        new_stick = Stick(new_frame, length=self.length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)
        self.new_frames.append(new_frame)
        return [from_frame, to_frame, new_frame]

    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
    

class Collision:
    def __init__(self, frame):
        self.frame = frame
