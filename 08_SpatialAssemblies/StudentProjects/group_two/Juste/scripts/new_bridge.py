from compas.geometry import Line, Frame, Vector, Point, Transformation
from Sticks import Stick


class FrameToFrameBridge:
    def __init__(self, root_frame, target_frame, stick_length, width=None, depth=None, offset=0.0, grow_mode="both"):
        """
        Bridge two frames using a transformation-based alignment approach.

        Args:
            root_frame (Frame): Starting frame.
            target_frame (Frame): Ending frame.
            stick_length (float): Length of the sticks.
            width (float): Optional stick width.
            depth (float): Optional stick depth.
            offset (float): Optional offset from frame origin.
            grow_mode (str): "both", "root", or "target" — determines which stick(s) to grow.
        """
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset = offset
        self.grow_mode = grow_mode.lower()

        self.root_frame = root_frame
        self.target_frame = target_frame

        self.sticks = []
        self.debug_lines = []  # Optional debug visuals

        self._build_bridge()

    def _build_bridge(self):
        if self.grow_mode in ["both", "root"]:
            root_start = self.root_frame.point + self.root_frame.xaxis * self.offset
            root_axis = Line.from_point_and_vector(root_start, self.root_frame.xaxis * self.stick_length)
            root_stick = Stick(root_axis, self.root_frame.yaxis)
            self.sticks.append(root_stick)
            self.debug_lines.append(root_axis)

        if self.grow_mode in ["both", "target"]:
            target_start = self.target_frame.point + self.target_frame.xaxis * self.offset
            target_axis = Line.from_point_and_vector(target_start, -self.target_frame.xaxis * self.stick_length)
            target_stick = Stick(target_axis, self.target_frame.yaxis)
            self.sticks.append(target_stick)
            self.debug_lines.append(target_axis)

    def visualize(self):
        return [stick.geometry for stick in self.sticks]

    def debug(self):
        return self.debug_lines
