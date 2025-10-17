
import math
from compas.geometry import Frame
from compas.geometry import Box

# There's a data type called Brick
# that has attributes: length, width, height and a point
# and also has a rotate behavior that accepts degrees of rotation
class Brick:
    # Class attribute for default brick dimensions
    LENGTH = 20
    WIDTH = 10
    HEIGHT = 5

    def __init__(self, frame=None):
        self.length = Brick.LENGTH
        self.width = Brick.WIDTH
        self.height = Brick.HEIGHT
        self.frame = frame

    def __str__(self):
        return f"Brick of {self.length}cm x {self.width}cm x {self.height}cm (frame: {self.frame})"

    def draw(self):
        box = Box(self.length, self.width, self.height)
        box.frame = self.frame.copy()
        return box

    def rotate(self, rotation_in_degrees):
        self.frame.rotate(math.radians(rotation_in_degrees), (0, 0, 1), self.frame.point)

    def get_pick_frame(self):
        pick_frame = self.frame.copy()
        pick_frame.point.z += self.height / 2

        if pick_frame.zaxis.z > 0:
            pick_frame.xaxis = -pick_frame.xaxis
        return pick_frame
