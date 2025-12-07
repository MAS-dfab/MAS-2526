# stick_bridge_v2.py (or inside RootFrames if you prefer)
import math
from compas.geometry import Line, Point, Vector, Plane, Rotation, closest_point_on_line
from RootFrames import Stick  # or from RootFrames import Stick

class GrowTowards:
    def __init__(self, root_frame, target_frame,
                 offset_root_child=0.0, offset_target_child=0.0,
                 stick_length=None, width=None, depth=None):
        """
        root_frame      : Frame at end of the root stick (from your RootModule / RootFrames)
        target_frame    : Frame you want to grow towards
        offset_root_child   : distance along root-child axis from root child → intersection
        offset_target_child : same for target child
        """

        self.len   = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_frame   = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child   = offset_root_child
        self.offset_target_child = offset_target_child

        self.sticks = []

        # 1) make child frames on faces of root/target sticks
        self.root_child_frame   = self._make_child_frame(self.root_frame,  flip=False)
        self.target_child_frame = self._make_child_frame(self.target_frame, flip=True)

        # 2) compute intersection line between planes normal to child_frame.yaxis
        plane0 = Plane.from_frame(self.root_child_frame)
        plane0.normal = self.root_child_frame.yaxis

        plane1 = Plane.from_frame(self.target_child_frame)
        plane1.normal = self.target_child_frame.yaxis

        line = plane0.intersection_with_plane(plane1)

        if line:
            self.intersection_line = line
            self.intersection_point = Point(*closest_point_on_line(self.root_child_frame.point, line))
        else:
            # fallback: midpoint
            self.intersection_line = None
            self.intersection_point = Point(
                0.5 * (self.root_child_frame.point.x + self.target_child_frame.point.x),
                0.5 * (self.root_child_frame.point.y + self.target_child_frame.point.y),
                0.5 * (self.root_child_frame.point.z + self.target_child_frame.point.z),
            )

        # 3) create root child stick and target child stick with half-depth offsets at the intersection
        self.root_child_stick   = self._build_child_stick(self.root_child_frame,
                                                          self.intersection_point,
                                                          self.offset_root_child)
        self.target_child_stick = self._build_child_stick(self.target_child_frame,
                                                          self.intersection_point,
                                                          self.offset_target_child)

        self.sticks.extend([self.root_child_stick, self.target_child_stick])

    # ------------------------------------------------------------------
    # step 1: child frames (root & target)
    # ------------------------------------------------------------------
    def _make_child_frame(self, frame, flip=False):
        """
        Place a child frame halfway along the local x-axis, then rotate to a side face
        and offset outward by half the stick depth so its Y+ face is on the stick surface.
        """
        f = frame.copy()

        # center of the parent stick axis
        mid = Line.from_point_and_vector(f.point, f.xaxis * self.len).midpoint
        f.point = mid

        # choose which side face
        face_index = 1 if not flip else 3   # your existing convention: 1 / 3 around x-axis
        angle = face_index * math.pi / 2.0

        R_face = Rotation.from_axis_and_angle(f.xaxis, angle, point=f.point)
        f.transform(R_face)

        # move out to stick face (half depth → face-to-face contact plane)
        f.point += f.yaxis * (self.depth * 0.5)

        return f

    # ------------------------------------------------------------------
    # step 2+3: build child sticks towards intersection
    # ------------------------------------------------------------------
    def _build_child_stick(self, child_frame, intersection, offset):
        """
        Build a child stick so that its *near face* meets the intersection point,
        and the stick extends away from the parent (no penetration).
        """

        # Direction from child frame to intersection (fall back to local xaxis)
        direction = Vector.from_start_end(child_frame.point, intersection)
        if direction.length < 1e-6:
            direction = child_frame.xaxis.copy()
        direction.unitize()

        axis_dir = direction

        # Attach point: intersection is the center of the near face.
        attach = intersection.copy()

        # Optional offset along the stick axis (to push it further out)
        if offset:
            attach += axis_dir * offset

        # Axis starts at the attachment (near face) and goes outward
        axis_start = attach
        axis_end   = attach + axis_dir * self.len

        axis = Line(axis_start, axis_end)
        return Stick(axis, length=self.len, width=self.width, depth=self.depth)


    # ------------------------------------------------------------------
    def visualize(self):
        return [s.geometry for s in self.sticks]
