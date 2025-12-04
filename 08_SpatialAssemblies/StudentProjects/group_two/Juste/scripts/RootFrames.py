# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box, Plane,
    Rotation, Transformation, closest_point_on_line
)

# =============================================================================
# HELPERS
# =============================================================================

def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


# =============================================================================
# STICK CLASS
# =============================================================================

class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None, z_vector=None):
        """
        axis   : compas.geometry.Line (centerline)
        length : box length (local X)
        width  : box width  (local Y)
        depth  : box depth  (local Z)
        """
        self.axis = axis
        self.length = length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self.compute_frame(z_vector=z_vector)

    def compute_frame(self, z_vector=None):
        x = self.axis.direction.unitized()

        if z_vector:
            z = z_vector.unitized()
            y = z.cross(x).unitized()
        else:
            y = _stable_perp(x)
            z = x.cross(y).unitized()

        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# =============================================================================
# BRANCHING MODULE – PURE L-SYSTEM CHAIN
# =============================================================================

class BranchingModule:
    """
    Local L-system:
      - start from root stick
      - each step grows one child from the *last* stick
      - child grows along the chosen face normal (no overlap with parent).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset=1.0):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset = float(offset)

    def _face_frame_and_normal(self, parent_stick, face_index, offset01):
        """
        Compute a frame at the center of a chosen face, plus that face's outward normal.

        face_index: 0,1,2,3 around x-axis
            0 → +Y, 1 → +Z, 2 → -Y, 3 → -Z
        offset01 : 0..1 param along parent axis
        """
        f = parent_stick.frame.copy()
        fi = int(face_index) % 4

        # 1) slide along parent axis
        t = max(0.0, min(1.0, offset01))
        axis_point = parent_stick.axis.point_at(t)
        f.point = axis_point

        # 2) rotate frame around local X to select face
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        # 3) determine face normal and move to face center
        if fi % 2 == 0:
            # Y faces → normal = ±Y, thickness = width
            face_normal = f.yaxis
            f.point += face_normal * (self.width * 0.5)
        else:
            # Z faces → normal = ±Z, thickness = depth
            face_normal = f.zaxis
            f.point += face_normal * (self.depth * 0.5)

        face_normal.unitize()
        return f, face_normal

    def grow_once(self, face_index=0, angle=0.0, offset01=None):
        """
        Grow one child from the last stick in the chain.
        Child axis is aligned with the face normal (no penetration).
        """
        parent = self.sticks[-1]
        off = self.offset if offset01 is None else float(offset01)

        f, face_normal = self._face_frame_and_normal(parent, face_index, off)

        # Optional twist around the *face normal* itself
        if angle:
            R = Rotation.from_axis_and_angle(face_normal, math.radians(angle), point=f.point)
            f.transform(R)

        # Child grows away from parent along the face normal
        axis = Line.from_point_and_vector(f.point, face_normal * self.stick_length)

        child = Stick(axis,
                      length=self.stick_length,
                      width=self.width,
                      depth=self.depth)

        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, angle=0.0, offset01=None):
        """Grow a chain of N children."""
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, angle=angle, offset01=offset01)

    def visualize(self):
        return [s.geometry for s in self.sticks]



# =============================================================================
# GROW-TOWARDS (BRIDGE) – CLEAN FACE CONTACT
# =============================================================================

class GrowTowards:
    """
    Build two child sticks that start on chosen faces of root/target frames
    and grow approximately toward a joint point defined by their face planes.
    """

    def __init__(
        self,
        root_frame,
        target_frame,
        offset_root_child=0.0,
        offset_target_child=0.0,
        stick_length=None,
        width=None,
        depth=None,
        face_index_root=0,
        face_index_target=None
    ):
        self.len = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_frame = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child = float(offset_root_child or 0.0)
        self.offset_target_child = float(offset_target_child or 0.0)

        self.face_index_root = int(face_index_root) % 4
        self.face_index_target = (
            (self.face_index_root + 2) % 4
            if face_index_target is None
            else int(face_index_target) % 4
        )

        self.sticks = []

        # child frames on faces
        self.root_child_frame = self._face_child_frame(
            self.root_frame, self.face_index_root, self.offset_root_child
        )
        self.target_child_frame = self._face_child_frame(
            self.target_frame, self.face_index_target, self.offset_target_child
        )

        # planes & intersection
        plane0 = Plane.from_frame(self.root_child_frame)
        plane0.normal = self.root_child_frame.yaxis

        plane1 = Plane.from_frame(self.target_child_frame)
        plane1.normal = self.target_child_frame.yaxis

        line = plane0.intersection_with_plane(plane1)
        if line:
            joint = Point(*closest_point_on_line(self.root_child_frame.point, line))
        else:
            joint = Point(
                0.5 * (self.root_child_frame.point.x + self.target_child_frame.point.x),
                0.5 * (self.root_child_frame.point.y + self.target_child_frame.point.y),
                0.5 * (self.root_child_frame.point.z + self.target_child_frame.point.z),
            )

        self.joint_point = joint

        # build children
        self.root_child_stick = self._build_stick_to_joint(self.root_child_frame, joint)
        self.target_child_stick = self._build_stick_to_joint(self.target_child_frame, joint)

        self.sticks.extend([self.root_child_stick, self.target_child_stick])

    # ------------------------------------------------------------------  

    def _face_child_frame(self, frame, face_index, offset_dist):
        """
        Create a child frame on a face of the given frame:
        - move along local X by offset_dist
        - rotate around X to select face
        - offset by half-width / half-depth to land on face center
        """
        f = frame.copy()

        # slide along local x from frame origin
        f.point = f.point + f.xaxis * float(offset_dist)

        fi = int(face_index) % 4
        angle = fi * math.pi / 2.0

        R = Rotation.from_axis_and_angle(f.xaxis, angle, point=f.point)
        f.transform(R)

        if fi % 2 == 0:
            f.point += f.yaxis * (self.width * 0.5)
        else:
            f.point += f.zaxis * (self.depth * 0.5)

        return f

    def _build_stick_to_joint(self, child_frame, joint):
        """
        Build a stick that starts at the child face center and aims at the joint.
        The near face stays coincident with the parent face; we don't try to
        force the far face exactly to the joint, but direction is toward it.
        """
        origin = child_frame.point
        direction = Vector.from_start_end(origin, joint)
        if direction.length < 1e-6:
            direction = child_frame.xaxis.copy()
        direction.unitize()

        axis = Line(origin, origin + direction * self.len)

        return Stick(axis, length=self.len, width=self.width, depth=self.depth)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# ROOTFRAMES ENGINE
# =============================================================================

class RootFrames:
    """
    Pipeline:
      1) Surface/Curve → points
      2) Points → frames
      3) Frames → edge frames + edge vectors
      4) Growth: branch (L-system) or bridge (GrowTowards)
    """

    def __init__(
        self,
        surface=None, curve=None,
        height_subdiv=5, point_density=10, twist_angle=0,
        stick_length=None, stick_width=None, stick_depth=None
    ):
        self.surface_input = surface
        self.curve_input = curve

        self.height_subdiv = int(height_subdiv)
        self.point_density = int(point_density)
        self.twist_angle = float(twist_angle)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width = stick_width or Stick.WIDTH
        self.stick_depth = stick_depth or Stick.DEPTH

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []
        self.sticks = []

    # ------------------------------------------------------------------  
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------  

    def surface_to_points(self):
        import Rhino.Geometry as rg
        pts = []

        # Curve mode: extrude and sample
        if self.curve_input is not None and self.surface_input is None:
            surf = rg.Surface.CreateExtrusion(self.curve_input, rg.Vector3d(0, 0, 1) * 10.0)
            if surf is None:
                raise Exception("Failed to extrude curve_input.")
            brep = surf.ToBrep()
            if brep is None or brep.Faces.Count == 0:
                raise Exception("Extruded brep has no faces.")
            face = brep.Faces[0]

            for k in range(max(1, self.height_subdiv)):
                z = 10.0 * k / self.height_subdiv
                twist = math.radians(self.twist_angle * k)

                layer = []
                for _ in range(self.point_density):
                    u, v = random.random(), random.random()
                    p = face.PointAt(u, v)
                    layer.append(rg.Point3d(p.X, p.Y, z))

                cx = sum(p.X for p in layer) / len(layer)
                cy = sum(p.Y for p in layer) / len(layer)

                for p in layer:
                    dx, dy = p.X - cx, p.Y - cy
                    X = cx + dx * math.cos(twist) - dy * math.sin(twist)
                    Y = cy + dx * math.sin(twist) + dy * math.cos(twist)
                    pts.append(rg.Point3d(X, Y, p.Z))

        # Surface / Brep mode
        else:
            if self.surface_input is None:
                raise Exception("No surface_input for sampling.")
            brep = self.surface_input.ToBrep()
            if brep is None or brep.Faces.Count == 0:
                raise Exception("surface_input.ToBrep() has no faces.")
            face = brep.Faces[0]

            udom = face.Domain(0)
            vdom = face.Domain(1)

            count = max(1, self.point_density * max(1, self.height_subdiv))
            for _ in range(count):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------  
    # BLOCK 2 – FRAMES
    # ------------------------------------------------------------------  

    def points_to_frames(self, rot_tan=0, rot_norm=0):
        pts = self.points
        N = len(pts)
        if N == 0:
            self.frames = []
            return []

        frames = []
        Z = Vector(0, 0, 1)

        for i, p in enumerate(pts):
            p_prev = pts[max(0, i - 1)]
            p_next = pts[min(N - 1, i + 1)]

            t = Vector(p_next.x - p_prev.x, p_next.y - p_prev.y, 0.0)
            if t.length:
                t.unitize()
            else:
                t = Vector(1, 0, 0)

            nrm = Z.cross(t)
            if nrm.length:
                nrm.unitize()
            else:
                nrm = Vector(0, 1, 0)

            f = Frame(p, t, nrm)

            if rot_tan:
                R = Rotation.from_axis_and_angle(t, math.radians(rot_tan), point=p)
                f.transform(R)

            if rot_norm:
                R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=p)
                f.transform(R)

            frames.append(f)

        self.frames = frames
        return frames

    # ------------------------------------------------------------------  
    # BLOCK 3 – EDGE FRAMES / VECTORS
    # ------------------------------------------------------------------  

    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)

        if N < 2:
            self.edge_frames = []
            self.edge_vectors = []
            self.edges = []
            return [], []

        edges = set()
        for i in range(N):
            pi = pts[i]
            best = 1e9
            j_best = None
            for j in range(N):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j
            if j_best is not None:
                edges.add(tuple(sorted((i, j_best))))

        edges = [(i, j) for (i, j) in edges if i < N and j < N]
        self.edges = edges

        eframes = []
        evectors = []

        for i, j in edges:
            p0 = self.frames[i].point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue

            f = self.frames[i]
            v_proj = v - f.zaxis * v.dot(f.zaxis)
            if v_proj.length < 1e-6:
                v_proj = f.xaxis.copy()
            v_proj.unitize()

            eframes.append(Frame(p0, v_proj, _stable_perp(v_proj)))
            evectors.append(v_proj)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------------  
    # BLOCK 4 – GROWTH
    # ------------------------------------------------------------------  

    def grow_sticks(
        self,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None
    ):
        """
        mode: 'branch' or 'bridge'
        face_index: which face to grow from (0–3)
        angle: yaw angle in degrees (branch mode)
        offset01: [0,1] param along axis for branching,
                  scaled length for bridging
        steps: number of L-system steps (branching only)
        bridge_index: if not None, only edges touching this frame index are bridged
        """
        mode = str(mode).strip().lower()
        sticks_out = []

        if not self.edge_frames:
            return sticks_out

        # create root sticks for each edge frame
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(axis,
                         length=self.stick_length,
                         width=self.stick_width,
                         depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        # ---------------- BRANCH MODE ----------------
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(
                    r,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth,
                    offset=offset01
                )
                mod.grow_chain(
                    steps=steps,
                    face_index=face_index,
                    angle=angle,
                    offset01=offset01
                )
                sticks_out.extend(mod.sticks[1:])  # exclude root duplicate

            self.sticks = sticks_out
            return sticks_out

        # ---------------- BRIDGE MODE ----------------
        if mode == "bridge":
            offset_abs = offset01 * self.stick_length

            for (i, j) in self.edges:
                if bridge_index is not None:
                    bi = int(bridge_index)
                    if i != bi and j != bi:
                        continue

                f0 = self.frames[i]
                f1 = self.frames[j]

                try:
                    grow = GrowTowards(
                        root_frame=f0,
                        target_frame=f1,
                        offset_root_child=offset_abs,
                        offset_target_child=offset_abs,
                        stick_length=self.stick_length,
                        width=self.stick_width,
                        depth=self.stick_depth,
                        face_index_root=face_index
                    )
                    sticks_out.extend(grow.sticks)
                except Exception as e:
                    print("GrowTowards failed on edge ({}, {}): {}".format(i, j, e))
                    continue

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode: {}".format(mode))

    # ------------------------------------------------------------------  
    # RUN
    # ------------------------------------------------------------------  

    def run(
        self,
        rot_tan=0,
        rot_norm=0,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None
    ):
        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        return self.grow_sticks(
            mode=mode,
            face_index=face_index,
            angle=angle,
            offset01=offset01,
            steps=steps,
            bridge_index=bridge_index
        )
