# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box, Plane,
    Rotation, Translation, Transformation, closest_point_on_line
)

# =============================================================================
# STICK CLASS
# =============================================================================

def _stable_perp(xaxis):
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None, z_vector=None):
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
# BRANCHING MODULE (Mode A)
# =============================================================================

class BranchingModule:
    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset=1.0):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset = offset

    def get_face_frame(self, idx, face, offset):
        parent = self.sticks[idx]
        f = parent.frame.copy()

        # rotate around X to choose face
        R = Rotation.from_axis_and_angle(f.xaxis, face * math.pi / 2, point=f.point)
        f.transform(R)

        # place along axis
        t = max(0, min(1, offset))
        f.point = parent.axis.point_at(t)

        # move to face
        f.point += f.yaxis * (self.depth * 0.5)

        return f

    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=None):
        offset = offset if offset is not None else self.offset
        f = self.get_face_frame(from_stick_index, face_index, offset).copy()

        if angle != 0:
            R = Rotation.from_axis_and_angle(f.yaxis, math.radians(angle), point=f.point)
            f.transform(R)

        axis = Line.from_point_and_vector(f.point, f.xaxis * self.stick_length)
        new_s = Stick(axis, width=self.width, depth=self.depth)
        self.sticks.append(new_s)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# STICK-BRIDGE MODULE (Mode C)
# =============================================================================

class StickBridge:
    """Builds two sticks connecting root_frame → target_frame via plane intersections."""

    def __init__(
        self, root_frame, target_frame,
        offset_root=0.0, offset_target=0.0,
        stick_length=None, width=None, depth=None
    ):

        self.len = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.sticks = []

        self.root = root_frame.copy()
        self.target = target_frame.copy()

        self.offset_root = offset_root
        self.offset_target = offset_target

        # orientation logic
        dev = math.degrees(Vector.angle(self.root.zaxis, self.target.zaxis))
        if dev > 90:
            R = Rotation.from_axis_and_angle(self.target.xaxis, math.pi / 2, point=self.target.point)
            self.target.transform(R)

        # construct child frames
        self.f_root = self.make_child_frame(self.root)
        self.f_target = self.make_child_frame(self.target)

        # plane intersection
        plane0 = Plane.from_frame(self.f_root)
        plane0.normal = self.f_root.yaxis

        plane1 = Plane.from_frame(self.f_target)
        plane1.normal = self.f_target.yaxis

        line = plane0.intersection_with_plane(plane1)
        if line:
            cp_root = Point(*closest_point_on_line(self.f_root.point, line))
        else:
            cp_root = self.f_target.point

        # place and generate sticks
        self.sticks.append(self.build_stick(self.f_root, cp_root, self.offset_root))
        self.sticks.append(self.build_stick(self.f_target, cp_root, self.offset_target))

    def make_child_frame(self, f):
        c = f.copy()
        c.point = Line(f.point, f.point + f.xaxis * self.len).midpoint
        R = Rotation.from_axis_and_angle(c.xaxis, math.pi / 2, point=c.point)
        c.transform(R)
        c.point += c.yaxis * (self.depth * 0.5)
        return c

    def build_stick(self, f, cp, offset):
        direction = Vector.from_start_end(f.point, cp).unitized()
        origin = f.point + direction * offset
        axis = Line(origin, origin + direction * self.len)
        return Stick(axis, width=self.width, depth=self.depth)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# ROOTFRAMES GEOMETRY ENGINE
# =============================================================================

class RootFrames:
    """
    Full pipeline:
        1) GH Surface/Curve → Points
        2) Points → Tangent Frames
        3) Frames → Edge Frames + Vectors
        4) Growth (BranchingModule OR StickBridge)
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

    # ----------------------------------------------------------------------
    # BLOCK 1 — Points
    # ----------------------------------------------------------------------
    def surface_to_points(self):
        """Unified UV sampling for surfaces and twisted extrusion for curves."""
        import Rhino.Geometry as rg

        if self.curve_input and not self.surface_input:
            # CURVE → EXTRUSION → TWISTED LAYERS
            surf = rg.Surface.CreateExtrusion(self.curve_input, rg.Vector3d(0, 0, 1) * 10)
            face = surf.ToBrep().Faces[0]

            pts = []
            for k in range(self.height_subdiv):
                z = 10 * k / self.height_subdiv
                twist = math.radians(self.twist_angle * k)
                layer = []
                for _ in range(self.point_density):
                    u = random.random()
                    v = random.random()
                    p = face.PointAt(u, v)
                    layer.append(rg.Point3d(p.X, p.Y, z))
                # twist
                cx = sum(p.X for p in layer) / len(layer)
                cy = sum(p.Y for p in layer) / len(layer)
                pts += [
                    rg.Point3d(
                        cx + (p.X - cx) * math.cos(twist) - (p.Y - cy) * math.sin(twist),
                        cy + (p.X - cx) * math.sin(twist) + (p.Y - cy) * math.cos(twist),
                        p.Z
                    )
                    for p in layer
                ]
        else:
            # BREPS / SURFACES
            srf = self.surface_input.ToBrep().Faces[0]
            udom = srf.Domain(0)
            vdom = srf.Domain(1)

            pts = []
            N = self.point_density * self.height_subdiv
            for _ in range(N):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = srf.PointAt(u, v)
                pts.append(p)

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # BLOCK 2 — Frames
    # ----------------------------------------------------------------------
    def points_to_frames(self, rot_tan=0, rot_norm=0):
        pts = self.points
        N = len(pts)
        Z = Vector(0, 0, 1)
        frames = []

        for i, p in enumerate(pts):
            p_prev = pts[max(i - 1, 0)]
            p_next = pts[min(i + 1, N - 1)]

            t = Vector(p_next.x - p_prev.x, p_next.y - p_prev.y, 0)
            t = t.unitized() if t.length else Vector(1, 0, 0)

            nrm = Z.cross(t).unitized()
            f = Frame(p, t, nrm)

            # tangent rotation
            if rot_tan != 0:
                R = Rotation.from_axis_and_angle(t, math.radians(rot_tan), point=p)
                f.transform(R)

            # normal rotation
            if rot_norm != 0:
                R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=p)
                f.transform(R)

            frames.append(f)

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # BLOCK 3 — Edge Frames & Vectors
    # ----------------------------------------------------------------------
    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)

        edges = set()
        for i in range(N):
            pi = pts[i]
            j_best, best = None, 1e9
            for j in range(N):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    j_best, best = j, d
            edges.add(tuple(sorted((i, j_best))))

        self.edges = list(edges)

        edge_frames = []
        edge_vectors = []

        for i, j in self.edges:
            f = self.frames[i]
            v = Vector.from_start_end(f.point, self.frames[j].point)
            v = (v - f.zaxis * v.dot(f.zaxis)).unitized()

            edge_vectors.append(v)
            edge_frames.append(Frame(f.point, v, _stable_perp(v)))

        self.edge_vectors = edge_vectors
        self.edge_frames = edge_frames
        return edge_frames, edge_vectors

    # ----------------------------------------------------------------------
    # BLOCK 4 — Growth (Branch vs Bridge)
    # ----------------------------------------------------------------------
    def grow_sticks(self, mode="branch", face_index=0, angle=0, offset01=1.0):
        """
        mode = "branch"    → BranchingModule
        mode = "bridge"    → StickBridge
        """
        sticks_out = []

        # Root sticks
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(axis, length=self.stick_length, width=self.stick_width, depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        # Branching (A)
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(r, stick_length=self.stick_length)
                mod.grow_stick(face_index=face_index, angle=angle, offset=offset01)
                sticks_out.extend(mod.sticks)

        # Bridging (C)
        elif mode == "bridge":
            for (i, j) in self.edges:
                f0 = self.edge_frames[i]
                f1 = self.edge_frames[j]
                bridge = StickBridge(
                    f0, f1,
                    offset_root=offset01 * self.stick_length,
                    offset_target=offset01 * self.stick_length,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth
                )
                sticks_out.extend(bridge.sticks)

        return sticks_out

    # ----------------------------------------------------------------------
    # FULL PIPELINE
    # ----------------------------------------------------------------------
    def run(
        self,
        rot_tan=0, rot_norm=0,
        mode="branch", face_index=0, angle=0, offset01=1.0
    ):
        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()
        sticks = self.grow_sticks(
            mode=mode,
            face_index=face_index,
            angle=angle,
            offset01=offset01
        )
        return sticks
