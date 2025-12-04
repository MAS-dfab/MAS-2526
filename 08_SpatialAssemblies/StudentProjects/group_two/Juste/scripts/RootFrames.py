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
# BRANCHING MODULE – PURE L-SYSTEM CHAIN (FACE CONTACT, NO COLLISION)
# =============================================================================

class BranchingModule:
    """
    Branching system:
    - Each generation grows from the last stick.
    - Child grows away from the parent, starting at a chosen face.
    - Full-thickness offset from parent axis → true face contact without overlap.

    Designer controls:
        face_index   : 0..3 around parent x-axis
        stick_angle  : tilt in the plane spanned by (face normal, parent x-axis)
        offset01     : 0..1 param along parent axis
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)  # 0–1 along parent axis

    # ----------------------------------------------------------------------
    # Compute parent face geometry + face normal with FULL thickness offset
    # ----------------------------------------------------------------------
    def _compute_face_frame(self, parent, face_index):
        """
        Returns:
            base_frame : Frame whose origin is placed at the *outside* of parent,
                         on the chosen face.
            normal     : outward face normal (unit vector).
        """
        f = parent.frame.copy()
        fi = int(face_index) % 4

        # 1) slide along parent axis to offset position
        t = max(0.0, min(1.0, self.offset01))
        f.point = parent.axis.point_at(t)

        # 2) rotate frame around X to select which face
        #    face 0 → +Y, 1 → +Z, 2 → -Y, 3 → -Z
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        # 3) determine face normal and move to full-thickness offset
        if fi in (0, 2):  # ±Y faces → offset by FULL width
            normal = f.yaxis
            offset = self.width
        else:             # ±Z faces → offset by FULL depth
            normal = f.zaxis
            offset = self.depth

        normal.unitize()
        # Move origin fully outside parent bounding box
        f.point += normal * offset

        return f, normal

    # ----------------------------------------------------------------------
    # Grow one child from last stick
    # ----------------------------------------------------------------------
    def grow_once(self, face_index=0, stick_angle=0.0):
        """
        Grow one child from the last stick in the chain.

        - Child origin lies on the parent face plane, but shifted by FULL thickness.
        - Direction is a blend between face normal and parent.xaxis, controlled
          by stick_angle (in degrees).
        """
        parent = self.sticks[-1]

        # Get face frame + outward normal with full-thickness offset
        base_frame, normal = self._compute_face_frame(parent, face_index)

        # Base normal direction
        n = normal
        # Tangential direction taken from the rotated frame's x-axis
        side = base_frame.xaxis

        theta = math.radians(stick_angle)
        # Blend normal and side to get final direction
        d = n * math.cos(theta) + side * math.sin(theta)
        if d.length < 1e-6:
            d = n
        d.unitize()

        axis = Line(base_frame.point, base_frame.point + d * self.stick_length)

        child = Stick(axis,
                      length=self.stick_length,
                      width=self.width,
                      depth=self.depth)

        self.sticks.append(child)

    # ----------------------------------------------------------------------
    # N-step growth (L-system style)
    # ----------------------------------------------------------------------
    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# GROW-TOWARDS (BRIDGE) – FACE-BASED CHILDREN
# =============================================================================

class GrowTowards:
    """
    Build two child sticks that start on chosen faces of root/target frames
    and grow approximately toward a joint point defined by their face planes.

    Uses the same full-thickness face offset logic as BranchingModule.
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
        # By default, use opposite face on the target
        self.face_index_target = (
            (self.face_index_root + 2) % 4
            if face_index_target is None
            else int(face_index_target) % 4
        )

        self.sticks = []

        # child frames on faces (full-thickness offset)
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

        - Move along local X by offset_dist.
        - Rotate around X to select face.
        - Offset by FULL width / depth so axis is outside the parent.
        """
        f = frame.copy()

        # slide along local x from frame origin
        f.point = f.point + f.xaxis * float(offset_dist)

        fi = int(face_index) % 4
        angle = fi * math.pi / 2.0

        R = Rotation.from_axis_and_angle(f.xaxis, angle, point=f.point)
        f.transform(R)

        if fi in (0, 2):  # ±Y faces → full width
            f.point += f.yaxis * self.width
        else:             # ±Z faces → full depth
            f.point += f.zaxis * self.depth

        return f

    def _build_stick_to_joint(self, child_frame, joint):
        """
        Build a stick that starts at the child face origin and aims at the joint.
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
        import Rhino.Geometry as rg

        if not self.surface_input:
            raise Exception("points_to_frames requires a surface input")

        brep = self.surface_input.ToBrep()
        face = brep.Faces[0]

        frames = []

        for p in self.points:

            # Convert COMPAS point → Rhino point
            rp = rg.Point3d(p.x, p.y, p.z)

            # Get UV closest point on surface
            ok, u, v = face.ClosestPoint(rp)
            if not ok:
                continue

            # Evaluate true 3D surface frame:
            # - tangent_u
            # - tangent_v
            # - normal
            _, du, dv = face.Evaluate(u, v, 1)

            du = Vector(du.X, du.Y, du.Z)
            dv = Vector(dv.X, dv.Y, dv.Z)
            nrm = du.cross(dv)

            if not nrm.length:
                nrm = Vector(0,0,1)
            nrm.unitize()

            # Build orthonormal frame:
            xaxis = du.unitized()
            yaxis = nrm.cross(xaxis).unitized()
            zaxis = nrm.unitized()

            f = Frame(p, xaxis, yaxis)

            # designer-controlled rotations
            if rot_tan:
                R = Rotation.from_axis_and_angle(xaxis, math.radians(rot_tan), point=p)
                f.transform(R)

            if rot_norm:
                R = Rotation.from_axis_and_angle(zaxis, math.radians(rot_norm), point=p)
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
        angle: stick_angle in degrees (branch mode)
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
                    offset01=offset01
                )
                mod.grow_chain(
                    steps=steps,
                    face_index=face_index,
                    stick_angle=angle
                )
                # skip the first (root) because we already added it
                sticks_out.extend(mod.sticks[1:])

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
