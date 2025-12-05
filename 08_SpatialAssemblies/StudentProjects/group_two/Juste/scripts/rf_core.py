# rf_core.py
# RootFrames Core – COMPAS-based branching + optional bridging
# Fully compatible with GH RhinoCode environment

import random
import Rhino.Geometry as rg  # type: ignore

from compas.geometry import Point, Vector, Line, Frame
from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


# ======================================================================
# Helper: fix GH MethodBinding domain issue
# ======================================================================
def _ensure_interval(method_or_interval, face, idx):
    """
    Grasshopper sometimes wraps BrepFace.Domain as a MethodBinding.
    This function forces proper Interval extraction.
    """
    # already a normal Interval
    if hasattr(method_or_interval, "T0") and hasattr(method_or_interval, "T1"):
        return method_or_interval

    # try calling the domain explicitly
    try:
        dom = face.Domain(idx)  # force evaluation
        if hasattr(dom, "T0") and hasattr(dom, "T1"):
            return dom
    except:
        pass

    raise RuntimeError("Unable to extract UV domain (GH MethodBinding issue).")


# ======================================================================
# RootFrames Engine
# ======================================================================
class RootFrames:
    """
    RootFrames engine:

      1) Sample points on a curve or a surface
      2) Build 3D frames (no flattening)
      3) Build nearest-neighbour edges
      4) Branching (L-system growth)
      5) Optional bridging
      6) Optional collisions

    GH outputs geometry, frames, sticks, debug channels.
    """

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=10,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        # --------------------------------------------------------------
        # Inputs
        # --------------------------------------------------------------
        self.surface_input = surface
        self.curve_input = curve
        self.point_density = int(point_density)

        # Stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # --------------------------------------------------------------
        # Storage
        # --------------------------------------------------------------
        self.points = []
        self.frames = []
        self.edges = []
        self.edge_frames = []
        self.edge_vectors = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # Internal sampling data
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ==================================================================
    # 1 — POINT SAMPLING
    # ==================================================================
    def sample_points(self):
        pts = []
        self._uv_params = []
        self._curve_t = []

        # ----------------------------------------------------------
        # CURVE MODE
        # ----------------------------------------------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        else:
            # ------------------------------------------------------
            # SURFACE MODE
            # ------------------------------------------------------
            brep = self.surface_input.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face

            # Fix GH MethodBinding bug
            udom = _ensure_interval(face.Domain(0), face, 0)
            vdom = _ensure_interval(face.Domain(1), face, 1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            pts.sort(key=lambda p: p.Z)

        # Convert to COMPAS points
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ==================================================================
    # 2 — BUILD FRAMES
    # ==================================================================
    def frames_from_geometry(self):
        frames = []

        # ----------------------------------------------------------
        # CURVE MODE
        # ----------------------------------------------------------
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(*plane.XAxis)
                    y = Vector(*plane.YAxis)
                    x.unitize()
                    y.unitize()
                else:
                    tangent = crv.TangentAt(t)
                    x = Vector(tangent.X, tangent.Y, tangent.Z)
                    x.unitize()
                    y = Vector(0, 0, 1).cross(x)
                    y.unitize()

                frames.append(Frame(pt, x, y))

        # ----------------------------------------------------------
        # SURFACE MODE
        # ----------------------------------------------------------
        elif self._rg_face and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)
                if ok:
                    x = Vector(*plane.XAxis)
                    y = Vector(*plane.YAxis)
                    x.unitize()
                    y.unitize()
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)
                frames.append(Frame(pt, x, y))

        # ----------------------------------------------------------
        # FALLBACK
        # ----------------------------------------------------------
        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ==================================================================
    # 3 — NEAREST NEIGHBOUR EDGES
    # ==================================================================
    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            return [], []

        edges = set()

        for i in range(n):
            pi = pts[i]
            best = 1e9
            j_best = None
            for j in range(n):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j
            edges.add(tuple(sorted((i, j_best))))

        edges = list(edges)
        self.edges = edges

        edge_frames = []
        edge_vectors = []

        for i, j in edges:
            p0 = self.frames[i].point
            p1 = self.frames[j].point
            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue
            v.unitize()

            z = Vector(0, 0, 1)
            y = z.cross(v)
            if y.length < 1e-6:
                y = Vector(0, 1, 0)
            y.unitize()

            edge_frames.append(Frame(p0, v, y))
            edge_vectors.append(v)

        self.edge_frames = edge_frames
        self.edge_vectors = edge_vectors

        return edge_frames, edge_vectors

    # ==================================================================
    # 4 — RULE PARSER
    # ==================================================================
    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        vals = []
        for t in rule_str.split(","):
            t = t.strip()
            if not t:
                continue
            try:
                vals.append(float(t) if "." in t else int(t))
            except:
                pass
        return vals

    # ==================================================================
    # 5 — BRANCHING
    # ==================================================================
    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # Build initial sticks from edge frames
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            s = Stick(axis,
                      length=self.stick_length,
                      width=self.stick_width,
                      depth=self.stick_depth,
                      parent_frame=f)
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        # L-system rule arrays
        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            for k in range(int(steps)):
                fi = face_seq[k % len(face_seq)] if face_seq else 0
                ang = angle_seq[k % len(angle_seq)] if angle_seq else stick_angle
                B.grow_once(face_index=int(fi), stick_angle=float(ang))

            self.branch_sticks.extend(B.sticks[1:])  # skip original root

        return self.branch_sticks

    # ==================================================================
    # 6 — BRIDGING
    # ==================================================================
    def grow_bridging(self):
        if not self.branch_sticks:
            self.bridge_sticks = []
            return []

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
        )
        self.bridge_sticks = BM.build()
        return self.bridge_sticks

    # ==================================================================
    # 7 — COLLISIONS
    # ==================================================================
    def detect_collisions(self, clearance=0.0):
        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)
        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):
                if sticks[i].intersects(sticks[j], clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ==================================================================
    # 8 — FULL PIPELINE
    # ==================================================================
    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        do_bridging=False,
        clearance=0.0,
        face_rule=None,
        angle_rule=None,
        verbose=False,
    ):
        self.sample_points()
        self.frames_from_geometry()
        self.frames_to_edges()

        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
        )

        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []

        if detect_collisions:
            self.detect_collisions(clearance)
        else:
            self.collision_flags = [False] * (
                len(self.branch_sticks) + len(self.bridge_sticks)
            )

        return self.branch_sticks + self.bridge_sticks