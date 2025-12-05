# rf_core.py
# Clean 3D RootFrames core using COMPAS only.

import random
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def rhino_plane_to_frame(plane):
    """Convert Rhino Plane to a robust COMPAS Frame."""
    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)

    if x.length < 1e-9:
        x = Vector(1, 0, 0)
    if y.length < 1e-9:
        y = Vector(0, 1, 0)

    x.unitize()
    y.unitize()

    # re-orthogonalise
    z = x.cross(y)
    if z.length < 1e-9:
        y = Vector(0, 0, 1).cross(x)
    y.unitize()

    origin = Point(plane.OriginX, plane.OriginY, plane.OriginZ)
    return Frame(origin, x, y)


def get_surface_domain(face, idx):
    """Robust wrapper around face.Domain(idx)."""
    dom = face.Domain(idx)  # this is where MethodBinding lived
    if hasattr(dom, "T0") and hasattr(dom, "T1"):
        return dom
    raise RuntimeError("Unexpected domain type: {}".format(type(dom)))


# ----------------------------------------------------------------------
# RootFrames
# ----------------------------------------------------------------------

class RootFrames:
    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=10,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        # geometry inputs
        self.surface_input = surface
        self.curve_input = curve

        # sampling density
        self.point_density = int(point_density)

        # stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # core storage
        self.points = []
        self.frames = []
        self.edges = []          # [(i, j)]
        self.edge_vectors = []   # [Vector]

        # result groups
        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # internals
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ------------------------------------------------------------------
    # 1. Sample points
    # ------------------------------------------------------------------

    def sample_points(self):
        pts = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # CURVE MODE
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

        # SURFACE MODE
        else:
            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise RuntimeError("RootFrames.sample_points: brep has no faces.")

            face = brep.Faces[0]
            self._rg_face = face

            udom = get_surface_domain(face, 0)
            vdom = get_surface_domain(face, 1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------
    # 2. Frames from geometry
    # ------------------------------------------------------------------

    def frames_from_geometry(self):
        frames = []

        # CURVE
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if ok:
                    frames.append(rhino_plane_to_frame(plane))
                else:
                    # tangent fallback
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    if x.length < 1e-9:
                        x = Vector(1, 0, 0)
                    x.unitize()
                    y = Vector(0, 0, 1).cross(x)
                    if y.length < 1e-9:
                        y = Vector(0, 1, 0)
                    y.unitize()
                    frames.append(Frame(pt, x, y))

        # SURFACE
        elif self._rg_face and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)
                if ok:
                    frames.append(rhino_plane_to_frame(plane))
                else:
                    frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        else:
            # fallback world XY
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ------------------------------------------------------------------
    # 3. Nearest-neighbour edges
    # ------------------------------------------------------------------

    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_vectors = []
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
            if j_best is not None:
                edges.add(tuple(sorted((i, j_best))))

        self.edges = list(edges)

        evectors = []
        for i, j in self.edges:
            p0 = self.frames[i].point
            p1 = self.frames[j].point
            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-9:
                continue
            v.unitize()
            evectors.append(v)

        self.edge_vectors = evectors
        return self.edges, evectors

    # ------------------------------------------------------------------
    # Utility for L-rules
    # ------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        if isinstance(rule_str, (int, float)):
            return [rule_str]
        vals = []
        for tok in str(rule_str).split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                if "." in tok:
                    vals.append(float(tok))
                else:
                    vals.append(int(tok))
            except Exception:
                continue
        return vals

    # ------------------------------------------------------------------
    # 4. Branching
    # ------------------------------------------------------------------

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # build root sticks on each edge, using the frame at the first vertex
        for k, (i, j) in enumerate(self.edges):
            fi = self.frames[i]
            p0 = fi.point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-9:
                continue
            v.unitize()

            axis = Line(p0, p0 + v * self.stick_length)
            s = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=fi
            )
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

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
                fi = int(face_seq[k % len(face_seq)]) if face_seq else 0
                ang = float(angle_seq[k % len(angle_seq)]) if angle_seq else float(stick_angle)
                B.grow_once(face_index=fi, stick_angle=ang)

            # skip root duplicate
            self.branch_sticks.extend(B.sticks[1:])

        return self.branch_sticks

    # ------------------------------------------------------------------
    # 5. Bridging
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # 6. Collision detection
    # ------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)
        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):
                if sticks[i].intersects(sticks[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------------
    # 7. Run
    # ------------------------------------------------------------------

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
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * (len(self.branch_sticks) + len(self.bridge_sticks))

        return self.branch_sticks + self.bridge_sticks
