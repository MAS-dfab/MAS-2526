# rf_core.py
# Clean RootFrames engine for Grasshopper integration.
#
# Responsibilities:
#   1. Sample points on curve or surface
#   2. Build COMPAS Frames from Rhino local frames
#   3. Build nearest-neighbour edge frames
#   4. Grow branching L-system (with optional collision-safe mode)
#   5. Grow bridging sticks (optional)
#   6. Detect collisions (optional)
#   7. Provide debug channels + color-coded sticks

import random
import Rhino.Geometry as rg  # type: ignore
import System.Drawing as sd  # type: ignore # for GH colors

from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


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
        # Geometry inputs
        self.surface_input = surface
        self.curve_input = curve

        # Sampling
        self.point_density = int(point_density)

        # Stick geometry
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # Storage
        self.points = []          # sampled COMPAS Points
        self.frames = []          # root sample Frames
        self.edge_frames = []     # Frames along nearest-neighbour edges
        self.edge_vectors = []    # edge directions
        self.edges = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []
        self.colors = []          # parallel to (branch + bridge) sticks

        # Debug geometry (packed into existing GH debug outputs)
        self.dbg_root_frames = []   # Frames at samples (same as self.frames)
        self.dbg_root_axes = []     # Lines along edge vectors (roots)
        self.dbg_branch_geos = []   # Lines for branch sticks
        self.dbg_bridge_geos = []   # Lines for bridge sticks

        # Internals for sampling
        self._rg_curve = None
        self._curve_t = []
        self._rg_face = None
        self._uv_params = []

    # ----------------------------------------------------------------------
    # 1. POINT SAMPLING
    # ----------------------------------------------------------------------

    def sample_points(self):
        pts = []
        self._curve_t = []
        self._uv_params = []
        self._rg_curve = None
        self._rg_face = None

        # ------------------------
        # CURVE MODE
        # ------------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            # FIXED: Domain is a method in RhinoCode
            dom = crv.Domain()
            t0, t1 = dom.Min, dom.Max

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # ------------------------
        # SURFACE MODE
        # ------------------------
        else:
            surf = self.surface_input
            brep = surf.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.Min, udom.Max)
                v = random.uniform(vdom.Min, vdom.Max)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            pts.sort(key=lambda p: p.Z)

        # convert to COMPAS Points
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points


    # ----------------------------------------------------------------------
    # 2. FRAME GENERATION (root frames at sample points)
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        frames = []

        # CURVE MODE
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    y = Vector(0, 0, 1).cross(x)

                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)

                x.unitize()
                y.unitize()

                frames.append(Frame(pt, x, y))

        # SURFACE MODE
        elif self._rg_face and self._uv_params:
            face = self._rg_face

            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)

                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)

                x.unitize()
                y.unitize()

                frames.append(Frame(pt, x, y))

        # FALLBACK
        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        self.dbg_root_frames = frames[:]  # debug alias
        return frames

    # ----------------------------------------------------------------------
    # 3. NEAREST NEIGHBOUR EDGE-FRAMES
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            self.dbg_root_axes = []
            return [], []

        edges = set()

        # NN search
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

        eframes = []
        evectors = []
        dbg_axes = []

        for i, j in edges:
            f0 = self.frames[i]
            p0 = f0.point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue

            v.unitize()

            # Use surface normal to keep frames 3D
            surf_normal = f0.zaxis
            y = surf_normal.cross(v)
            if y.length < 1e-6:
                y = surf_normal.cross(Vector(1, 0, 0))
            y.unitize()

            eframes.append(Frame(p0, v, y))
            evectors.append(v)

            # Debug axis line
            dbg_axes.append(Line(p0, p0 + v * self.stick_length))

        self.edge_frames = eframes
        self.edge_vectors = evectors
        self.dbg_root_axes = dbg_axes

        return eframes, evectors

    # ----------------------------------------------------------------------
    # Utility: parse rule strings
    # ----------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        vals = []
        for tok in rule_str.split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except:
                pass
        return vals

    # ----------------------------------------------------------------------
    # 4. BRANCHING
    # ----------------------------------------------------------------------

    def grow_branching(self,
                       steps,
                       stick_angle,
                       offset01,
                       face_rule=None,
                       angle_rule=None,
                       collision_safe=True):

        self.root_sticks = []
        self.branch_sticks = []
        self.dbg_branch_geos = []

        # Build root sticks from edge-frames
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)

            s = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f
            )
            s.family = None  # explicit
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # L-system
        for root in self.root_sticks:

            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
                collision_safe=collision_safe,
            )

            for k in range(int(steps)):

                if face_seq:
                    fi = int(face_seq[k % len(face_seq)])
                else:
                    fi = 0

                if angle_seq:
                    ang = float(angle_seq[k % len(angle_seq)])
                else:
                    ang = stick_angle

                child = B.grow_once(face_index=fi, stick_angle=ang)

                # If branch died (collision or invalid face), stop expanding
                if child is None:
                    break

            # Skip duplicate root
            new_sticks = B.sticks[1:]
            self.branch_sticks.extend(new_sticks)

        # Debug branch axes
        self.dbg_branch_geos = [stick.axis for stick in self.branch_sticks]
        return self.branch_sticks

    # ----------------------------------------------------------------------
    # 5. BRIDGING
    # ----------------------------------------------------------------------

    def grow_bridging(self):
        if not self.branch_sticks:
            self.bridge_sticks = []
            self.dbg_bridge_geos = []
            return []

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
        )

        self.bridge_sticks = BM.build()
        self.dbg_bridge_geos = [stick.axis for stick in self.bridge_sticks]
        return self.bridge_sticks

    # ----------------------------------------------------------------------
    # 6. COLLISION DETECTION
    # ----------------------------------------------------------------------

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

    # ----------------------------------------------------------------------
    # 7. COLOR ASSIGNMENT
    # ----------------------------------------------------------------------

    def assign_colors(self):
        """
        Build color list parallel to (branch_sticks + bridge_sticks).

        Color scheme:
            Collision-flagged : Red
            Root sticks       : Yellow
            Y-family          : Light Pink
            Z-family          : Light Green
            Bridges           : Beige
        """
        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)
        flags = self.collision_flags or [False] * n

        colors = []

        for i, s in enumerate(sticks):
            if i < len(flags) and flags[i]:
                c = sd.Color.Red
            elif s in self.root_sticks:
                c = sd.Color.Yellow
            else:
                fam = getattr(s, "family", None)
                if fam == "Y":
                    # light pink
                    c = sd.Color.FromArgb(255, 255, 182, 193)
                elif fam == "Z":
                    # light green
                    c = sd.Color.FromArgb(255, 144, 238, 144)
                elif fam == "BRIDGE":
                    # beige
                    c = sd.Color.FromArgb(255, 245, 245, 220)
                else:
                    # default non-root
                    c = sd.Color.Yellow

            colors.append(c)

        self.colors = colors
        return colors

    # ----------------------------------------------------------------------
    # 8. RUN PIPELINE
    # ----------------------------------------------------------------------

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        do_bridging=False,
        face_rule=None,
        angle_rule=None,
        collision_safe=True,
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
            collision_safe=collision_safe,
        )

        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []
            self.dbg_bridge_geos = []

        sticks_all = self.branch_sticks + self.bridge_sticks

        if detect_collisions:
            self.detect_collisions(clearance=0.0)
        else:
            self.collision_flags = [False] * len(sticks_all)

        self.assign_colors()

        return sticks_all
