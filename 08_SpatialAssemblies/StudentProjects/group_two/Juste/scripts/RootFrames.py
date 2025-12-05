# rootframes.py
# r: compas>=2.14.1

import random
import Rhino.Geometry as rg
from compas.geometry import Point, Vector, Line, Frame

from stick import Stick
from branch import BranchingModule
from bridge import BridgingModule


class RootFrames:
    """
    RootFrames engine:
      1) Sample points on a curve or surface
      2) Build 3D frames using Rhino's native frames (no flattening)
      3) Build nearest-neighbour edges & edge-frames
      4) Branching phase (L-system style growth rules)
      5) Optional bridging phase (only between non-coplanar sticks)
      6) Optional collision detection via Stick AABBs

    Debug channels:
      - self.root_sticks
      - self.branch_sticks
      - self.bridge_sticks
      - self.collision_flags
      - self.frames
      - self.edge_frames
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
        # input geometry
        self.surface_input = surface    # Rhino Brep/Surface
        self.curve_input   = curve      # Rhino Curve

        # sampling
        self.point_density = int(point_density)

        # stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width  = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth  = stick_depth or Stick.DEFAULT_SIZE

        # core data
        self.points       = []   # [compas Point]
        self.frames       = []   # [compas Frame]
        self.edge_frames  = []   # [compas Frame]
        self.edge_vectors = []   # [compas Vector]
        self.edges        = []   # [(i, j)]

        # debug / result groups
        self.root_sticks   = []  # initial sticks on edges
        self.branch_sticks = []  # root + branch
        self.bridge_sticks = []  # bridging sticks
        self.collision_flags = []  # parallel to (branch+bridge)

        # internal for 3D frame reconstruction
        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

    # ----------------------------------------------------------------------
    # 1. SAMPLING
    # ----------------------------------------------------------------------

    def sample_points(self):
        """Sample points on a curve or surface and store parameter data for 3D frames."""
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

        # SURFACE/BREP MODE
        else:
            if self.surface_input is None:
                raise Exception("RootFrames.sample_points: no surface_input or curve_input.")

            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("RootFrames.sample_points: Brep has no faces.")

            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            # keep sorted by Z for some visual consistency
            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. 3D FRAMES (NO FLATTENING)
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        """Use Rhino's 3D frames on curve/surface to build compas Frames."""
        frames = []

        # CURVE MODE
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    # fallback: tangent-based frame
                    tangent = crv.TangentAt(t)
                    x = Vector(tangent.X, tangent.Y, tangent.Z)
                    if x.length < 1e-6:
                        x = Vector(1, 0, 0)
                    x.unitize()
                    y = Vector(0, 0, 1).cross(x)
                    if y.length < 1e-6:
                        y = Vector(0, 1, 0)
                    y.unitize()
                else:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    if x.length < 1e-6:
                        x = Vector(1, 0, 0)
                    else:
                        x.unitize()
                    if y.length < 1e-6:
                        y = Vector(0, 1, 0)
                    else:
                        y.unitize()

                frames.append(Frame(pt, x, y))

        # SURFACE MODE
        elif self._rg_face is not None and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)
                if not ok:
                    frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))
                    continue

                x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)

                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                else:
                    x.unitize()
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)
                else:
                    y.unitize()

                frames.append(Frame(pt, x, y))

        else:
            # last-resort fallback: world XY
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. EDGES & EDGE FRAMES
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        """Build nearest-neighbour edges and associated edge frames/vectors."""
        pts = [f.point for f in self.frames]
        n = len(pts)
        if n < 2:
            self.edges = []
            self.edge_frames = []
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

        eframes = []
        evectors = []
        for i, j in self.edges:
            f0 = self.frames[i]
            p0 = f0.point
            p1 = self.frames[j].point
            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue
            v.unitize()
            # simple edge frame: x along v, y as any stable perp
            z = Vector(0, 0, 1)
            y = z.cross(v)
            if y.length < 1e-6:
                y = Vector(0, 1, 0)
            y.unitize()
            eframes.append(Frame(p0, v, y))
            evectors.append(v)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ----------------------------------------------------------------------
    # 4. BRANCHING (WITH L-STYLE RULES)
    # ----------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        """
        Convert a comma-separated string to a list of ints/floats.
        e.g. "0,1,2,3" or "0, 30, -30".
        Returns [] on empty/invalid.
        """
        if not rule_str:
            return []
        if isinstance(rule_str, (int, float)):
            return [rule_str]
        if not isinstance(rule_str, str):
            return []
        tokens = [t.strip() for t in rule_str.split(",") if t.strip() != ""]
        vals = []
        for t in tokens:
            try:
                if "." in t:
                    vals.append(float(t))
                else:
                    vals.append(int(t))
            except Exception:
                continue
        return vals

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):
        """
        Build root sticks on edges, then grow L-style branching chains.

        face_rule  : string like "0,1,2,3" or None
        angle_rule : string like "0,30,-30" or None

        At generation k:
          face_index = face_rule[k % len(face_rule)] if provided
          angle      = angle_rule[k % len(angle_rule)] if provided
        """
        self.root_sticks = []
        self.branch_sticks = []

        # build root sticks
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            s = Stick(axis, self.stick_length, self.stick_width, self.stick_depth)
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        # parse rules
        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # branch from each root with L-style rules
        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            for k in range(int(steps)):
                # face index for this generation
                if face_seq:
                    fi = int(face_seq[k % len(face_seq)]) % 4
                else:
                    fi = 0  # default to +Y face

                # angle for this generation
                if angle_seq:
                    ang = float(angle_seq[k % len(angle_seq)])
                else:
                    ang = float(stick_angle)

                B.grow_once(face_index=fi, stick_angle=ang)

            # skip root duplicate
            self.branch_sticks.extend(B.sticks[1:])

        return self.branch_sticks

    # ----------------------------------------------------------------------
    # 5. BRIDGING (AUTOMATIC, NON-COPLANAR ONLY)
    # ----------------------------------------------------------------------

    def grow_bridging(self):
        """
        Use BridgingModule to connect non-coplanar sticks.
        (The logic for selecting which pairs are bridged lives in bridge.py)
        """
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

    # ----------------------------------------------------------------------
    # 6. COLLISION DETECTION (AABB-BASED)
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        """
        Approximate collisions via Stick AABBs.
        Flags are parallel to branch_sticks + bridge_sticks.
        """
        all_sticks = self.branch_sticks + self.bridge_sticks
        n = len(all_sticks)
        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):
                if all_sticks[i].intersects(all_sticks[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ----------------------------------------------------------------------
    # 7. RUN
    # ----------------------------------------------------------------------

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
    ):
        """
        Main pipeline entry.
        """
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

        # return full set (branch + bridge)
        return self.branch_sticks + self.bridge_sticks
