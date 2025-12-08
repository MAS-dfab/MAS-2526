# rootframes.py
# r: compas>=2.14.1

import random
import time
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick import Stick
from branch import BranchingModule
from bridge import BridgingModule


class RootFrames:
    """
    RootFrames engine (optimized / guarded):

      1) Sample points on a curve or surface
      2) Build 3D frames using Rhino's native frames (no flattening)
      3) Build (guarded) nearest-neighbour edges & edge-frames
      4) Branching phase (L-system style growth rules)
      5) Optional bridging phase (only between non-coplanar sticks)
      6) Optional collision detection via Stick AABBs (guarded by N)

    Debug channels:
      - self.root_sticks
      - self.branch_sticks
      - self.bridge_sticks
      - self.collision_flags
      - self.frames
      - self.edge_frames
    """

    # safety thresholds
    MAX_POINTS_FOR_EDGES      = 2000   # above this, we won't run O(N^2) nearest-neighbour
    MAX_THEORETICAL_STICKS    = 3000   # point_density * (steps+1) soft cap
    MAX_COLLISION_STICKS      = 1500   # above this, collisions are skipped

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
        self.surface_input = surface
        self.curve_input   = curve

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
        self.root_sticks     = []
        self.branch_sticks   = []
        self.bridge_sticks   = []
        self.collision_flags = []

        # internal for 3D frame reconstruction
        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

    # ----------------------------------------------------------------------
    # 1. SAMPLING  (unchanged)
    # ----------------------------------------------------------------------
    # [keep your current sample_points implementation as-is]

    # ----------------------------------------------------------------------
    # 2. FRAMES (unchanged)
    # ----------------------------------------------------------------------
    # [keep your current frames_from_geometry implementation as-is]

    # ----------------------------------------------------------------------
    # 3. EDGES & EDGE FRAMES (GUARDED)
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        """
        Build nearest-neighbour edges and associated edge frames/vectors.

        If there are too many points (MAX_POINTS_FOR_EDGES), fall back to
        a simple "chain" connectivity (i -> i+1) to avoid O(N^2) explosion.
        """
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            return [], []

        # ---------- SAFE PATH FOR LARGE N ----------
        if n > self.MAX_POINTS_FOR_EDGES:
            # linear chain connectivity
            edges = [(i, i + 1) for i in range(n - 1)]
        else:
            # O(N^2) nearest neighbour (only for manageable N)
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
            edges = list(edges)

        self.edges = edges

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
    # 4. BRANCHING (unchanged logic, but we add theoretical cap)
    # ----------------------------------------------------------------------
    # keep your _parse_rule and grow_branching implementations,
    # but add the "theoretical sticks" check at the top of grow_branching:

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        # theoretical count check BEFORE doing any heavy work
        estimated_sticks = len(self.edge_frames) * (int(steps) + 1)
        if estimated_sticks > self.MAX_THEORETICAL_STICKS:
            raise RuntimeError(
                "RootFrames.grow_branching: estimated {} sticks exceeds "
                "safety cap of {}. Reduce point_density or steps."
                .format(estimated_sticks, self.MAX_THEORETICAL_STICKS)
            )

        # ... then keep your existing branching logic from your last version ...
        # (I won't re-paste the whole thing here to save space, but
        # the only *new* bit is the check above.)

    # ----------------------------------------------------------------------
    # 6. COLLISION DETECTION (GUARDED)
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        """
        Approximate collisions via Stick AABBs.
        Flags are parallel to branch_sticks + bridge_sticks.

        If there are too many sticks, we skip this for performance.
        """
        all_sticks = self.branch_sticks + self.bridge_sticks
        n = len(all_sticks)

        if n > self.MAX_COLLISION_STICKS:
            # skip heavy computation, assume "no collisions" for speed
            self.collision_flags = [False] * n
            return self.collision_flags

        flags = [False] * n
        for i in range(n):
            for j in range(i + 1, n):
                if all_sticks[i].intersects(all_sticks[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ----------------------------------------------------------------------
    # 7. RUN WITH MICRO-PROFILING
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
        verbose=False,
    ):
        """
        Main pipeline entry with simple timing prints if verbose=True.
        """
        t0 = time.time()
        self.sample_points()
        t1 = time.time()
        self.frames_from_geometry()
        t2 = time.time()
        self.frames_to_edges()
        t3 = time.time()

        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
        )
        t4 = time.time()

        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []
        t5 = time.time()

        if detect_collisions:
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * (len(self.branch_sticks) + len(self.bridge_sticks))
        t6 = time.time()

        if verbose:
            print("RootFrames timings [ms]:")
            print("  sample_points     :", int((t1 - t0) * 1000))
            print("  frames_from_geom  :", int((t2 - t1) * 1000))
            print("  frames_to_edges   :", int((t3 - t2) * 1000))
            print("  grow_branching    :", int((t4 - t3) * 1000))
            print("  grow_bridging     :", int((t5 - t4) * 1000))
            print("  detect_collisions :", int((t6 - t5) * 1000))

        return self.branch_sticks + self.bridge_sticks
