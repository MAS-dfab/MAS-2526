from compas_rhino.conversions import mesh_to_compas
from compas.geometry import Polyhedron, is_point_in_polyhedron
from J3RRY_SingleStick_v1 import Stick
from J3RRY_Collision_v1 import Collision
import math, random


class AggregationManager:
    def __init__(self):
        """
        Constructor for Aggregation Manager to handle multiple aggregations.

        graph example:
        [0, 0]; round = 0, root = 0
        [1, 0, 1]; round = 1, root = 0, branch = [1]
        [2, 1, 0, 2]; round = 2 root = 1, branch = [0,2]
        """
        self.graph = []  # {round, root, branch, sub-branch, ...}
        self.aggs = []  # all aggregations, [[sticks], [sticks], [sticks],...[sticks]]


    def run_multiround_aggregation(self, first_frames, length_pattern, angle_pattern,
                                agg_type, seed, agg_count, agg_round=2, branch_count=2,
                                boundary_mesh=None, max_attempts=10):
        
        global_sticks = []
        # ------- Round 0 -------
        current_iters = []
        for root_idx, frame in enumerate(first_frames):
            agg = Aggregation(frame, length_pattern, angle_pattern, agg_type, seed+root_idx*100, init=True)
            success = False
            for j in range(agg_count):
                new_stick = agg.spawn_next_stick_random_in_boundary(
                            boundary_mesh, max_attempts, seed+root_idx*100 + j, global_sticks
                            )
                if new_stick is not None:
                    global_sticks.append(new_stick)
                    success = True
            if not success:
                continue 
            self.aggs.append(agg)
            # Add to current index of aggregations
            current_iters.append(len(self.aggs)-1)  # store current agg index. e.g. [0,1,2,..]
            # Add to graph
            self.graph.append([0, root_idx])  # init round 0, how many root idx. e.g. [0,2,3,...]
        
        # -------Subsequent Rounds -------
        for r in range(1, agg_round):
            next_iters = []
            for parent_iter in current_iters:
                parent_agg = self.aggs[parent_iter]
                parent_last_stick = parent_agg.sticks[-1]

                branch_frame = parent_last_stick.frame
                # Get parent path
                parent_path = self.graph[parent_iter][1:]  # exclude round info

                for b in range(branch_count):
                    child_seed = r*1000 + parent_iter*branch_count + b
                    child_agg = Aggregation(branch_frame, length_pattern, angle_pattern, agg_type, child_seed,
                                            init=False, parent_stick=parent_last_stick
                                            )
                    success = False
                    for j in range(agg_count):
                        new_stick = child_agg.spawn_next_stick_random_in_boundary(
                                    boundary_mesh, max_attempts, child_seed + j, global_sticks
                                    )
                        if new_stick is not None:
                            global_sticks.append(new_stick)
                            success = True
                    if not success:
                        continue
                    self.aggs.append(child_agg)
                    # Add to next index of aggregations
                    next_iters.append(len(self.aggs)-1)  # store current agg index. e.g. [...,6,7,8,..]
                    # Add to graph
                    child_path = parent_path + [b]
                    self.graph.append([r] + child_path)  # agg round, parent idx, branch b
                
            if not next_iters:
                break
            current_iters = next_iters
        return self.aggs



class Aggregation:
    def __init__(self, first_frame, length_pattern=[100], angle_pattern=[0],
                 aggregation_type=0, global_seed=None, init=True, parent_stick=None):
        """
        Constructor for a single Aggregation spawn.
        
        Args:
            first_frame: Frame for the first stick.
            length_pattern: List of lengths or single length value for sticks.
            angle_pattern: list of angles or single angle value for sticks.
            aggregation_type: 0 = regular, 1 = random
            global_seed: Seed for random generator (defaults to None)
            init: bool, whether to initialize the first stick.
            parent_stick: optional, type Stick, parent stick from previous aggregation round.
        """
        self.length_pattern = length_pattern
        self.angle_pattern = angle_pattern
        self.sticks = []
        self.axes = []
        self.frames = []

        self.failed_sticks = []
        self.valid_sticks = []
        self.scores = []
        self.face_indices = []
        self.t_values = []
        self.collision_log = []


        self._boundary_polyhedron = None

        self.parent_stick = parent_stick
        
        # 0 = regular, 1 = random
        self.aggregation_type = aggregation_type
        self.global_seed = global_seed
        if global_seed is not None:
            random.seed(global_seed)

        if init:
            self._init_first_stick(first_frame)
            self.face_indices.append(None)
            self.t_values.append(None)

    def _init_first_stick(self, first_frame):
        """
        Private method for creating the first stick with the first index of length_pattern.
        
        Args:
            first_frame: Frame for the first stick.
        """
        first_stick = Stick(first_frame, self.length_pattern[0])
        self.sticks.append(first_stick)
        self.axes.append(first_stick.axis)
        self.frames.append(first_frame)

    
    def _next_length(self):
        """
        Private method to get the next length from the list of length pattern.
        
        Returns:
            float: next length value.
        """
        return random.choice(self.length_pattern)
    

    def _next_angle(self):
        """
        Private method to get the next angle from the list of angle pattern.
        
        Returns:
            float: next angle in degrees.
        """
        return random.choice(self.angle_pattern)


    def _remap_t_with_margin(self, t, length, margin=None):
        """
        Private method to remap t value with margin.

        Args:
            t: float, original t value (0.0 to 1.0).
            margin: float, margin to apply (defaults to stick width).

        Returns:
            float: remapped t value within margin constraints.
        """
        if margin is None:
            margin = self.width
        
        # Avoid 2 * margin > length
        max_margin = length * 0.5 * 0.999
        m = min(margin, max_margin)

        t = max(0.0 ,min(1.0, t))

        u_min = m / length
        u_max = 1.0 - (m / length)
        u = u_min + t * (u_max - u_min)
        return u
    

    def _center_to_existing(self, candidate, external_sticks=None):
        """
        Private method to find the smallest squared distance between the center point of a candidate stick to all existing sticks, excluding the last stick(parent).
        
        Args:
            candidate: type Stick, candidate stick.
            external_sticks: list of type Stick, from other Aggregations.
        
        Returns:
            float: smallest squared distance between candidate center to existing sticks, excluding the last stick(parent).
        """
        all_sticks = self.sticks[:-1]
        if external_sticks:
            all_sticks += external_sticks

        if not all_sticks:
            return float('inf')
        
        min_dist = float('inf')
        cx, cy, cz = candidate.midframe.point

        for s in all_sticks:
            px, py, pz = s.midframe.point
            dx = cx - px
            dy = cy - py
            dz = cz - pz
            dist = dx*dx + dy*dy + dz*dz

            if dist < min_dist:
                min_dist = dist
        # return math.sqrt(min_dist)
        return min_dist
    

    def _set_boundary_polyhedron(self, mesh):
        """
        Private method to convert Rhino Mesh to COMPAS Polyhedron for boundary checking, and store it internally.
        
        Args:
            mesh: type Rhino Mesh, must be closed mesh. Will convert to COMPAS Polyhedron.
            
        Returns:
            type Polyhedron: COMPAS Polyhedron.
        """
        if mesh is None:
            return None
        if self._boundary_polyhedron is None:
            compas_mesh = mesh_to_compas(mesh)
            vertices, faces = compas_mesh.to_vertices_and_faces()
            self._boundary_polyhedron = Polyhedron(vertices, faces)
    

    def is_stick_in_mesh(self, stick, mesh):
        """
        Check if all corners of a stick are inside a given closed mesh.

        Args:
            point: type Point, point to check.
            mesh: type Rhino Mesh, must be closed mesh. Will convert to COMPAS Polyhedron.

        Returns:/
            bool: True if point is inside mesh.
        """
        self._set_boundary_polyhedron(mesh)  # Lazy init
        if self._boundary_polyhedron is None:
            return True  # No boundary defined, view all points as inside
        
        return all(is_point_in_polyhedron(c, self._boundary_polyhedron) for c in stick.corners)


    def spawn_next_stick(self, from_index=0, from_t=0.5, to_index=1, to_t=0.5):
        """
        Spawns next stick based on specified face index and t value.
        
        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            from_index: Face index (0-3) on the current stick to attach from.
            from_t: Relative position along the from-face (0.0 to 1.0).
            to_index: Face index (0-3) on the current stick to attach to.
            to_t: Relative position along the to-face (0.0 to 1.0).
        """
        current_stick = self.sticks[-1]
        new_length = self._next_length()
        angle = self._next_angle()

        # Apply margin to from_t
        safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
        # Evlaute frames for next stick
        from_frame = current_stick.eval_frame(from_index, safe_from_t)
        dir_vec = from_frame.zaxis
        dir_vec.unitize()
        next_frame = from_frame.copy()
        # Move next_frame along normal by half current stick depth
        next_frame.point += dir_vec * (current_stick.depth / 2)
        
        # Define which face relative to new stick attaches to previous stick
        which_face = to_index * -90
        next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

        # Rotate next_frame around normal
        if angle != 0:
            next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

        # Apply margin to to_t
        safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)
        # Adjust start point along negative x-axis by new stick length
        offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
        next_frame.point -= offset

        # Create new stick
        new_stick = Stick(next_frame, length=new_length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.frames.append(next_frame)


    def spawn_next_stick_random(self, local_seed=None):
        """
        Spawns next stick based on random face index and t value.

        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            local_seed: Seed for random generator (overrides global seed if provided).
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Create random index and t value for the next stick
        from_index = random.randint(0, 3)
        from_t = random.random()
        to_index = random.randint(0, 3)
        to_t = random.random()
        self.spawn_next_stick(from_index, from_t, to_index, to_t)


    def spawn_next_stick_random_with_rejection(self, max_attempts=10, local_seed=None):
        """
        Spawns next stick based on random face index and t value with collision rejection sampling.

        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            max_attempts: Maximum number of attempts to find a non-colliding position.
            local_seed: Seed for random generator (overrides global seed if provided).

        Records:
            - collision_log: pure metadata
            - failed_sticks: list of Stick objects colliding in each spawn attempt
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Preparation for logging
        failed_candidates = []
        params_length = []
        params_angle = []
        params_from_index = []
        params_from_t = []
        params_to_index = []
        params_to_t = []


        current_stick = self.sticks[-1]
        for attempt in range(max_attempts):
            new_length = self._next_length()
            angle = self._next_angle()
            # Create random index and t value for the next stick
            from_index = random.randint(0, 3)
            # Apply margin to from_t
            from_t = random.random()
            safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
            to_index = random.randint(0, 3)
            # Apply margin to to_t
            to_t = random.random()
            safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)

            # Collect Data for logging
            params_from_index.append(from_index)
            params_from_t.append(safe_from_t)
            params_to_index.append(to_index)
            params_to_t.append(safe_to_t)
            params_length.append(new_length)
            params_angle.append(angle)

            # Evaluate frames for next stick
            from_frame = current_stick.eval_frame(from_index, safe_from_t)
            dir_vec = from_frame.zaxis
            dir_vec.unitize()
            next_frame = from_frame.copy()
            # Move next_frame along normal by half current stick depth
            next_frame.point += dir_vec * (current_stick.depth / 2)
            
            # Define which face of new stick attaches to previous stick
            which_face = to_index * -90
            next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

            # Rotate next_frame around normal
            if angle != 0:
                next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

            # Adjust start point along negative x-axis by new stick length
            offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
            next_frame.point -= offset

            # Create new stick
            new_stick = Stick(next_frame, length=new_length)

            # --- collision checking
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    break
                    
            if collision_found:
                failed_candidates.append(new_stick)
                continue  # try again
            self.failed_sticks.append(failed_candidates)

            self.collision_log.append({
                "attempts": attempt + 1,
                "fail_count": len(failed_candidates),
                "success": True,
                "reason": "No Collision",
                "params": {
                    "from_index": params_from_index,
                    "from_t": params_from_t,
                    "to_index": params_to_index,
                    "to_t": params_to_t,
                    "length": params_length,  
                    "angle": params_angle
                }
            })

            # SUCCESS: append and return
            self.sticks.append(new_stick)
            self.axes.append(new_stick.axis)
            self.frames.append(next_frame)
            return new_stick
        
        # If all attempts fail
        self.failed_sticks.append(failed_candidates)
        self.collision_log.append({
            "attempts": max_attempts,
            "fail_count": len(failed_candidates),
            "success": False,
            "reason": "Max Attempts Reached",
            "params": {
                "from_index": params_from_index,
                "from_t": params_from_t,
                "to_index": params_to_index,
                "to_t": params_to_t,
                "length": params_length,
                "angle": params_angle
            }
        })
        return None


    def spawn_next_stick_random_in_boundary(self, boundary=None, max_attempts=10, local_seed=None, external_sticks=None):
        """
        Spawns next stick based on random face index and t value with collision rejection sampling.

        Args:
            boundary: type Rhino Mesh, to constrain sticks within boundary.
            max_attempts: Maximum number of attempts to find a non-colliding position.
            local_seed: Seed for random generator (overrides global seed if provided).
            external_sticks: list of type Stick for additional collision checking, basically from other Aggregations.

        Records:
            - collision_log: pure metadata
            - failed_sticks: list of Stick objects colliding in each spawn attempt
        
        Returns:
            type Stick: successfully spawned stick, or None if all attempts fail.
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Preparation for selecting candidate
        valid_candidates = []
        valid_scores = []
        valid_face_idx = []
        valid_t = []

        # Preparation for logging
        failed_candidates = []
        params_length = []
        params_angle = []
        params_from_index = []
        params_from_t = []
        params_to_index = []
        params_to_t = []

        # Initial boundary check for the first stick
        if boundary is not None and len(self.sticks) == 1:
            if not self.is_stick_in_mesh(self.sticks[0], boundary):
                raise ValueError("The first stick is outside the boundary mesh.")

        # Determine current stick, if running multiple rounds of aggregation, use parent stick as the last stick of previous aggregation
        if self.sticks:
            current_stick = self.sticks[-1]
        elif self.parent_stick is not None:
                current_stick = self.parent_stick
        else:
            return None
        
        for _ in range(max_attempts):
            new_length = self._next_length()
            angle = self._next_angle()
            # Create random face index and t value for the next stick
            from_index = random.randint(0, 3)
            # Apply margin to from_t
            from_t = random.random()
            safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
            to_index = random.randint(0, 3)
            # Apply margin to to_t
            to_t = random.random()
            safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)
            
            # Collect Data for logging
            params_from_index.append(from_index)
            params_from_t.append(safe_from_t)
            params_to_index.append(to_index)
            params_to_t.append(safe_to_t)
            params_length.append(new_length)
            params_angle.append(angle)
            
            # Evaluate frames for next stick
            from_frame = current_stick.eval_frame(from_index, safe_from_t)
            dir_vec = from_frame.zaxis
            dir_vec.unitize()
            next_frame = from_frame.copy()
            # Move next_frame along normal by half current stick depth
            next_frame.point += dir_vec * (current_stick.depth / 2)
            
            # Define which face of new stick attaches to previous stick
            which_face = to_index * -90  # should be -90
            next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)
            # Rotate next_frame around normal
            if angle != 0:
                next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

            # Adjust start point along negative x-axis by new stick length
            offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
            next_frame.point -= offset
            # Create new stick
            new_stick = Stick(next_frame, length=new_length)

            ### ------- boundary checking -------
            if boundary is not None:
                if not self.is_stick_in_mesh(new_stick, boundary):
                    failed_candidates.append(new_stick)
                    continue

            ### ------- collision checking -------
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    break
            ### ------- Check against multiple aggregations' sticks if provided
            if (not collision_found) and external_sticks:
                for other in external_sticks:
                    if Collision(new_stick, other).check_collision():
                        collision_found = True
                        break
            
            if collision_found:
                failed_candidates.append(new_stick)
            else:
                valid_candidates.append(new_stick)
                valid_scores.append(self._center_to_existing(new_stick, external_sticks))
                valid_face_idx.append((to_index + 2) % 4)  # face on new stick that connects to current stick
                valid_t.append(safe_to_t)

        # Select best candidate based on score
        if not valid_candidates:
            return None
        best_idx = max(range(len(valid_scores)), key=lambda i: valid_scores[i])
        best_candidate = valid_candidates[best_idx]
        best_face_idx = valid_face_idx[best_idx]
        best_t = valid_t[best_idx]

        # SUCCESS: append and return
        self.failed_sticks.append(failed_candidates)
        self.valid_sticks.append(valid_candidates)
        self.scores.append(valid_scores)
        self.sticks.append(best_candidate)
        self.axes.append(best_candidate.axis)
        self.frames.append(best_candidate.frame)

        self.face_indices.append(best_face_idx)
        self.t_values.append(best_t)
        
        self.collision_log.append({
            "attempts": max_attempts,
            "fail_count": len(failed_candidates),
            "success": True,
            "reason": "Pass",
            "params": {
                "from_index": params_from_index,
                "from_t": params_from_t,
                "to_index": params_to_index,
                "to_t": params_to_t,
                "length": params_length,
                "angle": angle
                
            }
        })
        return best_candidate