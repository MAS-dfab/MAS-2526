from J3RRY_SingleStick_v1 import Stick
from J3RRY_Collision_v1 import Collision
import math, random


class Aggregation:
    def __init__(self, first_frame, length_pattern=[100], angle_pattern=[0], aggregation_type=0, global_seed=None):
        """
        Constructor for Stick Aggregation.
        
        Args:
            first_frame: Frame for the first stick.
            length_pattern: List of lengths or single length value for sticks.
            aggregation_type: 0 = regular, 1 = random
            global_seed: Seed for random generator (defaults to None)
        """
        self.sticks = []
        self.axes = []
        self.frames = []
        self.length_pattern = length_pattern
        self.angle_pattern = angle_pattern

        self.failed_sticks = []
        self.collision_log = []

        # 0 = regular, 1 = random
        self.aggregation_type = aggregation_type
        self.global_seed = global_seed
        if global_seed is not None:
            random.seed(global_seed)

        self._init_first_stick(first_frame)


    def _init_first_stick(self, first_frame):
        """
        Private method for creating the first stick.
        
        Args:
            first_frame: Frame for the first stick.
        """
        first_stick = Stick(first_frame, self._next_length())
        self.sticks.append(first_stick)
        self.axes.append(first_stick.axis)
        self.frames.append(first_frame)

    
    def _next_length(self):
        """
        Private method to get the next length from the length pattern.
        
        Returns:
            float: next length value.
        """
        return random.choice(self.length_pattern)
    

    def _next_angle(self):
        """
        Private method to get the next angle from the angle pattern.
        
        Returns:
            float: next angle in degrees.
        """
        return random.choice(self.angle_pattern)
        

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
        # Evlaute frames for next stick
        from_frame = current_stick.eval_frame(from_index, from_t)
        dir_vec = from_frame.zaxis
        dir_vec.unitize()
        next_frame = from_frame.copy()
        # Move next_frame along normal by half current stick depth
        next_frame.point += dir_vec * (current_stick.depth / 2)
        
        # Define which face relative to new stick attaches to previous stick
        which_face = to_index * 90
        next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

        # Rotate next_frame around normal
        if angle != 0:
            next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

        # Adjust start point along negative x-axis by new stick length
        offset = next_frame.xaxis.unitized() * (to_t * new_length)
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
            from_t = random.random()
            to_index = random.randint(0, 3)
            to_t = random.random()

            # Evlaute frames for next stick
            from_frame = current_stick.eval_frame(from_index, from_t)
            dir_vec = from_frame.zaxis
            dir_vec.unitize()
            next_frame = from_frame.copy()
            # Move next_frame along normal by half current stick depth
            next_frame.point += dir_vec * (current_stick.depth / 2)
            
            # Define which face relative to new stick attaches to previous stick
            which_face = to_index * 90
            next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

            # Rotate next_frame around normal
            if angle != 0:
                next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

            # Adjust start point along negative x-axis by new stick length
            offset = next_frame.xaxis.unitized() * (to_t * new_length)
            next_frame.point -= offset

            # Create new stick
            new_stick = Stick(next_frame, length=new_length)

            # --- collision checking
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    
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
                    "length": new_stick.length,  
                    "angle": angle
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
                "length": new_stick.length,
                "angle": angle
            }
        })
        return None


    def spawn_next_stick_random_in_boundary_with_rejection(self, boundary, angle=0, max_attempts=10):
        

        return None


    def visualize(self):
        """
        Compute all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
