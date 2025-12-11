from J3RRY_SingleStick_v1 import Stick
from compas.geometry import Frame, Transformation, Polyline, Point
import math


class Fabrication:
    def __init__(self, aggregation_manager):

        self.aggregation_manager = aggregation_manager
        self.aggs = aggregation_manager.aggs
        self.graph = aggregation_manager.graph

        self.original_modules = [agg.sticks for agg in self.aggs]
        self.modules = [list(branch) for branch in self.original_modules]  # copy of original modules
        self.agg_indices = []
        self.stick_indices = []

        self.erected_sticks = []
        self.sticks_with_adjacent = []

        
    def _round_to_indices(self):
        """
        Create a dictionary to find indices by round number.
        e.g. round = 0: [0, 1, 2]; round = 1: [3,4,5,6,8,...]
        """
        mapping = {}
        for idx, g in enumerate(self.graph):
            round = g[0]
            mapping.setdefault(round, []).append(idx)
        return mapping
    

    def get_face_t(self, agg_idx, stick_idx):
        agg = self.aggs[agg_idx]
        face_indices = agg.face_indices
        t_values = agg.t_values
        return face_indices[stick_idx], t_values[stick_idx]
    

    def get_joint_frames(self, modules=None):
        """
        Get joint frames which belong to the spawned sticks in the module.
        
        Returns:
            joint_frames: list of type Frame, frames belonging to spawned sticks.
        """
        if modules is None:
            joint_frames = []
            for agg in self.aggs:
                joint_branch = []
                for f, t, s in zip(agg.face_indices, agg.t_values, agg.sticks):
                    if f is not None and t is not None:
                        frame = s.eval_frame(f, t)
                        joint_branch.append(frame)
                joint_frames.append(joint_branch)
            return joint_frames


        if not self.agg_indices or not self.stick_indices:
            raise ValueError("Please run 'add_adjacent_sticks' method first to populate agg_indices and stick_indices.")

        joint_frames = []
        for module_idx, branch in enumerate(modules):
            joint_branch = []
            agg_idx_list = self.agg_indices[module_idx]
            stick_idx_list = self.stick_indices[module_idx]
            
            for agg_idx, stick_idx, stick in zip(agg_idx_list, stick_idx_list, branch):
                f, t = self.get_face_t(agg_idx, stick_idx)
                if f is not None or t is not None:
                    frame = stick.eval_frame(f, t)
                    joint_branch.append(frame)
            joint_frames.append(joint_branch)
        return joint_frames


    def add_adjacent_sticks(self, both=False):  #  both=True script has not been constructed yet
        """
        Add adjacent sticks (child = upper, parent = lower) to each module,
        in order to mark the overlap areas.

        if both = True, add both upper and lower adjacent sticks.
        if both = False, only add upper adjacent sticks.
    
        graph example:
        [0, 0]; round = 0, root = 0
        [1, 0, 1]; round = 1, root = 0, branch = [1]
        [2, 1, 0, 2]; round = 2 root = 1, branch = [0,2]

        Args:
            both (bool): Whether to include both upper and lower adjacent sticks.
            modules (list): Optional list of modules to use instead of the original ones.
        """ 
        graph = self.graph
        max_round = max(g[0] for g in graph)  # Get the maximum round number
        

        base_modules = self.original_modules

        self.sticks_with_adjacent = [list(branch) for branch in base_modules]  # Copy base modules
        self.agg_indices = []
        self.stick_indices = []
        for agg_idx, agg in enumerate(self.aggs):
            n = len(agg.sticks)
            self.agg_indices.append([agg_idx] * n)
            self.stick_indices.append(list(range(n)))

        mapping = self._round_to_indices()
        for idx, g in enumerate(graph):
            round = g[0]  
            root = g[1]
            parent_branch = g[2:]  # Branch path for current module
    
            # ----Find child (upper) adjacent sticks----
            if round < max_round:
                child_round = round + 1
                for j in mapping.get(child_round, []):
                    child_graph = graph[j]
                    if child_graph[1] != root:
                        continue
                    child_branch = child_graph[2:]

                    if child_branch[:len(parent_branch)] != parent_branch:
                        continue
                    child_sticks = base_modules[j]
                    if not child_sticks:
                        continue

                    child_first_stick = child_sticks[0]
                    self.sticks_with_adjacent[idx].append(child_first_stick)

                    agg_idx = j
                    stick_idx = 0  # first stick of the child module
                    self.agg_indices[idx].append(agg_idx)
                    self.stick_indices[idx].append(stick_idx)

            # ----Find parent (lower) adjacent sticks----
            if both and round > 0:
                parent_round = round - 1
                for j in mapping.get(parent_round, []):
                    pass  # do this later

        return self.sticks_with_adjacent, self.agg_indices, self.stick_indices


    def erect_modules(self, modules=None):
        """
        Erect modules to vertical position for fabrication.
        The first stick of each module will be aligned to global Z axis.

        Returns:
            erected_modules: list of type Stick, list of sticks in erected position.
        """
        new_modules = []
        if modules is None:
            current_modules = self.original_modules
        else: 
            current_modules = modules

        for sticks in current_modules:
            # frame to orient from
            from_frame = sticks[0].frame.copy()
            # frame to orient to
            to_frame = Frame(from_frame.point, [0,0,1], [0,1,0])
            # Create Orientation
            O = Transformation.from_frame_to_frame(from_frame, to_frame)
            branch_sticks = []
            for stick in sticks:
                new_frame = stick.frame.copy()
                new_frame.transform(O)
                ori_stick = Stick(new_frame, stick.length)
                branch_sticks.append(ori_stick)
            new_modules.append(branch_sticks)
        self.modules = new_modules

        return self.modules
        

    def erect_stick(self, frames):
        """
        Conversion for any types of sticks using type Frame.
        Michael can follow the section "erect_modules" above as the guidance.
        """
        # frame to orient from

        # from to orient to

        # Create Orientation

        # Return type ???
        pass


    def plot_modules(self, modules=None, origin=(0,0,0), x_size=400, y_size=240):
        # Create plotting points
        cols, rows = [], []
        for idx, g in enumerate(self.graph):
            mapping = self._round_to_indices()
            row = mapping.get(g[0], []).index(idx)
            col = g[0]
            cols.append(col)
            rows.append(row)
            
        to_frames = []
        for col, row in zip(cols, rows):
            pt = Point(origin[0] + col * x_size, origin[1] + row * y_size, origin[2])
            to_frame = Frame(pt, [0,0,1], [0,1,0])
            to_frames.append(to_frame)

        # Create Border
        half_x = x_size / 2
        half_y = y_size / 2
        recs = []
        for f in to_frames:
            X, Y = f.point.x, f.point.y
            p0 = (X + half_x, Y + half_y, 0)
            p1 = (X - half_x, Y + half_y, 0)
            p2 = (X - half_x, Y - half_y, 0)
            p3 = (X + half_x, Y - half_y, 0)
            rec = Polyline([p0, p1, p2, p3, p0])
            recs.append(rec)

        new_modules = []
        if modules is None:
            current_modules = self.original_modules
        else: 
            current_modules = modules

        for idx, sticks in enumerate(current_modules):
            to_frame = to_frames[idx]
            from_frame = sticks[0].frame.copy()
            branch_sticks = []
            for stick in sticks:
                new_frame = stick.frame.copy()
                translation = Transformation.from_frame_to_frame(from_frame, to_frame)
                new_frame.transform(translation)
                ori_stick = Stick(new_frame, stick.length)
                branch_sticks.append(ori_stick)
            new_modules.append(branch_sticks)

        return new_modules, recs


    def send_modules_to_place_frame(self, frame):
        pass


    def eval_pick_frames(self, modules, robot_position=Point(0,0,0)):
        """
        Compute default pick frames for each module.

        Args:
            modules: list of type Stick, modules to compute pick frames for.
            robot_position: type Point, will choose the face which closest to the robot position base on the center of the faces.
            
        Returns:
            pick_frames: list of type Frame, first pick frames to try.    
        """
        pick_frames = []
        for module_idx, branch in enumerate(modules):

            pick_branch = []
            agg_idx_list = self.agg_indices[module_idx]
            stick_idx_list = self.stick_indices[module_idx]
            
            for agg_idx, stick_idx, stick in zip(agg_idx_list, stick_idx_list, branch):
                f, t = self.get_face_t(agg_idx, stick_idx)
                if f is not None or t is not None:
                    if t < 0.5:
                        new_t = t + ((1-t) / 2)
                    else:
                        new_t = t / 2
                    # Redifine face index based on robot position
                    pts = [stick.eval_frame(f_idx, 0.5).point for f_idx in range(4)]
                    dist = [pt.distance_to_point(robot_position) for pt in pts]
                    new_face_idx = min(list(range(4)), key=lambda i: dist[i])

                    frame = stick.eval_frame(new_face_idx, new_t)
                    frame.rotate(math.pi, stick.frame.xaxis, frame.point)
                    pick_branch.append(frame)
            pick_frames.append(pick_branch)
        return pick_frames