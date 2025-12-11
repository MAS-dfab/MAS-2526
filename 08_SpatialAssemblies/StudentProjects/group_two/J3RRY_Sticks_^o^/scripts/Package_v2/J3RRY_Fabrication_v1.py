from J3RRY_SingleStick_v1 import Stick
from compas.geometry import Frame, Transformation

class Fabrication:
    def __init__(self, aggregation_manager):
        
        self.aggregation_manager = aggregation_manager
        self.modules = aggregation_manager.aggs
        self.sticks = []

    
    def _init_sticks(self):
        if not self.sticks:
            for module in self.modules:
                sticks = module.sticks
                self.sticks.append(sticks)


    def get_joint_frames(self):
        """
        Get joint frames which belong to the spawned sticks in the module.
        
        Returns:
        """
        joint_frames = []

        for module in self.modules:
            joint_branch = []
            for f, t, s in zip(module.face_indices, module.t_values, module.sticks):
                if f is not None and t is not None:
                    frame = s.eval_frame(f, t)
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
        """ 
        self._init_sticks()

        graph = self.aggregation_manager.graph
        modules = self.modules
        max_round = max(g[0] for g in graph)  # Get the maximum round number
        
        # Create a dictionary to find indices by round number.
        # e.g. round = 0: [0, 1, 2]; round = 1: [3,4,5,6,8,...]
        round_to_indices = {}
        for idx, g in enumerate(graph):
            round = g[0]
            round_to_indices.setdefault(round, []).append(idx)
        
        for idx, g in enumerate(graph):
            round = g[0]  
            root = g[1]
            parent_branch = g[2:]  # Branch path for current module
    
            # ----Find chlid (upper) adjacent sticks----
            if round < max_round:
                child_round = round + 1
                for j in round_to_indices.get(child_round, []):
                    child_graph = graph[j]
                    if child_graph[1] != root:
                        continue
                    child_branch = child_graph[2:]

                    if child_branch[:len(parent_branch)] != parent_branch:
                        continue
                    child_module = modules[j]
                    child_first_stick = child_module.sticks[0]
                    self.sticks[idx].append(child_first_stick)

            # ----Find parent (lower) adjacent sticks----
            if both and round > 0:
                parent_round = round - 1
                for j in round_to_indices.get(parent_round, []):
                    pass  # do this later


    def erect_modules(self):
        """
        Erect modules to vertical position for fabrication.
        The first stick of each module will be aligned to global Z axis.

        Returns:
            erected_modules: list of type Stick, list of sticks in erected position.
        """
        erected_modules = []
        for module in self.modules:
            sticks = module.sticks
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
            erected_modules.append(branch_sticks)
        return erected_modules
    

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


    def plot_modules(self):
        pass


    def send_modules_to_place_frames(self):
        pass


    def eval_pick_frame(self):
        pass