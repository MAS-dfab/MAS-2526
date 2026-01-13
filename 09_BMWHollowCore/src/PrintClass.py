from compas.geometry import Point, Polyline, Frame, Vector, distance_point_line, Translation
import math


class PrintPoint:
    def __init__(self, point, velocity = 18.0, air_pressure = 8.0, blend= 1.0, wait_time=0.0, toggle=True, layer_idx=None, trigger_motor_0=False, trigger_motor_1=False):
        self.point = point
        self.velocity = velocity
        self.air_pressure = air_pressure # 4.16 (flat) to 20.00 (max) layer_height 18.00 = mid(range)
        self.blend = blend
        self.wait_time = wait_time
        self.toggle = toggle
        self.layer_idx = layer_idx
        self.frame = self.get_frame()
        self.hc_set_point = 0.0134

        # NEW: color + motor setpoints
        self.rgb = None       
        self.gray_scale = None

        self.trigger_motor_0 = trigger_motor_0
        self.trigger_motor_1 = trigger_motor_1

    def get_frame(self):
        return Frame(self.point, Vector(-1, 5, 0), Vector(0, -1, 0))
      
    def to_dict(self):
        return {
            "frame": self.frame,
            "point": self.point,
            "velocity": self.velocity,
            "air_pressure": self.air_pressure,
            "blend": self.blend,
            "wait_time": self.wait_time,
            "toggle": self.toggle,
            "layer_idx": self.layer_idx,
            "hc_set_point": self.hc_set_point,
            "trigger_motor_0": self.trigger_motor_0,
            "trigger_motor_1": self.trigger_motor_1

        }


class PrintPath:
    """
    A class to represent a print path consisting of multiple layers.
    ...
    Attributes
    ----------
    layers : list
        a list of layers
    prinpoints : list
        a list of printpoints
    path : Polyline
        a polyline representing the path
    """
    def __init__(self, layers, average_robot_speed =6):
        self.layers = layers
        self.printpoints = self.get_printpoints()
        self.path = Polyline([printpoint.point for printpoint in self.printpoints])
        self.average_robot_speed = average_robot_speed
        self.length = self.path.length
        self.nozzle_outer_rad = 12
        self.nozzle_inner_rad = 10.5
        self.area_nozzle = math.pi * (self.nozzle_outer_rad**2 - self.nozzle_inner_rad**2)
    
    def add_safety_point(self, vector, safety_distance = 50.0):
        vec = vector * safety_distance
        T = Translation.from_vector(vec)
        TT = Translation.from_vector(Vector(0, 0, 50))
        tail_pt = PrintPoint(self.printpoints[0].point.transformed(T), toggle = True, layer_idx = 0)
        safe_pt = PrintPoint(tail_pt.point.transformed(TT), velocity=18.0, toggle=True, layer_idx=0)
        self.printpoints.insert(0,tail_pt)
        self.printpoints.insert(0, safe_pt)
    
    def get_printpoints(self):
        printpoints = []
        for layer in self.layers:
            for printpoint in layer.printpoints:
                printpoints.append(printpoint)
        return printpoints
                
    def get_print_time(self):
        print_time = self.length / self.average_robot_speed
        return print_time/3600 #convert seconds to hours
    
    def get_print_weight(self, material_density = 1.27):
        length_path = self.length/10 #convert mm to cm
        volume = length_path * self.area_nozzle /100 #cm3
        grams = volume * material_density
        return grams/1000 # grams to kg
    
    def to_dict(self):
        return {i : printpoint.to_dict() for i, printpoint in enumerate(self.printpoints)}


class Layer:
    def __init__(self, printpoints, layer_idx, layer_height = 18.0):
        self.printpoints = printpoints
        self.layer_idx = layer_idx
        self.layer_height = layer_height  
        self.path = Polyline([printpoint.point for printpoint in printpoints])
    

    def simplify_path(self, epsilon=0.1):
        """
        Simplify a polyline using the Ramer-Douglas-Peucker algorithm.
        Parameters
        ----------
        polyline : compas.geometry.Polyline
            A polyline.
        Returns
        -------
        compas.geometry.Polyline
            The simplified polyline.
        
        Source
        ------
        https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

        """
        def rdp(points, epsilon):
            dmax = 0
            index = 0
            end = len(points) - 1
            for i in range(1, end):
                d = distance_point_line(points[i], (points[0], points[end]))
                if d > dmax:
                    index = i
                    dmax = d
            if dmax > epsilon:
                results = rdp(points[:index + 1], epsilon)[:-1] + rdp(points[index:], epsilon)
            else:
                results = [points[0], points[end]]
            return results
        
        self.path = Polyline(rdp(self.path.points, epsilon))
        self.printpoints = [PrintPoint(point) for point in self.path.points]