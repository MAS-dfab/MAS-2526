from compas.geometry import Point, Polyline, Frame, Vector, distance_point_line, Translation, closest_point_on_polyline, Line, Curve
import math

def remap(value, from_min, from_max, to_min, to_max):
    # Remap a value from one range to another
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled_value = float(value - from_min) / float(from_range)
    return to_min + (scaled_value * to_range)

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
        self.rgb_raw = None

        self.motor_106 = trigger_motor_0
        self.motor_107 = trigger_motor_1

    def get_frame(self):
        return Frame(self.point, Vector(1, -5, 0), Vector(0, -1, 0))
      
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
            "trigger_motor_0": self.motor_106,
            "trigger_motor_1": self.motor_107

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
    def __init__(self, layers, average_robot_speed = 18.0):
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
        tail_pt = PrintPoint(self.printpoints[0].point.transformed(T), toggle = True, layer_idx = 0, velocity=7.0)
        safe_pt = PrintPoint(tail_pt.point.transformed(TT), toggle=True, layer_idx=0, velocity=7.0)
        self.printpoints.insert(0,tail_pt)
        self.printpoints.insert(0, safe_pt)
    
    def end_safety_point(self, vector, safety_distance = 50.0):
        vec = vector * safety_distance
        T = Translation.from_vector(vec)
        head_pt = PrintPoint(self.printpoints[-1].point.transformed(T), toggle = False, layer_idx = self.layers[-1].layer_idx, velocity=7.0)
        self.printpoints.append(head_pt)
       

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
    
    def get_print_angles(self):
        # this function is exactly the same as the one from Nik's tutorial (but uses compas)

        vert_vec = Vector(0, 0, -1)

        print_offset = []
        print_angles = []

        # get directly the contours from the layers
        contours = [layer.path for layer in self.layers]
        # print (len(contours))
        for i, crv in enumerate(contours):
            if i == 0:
                print_angles.extend([0] * len(crv))
                print_offset.extend([0] * len(crv))

            else:
                prev_pl = contours[i - 1]
                for pt in crv:
                    # layer.path is a polyline = list of points
                    par = Point(*closest_point_on_polyline(pt, prev_pl))
                    # print(par)
                    vect = par - pt
                    ang = vect.angle(vert_vec, True)
                    print_angles.append(ang)

                    dist = par.distance_to_point(Point(pt.x, pt.y, par.z))
                    print_offset.append(dist)

        return print_angles, print_offset
    
    def build_spiral_path(self, turns_per_layer=1):
        """
        Build a single continuous polyline that ramps from layer 0 -> 1 -> 2 -> ...

        Parameters
        ----------
        turns_per_layer : int
            How many "index turns" to do per layer transition. 1 matches your original.
        Returns
        -------
        Polyline
        """
        
        all_pts = []

        for i in range(len(self.layers) - 1):
            layer_points = self.layers[i].printpoints
            pts1 = [pp.point for pp in layer_points]
            pts2 = [pp.point for pp in self.layers[i + 1].printpoints]
            div = turns_per_layer * len(layer_points)


            for i in range(len(layer_points)):
                
                point1 = pts1[i]
                point2 = pts2[i]
            
                line = Line(point1, point2)
                a = remap(i, 0, div, 0, 1)
                point = line.point_at(a)

                all_pts.append(point)
        spiral = Polyline(all_pts)
        self.path = spiral
        self.printpoints = [PrintPoint(point) for point in spiral.points]
        return spiral


    
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