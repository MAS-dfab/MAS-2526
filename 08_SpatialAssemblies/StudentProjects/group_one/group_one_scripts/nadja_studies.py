from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane
from group_one_sticks import Stick
import math



class Planarize:
    def __init__(self, point, translation_bottom, translation_top, heigth, angles):
        self.point = point
        self.translation_bottom = translation_bottom
        self.translation_top = translation_top
        self.heigth = heigth
        self.angles = angles
        self.initial_face_points = []
        self.row_points = []
        self.column_points = []    
        self.points_list = []
        self.stick_lines = self.module_stick()
        
    
    def initial_face(self, angle_x = 0, angle_z = 0):
        """
        Docstring for initial_face
        
        :param self: initial point, vector angles in two directions, what is the heigth and width 
        :return: four points
        """
        # STEP 1: create rotation planes for both points
        base_frame = Frame(self.point)
        
        frame_x_rotation = base_frame.rotated(math.radians(self.angles[1]), base_frame.zaxis, base_frame.point)
        v_x = frame_x_rotation.xaxis
        v_x.unitize()
        v_p1 = v_x.scaled(self.translation_bottom)

        frame_z_rotation = base_frame.rotated(math.radians(self.angles[1]), base_frame.xaxis, base_frame.point)
        v_z = frame_z_rotation.zaxis
        v_z.unitize()
        v_p2 = v_z.scaled(self.heigth)
        
        # STEP 2: move root point in v_x direction and v_z direction
        p0 = self.point
        p1 = self.point.translated(v_p1)
        p2 = self.point.translated(v_p2)
        self.initial_face_points += p0, p1, p2, p0
        
        # STEP 3: construct fourt point from v_z point in v_x direction
        v_p3 = v_x.scaled(self.translation_top)
        p3 = p2.translated(v_p3)
        self.initial_face_points.insert(-2, p3)


    @property
    def initial_face_plane(self):
        plane = Frame.from_points(self.point, self.initial_face_points[1], self.initial_face_points[3])
        return plane
    
    def row_faces(self, row_size, angles = 0, translation_bottom = [2, 4], translation_top = [2, 4]):
        """
        Docstring for row_faces
        
        :param self: num of faces in the row, list of angles in x dir, list of Translation bottom point and top point
        """
        # STEP 1: create a list of points inside existing list of initial face
        face_points = [self.initial_face_points]
        
        for i in range(row_size):
            if i % 2 == 0:
                # STEP 1: extract points from initial plain
                p0 = face_points[i][1]
                p3 = face_points[i][2]
                
                # STEP 2: move and rotate points in the face 
                frame_p0 = Frame(p0)
                frame_p0_rotation = frame_p0.rotated(math.radians(self.angles[0]), frame_p0.zaxis, frame_p0.point)
                v_p0 = frame_p0_rotation.xaxis
                v_p0.unitize()
                v_p0_scale = v_p0.scaled(self.translation_bottom)
                p1 = p0.translated(v_p0_scale)
                
                v_p3_scale = v_p0.scaled(self.translation_bottom)
                p2 = p3.translated(v_p3_scale)

                # STEP 3: append to the main list
                face_points.append([p0, p1, p2, p3, p0])
            if i % 2 != 0:
                # STEP 1: extract points from initial plain
                p0 = face_points[i][1]
                p3 = face_points[i][2]
                
                # STEP 2: move and rotate points in the face 
                frame_p0 = Frame(p0)
                frame_p0_rotation = frame_p0.rotated(math.radians(self.angles[1]), frame_p0.zaxis, frame_p0.point)
                v_p0 = frame_p0_rotation.xaxis
                v_p0.unitize()
                v_p0_scale = v_p0.scaled(self.translation_top)
                p1 = p0.translated(v_p0_scale)
                
                v_p3_scale = v_p0.scaled(self.translation_top)
                p2 = p3.translated(v_p3_scale)

                # STEP 3: append to the main list
                face_points.append([p0, p1, p2, p3, p0])
        self.row_points += face_points
        return face_points
    
    def initial_column_faces(self, column_size, angles = 0, translation_bottom = [2, 4], translation_top = [2, 4]):
        """
        Docstring for row_faces
        
        :param self: num of faces in the row, list of angles in x dir, list of Translation bottom point and top point
        """
        # STEP 1: create a list of points inside existing list of initial face
        face_points = [self.initial_face_points]
        
        for i in range(column_size):
            if i % 2 == 0:
                # STEP 1: extract points from initial plain
                p0 = face_points[i][3]
                p1 = face_points[i][2]
                
                # STEP 2: move and rotate points in the face 
                frame_p0 = Frame(p0)
                frame_p0_rotation = frame_p0.rotated(math.radians(self.angles[0]), frame_p0.xaxis, frame_p0.point)
                v_p0 = frame_p0_rotation.zaxis
                v_p0.unitize()
                v_p0_scale = v_p0.scaled(self.heigth)
                p3 = p0.translated(v_p0_scale)
                
                v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                v_p2.unitize()
                v_p0_scale = v_p2.scaled(self.translation_bottom)
                p2 = p3.translated(v_p0_scale)

                # STEP 3: append to the main list
                face_points.append([p0, p1, p2, p3, p0])
            if i % 2 != 0:
                # STEP 1: extract points from initial plain
                p0 = face_points[i][3]
                p1 = face_points[i][2]
                
                # STEP 2: move and rotate points in the face 
                frame_p0 = Frame(p0)
                frame_p0_rotation = frame_p0.rotated(math.radians(self.angles[1]), frame_p0.xaxis, frame_p0.point)
                v_p0 = frame_p0_rotation.zaxis
                v_p0.unitize()
                v_p0_scale = v_p0.scaled(self.heigth)
                p3 = p0.translated(v_p0_scale)
                
                v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                v_p2.unitize()
                v_p0_scale = v_p2.scaled(self.translation_top)
                p2 = p3.translated(v_p0_scale)

                # STEP 3: append to the main list
                face_points.append([p0, p1, p2, p3, p0])
        self.column_points += face_points
        return face_points   
                       
    def column_faces(self, translation_top = [2, 3]):
        """
        Docstring for row_faces
        
        :param self: num of faces in the row, list of angles in x dir, list of Translation bottom point and top point
        """
        # STEP 1: create a list of points inside existing list of initial face
        column_list = self.column_points[1:] # columns [[pt]]
        row_list = self.row_points[1:] # row [[pt]]
        
        # STEP2: create a new structure ofthe lists in columns and rows
        rows_bottom = [row_list] # [[[pt]][[pt]]]
        rows_left = [] # [[pt]]
        
        for r in column_list:
            rows_left.append([r])
        
        # return rows_left[1][0][1] # [1] - row and [0] face in the row
        # return rows_bottom[0][0][2].x # [0] - row and [2] face in the row
        # return len(rows_left)
        
        my_list = [self.row_points]
        
        for i in range(len(rows_left)):
            for j in range(len(rows_bottom[i])):
                if i % 2 == 0:
                    if j % 2 == 0:
                        p0 = rows_bottom[i][j][3]
                        p3 = rows_left[i][j][2]
                        p1 = rows_bottom[i][j][2]
                        
                        # move the point p3 in the direction of the vector of points p0-p1 or p0-p3
                        v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                        v_p2.unitize()
                        v_p0_scale = v_p2.scaled(self.translation_bottom)
                        p2 = p3.translated(v_p0_scale)
                        
                        # append
                        rows_left[i].append([p0, p1, p2, p3, p0])
                    else:
                        p0 = rows_bottom[i][j][3]
                        p3 = rows_left[i][j][2]
                        p1 = rows_bottom[i][j][2]
                        
                        # move the point p3 in the direction of the vector of points p0-p1 or p0-p3
                        v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                        v_p2.unitize()
                        v_p0_scale = v_p2.scaled(self.translation_top)
                        p2 = p3.translated(v_p0_scale)
                        
                        # append
                        rows_left[i].append([p0, p1, p2, p3, p0])
                elif i % 2 != 0:
                    if j % 2 != 0:
                        p0 = rows_bottom[i][j][3]
                        p3 = rows_left[i][j][2]
                        p1 = rows_bottom[i][j][2]
                        
                        # move the point p3 in the direction of the vector of points p0-p1 or p0-p3
                        v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                        v_p2.unitize()
                        v_p0_scale = v_p2.scaled(self.translation_top)
                        p2 = p3.translated(v_p0_scale)
                        
                        # append
                        rows_left[i].append([p0, p1, p2, p3, p0])
                    else:
                        p0 = rows_bottom[i][j][3]
                        p3 = rows_left[i][j][2]
                        p1 = rows_bottom[i][j][2]
                        
                        # move the point p3 in the direction of the vector of points p0-p1 or p0-p3
                        v_p2 = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
                        v_p2.unitize()
                        v_p0_scale = v_p2.scaled(self.translation_bottom)
                        p2 = p3.translated(v_p0_scale)
                        
                        # append
                        rows_left[i].append([p0, p1, p2, p3, p0])
            rows_bottom.append(rows_left[i][1:])
            my_list.append(rows_left[i])
        
        rows_bottom.append(column_list)
        self.points_list.extend(my_list)
    
        return my_list

    def module_stick(self):
        stick_lines = []
        for i,row in enumerate(self.points_list):
            for j,face in enumerate(row):
                for l in range(len(face)-1):
                    stick_line = Line(face[l], face[l+1])
                    stick_lines.append(stick_line)
        
        return stick_lines