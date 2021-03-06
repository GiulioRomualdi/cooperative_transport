from __future__ import division
import random
import rospy
import threading
import numpy as np
import math3d as m3d
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState
from cooperative_transport.utils import quaternion_to_yaw
from std_srvs.srv import Empty
from utils import Segment, Line, angle_normalization
from cooperative_transport.filter import LowPassFilter
from cooperative_transport.srv import GetUncertaintyAreaInfo
from cooperative_transport.srv import GetUncertaintyAreaInfoRequest
from cooperative_transport.srv import GetUncertaintyAreaInfoResponse
from cooperative_transport.srv import IsInitialGuessFound
from cooperative_transport.srv import IsInitialGuessFoundRequest
from cooperative_transport.srv import IsInitialGuessFoundResponse

# debug
from gazebo_msgs.msg import ModelStates
import csv

class BoxGeometry:
    """Box geometry"""
    
    def __init__(self, length, width, center, theta):
        """Initialize box geometry.

        Attributes:
            length (float): box length in meters
            width (float): box length in meters
            center (float[2]): box center [x_c, y_c] in meters
            orientation (float): box orientation
        """
        self.length = length
        self.width = width
        self.center = center
        self.theta = theta

    def vertex(self, vertex_index):
        """Return the requested box vertex.

        Arguments:
            vertex (int): the index of the requested vertex
        """
        
        # Find the vertex coordinates in a box-fixed reference frame
        #
        #  D --------- C
        #  -           -
        #  -   (0,0)   -
        #  -           -
        #  A --------- B
        #
        vertex_index %= 4
        delta_x = float(self.length)/2
        delta_y = float(self.width)/2
        x_vertex = 0
        y_vertex = 0
        #A
        if vertex_index == 0:
            x_vertex = - delta_x
            y_vertex = - delta_y
        #B
        elif vertex_index == 1:
            x_vertex = + delta_x
            y_vertex = - delta_y
        #C
        elif vertex_index == 2:
            x_vertex = + delta_x
            y_vertex = + delta_y
        #D
        else:
            x_vertex = - delta_x
            y_vertex = + delta_y

        # Calculate the vector
        vector = m3d.Vector(x_vertex, y_vertex)
        # Rotatate the vector by theta
        rotation = m3d.Orientation.new_rot_z(self.theta)
        rotated_vector = rotation * vector        
        # Translate the vector
        p_c = m3d.Vector(self.center)
        transl_vector = p_c + rotated_vector

        vector = transl_vector.get_list()
        # we need 2D vector
        vector.pop()

        return vector

    def vertices(self):
        """Return the vertices in ccw order."""

        return [self.vertex(i) for i in range(4)]

    def edge(self, edge_i, edge_j):
        """Return a segment representing the edge between vertices (edge_i) and (edge_j).

        Arguments:
            edge_i (int): i-th edge index
            edje_j (int): j-th edge index
        """

        edge_i %= 4
        edge_j %= 4

        # Get the vertices
        vertices = self.vertices()

        # Create the segment
        segment = Segment (vertices[edge_i], vertices[edge_j])

        return segment

class BoxStateObserver:
    """Estimate box state using noisy coordinates of points that belong to a rectangular perimeter

    Attributes:
        length (float): box length in meters
        width (float): box width in meters
        state (float[]): box state [x_c, y_c, theta]
    """

    def __init__(self, length, width):
        """Initialize the object.
        
        Arguments:
            length (float): box length in meters
            width (float): box width in meters
        """
        self.length = length
        self.width = width
        self._state = [0, 0, 0]

    @property
    def state(self):
        return self._state

    def __rectangle (self, x, y, state):
        """Evaluate the rectangle equation based on a Lame' curve.

        Arguments:
            x (float): x coordinate of a point that belongs to a rectangular perimeter
            y (float): y coordinate of a point that belongs to a rectangular perimeter
            state (float[]): rectangle state [x_c, y_c, theta]
        """
        x_c = state[0]
        y_c = state[1]
        theta = state[2]
    
        value = 2 * (-1 + abs(((self.width * x - self.width * x_c -\
                self.length * y + self.length * y_c) * np.cos(theta) +\
                (self.length * x - self.length * x_c + self.width * y -\
                self.width * y_c) * np.sin(theta)) / (self.length * self.width)) +\
                abs(((self.width * x - self.width * x_c + self.length * y -\
                self.length * y_c) * np.cos(theta) + (- self.length * x +\
                self.length * x_c + self.width * y - self.width * y_c) *\
                np.sin(theta)) / (self.length * self.width)))

        return abs(value)

    def __loss_function(self, state, coordinates):
        """Evaluate the loss function.

        Arguments:
            state (float[]): rectangle state [x_c, y_c, theta]
            coordinates (float[]): coordinates of points that belong to a rectangular perimeter
        """
        loss_function = 0

        for coordinate in coordinates:
            loss_function += self.__rectangle(coordinate[0], coordinate[1], state)

        return loss_function

    def find_initial_guess(self, coordinates, robots_pose):
        """Find the initial guess using a montecarlo-based approach.

        Argument:
            coordinates (float[]): points coordinatesn
        """
        # Get the box size
        box_params = rospy.get_param('box')
        box_length = box_params['length']
        box_width = box_params['width']
        max_length = max(box_length, box_width)    

        # Get info on the uncertainty area
        rospy.wait_for_service('uncertainty_area_get_info')
        info = rospy.ServiceProxy('uncertainty_area_get_info', GetUncertaintyAreaInfo)
        try:
            response = info(0)
        except rospy.ServiceException:
            pass
        unc_area = BoxGeometry(2 * max_length, max_length,\
                               [response.pose[0], response.pose[1]],\
                               response.pose[2])
        # Robot that found the box
        discoverer_id = response.discoverer_id
        discoverer_pose = robots_pose[discoverer_id]

        # Evaluate the segment between the two other robot
        indexes = [0, 1, 2]
        indexes.remove(discoverer_id)
        poses = [robots_pose[i] for i in indexes]

        segment_between = Segment([poses[0][0],poses[0][1]],\
                                  [poses[1][0],poses[1][1]])
        half_way = np.array(segment_between.point(0.5))
        segment_length = segment_between.length()

        # Find the length of the side on which the other robots are attached
        robot_radius = float(rospy.get_param('robot_radius'))
        effective_length = segment_length - 2 * robot_radius

        side_length = box_width
        if abs(effective_length - box_width) < abs(effective_length - box_length):
            side_length = box_length
            
        # Find an uncertainty area for the center of the box
        direction = np.array([np.cos(discoverer_pose[2]), np.sin(discoverer_pose[2])])
        line_half_way = Line(half_way.tolist(), (half_way + direction).tolist())
        side0 = Line(unc_area.vertex(0), unc_area.vertex(1))
        intersection = line_half_way.intersect(side0)
        center_guess = (np.array(intersection) + (side_length / 2) * direction).tolist()
        theta_guess = discoverer_pose[2]
        if side_length == box_width:
            theta_guess -= np.pi / 2

        # Estimate the initial guess
        min_value = float('inf')
        random.seed()
        theta = angle_normalization(theta_guess)
        for i in range(10000):
            x_c = random.gauss(center_guess[0], 0.01)
            y_c = random.gauss(center_guess[1], 0.01)

            estimated_state = [x_c, y_c, theta]

            new_value = self.__loss_function(estimated_state, coordinates)
            min_value = min(min_value, new_value)
            
            if new_value == min_value:
                self._state = estimated_state

        print estimated_state
        print center_guess
        print theta_guess

    def state_estimation(self, coordinates):
        """Evaluate the box state using a montecarlo-based approach.
        
        Argument:
            coordinates (float[]): points coordinates
        """
        random.seed()
        min_value = float('inf')

        # Params
        estimation_rate = rospy.get_param('estimation_rate')
        delta_linear = 0.02
        delta_theta = 0.01
        # Estimation
        for i in range(300):
            x_c = round(random.uniform(self._state[0]- delta_linear, self._state[0] + delta_linear), 2)
            y_c = round(random.uniform(self._state[1]- delta_linear, self._state[1] + delta_linear), 2)
            theta = round(angle_normalization(random.uniform(self._state[2] - delta_theta,\
                                                             self._state[2] + delta_theta)), 2)
            
            estimated_state = [x_c, y_c, theta]

            new_value = self.__loss_function(estimated_state, coordinates)
            min_value = min(min_value, new_value)
            
            if new_value == min_value:
                self._state = estimated_state

        return self._state

class BoxStatePublisher:
    """Publish the box state in a topic."""

    def __init__(self):
        """Initialize the object."""
        rospy.init_node('box_state_publisher')

        # Topics subscription
        topics_names = rospy.get_param('topics_names')
        for robot_index, item in enumerate(topics_names, start=0):
            rospy.Subscriber(item['irbumper'], RoombaIR, self.irsensors_callback, callback_args = robot_index)
            rospy.Subscriber(item['odom'], Odometry, self.pose_callback, callback_args = robot_index)
            
        self.robot_radius = float(rospy.get_param('robot_radius'))
        self.min_range = 0.169265 - self.robot_radius
        self.max_range = 0.218445 - self.robot_radius
        self.sensors_angles = rospy.get_param('sensors_angles')

        # Publish to the box_state topic
        self.state_pub = rospy.Publisher('box_state', BoxState, queue_size=50)
        
        # Service declaration
        self.release_estimation = False
        self.found_initial_guess = False
        rospy.Service('release_box_state', Empty, self.release_box_state)
        rospy.Service('hold_box_state', Empty, self.hold_box_state)
        rospy.Service('find_initial_guess', Empty, self.find_initial_guess)
        rospy.Service('is_initial_guess_found', IsInitialGuessFound, self.is_initial_guess_found)
        
        # Initialize the box state observer
        box_params = rospy.get_param('box')
        self.observer = BoxStateObserver(box_params['length'], box_params['width'])
        
        self.number_robots = len(topics_names)
        self.robots_state = [{'state':{}, 'irbumper':{}} for index in range(self.number_robots)]

        # Lock used to avoid concurrent access to robots_state
        self.robots_state_lock = threading.Lock()
        # Lock used to avoid concurrent access to release_estimation
        self.flag_lock = threading.Lock()

        # Node rate
        self.rate = rospy.get_param('estimation_rate')
        self.clock = rospy.Rate(self.rate)

        # Enable box state from gazebo
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.gazebo_box = [box_params['posx'], box_params['posy'], box_params['yaw']]

    def find_initial_guess(self, request):
        self.robots_state_lock.acquire()
        angle_keys = self.sensors_angles.keys()
        points = [self.point_coordinates(robot_index, key) \
                  for robot_index in range(self.number_robots) for key in angle_keys \
                  if self.robots_state[robot_index]['irbumper'][key] < self.max_range]

        robots_pose = [[self.robots_state[i]['state']['x'],\
                         self.robots_state[i]['state']['y'],\
                         self.robots_state[i]['state']['theta']] for i in range(3)]
        self.robots_state_lock.release()

        # Run Estimation process
        self.observer.find_initial_guess(points, robots_pose)
        
        # Initial guess found
        self.flag_lock.acquire()
        self.found_initial_guess = True        
        self.flag_lock.release()

    def release_box_state(self, request):
        print 'release box state'
        self.flag_lock.acquire()
        self.release_estimation = True
        self.flag_lock.release()

    def hold_box_state(self, request):
        self.flag_lock.acquire()
        self.release_estimation = False
        self.flag_lock.release()

    def is_initial_guess_found(self, request):
        self.flag_lock.acquire()
        is_found = self.found_initial_guess
        self.flag_lock.release()
        response = IsInitialGuessFoundResponse(is_found)
        return response

    def irsensors_callback(self, data, robot_index):
        """Update robots_state using data from IR sensor.
        
        Arguments:
            data (RoombaIR): data from IR sensor 
            robot (string): robot name 
        """
        self.robots_state_lock.acquire()

        # convert IR signal in meters
        delta_range = self.max_range - self.min_range
        robot_max_ir = 4095
        self.robots_state[robot_index]['irbumper'][data.header.frame_id] = self.max_range - delta_range / robot_max_ir * data.signal

        self.robots_state_lock.release()

    def pose_callback(self, data, robot_index):
        """Update robots_state using robot odometry.
        
        Arguments:
            data (Odometry): robot pose
            robot (string): robot name 
        """

        self.robots_state_lock.acquire()

        self.robots_state[robot_index]['state']['x'] = data.pose.pose.position.x
        self.robots_state[robot_index]['state']['y'] = data.pose.pose.position.y
        self.robots_state[robot_index]['state']['theta'] = quaternion_to_yaw(data.pose.pose.orientation)

        self.robots_state_lock.release()

    def gazebo_callback(self, data):
        box = data.pose[4]
        position = box.position
        orientation = box.orientation

        self.gazebo_box[0] = position.x
        self.gazebo_box[1] = position.y
        self.gazebo_box[2] = quaternion_to_yaw(orientation)

        
    def point_coordinates(self, robot_index, angle_key):
        """Convert the <angle_key>-th range measure of the <robot_index>-th robot
        from the robot-fixed reference frame to a ground-fixed reference frame
        using an SE2 matrix.

        p_g = T * p_b
        
            | cos(theta) -sin(theta) x_o |   
        T = | sin(theta)  cos(theta) y_o |
            |      0          0       1  |
        
        where theta is the robot yaw angle and [x_o, y_0]' is the robot pose

        Arguments:
            robot (string): robot name
            angle_key (string): ir sensor key representing its angular position in the body-fixed reference frame
        """
        
        x_robot = self.robots_state[robot_index]['state']['x']
        y_robot = self.robots_state[robot_index]['state']['y']
        theta_robot = self.robots_state[robot_index]['state']['theta']
        sensor_angle = self.sensors_angles[angle_key]

        local_radius = self.robot_radius + self.robots_state[robot_index]['irbumper'][angle_key]
        x_global = round(x_robot + local_radius * np.cos(sensor_angle + theta_robot), 2)
        y_global = round(y_robot + local_radius * np.sin(sensor_angle + theta_robot), 2)

        point = [x_global, y_global]
        return point

    def publish_box_state(self):
        """ Publish box state in a topic."""
        
        self.flag_lock.acquire()
        flag = self.release_estimation
        self.flag_lock.release()

        if flag:
            self.robots_state_lock.acquire()
            angle_keys = self.sensors_angles.keys()
            points = [self.point_coordinates(robot_index, key) \
                      for robot_index in range(self.number_robots) for key in angle_keys \
                      if self.robots_state[robot_index]['irbumper'][key] < self.max_range]
            self.robots_state_lock.release()
            
            # Run Estimation process
            state = self.observer.state_estimation(points)

            # Debugging
            # path = ?
            # with open(path + 'points.csv', 'a') as csvfile:
            #      fieldnames = ['x', 'y']
            #      writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #      for point in points:
            #          writer.writerow({'x':point[0], 'y':point[1]})
 
            # with open(path + '/center.csv', 'a') as csvfile:
            #     fieldnames = ['x', 'y']
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #     writer.writerow({'x':state[0], 'y':state[1]})
            # with open(path + '/center_gazebo.csv', 'a') as csvfile:
            #     fieldnames = ['x', 'y']
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #     writer.writerow({'x':self.gazebo_box[0], 'y':self.gazebo_box[1]})
        else:
            state = self.observer.state

        msg = BoxState()
        msg.header.stamp = rospy.Time.now()
        msg.x = state[0]
        msg.y = state[1]
        msg.theta = state[2]

        # Enable box state from gazebo
        # msg = BoxState()
        # msg.header.stamp = rospy.Time.now()
        # msg.x = self.gazebo_box[0]
        # msg.y = self.gazebo_box[1]
        # msg.theta = self.gazebo_box[2]

        if not rospy.is_shutdown():        
            self.state_pub.publish(msg)

    def run(self):
        """Run main method of the class."""
        
        # do we need some sleep here?!?

        while not rospy.is_shutdown():

            self.publish_box_state()

            self.clock.sleep()

def main():
    """Initialize BoxStatePublisher."""
    box_state_publisher = BoxStatePublisher()
    try:
        box_state_publisher.run()
    except rospy.ROSInterruptException:
        pass
