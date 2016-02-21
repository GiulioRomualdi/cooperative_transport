from __future__ import division
import random
import rospy
import threading
import numpy as np
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState
from cooperative_transport.utils import quaternion_to_yaw
from std_srvs.srv import Empty

class BoxStateObserver:
    """Estimate box state using noisy coordinates of points that belong to rectangle perimeter

    Attributes:
    length (float): box length in meters
    width (float): box width in meters
    state (float[]): box state [x_c, y_c, theta]
    """

    def __init__(self, length, width, x_0, y_0, theta_0):
        """Initialize the object.
        
        Arguments:
        length (float): box length in meters
        width (float): box width in meters
        x_0 (float): initial x coordinate of the box
        y_0 (float): initial y coordinate of the box
        theta_0 (float): initial yaw angle
        """
        self.length = length
        self.width = width
        self.state = [x_0, y_0, theta_0]

    def __rectangle (self, x, y, state):
        """Rectangle equation based on Lame' curve.

        Arguments:
        x (float): x coordinate of a point belongs to rectangle perimeter 
        y (float): y coordinate of a point belongs to rectangle perimeter 
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

    def __loss_function(self, state, data):
        """Loss function evaluation

        Arguments:
        state (float[]): rectangle state [x_c, y_c, theta]
        data (float[]): points coordinates that belong ti rectangle perimeter
        """
        loss_function = 0
        coordinates = data

        for coordinate in coordinates:
            loss_function += self.__rectangle(coordinate[0], coordinate[1], state)

        return loss_function
        
    def state_estimation(self, data):
        """Evaluation of box state using Monte-Carlo method.
        
        Argument:
        data (float[]): points coordinates that belong ti rectangle perimeter
        """
        random.seed()
        min_value = float('inf')
        
        for i in range(50):
            x_c = random.gauss(self.state[0], 0.02)
            y_c = random.gauss(self.state[1], 0.02)
            theta = random.gauss(self.state[2], 0.002)
            
            estimated_state = [x_c, y_c, theta]
            
            new_value = self.__loss_function(estimated_state, data)
            min_value = min(min_value, new_value)
            
            if new_value == min_value:
                self.state = estimated_state

        return self.state

class BoxStatePublisher:
    """Publish the box state in a topic.

    Attributes:
    robot_radius (float): robot radius in meters
    sensors_angles (dict): keys sensors name, value sensors angle position (radians) 
    service_enabled (boolean): flag that allows to start box estimation
    observer (BoxStateObserver): BoxStateObserver object
    robots_state (dict): state and IR sensors data of each robot are saved
    """

    def __init__(self):
        """Initialize the object."""
        rospy.init_node('box_state_publisher')

        # Topics subscription
        topics_names = rospy.get_param('topics_names')
        for robot_index, item in enumerate(topics_names, start=0):
            rospy.Subscriber(item['irbumper'], RoombaIR, self.irsensors_callback, callback_args = robot_index)
            rospy.Subscriber(item['odom'], Odometry, self.pose_callback, callback_args = robot_index)
            
        self.robot_radius = float(rospy.get_param('robot_radius'))
        self.sensors_angles = rospy.get_param('sensors_angles')

        # Publish to the box_state topic
        self.state_pub = rospy.Publisher('box_state', BoxState, queue_size=50)
        
        # Service declaration
        self.service_enabled = False
        rospy.Service('start_box_estimation', Empty, self.start_box_estimation)
        rospy.Service('hold_latest_estimate', Empty, self.hold_latest_estimate)
        
        box_params = rospy.get_param('box')
        self.observer = BoxStateObserver(box_params['length'], box_params['width'], box_params['posx'], box_params['posy'], box_params['yaw'])
        
        self.number_robots = len(topics_names)
        self.robots_state = [{'state':{}, 'irbumper':{}} for index in range(self.number_robots)]

        # Lock used to avoid contemporary access to robots_state 
        self.robots_state_lock = threading.Lock()

        # Node rate
        self.clock = rospy.Rate(100)

    def start_box_estimation(self):
        self.service_enabled = True

    def hold_latest_estimate(self):
        self.service_enabled = False

    def irsensors_callback(self, data, robot_index):
        """Update robots_state using data that come from IR sensor.
        
        Arguments:
        data (RoombaIR): data from IR sensor 
        robot (string): robot name 
        """
        self.robots_state_lock.acquire()

        # IR signal conversion into meters 
        max_range = 0.233045
        scale_factor =  max_range - self.robot_radius
        robot_max_ir = 4095
        self.robots_state[robot_index]['irbumper'][data.header.frame_id] = data.signal * scale_factor / robot_max_ir

        self.robots_state_lock.release()

    def pose_callback(self, data, robot_index):
        """Update robots_state using robot's pose.
        
        Arguments:
        data (Odometry): robot pose
        robot (string): robot name 
        """

        self.robots_state_lock.acquire()

        self.robots_state[robot_index]['state']['x'] = data.pose.pose.position.x
        self.robots_state[robot_index]['state']['y'] = data.pose.pose.position.y
        self.robots_state[robot_index]['state']['theta'] = quaternion_to_yaw(data.pose.pose.orientation)

        self.robots_state_lock.release()

    def point_coordinates(self, robot_index, angle_key):
        """Convert point expressed in local coordinate (robot frame) into global frame using roto-translation matrix (SE(2))
        p_g = T * p_l
        
            | cos(theta) -sin(theta) x_o |   
        T = | sin(theta)  cos(theta) y_o |
            |      0          0       1  |
        
        Where theta is robot yaw angle, x_o and y_o are robot pose

        Arguments:
        robot (string): robot name
        angle_key (string): ir sensor angle in local frame
        """
        
        x_robot = self.robots_state[robot_index]['state']['x']
        y_robot = self.robots_state[robot_index]['state']['y']
        theta_robot = self.robots_state[robot_index]['state']['theta']
        sensor_angle = self.sensors_angles[angle_key]

        local_radius = self.robot_radius + self.robots_state[robot_index]['irbumper'][angle_key]
        x_global = x_robot + local_radius * np.cos(sensor_angle + theta_robot)
        y_global = y_robot + local_radius * np.sin(sensor_angle + theta_robot)

        point = [x_global, y_global]
        return point

    def publish_box_state(self):
        """ Publish box state in a topic."""
        self.robots_state_lock.acquire()

        angle_keys = self.sensors_angles.keys()

        points = [self.point_coordinates(robot_index, key) for robot_index in range(self.number_robots) for key in angle_keys \
                       if self.robots_state[robot_index]['irbumper'][key] != 0]

        self.robots_state_lock.release()

        # Run Estimation process
        state = self.observer.state_estimation(points)

        msg = BoxState()
        msg.header.stamp = rospy.Time.now()
        msg.x = state[0]
        msg.y = state[1]
        msg.theta = state[2]

        if not rospy.is_shutdown():        
            self.state_pub.publish(msg)
        

    def run(self):
        """Run iterative process, main method of the class."""
        rospy.sleep(10)
        while not rospy.is_shutdown():

            if not self.service_enabled:
                continue
                
            self.publish_box_state()

            self.clock.sleep()


def main():
    """Initialize BoxStatePublisher."""
    box_state_publisher = BoxStatePublisher()
    box_state_publisher.run()
