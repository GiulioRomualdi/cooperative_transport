import rospy
import utils
from point_to_point import PointToPoint
from threading import Lock
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState
from geometry_msgs.msg import Twist

class Subscriber:
    """Topic subscription with locks."""

    def __init__(self, topic_name, msg_type):

        rospy.Subscriber(topic_name, msg_type, self.callback)
        self._is_ready = False
        self.lock = Lock()

    @property
    def is_ready(self):
        self.lock.acquire()
        value = self._is_ready
        self.lock.release()
        return value
        
    @is_ready.setter
    def is_ready(self, value):
        self.lock.acquire()
        self._is_ready = value
        self.lock.release()

    @property
    def data(self):
        self.lock.acquire()
        value = self._data
        self.lock.release()
        return value
        
    @data.setter
    def data(self, value):
        self.lock.acquire()
        self._is_ready = value
        self.lock.release()

    def callback(self, data):
        self.is_ready = True
        self.data = data

    
class Controller:
    """Main controller for cooperative transport."""

    def __init__(self, controller_index):

        # Init the node
        self.controller_index = controller_index
        rospy.init_node('controller' +  str(controller_index))

        # Get the topics names for all the robots
        topics_names = rospy.get_param('topics_names')

        # Subscribe to robots odometry
        self.robots_state = [Subscriber(names['odom'], Odometry) for names in topics_names]
        
        # Subscribe to irbumper topic
        self.irbumper = Subscriber(topics_names[controller_index]['irbumper'], RoombaIR)

        # Subscribe to box estimation topic
        self.box = Subscriber('box_state', BoxState)

        # Publish to cmdvel
        self.cmdvel_pub = rospy.Publisher(topics_names[controller_index]['cmdvel'], Twist, queue_size=50)

        # Controller rate
        self.clock = rospy.Rate(100)

        #
        self.max_forward_v = 0.5
        self.max_angular_v = 2

    def are_callbacks_ready(self):
        condition = True

        for robot_state in self.robots_state:
            condition = condition and robot_state.is_ready

        condition = condition and self.irbumper.is_ready

        return condition

    def set_control(self, forward_v, angular_v):
        twist = Twist()
        twist.linear.x = forward_v
        twist.angular.z = angular_v

        self.cmdvel_pub.publish(twist)

    def run(self):

        this_robot = self.robots_state[self.controller_index]

        point2point_ctl = PointToPoint(self.max_forward_v, self.max_angular_v)
        point2point_ctl.goal_point([-3,-2])

        while not rospy.is_shutdown():

            if not self.are_callbacks_ready():
                self.clock.sleep()
                continue

            data = this_robot.data
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            theta = utils.quaternion_to_yaw(data.pose.pose.orientation)
            forward_v = data.twist.twist.linear.x

            if self.controller_index == 0:
                is_controller_done, forward_v, angular_v = point2point_ctl.control_law([x, y, theta], forward_v)
                self.set_control(forward_v, angular_v)

            self.clock.sleep()
        
def main(controller_index):
    rospy.sleep(10)
    try:
        controller = Controller(controller_index)
        controller.run()
    except rospy.ROSInterruptException:
        pass
