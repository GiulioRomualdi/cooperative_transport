import rospy
from threading import Lock
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState

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
        #self.data = data

    
class Controller:
    """Main controller for cooperative transport."""

    def __init__(self, controller_index):

        # Init the node
        rospy.init_node('controller' +  str(controller_index))

        # Get the topics names for all the robots
        topics_names = rospy.get_param('topics_names')

        # Subscribe to robots odometry
        self.robots_state = [Subscriber(names['odom'], Odometry) for names in topics_names]
        
        # Subscribe to irbumper topic
        self.irbumper = Subscriber(topics_names[controller_index]['irbumper'], RoombaIR)

        # Subscribe to box estimation topic
        self.box = Subscriber('box_state', BoxState)

        # Controller rate
        self.clock = rospy.Rate(1)

    def are_callbacks_ready(self):
        condition = True

        for robot_state in self.robots_state:
            condition = condition and robot_state.is_ready

        condition = condition and self.irbumper.is_ready

        return condition

    def run(self):

        while not rospy.is_shutdown():

            print(self.are_callbacks_ready())

            self.clock.sleep()
        

def main(controller_index):
    try:
        controller = Controller(controller_index)
        controller.run()
    except rospy.ROSInterruptException:
        pass
