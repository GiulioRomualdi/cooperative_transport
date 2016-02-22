import rospy
import utils
import smach_ros
from subscriber import Subscriber
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState
from geometry_msgs.msg import Twist
from statemachine import construct_sm

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

        # Publish to cmdvel
        self.cmdvel_pub = rospy.Publisher(topics_names[controller_index]['cmdvel'], Twist, queue_size=50)

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

        # wait for callbacks
        while not self.are_callbacks_ready():
            rospy.sleep(1)
            continue

        # create top level state machine
        state_machine = construct_sm(self.robots_state, self.set_control, self.controller_index)

        # create and start introspection server
        sis = smach_ros.IntrospectionServer('cooperative_transport' + str(self.controller_index), 
                                            state_machine, 
                                            'COOPERATIVE_TRANSPORT' + str(self.controller_index))
        sis.start()

        # start state machine
        state_machine.execute()

def main(controller_index):
    # wait for gazebo startup
    #rospy.sleep(10)

    # start the controller
    try:
        controller = Controller(controller_index)
        controller.run()
    except rospy.ROSInterruptException:
        pass
