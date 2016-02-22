import rospy
import utils
import smach
from smach import StateMachine, Iterator, CBState
from subscriber import Subscriber
from point_to_point import PointToPoint
from irobotcreate2.msg import RoombaIR
from nav_msgs.msg import Odometry
from cooperative_transport.msg import BoxState
from geometry_msgs.msg import Twist

class GoToPoint(smach.State):
    def __init__(self, controller, robot_state, main_controller):
        smach.State.__init__(self, outcomes=['success'], 
                             input_keys=['goal', 'max_forward', 'max_angular'])
        self.clock = rospy.Rate(100)
        self.robot_state = robot_state
        self.controller = controller
        self.main_controller = main_controller

    def execute(self, inputs):
        self.controller.goal_point(inputs.goal)
        
        done = False

        while not done:
            x = self.robot_state.data.pose.pose.position.x
            y = self.robot_state.data.pose.pose.position.y
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            forward_v = self.robot_state.data.twist.twist.linear.x

            done, forward_v, angular_v = self.controller.control_law([x, y, theta], forward_v)

            self.main_controller.set_control(forward_v, angular_v)

            self.clock.sleep()

        return 'success'

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

        if self.controller_index != 0:
            return

        while not self.are_callbacks_ready():
            rospy.sleep(1)
            continue

        # !!
        p2p_ctl = PointToPoint(self.max_forward_v, self.max_angular_v)
        this_robot = self.robots_state[self.controller_index]

        # top level state machine
        sm_cooperative_transport = StateMachine(outcomes=['transport_successful'])
        sm_cooperative_transport.userdata.path = [[-2, -3], [-2, -2], [-3, -2], [-3, -3]]

        with sm_cooperative_transport:
            box_approach = Iterator(outcomes=['success'], input_keys=[], output_keys=[],
                                   it=sm_cooperative_transport.userdata.path, it_label='goal',
                                   exhausted_outcome='success')

            with box_approach:
                sm_box_approach = StateMachine(outcomes=['success', 'continue'],input_keys=['goal'])

                with sm_box_approach:
                    state = GoToPoint(main_controller=self, controller=p2p_ctl, robot_state=this_robot)
                    StateMachine.add('GO_TO_POINT', state, transitions={'success':'continue'})
                    
                Iterator.set_contained_state('BOX_APPROACH_SM', sm_box_approach, loop_outcomes=['continue'])

            StateMachine.add('BOX_APPROACH', box_approach, transitions={'success':'transport_successful'})

        sm_cooperative_transport.execute()
        
def main(controller_index):
    # wait for gazebo startup
    #rospy.sleep(10)

    # start the controller
    try:
        controller = Controller(controller_index)
        controller.run()
    except rospy.ROSInterruptException:
        pass
