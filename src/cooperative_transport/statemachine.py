import rospy
import smach
from subscriber import Subscriber
from planner import Planner
from point_to_point import PointToPoint
from smach import StateMachine, Iterator
from cooperative_transport.msg import TaskState.msg
from threading import Lock

def construct_sm(robots_state, set_control, controller_index):
    this_robot = robots_state[controller_index]

    sm = StateMachine(outcomes=['transport_ok', 'transport_failed'])
    with sm:    
        box_approaching = StateMachine(outcomes=['approach_ok', 'approach_failed'])
        move_box = StateMachine(outcomes=['moved_ok', 'moved_failed'])

        with box_approaching:
            StateMachine.add('WAIT_FOR_TURN', WaitForTurn(controller_index),\
                             transitions={'my_turn':'PLAN_TRAJECTORY'})

            StateMachine.add('PLAN_TRAJECTORY', PlanTrajectory(),\
                             transitions={'path_found':'TODO', 'plan_failed':'approach_failed'})
            
                        
        #with move_box:
        #   pass

        StateMachine.add('BOX_APPROACHING', box_approaching,\
                         transitions={'approach_failed':'transport_failed',\
                                      'approach_ok':'MOVE_BOX'})

        # StateMachine.add('MOVE_BOX', move_box,\
        #                transitions={'moved_failed':'transport_failed',\
        #                         'moved_ok':'transport_ok'})
            
    return sm
        
class WaitForTurn(smach.State):
    def __init__(self, controller_index):
        smach.State.__init__(self, outcomes=['my_turn', 'aborted'])
        
        self.controller_index = controller_index
        self.task_state_pub = rospy.Publisher('task_state', TaskState, queue_size=50)
        self.task_state_sub = rospy.Subscriber('task_state', TaskState, self.callback)
        self.task_state = []
        self.lock = Lock()

    def callback(data):
        self.lock.acquire()
        if not data.robot_id in self.task_state:
            self.task_state.append(data.robot_id)
        self.lock.release()
        
    def turn_number(self):
        self.lock.acquire()
        number = len(self.task_state)
        self.lock.release()

        return number
        
    def execute(self, userdata):
        while self.turn_number != self.controller_index:
            rospy.sleep(1)

        return 'my_turn'

class PlanTrajectory(smach.State):
    def __init__(self, robots_state, controller_index):
        smach.State.__init__(self, outcomes=['path_found','plan_failed'], output_keys=['path'])
        
        # Subscribe to box estimation topic
        box_state = Subscriber('box_state', BoxState)
        #!!!
        
        lower_bound = rospy.get_param('planner_lower_bound')
        upper_bound = rospy.get_param('planner_upper_bound')
        self.planner = Planner(lower_bound, upper_bound)
        

class GoToPoint(smach.State):
    def __init__(self, robot_state, set_control):
        smach.State.__init__(self, outcomes=['success'], input_keys=['goal'])
        
        self.clock = rospy.Rate(100)
        self.robot_state = robot_state
        self.controller = controller
        self.set_control = set_control
    
        self.max_forward_v = float(rospy.get_param('max_forward_v'))
        self.max_angular_v = float(rospy.get_param('max_angular_v'))

        self.controller = PointToPoint(self.max_forward_v, self.max_angular_v)

    def execute(self, userdata):
        self.controller.goal_point(userdata.goal)
        
        done = False

        while not done:
            x = self.robot_state.data.pose.pose.position.x
            y = self.robot_state.data.pose.pose.position.y
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            forward_v = self.robot_state.data.twist.twist.linear.x

            done, forward_v, angular_v = self.controller.control_law([x, y, theta], forward_v)

            self.set_control(forward_v, angular_v)

            self.clock.sleep()

        return 'success'
