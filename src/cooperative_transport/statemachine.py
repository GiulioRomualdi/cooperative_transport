import rospy
import smach
from point_to_point import PointToPoint
from smach import StateMachine, Iterator

def construct_sm(robots_state, set_control, controller_index):
    this_robot = robots_state[controller_index]

    sm = StateMachine(outcomes=['transport_ok', 'transport_failed'])
    sm.userdata.controller_index = controller_index
    with sm:    
        box_approaching = StateMachine(outcomes=['approach_ok', 'approach_failed'],\
                                       input_keys=['controller_index'])
        move_box = StateMachine(outcomes=['moved_ok', 'moved_failed'])

        with box_approaching:
            construct_box_approaching_sm()

        #with move_box:
        #   pass

        StateMachine.add('BOX_APPROACHING', box_approaching,\
                         transitions={'approach_failed':'transport_failed',\
                                      #'approach_ok':'MOVE_BOX'},
                                      'approach_ok':'transport_ok'},
                         remapping={'controller_index':'controller_index'})

        # StateMachine.add('MOVE_BOX', move_box,\
        #                transitions={'moved_failed':'transport_failed',\
        #                         'moved_ok':'transport_ok'})
            
    return sm
        
def construct_box_approaching_sm():
    StateMachine.add('WAIT_FOR_TURN', WaitForTurn(),\
                     transitions={'my_turn':'approach_ok','aborted':'approach_failed'},\
                     remapping={'controller_index':'controller_index'})


class WaitForTurn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['my_turn','aborted'], input_keys=['controller_index'])

    def execute(self, inputs):
        if inputs.controller_index == 0:
            return 'my_turn'
            
        return 'aborted'


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

    def execute(self, inputs):
        self.controller.goal_point(inputs.goal)
        
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
