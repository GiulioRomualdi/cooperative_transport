import rospy
import smach
import numpy as np
import utils
from subscriber import Subscriber
from irobotcreate2.msg import RoombaIR
from planner import Planner, RectangularObstacle, CircularObstacle
from control.point_to_point import PointToPoint
from control.proportional_control import proportional_control
from smach import StateMachine, Iterator
from cooperative_transport.msg import TaskState
from threading import Lock
from cooperative_transport.srv import BoxGetDockingPoint
from cooperative_transport.srv import BoxGetDockingPointResponse
from cooperative_transport.srv import BoxGetDockingPointRequest

def construct_sm(controller_index, robots_state, boxstate, set_control):
    """Construct the top level state machine for cooperative transport.

    Arguments:
    controller_index (int): index of the robot
    robots_state (Subscriber[]): list of Subscribers to robots odometry
    irbumper (Subscriber): Subscriber to this robot ir bumper
    boxstate (Subscriber): Subscriber to the box state estimation
    set_control (function): function that publish a twist
    """
    sm = StateMachine(outcomes=['transport_ok', 'transport_failed'])

    with sm:    
    ##############################################################################################
    # TOP LEVEL STATE MACHINE
    ##############################################################################################
    #
        box_docking = StateMachine(outcomes=['docking_ok', 'docking_failed'])
        box_docking.userdata.path = []

        with box_docking:
        ##########################################################################################
        # BOX DOCKING STATE MACHINE
        ##########################################################################################
        #
            box_approach = Iterator(outcomes=['approach_ok'],\
                                    input_keys=['path'],\
                                    output_keys=[],\
                                    it=box_docking.userdata.path,\
                                    it_label='goal',\
                                    exhausted_outcome='approach_ok')

            with box_approach:
            ######################################################################################
            # BOX APPROACH ITERATOR
            ######################################################################################
            #
                box_approach_container = StateMachine(outcomes=['approach_continue'],\
                                                      input_keys=['goal'])
                with box_approach_container:
                ###################################################################################
                # BOX APPROACH CONTAINER
                ###################################################################################
                #
                    alignment = Alignment(robots_state[controller_index], set_control)
                    StateMachine.add('ALIGNMENT',\
                                     alignment,\
                                     transitions={'alignment_ok':'GO_TO_POINT'})
                    
                    go_to_point = GoToPoint(robots_state[controller_index], set_control)
                    StateMachine.add('GO_TO_POINT',\
                                     go_to_point,\
                                     transitions={'point_reached':'approach_continue'})
                    

                #
                ###################################################################################
                
                Iterator.set_contained_state('CONTAINER_STATE', 
                                             box_approach_container, 
                                             loop_outcomes=['approach_continue'])

            #
            ######################################################################################

            

            wait_for_turn = WaitForTurn(controller_index)
            StateMachine.add('WAIT_FOR_TURN',\
                             wait_for_turn,\
                             transitions={'my_turn':'PLAN_TRAJECTORY'})

            plan_trajectory = PlanTrajectory(controller_index, robots_state, boxstate)
            StateMachine.add('PLAN_TRAJECTORY', plan_trajectory,\
                             transitions={'path_found':'BOX_APPROACH',\
                                          'plan_failed':'docking_failed'},
                             remapping={'path':'path'})

            StateMachine.add('BOX_APPROACH',\
                             box_approach,\
                             transitions={'approach_ok':'BOX_FINE_APPROACH'})

            box_fine_approach = BoxFineApproach(robots_state[controller_index], controller_index, set_control)
            StateMachine.add('BOX_FINE_APPROACH',box_fine_approach,\
                             transitions={'fine_approach_ok':'docking_ok'})


        #
        ##########################################################################################

        StateMachine.add('BOX_DOCKING', box_docking,\
                         transitions={'docking_failed':'transport_failed',\
                                      'docking_ok':'transport_ok'})
    #
    ##############################################################################################
            
    return sm
        
class WaitForTurn(smach.State):
    """State of the fsm in which the robot waits its turn to approach the box.

    Outcomes:
    my_turn : time for the robot to approach the box

    Inputs: 
    none
    
    Outputs: 
    none
    """
    def __init__(self, controller_index):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        """
        smach.State.__init__(self, outcomes=['my_turn'])
        
        self.controller_index = controller_index

        # Subscribe and publish to the topic 'task_state'
        # Robots use this topic to wait for their turn to approach to the box

        self.turn_state_pub = rospy.Publisher('task_state', TaskState, queue_size=50)
        self.turn_state_sub = rospy.Subscriber('task_state', TaskState, self.callback)
        self.turn__state = []

        # Lock used to protect the variable turn_state
        self.lock = Lock()

        # State clock
        self.clock = rospy.Rate(1)

    def callback(data):
        """Update the turn_state variable.

        Arguments:
        data (TaskState): last task state received by the node.
        """
        self.lock.acquire()
        # Add robot_id to the turn_state list for every new robot
        if not data.robot_id in self.task_state:
            self.task_state.append(data.robot_id)
        self.lock.release()
        
    def turn_number(self):
        """Return the number of robots in the turn_state list."""
        self.lock.acquire()
        number = len(self.task_state)
        self.lock.release()

        return number
        
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        # Wait for turn
        if self.controller_index == 0:
            return 'my_turn'

        while self.turn_number != self.controller_index:
            self.clock.sleep()

        return 'my_turn'

class PlanTrajectory(smach.State):
    """State of the fsm in which the robot find its path to the box.

    Outcomes:
    path_found: a path was found
    plan_failed: the planning failed
    
    Inputs:
    none

    Outputs:
    path: the path found by the planning algorithm
    """
    def __init__(self, controller_index, robots_state, boxstate):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        boxstate (Subscriber): Subscriber to box state
        """
        smach.State.__init__(self, outcomes=['path_found','plan_failed'], output_keys=['path'])
        
        self.controller_index = controller_index
        self.robots_state = robots_state
        self.boxstate = boxstate
        
    def execute(self, userdata):

        # Initialize the planner
        lower_bound = rospy.get_param('planner_lower_bound')
        upper_bound = rospy.get_param('planner_upper_bound')
        self.planner = Planner(lower_bound, upper_bound)

        # Add obstacles for the neighbor robots
        robot_radius = rospy.get_param('robot_radius')
        for robot_index,robot_state in enumerate(self.robots_state):
            if robot_index != self.controller_index:

                x_robot = robot_state.data.pose.pose.position.x
                y_robot = robot_state.data.pose.pose.position.y
                obstacle = CircularObstacle(robot_radius, x_robot, y_robot , robot_radius)

                self.planner.add_obstacle(obstacle)
            
        # Add obstacle for the box
        box_parameters = rospy.get_param('box')
        length = box_parameters['length']
        width = box_parameters['width']
        x_box = self.boxstate.data.x
        y_box = self.boxstate.data.y
        theta_box = self.boxstate.data.theta
        obstacle = RectangularObstacle(length, width, x_box, y_box, theta_box, robot_radius)
        self.planner.add_obstacle(obstacle)

        # Get docking point from service box_get_docking_point
        rospy.wait_for_service('box_get_docking_point')
        docking = rospy.ServiceProxy('box_get_docking_point', BoxGetDockingPoint)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass
            
        # Plan trajectory
        this_robot = self.robots_state[self.controller_index].data.pose.pose.position

        tolerance = - np.array(response.normal) * 0.04 

        start_point = [this_robot.x, this_robot.y]
        goal_point = (np.array(response.point) + tolerance).tolist()

        state, path = self.planner.plan(start_point, goal_point)

        if state:
            userdata.path = path
            return 'path_found'

        else:
            return 'plan_failed'
        
class Alignment(smach.State):
    """State of the fsm in which the robot rotates in order to align
    itself with the line of sight between its center and the next goal.
    
    Outcomes:
    alignment_ok: the robot rotated successfully

    Inputs:
    goal (float[]): the goal position
    
    Outputs:
    none
    """
    def __init__(self, robot_state, set_control):
        """Initialize the state of the fsm.

        Arguments:
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['alignment_ok'], input_keys=['goal'])

        self.robot_state = robot_state
        self.set_control = set_control

        self.max_angular_v = float(rospy.get_param('max_angular_v'))
        self.kp = 10
        self.tolerance = 0.1

        # State clock
        self.clock = rospy.Rate(100)
        
    def execute(self, userdata):
        # Read the sensors
        x = self.robot_state.data.pose.pose.position.x
        y = self.robot_state.data.pose.pose.position.y
        theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

        reference_input = np.arctan2(userdata.goal[1] - y, userdata.goal[0] - x)  
        error = reference_input - theta
        
        while abs(error) > self.tolerance:
            # Read the sensors
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

            # Evaluate the control
            angular_v = proportional_control(self.kp, reference_input, theta, self.max_angular_v)
            angular_v *= -1 # Gazebo Bug

            # Evaluate error
            error = reference_input - theta

            # Set the control
            self.set_control(0, angular_v)

            self.clock.sleep()

        self.set_control(0, 0)
        
        return 'alignment_ok'
        
class GoToPoint(smach.State):
    """State of the fsm in which the robot moves to a precise location.

    Outcomes:
    point_reached: the robot moved successfully
    
    Inputs:
    goal (float[]): the goal position

    Outputs:
    none
    """
    def __init__(self, robot_state, set_control):
        """Initialize the state of the fsm.

        Arguments:
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['point_reached'], input_keys=['goal'])
        
        self.robot_state = robot_state
        self.set_control = set_control

        # Initialize the point2point controller
        self.max_forward_v = float(rospy.get_param('max_forward_v'))
        self.max_angular_v = float(rospy.get_param('max_angular_v'))
        self.controller = PointToPoint(self.max_forward_v, self.max_angular_v)

        # State clock
        self.clock = rospy.Rate(100)

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        # Set the goal point
        self.controller.goal_point(userdata.goal)

        # Move the robot to the goal point
        done = False
        while not done:
            # Read the sensors
            x = self.robot_state.data.pose.pose.position.x
            y = self.robot_state.data.pose.pose.position.y
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            forward_v = self.robot_state.data.twist.twist.linear.x

            # Evaluate the control
            done, forward_v, angular_v = self.controller.control_law([x, y, theta], forward_v)
            angular_v *= -1 # Gazebo Bug

            # Set the control
            self.set_control(forward_v, angular_v)

            self.clock.sleep()

        self.set_control(0, 0)        

        return 'point_reached'

class BoxFineApproach(smach.State):
    """State of the fsm in which the robot approaches the box.

    Outcomes:
    fine_approach_ok: the robot moved successfully
    
    Inputs:
    none

    Outputs:
    none
    """
    def __init__(self, robot_state, controller_index, set_control):
        """Initialize the state of the fsm.

        Arguments:
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        controller_index (int): index of the robot
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['fine_approach_ok'])

        self.robot_state = robot_state
        self.set_control = set_control
        self.controller_index = controller_index 
        self.max_forward_v = float(rospy.get_param('max_forward_v'))
        self.max_angular_v = float(rospy.get_param('max_angular_v'))
        
        self.angle_tolerance = 0.01
        self.kp = 10

        self.linear_tolerance = 4000

        # Topic subscrption
        topic_name = rospy.get_param('topics_names')[self.controller_index]
        rospy.Subscriber(topic_name['irbumper'], RoombaIR, self.irsensors_callback)
        
        self.ir_data = {}
        
        # Lock used to avoid concurrent access to ir_data
        self.ir_data_lock = Lock()

        # Get normal from service box_get_docking_point
        rospy.wait_for_service('box_get_docking_point')
        docking = rospy.ServiceProxy('box_get_docking_point', BoxGetDockingPoint)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass
        
        self.angle_ref = np.arctan2(response.normal[1], response.normal[0])

        # State clock
        self.clock = rospy.Rate(100)

    def min_ir_data(self):
        """Return the minimum value in ir_data"""
        self.ir_data_lock.acquire()
        min_value = min(self.ir_data.values())
        self.ir_data_lock.release()
        
        return min_value

    def irsensors_callback(self, data):
        """Update ir_data using data from IR sensor.
        
        Arguments:
        data (RoombaIR): data from IR sensor 
        """
        self.ir_data_lock.acquire()

        self.ir_data[data.header.frame_id] = data.signal

        self.ir_data_lock.release()

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""

        theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
        
        # Parameter of control
        angle_error = self.angle_ref - theta
                
        while abs(angle_error) > self.angle_tolerance:
            # Read the sensors
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

            # Evaluate the control
            angular_v = proportional_control(self.kp, self.angle_ref, theta, self.max_angular_v)
            angular_v *= -1 # Gazebo Bug

            # Evaluate error
            angle_error = self.angle_ref - theta
            
            # Set the control
            self.set_control(0, angular_v)

            self.clock.sleep()

        self.set_control(0, 0)
            
        linear_v = 0.01

        while self.min_ir_data() < self.linear_tolerance:
            self.set_control(0, linear_v)
        
        return 'fine_approach_ok'
