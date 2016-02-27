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

        with box_docking:
        ##########################################################################################
        # BOX DOCKING STATE MACHINE
        ##########################################################################################
        #
            box_approach = Iterator(outcomes=['approach_ok'],\
                                    input_keys=[],\
                                    output_keys=[],\
                                    it= lambda:[item for item in box_docking.userdata.path],\
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
                             remapping={'plan_trajectory_out':'path'})

            StateMachine.add('BOX_APPROACH',\
                             box_approach,\
                             transitions={'approach_ok':'BOX_FINE_APPROACH'})
 
            box_fine_approach = BoxFineApproach(robots_state[controller_index], controller_index, set_control)
            StateMachine.add('BOX_FINE_APPROACH',box_fine_approach,\
                             transitions={'fine_approach_ok':'SYNCHRONIZER'})

            synchronizer = Synchronizer(controller_index)
            StateMachine.add('SYNCHRONIZER',synchronizer,\
                             transitions={'synchronization_ok':'docking_ok'})
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

        # Subscribe and publish to the topic 'turn_state'
        # Robots use this topic to wait for their turn to approach to the box
        self.turn_state_pub = rospy.Publisher('wait', TaskState, queue_size=50)
        self.turn_state_sub = rospy.Subscriber('wait', TaskState, self.callback)
        self.turn_state = set()

        # Lock used to protect the variable turn_state
        self.lock = Lock()

        # State clock
        self.clock = rospy.Rate(1)

    def callback(self, data):
        """Update the turn_state variable.

        Arguments:
        data (TaskState): last task state received by the node.
        """
        if data.task_name == 'wait_for_turn':
            self.lock.acquire()
            # Add robot_id to the turn_state list for every new robot
            self.turn_state.add(data.robot_id)
            self.lock.release()
        
    def turn_number(self):
        """Return the number of robots in the turn_state list."""
        self.lock.acquire()
        number = len(self.turn_state)
        self.lock.release()
        return number
        
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        # Wait for turn
        while self.turn_number() != self.controller_index:
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
        smach.State.__init__(self, 
                             outcomes=['path_found','plan_failed'], 
                             output_keys=['plan_trajectory_out'])
        
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
        # Tolerance and robot radius are taken into account
        robot_radius = rospy.get_param('robot_radius')
        tolerance = - np.array(response.normal) * (0.06 + robot_radius)
        start_point = [this_robot.x, this_robot.y]
        goal_point = (np.array(response.point) + tolerance).tolist()
        state, path = self.planner.plan(start_point, goal_point)

        # the first point in the path is the actual robot position
        path.pop(0)

        if state:
            userdata.plan_trajectory_out = path
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

        # Tuning
        self.kp = 2
        self.tolerance = 0.017

        # State clock
        self.clock = rospy.Rate(200)
        
    def execute(self, userdata):
        # Read the sensors
        x = self.robot_state.data.pose.pose.position.x
        y = self.robot_state.data.pose.pose.position.y
        theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

        # Set point
        reference_input = np.arctan2(userdata.goal[1] - y, userdata.goal[0] - x)  
        error = reference_input - theta
        
        while abs(error) > self.tolerance:
            # Read the sensors
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

            # Evaluate the control effort
            angular_v = proportional_control(self.kp, reference_input, theta, self.max_angular_v)
            angular_v *= -1 # Gazebo Bug

            # Evaluate error
            error = reference_input - theta

            # Set the control
            self.set_control(0, angular_v)

            self.clock.sleep()

        # Stop the robot
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

        # Stop the robot
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

        # Topic subscription
        topic_name = rospy.get_param('topics_names')[self.controller_index]
        rospy.Subscriber(topic_name['irbumper'], RoombaIR, self.irsensors_callback)
        self.ir_data_lock = Lock()
        self.ir_data = {}

        # State clock
        self.clock = rospy.Rate(200)

        # Tuning
        self.kp = 2
        self.linear_tolerance = 4000
        self.angle_tolerance = 0.017

    def max_ir_data(self):
        """Return the maximum value in ir_data.values()"""
        self.ir_data_lock.acquire()
        max_value = max(self.ir_data.values())
        self.ir_data_lock.release()

        return max_value

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
        userdata: inputs and outputs of the fsm state.
        """
        # Let the other robots know that it's their turn
        pub = rospy.Publisher('wait', TaskState, queue_size=50)
        msg = TaskState()
        msg.robot_id = self.controller_index
        msg.task_name = 'wait_for_turn'
        pub.publish(msg)
        pub.publish(msg)
        pub.publish(msg)

        # Get normal from service box_get_docking_point
        rospy.wait_for_service('box_get_docking_point')
        docking = rospy.ServiceProxy('box_get_docking_point', BoxGetDockingPoint)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass

        # Read the sensors
        theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

        # Set point
        angle_reference = np.arctan2(response.normal[1], response.normal[0])
        angle_error = angle_reference - theta

        # Align the robot to the normal pointing inward
        while abs(angle_error) > self.angle_tolerance:
            # Read the sensors
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)

            # Evaluate the control
            angular_v = proportional_control(self.kp, angle_reference, theta, self.max_angular_v)
            angular_v *= -1 # Gazebo Bug

            # Evaluate error
            angle_error = angle_reference - theta
            
            # Set the control
            self.set_control(0, angular_v)

            self.clock.sleep()

        self.set_control(0, 0)
            
        # Move the robot as close to the box as possible
        linear_v = 0.01

        while self.max_ir_data() < self.linear_tolerance:
            self.set_control(linear_v, 0)

        self.set_control(0,0)

        return 'fine_approach_ok'


class Synchronizer(smach.State):
    """State of the fsm in which the robot synchronize themsel.

    Outcomes:
    fine_approach_ok: the robot moved successfully
    
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
        smach.State.__init__(self, outcomes=['synchronization_ok'])

        self.controller_index = controller_index 

        self.robots_state_pub = rospy.Publisher('wait', TaskState, queue_size=50)
        self.robots_state_sub = rospy.Subscriber('wait', TaskState, self.callback)
        self.robots_state = set()

        self.lock = Lock()
        
    def callback(self, data):
        """Update ir_data using data from IR sensor.
        
        Arguments:
        data (RoombaIR): data from IR sensor 
        """
        if data.task_name == 'synchronization':
            self.lock.acquire()
            self.robots_state.add(data.robot_id)
            self.lock.release()

    def get_dimension(self):
        """Return the dimension of robots_state."""
        self.lock.acquire()
        number = len(self.robots_state)
        self.lock.release()
        return number
    
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """

        msg = TaskState()
        msg.robot_id = self.controller_index
        msg.task_name = 'synchronization'
        
        while self.get_dimension() != 3:
            self.robots_state_pub.publish(msg)
            rospy.sleep(1)

        self.robots_state_pub.publish(msg)
        self.robots_state_pub.publish(msg)
        self.robots_state_pub.publish(msg)

        rospy.sleep(1)

        return 'synchronization_ok'
