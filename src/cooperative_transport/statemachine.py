import rospy
import smach
import utils
import numpy as np

# State Machine
from smach import StateMachine, Iterator, Sequence, State

# Threading
from threading import Lock

# Subscriber
from subscriber import Subscriber

# Planner 
from planner import Planner, RectangularObstacle, CircularObstacle

# Control
from control.point_to_point import PointToPoint
from control.proportional_control import proportional_control

# Msgs
from cooperative_transport.msg import TaskState, TimeSync
from irobotcreate2.msg import RoombaIR

# Services
from cooperative_transport.srv import BoxGetDockingPointPush
from cooperative_transport.srv import BoxGetDockingPointPushResponse
from cooperative_transport.srv import BoxGetDockingPointPushRequest
from cooperative_transport.srv import BoxGetDockingPointRotate
from cooperative_transport.srv import BoxGetDockingPointRotateResponse
from cooperative_transport.srv import BoxGetDockingPointRotateRequest

from std_srvs.srv import Empty

def construct_sm(controller_index, robots_state, boxstate, set_control):
    """Construct the top level state machine for cooperative transport.

    Arguments:
    controller_index (int): index of the robot
    robots_state (Subscriber[]): list of Subscribers to robots odometry
    boxstate (Subscriber): Subscriber to the box state estimation
    set_control (function): function that publish a twist
    """
    sm = StateMachine(outcomes=['transport_ok', 'transport_failed', 'step_ok'])

    with sm:
        rotate = Sequence(outcomes = ['sequence_ok', 'step_ok', 'rotation_not_needed'],
                            connector_outcome = 'step_ok')
        with rotate:
            Sequence.add('WAIT_ROTATION',\
                         Wait(controller_index, 'wait_for_turn_rotation'),\
                         transitions={'rotation_not_needed':'sequence_ok',\
                                      'step_ok':'PLAN_TRAJECTORY_ROTATION'})

            Sequence.add('PLAN_TRAJECTORY_ROTATION',\
                         PlanTrajectory(controller_index, robots_state,\
                                        boxstate, 'plan_for_rotation'),\
                         remapping={'plan_trajectory_out':'path'})

            Sequence.add('BOX_APPROACH_ROTATION',\
                         box_approach(rotate, robots_state,\
                                      controller_index, set_control))

            Sequence.add('ALIGNMENT_ROTATION',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_rotation'))

            Sequence.add('FINE_APPROACH_ROTATION',\
                         LinearMovement(robots_state[controller_index], \
                                            controller_index, set_control, \
                                            'fine_approach_rotation'))

            Sequence.add('SYNCHRONIZATION_BEFORE_ROTATION',
                         Wait(controller_index, 'synchronization_before_rotation'))

            Sequence.add('ALIGNMENT_BEFORE_ROTATION',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_before_rotation', boxstate))

            Sequence.add('ROTATE',\
                         Rotate(controller_index, robots_state[controller_index],\
                                boxstate, set_control))

            Sequence.add('PARTIAL_REVERSE_ROTATION',\
                         LinearMovement(robots_state[controller_index], \
                                            controller_index, set_control, \
                                            'partial_reverse'))

            Sequence.add('ALIGNMENT_AFTER_ROTATION',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_rotation'))

            Sequence.add('REVERSE_ROTATION',\
                         LinearMovement(robots_state[controller_index], \
                                            controller_index, set_control, \
                                            'reverse'))

        push = Sequence(outcomes = ['sequence_ok', 'step_ok'],
                                connector_outcome = 'step_ok')
        with push:

            Sequence.add('WAIT_PUSH',\
                         Wait(controller_index, 'wait_for_turn_push'))

            Sequence.add('PLAN_TRAJECTORY_PUSH',\
                         PlanTrajectory(controller_index, robots_state,\
                                        boxstate, 'plan_to_push'),\
                         remapping={'plan_trajectory_out':'path'})

            Sequence.add('BOX_APPROACH_PUSH',\
                         box_approach(push, robots_state,\
                                      controller_index, set_control))

            Sequence.add('ALIGNMENT_PUSH',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_push'))

            Sequence.add('FINE_APPROACH_PUSH',\
                         LinearMovement(robots_state[controller_index], \
                                            controller_index, set_control, \
                                            'fine_approach_push'))

            Sequence.add('SYNCHRONIZATION_PUSH',
                         Wait(controller_index, 'synchronization_before_pushing'))
            
            Sequence.add('PUSH_BOX_SM',
                         push_box_sm(robots_state, controller_index, set_control, boxstate))

        StateMachine.add('ROTATE', rotate,\
                         transitions={'sequence_ok':'PUSH',\
                                      'rotation_not_needed':'PUSH'})

        StateMachine.add('PUSH', push,\
                         transitions={'sequence_ok':'transport_ok'})
 
    return sm
        
def box_approach(sm, robots_state, controller_index, set_control):
    """ """
    box_approach = Iterator(outcomes=['step_ok'],\
                            input_keys=[],\
                            output_keys=[],\
                            it= lambda:[item for item in sm.userdata.path],\
                            it_label='goal',\
                            exhausted_outcome='step_ok')

    with box_approach:
        box_approach_container = StateMachine(outcomes=['approach_continue'],\
                                              input_keys=['goal'])
        with box_approach_container:
            robot_state = robots_state[controller_index]
            alignment = Alignment(robot_state, set_control, controller_index, 'iterator')
            StateMachine.add('ALIGNMENT',\
                             alignment,\
                             transitions={'alignment_ok':'GO_TO_POINT'})
            
            go_to_point = GoToPoint(robot_state, set_control)
            StateMachine.add('GO_TO_POINT',\
                             go_to_point,\
                             transitions={'point_reached':'approach_continue'})
                
        Iterator.set_contained_state('CONTAINER_STATE', 
                                     box_approach_container, 
                                     loop_outcomes=['approach_continue'])

    return box_approach

def push_box_sm(robots_state, controller_index, set_control, boxstate):
    """ """
    push_box_sm = StateMachine(outcomes=['sequence_ok'])

    with push_box_sm:
        alignment = Alignment(robots_state[controller_index], set_control,\
                              controller_index, 'alignment_before_push', boxstate)
        StateMachine.add('ALIGNMENT_BEFORE_PUSH', alignment,\
                         transitions={'alignment_ok':'PUSH_BOX'})
        
        push_box = PushBox(controller_index, boxstate, set_control)
        StateMachine.add('PUSH_BOX', push_box,\
                         transitions={'box_drifted':'ALIGNMENT_BEFORE_PUSH',\
                                      'box_at_goal':'sequence_ok'})
    return push_box_sm

class Wait(State):
    """State of the fsm in which the robot waits the other robots

    Outcomes:
    none

    Inputs: 
    none
    
    Outputs: 
    none
    """
    def __init__(self, controller_index, task_name):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        task_name (string) 
        """
        if task_name == 'wait_for_turn_rotation':
            State.__init__(self, outcomes=['step_ok','rotation_not_needed'])
        else:
            State.__init__(self, outcomes=['step_ok'])
        
        self.controller_index = controller_index
        self.task_name = task_name

        # Subscribe to the topic 'robots_common'
        rospy.Subscriber('robots_common', TaskState, self.callback)

        # Init robots_state
        self.robots_state = set()
        self.lock = Lock()

        # State clock
        self.clock = rospy.Rate(2)    

    def callback(self, data):
        """Update the robots_state variable.

        Arguments:
        data (TaskState): last task state received by the node.
        """
        if data.task_name == self.task_name:
            self.lock.acquire()
            # Add robot_id to the turn_state list for every new robot
            self.robots_state.add(data.robot_id)
            self.lock.release()
        
    def robots_state_length(self):
        """Return the number of robots in the robots_state set."""
        self.lock.acquire()
        number = len(self.robots_state)
        self.lock.release()
        return number
        
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        # Reset
        self.robots_state = set()
        
        if self.task_name == 'wait_for_turn_rotation':
            rospy.wait_for_service('box_get_docking_point_rotate')
            docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
          
            if not response.is_rotation_required:
                return 'rotation_not_needed'

 
        if self.task_name == 'wait_for_turn_rotation' or\
           self.task_name == 'wait_for_turn_push':
            
            if  self.task_name == 'wait_for_turn_push':
                # Clear docking point
                rospy.wait_for_service('clear_docking_point')
                clear_docking_point = rospy.ServiceProxy('clear_docking_point', Empty)
                try:
                    clear_docking_point()
                except rospy.ServiceException:
                    pass

            while self.robots_state_length() != self.controller_index:
                self.clock.sleep()
            

        elif self.task_name == 'synchronization_before_rotation' or\
             self.task_name == 'synchronization_before_pushing':
            # Publish to 'ropots_common'
            pub = rospy.Publisher('robots_common', TaskState, queue_size=50)

            # Synchronization message
            msg = TaskState()
            msg.robot_id = self.controller_index
            msg.task_name = self.task_name

            # Wait until all robots are ready
            while self.robots_state_length() != 3:
                self.clock.sleep()
                pub.publish(msg)

            # Start the box state estimation service
            rospy.wait_for_service('release_box_state')
            start_box_estimation = rospy.ServiceProxy('release_box_state', Empty)
            try:
                start_box_estimation()
            except rospy.ServiceException:
                pass

        return 'step_ok'
        
class PlanTrajectory(State):
    """State of the fsm in which the robot find its path to the box.

    Outcomes:
    none
    
    Inputs:
    none

    Outputs:
    path: the path found by the planning algorithm
    """
    def __init__(self, controller_index, robots_state, boxstate, task_name):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        boxstate (Subscriber): Subscriber to box state
        """
        State.__init__(self, output_keys=['plan_trajectory_out'], outcomes=['step_ok'])
        
        self.controller_index = controller_index
        self.robots_state = robots_state
        self.boxstate = boxstate
        self.task_name = task_name
        
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
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

        if self.task_name == 'plan_for_rotation':
            # Get docking point from service box_get_docking_point_rotate
            rospy.wait_for_service('box_get_docking_point_rotate')
            docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
            

        if self.task_name == 'plan_to_push':
            # Get docking point from service box_get_docking_point_push
            rospy.wait_for_service('box_get_docking_point_push')
            docking = rospy.ServiceProxy('box_get_docking_point_push', BoxGetDockingPointPush)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
            
        # Plan trajectory
        this_robot = self.robots_state[self.controller_index].data.pose.pose.position
        # Tolerance and robot radius are taken into account
        robot_radius = rospy.get_param('robot_radius')
        tolerance = - np.array(response.normal) * (0.07 + robot_radius)
        start_point = [this_robot.x, this_robot.y]
        goal_point = (np.array(response.point) + tolerance).tolist()

        state = False
        while not state:
            state, path = self.planner.plan(start_point, goal_point)

        # the first point in the path is the actual robot position
        path.pop(0)

        userdata.plan_trajectory_out = path

        return 'step_ok'

class Alignment(State):
    """State of the fsm in which the robot rotates in order to align
    itself with the line of sight between its center and the next goal.
    
    Outcomes:
    alignment_ok: the robot rotated successfully

    Inputs:
    goal (float[]): the goal position
    
    Outputs:
    none
    """
    def __init__(self, robot_state, set_control, controller_index, task_name, boxstate = None):
        """Initialize the state of the fsm.

        Arguments:
        robot_state (Subscriber): Subscriber to robot odometry
        set_control (function): function that publish a twist
        """
        if task_name == 'iterator':
            State.__init__(self, outcomes=['alignment_ok'], input_keys=['goal'])
        elif task_name == 'alignment_before_push':
            State.__init__(self, outcomes=['alignment_ok'])
        else:
            State.__init__(self, outcomes=['step_ok'])

        self.robot_state = robot_state
        self.set_control = set_control
        self.controller_index = controller_index
        self.task_name = task_name
        self.boxstate = boxstate
        self.max_angular_v = float(rospy.get_param('max_angular_v'))

        # Tuning
        self.kp = 2
        self.tolerance = 0.02

        # State clock
        self.clock = rospy.Rate(200)

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        robot_x = self.robot_state.data.pose.pose.position.x
        robot_y = self.robot_state.data.pose.pose.position.y

        # Set point
        if self.task_name == 'iterator':
            reference = np.arctan2(userdata.goal[1] - robot_y, userdata.goal[0] - robot_x)  
        
        if self.task_name == 'alignment_rotation' or\
           self.task_name == 'alignment_before_rotation':

            rospy.wait_for_service('box_get_docking_point_rotate')
            docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
            
            reference = np.arctan2(response.normal[1], response.normal[0]) 

        if self.task_name == 'alignment_before_rotation':
            latest_box = self.boxstate.data
            box_pose = np.array([latest_box.x, latest_box.y])
            difference = np.array([robot_x, robot_y]) - box_pose
            
            # Rotate the difference vector by +- 90 degrees
            rotated = response.direction * np.array([- difference[1], difference[0]])

            reference = np.arctan2(rotated[1], rotated[0])

        if self.task_name == 'alignment_push':
            rospy.wait_for_service('box_get_docking_point_push')
            docking = rospy.ServiceProxy('box_get_docking_point_push', BoxGetDockingPointPush)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
            reference = np.arctan2(response.normal[1], response.normal[0]) 

        if self.task_name == 'alignment_before_push':
            goal = rospy.get_param('box_goal')
            goal_x = goal['x']
            goal_y = goal['y']
            latest_boxstate = self.boxstate.data
            box_x = latest_boxstate.x
            box_y = latest_boxstate.y

            reference = np.arctan2(goal_y - box_y,  goal_x - box_x)

        # Perform alignment
        while True:
            # Check error
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            error = utils.angle_normalization(reference - theta)
            if abs(error) > self.tolerance:
                # Set control
                angular_v = proportional_control(self.kp, reference, theta, self.max_angular_v, True)
                self.set_control(0, angular_v)
            else:
                # Stop the robot
                self.set_control(0, 0)
                if self.task_name == 'iterator' or\
                   self.task_name == 'alignment_before_push':
                    return 'alignment_ok'
                else:
                    return 'step_ok'

            # Wait for next clock
            self.clock.sleep()

class GoToPoint(State):
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
        robot_state (Subscriber): Subscriber to robot odometry
        set_control (function): function that publish a twist
        """
        State.__init__(self, outcomes=['point_reached'], input_keys=['goal'])
        
        self.robot_state = robot_state
        self.set_control = set_control

        # Initialize the point to point controller
        self.max_forward_v = float(rospy.get_param('max_forward_v'))
        self.max_angular_v = float(rospy.get_param('max_angular_v'))
        self.controller = PointToPoint(self.max_forward_v, self.max_angular_v)

        # State clock
        self.clock = rospy.Rate(200)

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state."""
        # Set the goal point
        self.controller.goal_point(userdata.goal)

        # Move the robot to the goal point
        done = False
        while not done:
            # Current robot pose and forward velocity
            x = self.robot_state.data.pose.pose.position.x
            y = self.robot_state.data.pose.pose.position.y
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            forward_v = self.robot_state.data.twist.twist.linear.x

            # Set the control
            done, forward_v, angular_v = self.controller.control_law([x, y, theta], forward_v)
            self.set_control(forward_v, angular_v)

            # Wait for next clock
            self.clock.sleep()

        # Stop the robot
        self.set_control(0, 0)        

        return 'point_reached'

class LinearMovement(State):
    """State of the fsm in which the robot slowly and accurately approaches the box.

    Outcomes:
    fine_approach_ok: the robot approached the box
    
    Inputs:
    none

    Outputs:
    none
    """
    def __init__(self, robot_state, controller_index, set_control, task_name):
        """Initialize the state of the fsm.

        Arguments:
        robot_state (Subscriber): Subscriber to robot odometry
        controller_index (int): index of the robot
        set_control (function): function that publish a twist
        """
        if task_name == 'reverse':
            State.__init__(self, outcomes=['step_ok','sequence_ok'])
        else:
            State.__init__(self, outcomes=['step_ok'])

        self.robot_state = robot_state
        self.set_control = set_control
        self.controller_index = controller_index
        self.task_name = task_name
        self.max_forward_v = float(rospy.get_param('max_forward_v'))

        # Subscribe to irbumper topic
        names = rospy.get_param('topics_names')[self.controller_index]
        rospy.Subscriber(names['irbumper'], RoombaIR, self.irsensors_callback)
        self.ir_data_lock = Lock()
        self.ir_data = {}

        # Publish to 'robots_common'
        self.pub = rospy.Publisher('robots_common', TaskState, queue_size=50)

        # Tuning
        self.kp = 2

        # State clock
        self.clock = rospy.Rate(200)

    def max_ir_data(self):
        """Return the maximum ir bumper sensor reading."""
        self.ir_data_lock.acquire()
        max_value = max(self.ir_data.values())
        self.ir_data_lock.release()
        return max_value

    def irsensors_callback(self, data):
        """Update ir_data using data from IR sensors.
        
        Arguments:
        data (RoombaIR): data from one IR sensor 
        """
        self.ir_data_lock.acquire()
        self.ir_data[data.header.frame_id] = data.signal
        self.ir_data_lock.release()

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        msg = TaskState()
        if self.task_name == 'fine_approach_rotation':
            msg.task_name = 'wait_for_turn_rotation';

        if self.task_name == 'fine_approach_push':
            msg.task_name = 'wait_for_turn_push'

        if self.task_name == 'fine_approach_rotation' or\
           self.task_name == 'fine_approach_push':
            # We are approaching to the box
            # the IR readings increase up to 3300
            ir_tolerance = 3300
            
            # Let the other robots know that it's their turn
            # Three pubs should suffice
            msg.robot_id = self.controller_index
            self.pub.publish(msg)
            self.pub.publish(msg)
            self.pub.publish(msg)

        #########################################################################
        # Set the linear velocity and the ir tolerance depending on the task name
        #
        linear_v = 0.01

        if self.task_name == 'partial_reverse':
            linear_v = -0.1
            ir_tolerance = 3300
            
            if self.controller_index == 0:
                return 'step_ok'

        if self.task_name == 'reverse':
            linear_v = -0.1
            ir_tolerance = 0
        #
        ##########################################################################

        # Perform fine approach (in forward or reverse direction)
        if self.task_name == 'reverse' or\
           self.task_name == 'partial_reverse':
            # Move the robot away from the box
            while self.max_ir_data() > ir_tolerance:
                self.set_control(linear_v, 0)
                
        elif self.task_name == 'fine_approach_rotation' or\
             self.task_name == 'fine_approach_push':
            # Next move the robot as close as possible to the box using IR            
            while self.max_ir_data() < ir_tolerance:
                self.set_control(linear_v, 0)

        # In case of task 'reverse' the robot goes quite far from the box
        if self.task_name == 'reverse':
            x_0 = self.robot_state.data.pose.pose.position.x
            y_0 = self.robot_state.data.pose.pose.position.y

            while True:
                self.set_control(linear_v, 0)
                robot_state = self.robot_state.data
                x_current = robot_state.pose.pose.position.x
                y_current = robot_state.pose.pose.position.y
                if np.sqrt((x_0 - x_current) ** 2 + (y_0 - y_current) ** 2) > 0.6:
                    break

        #Stop the robot
        self.set_control(0,0)
        
        if self.task_name == 'reverse':
            return 'sequence_ok'

        return 'step_ok'

class Rotate(State):
    """State of the fsm in which the robot rotates the box.

    Outcomes:
    none

    Inputs:
    none

    Outputs:
    none
    """

    def __init__(self, controller_index, robot_state, boxstate, set_control):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        robot_state (Subscriber): Subscriber to robot odometry
        boxstate (Subscriber): Subscriber to the box state estimation
        set_control (function): function that publish a twist
        """
        State.__init__(self, outcomes=['step_ok'])

        self.controller_index = controller_index
        self.robot_state = robot_state
        self.boxstate = boxstate
        self.set_control = set_control
        self.tolerance = 0.02

        # State clock
        self.clock = rospy.Rate(200)

    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        # Setpoint
        rospy.wait_for_service('box_get_docking_point_rotate')
        docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass
        # Angular reference and direction of rotation
        reference = response.theta
        print (reference)
        direction = response.direction

        # Radius
        x_robot = self.robot_state.data.pose.pose.position.x
        y_robot = self.robot_state.data.pose.pose.position.y

        radius = np.linalg.norm(np.array([x_robot - self.boxstate.data.x,\
                                          y_robot - self.boxstate.data.y ]))

        # Linear and angular velocities
        omega_box = float(rospy.get_param('box_angular_v'))
        linear_v = omega_box * radius
        angular_v = omega_box

        # Synchronization required
        tolerance = 1000
        if self.controller_index == 0:
            time_sync_pub = rospy.Publisher('sync_time', TimeSync, tcp_nodelay = True, queue_size = 10)
            departure = rospy.Time.now() + rospy.Duration.from_sec(2)
            sync_time = TimeSync(departure)

            while not (departure - rospy.Time.now()).to_nsec() < tolerance:
                time_sync_pub.publish(sync_time)

            time_sync_pub.unregister()
        else:
            sync_time = rospy.wait_for_message("sync_time", TimeSync)
            departure = sync_time.departure

            while not (departure - rospy.Time.now()).to_nsec() < tolerance:
                pass

        print(`self.controller_index` + ': sync @ ' + str((departure - rospy.Time.now()).to_nsec()))
        
        # Perform rotation
        while True:
            theta = self.boxstate.data.theta
            error = utils.angle_normalization(reference - theta)
            if abs(error) > self.tolerance:
                self.set_control(linear_v, direction * angular_v)
            else:
                self.set_control(0, 0)

                # Clear docking point
                rospy.wait_for_service('clear_docking_point')
                clear_docking_point = rospy.ServiceProxy('clear_docking_point', Empty)
                try:
                    clear_docking_point()
                except rospy.ServiceException:
                    pass

                # Hold box state
                rospy.wait_for_service('hold_box_state')
                hold_box_state = rospy.ServiceProxy('hold_box_state', Empty)
                try:
                    hold_box_state()
                except rospy.ServiceException:
                    pass
                
                return 'step_ok'

            # Wait for next clock
            self.clock.sleep()

class PushBox(State):
    """State of the fsm in which the robot pushes the box.
    
    Outcomes:
    box_at_goal: box at goal point
    box_drifted: box is drifting
    Inputs:
    none

    Outputs:
    none
    """
    def __init__(self, controller_index, boxstate, set_control):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        boxstate (Subscriber): Subscriber to the box state estimation
        set_control (function): function that publish a twist
        """
        State.__init__(self, outcomes=['box_at_goal','box_drifted'])
        self.controller_index = controller_index
        self.boxstate  = boxstate
        self.set_control = set_control
        self.max_forward_v = float(rospy.get_param('max_forward_v'))

        # Box goal pose
        self.goal = rospy.get_param('box_goal')
        
        # Tuning
        self.distance_tolerance = 0.05
        self.drift_tolerance = 0.03
        
        # State Clock
        self.clock = rospy.Rate(100)
          
    def distance_error(self, current_box_pose):
        """Evaluate the feedback distance error.

        Arguments:
        reference (float): current_box_pose
        """
        box_x = current_box_pose[0]
        box_y = current_box_pose[1]
        error = np.sqrt((self.goal['x'] - box_x) ** 2 +\
                        (self.goal['y'] - box_y) ** 2)
        return error

    def drift_distance(self, current_box_pose):
        """Evaluate the current drift distance.

        Arguments:
        reference (float): current_box_pose
        """
        box_x = current_box_pose[0]
        box_y = current_box_pose[1]
        distance = self.desired_traj.distance([box_x, box_y])
        if self.controller_index == 0:
            print distance
        return distance
  
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        # Synchronization required
        tolerance = 1000
        if self.controller_index == 0:
            time_sync_pub = rospy.Publisher('sync_time', TimeSync, tcp_nodelay = True, queue_size = 10)
            departure = rospy.Time.now() + rospy.Duration.from_sec(2)
            sync_time = TimeSync(departure)

            while not (departure - rospy.Time.now()).to_nsec() < tolerance:
                time_sync_pub.publish(sync_time)

            time_sync_pub.unregister()
        else:
            sync_time = rospy.wait_for_message("sync_time", TimeSync)
            departure = sync_time.departure

            while not (departure - rospy.Time.now()).to_nsec() < tolerance:
                pass

        print(`self.controller_index` + ': sync @ ' + str((departure - rospy.Time.now()).to_nsec()))

        # The desired trajectory of the box center is a line
        latest_boxstate = self.boxstate.data
        box_x = round(latest_boxstate.x, 2)
        box_y = round(latest_boxstate.y, 2)
        self.desired_traj = utils.Line([box_x, box_y],
                                       [self.goal['x'], self.goal['y']])
        print(self.desired_traj.coefficients())

        # Push the box
        forward_v = float(rospy.get_param('box_forward_v'))
        while True:
            latest_boxstate = self.boxstate.data
            box_x = round(latest_boxstate.x, 2)
            box_y = round(latest_boxstate.y, 2)

            if self.drift_distance([box_x, box_y]) > self.drift_tolerance:
                # Stop the robot
                self.set_control(0, 0)
                return 'box_drifted'

            if self.distance_error([box_x, box_y]) > self.distance_tolerance:
                #Set the control
                self.set_control(forward_v, 0)
            else:
                #Stop the robot
                self.set_control(0,0)
                return 'box_at_goal'

            # Wait for next clock
            self.clock.sleep()
