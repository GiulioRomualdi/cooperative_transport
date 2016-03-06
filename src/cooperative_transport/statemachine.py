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
from control.consensus import consensus
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
        sequence = Sequence(outcomes = ['sequence_ok', 'step_ok'],
                            connector_outcome = 'step_ok')
        with sequence:
            Sequence.add('WAIT_ROTATION',\
                         Wait(controller_index, 'wait_for_turn'))

            Sequence.add('PLAN_TRAJECTORY_ROTATION',\
                         PlanTrajectory(controller_index, robots_state,\
                                        boxstate, 'plan_for_rotation'),\
                         remapping={'plan_trajectory_out':'path'})

            Sequence.add('BOX_APPROACH_ROTATION',\
                         box_approach(sequence, robots_state,\
                                      controller_index, set_control))

            Sequence.add('ALIGNMENT_ROTATION',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_rotation'))

            Sequence.add('FINE_APPROACH_ROTATION',\
                         BoxFineApproach(robots_state[controller_index], \
                                         controller_index, set_control))

            Sequence.add('SYNCHRONIZATION_BEFORE_ROTATION',
                         Wait(controller_index, 'synchronization'))

            Sequence.add('ALIGNMENT_BEFORE_ROTATION',\
                         Alignment(robots_state[controller_index],\
                                   set_control, controller_index, \
                                   'alignment_before_rotation', boxstate))
            Sequence.add('ROTATE',\
                         Rotate(controller_index, robots_state[controller_index],\
                                boxstate, set_control))
            # Sequence.add('ALIGNMENT_BEFORE_ROTATION', Alignment(robots_state[controller_index], set_control, controller_index, 'alignment_rotation')))
            # Sequence.add('ROTATION', Rotation(TODO))
            # Sequence.add('ALIGNMENT_AFTER_ROTATION', Alignment(TODO))
            # Sequence.add('REVERSE', Reverse(TODO))
            # Sequence.add('WAIT_PUSH',\
            #              Wait(controller_index, 'wait_for_turn'))
            # Sequence.add('PLAN_TRAJECTORY_PUSH',\
            #              PlanTrajectory(controller_index, robots_state,\
            #                             boxstate, 'plan_to_push'),\
            #              remapping={'plan_trajectory_out':'path'})
            # Sequence.add('BOX_APPROACH_PUSH', box_approach(sm))
            # Sequence.add('ALIGNMENT_PUSH', Alignment(TODO))
            # Sequence.add('FINE_APPROACH_PUSH', BoxFineApproach(TODO))

        StateMachine.add('SEQUENCE', sequence,\
                         transitions={'sequence_ok':'transport_ok'})
 
    return sm
        
def box_approach(sm, robots_state, controller_index, set_control):
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
            alignment = Alignment(robots_state[controller_index], set_control, controller_index, 'iterator')
            StateMachine.add('ALIGNMENT',\
                             alignment,\
                             transitions={'alignment_ok':'GO_TO_POINT'})
            
            go_to_point = GoToPoint(robots_state[controller_index], set_control)
            StateMachine.add('GO_TO_POINT',\
                             go_to_point,\
                             transitions={'point_reached':'approach_continue'})
                
        Iterator.set_contained_state('CONTAINER_STATE', 
                                     box_approach_container, 
                                     loop_outcomes=['approach_continue'])

    return box_approach

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
        State.__init__(self, outcomes=['step_ok'])
        
        self.controller_index = controller_index
        self.task_name = task_name

        # Publisher and subscribe to the topic 'robots_common'
        rospy.Subscriber('robots_common', TaskState, self.callback)
        self.pub = rospy.Publisher('robots_common', TaskState, queue_size=50)

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
        if self.task_name == 'wait_for_turn':
            while self.robots_state_length() != self.controller_index:
                self.clock.sleep()

        elif self.task_name == 'synchronization':
            # Synchronization message
            msg = TaskState()
            msg.robot_id = self.controller_index
            msg.task_name = self.task_name

            # Wait until all robots are ready
            while self.robots_state_length() != 3:
                self.clock.sleep()
                self.pub.publish(msg)

            # Start the box state estimation service
            rospy.wait_for_service('release_box_state')
            start_box_estimation = rospy.ServiceProxy('release_box_state', Empty)
            try:
                start_box_estimation()
            except rospy.ServiceException:
                pass

        self.robots_state = set()
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
            # Get docking point from service box_get_docking_point_push
            rospy.wait_for_service('box_get_docking_point_rotate')
            docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
            try:
                response = docking(self.controller_index)
            except rospy.ServiceException:
                pass
            
            if not response.is_rotation_required:
                userdata.plan_trajectory_out = []
                return 'step_ok'

        elif self.task_name == 'plan_to_push':
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
        tolerance = - np.array(response.normal) * (0.06 + robot_radius)
        start_point = [this_robot.x, this_robot.y]
        goal_point = (np.array(response.point) + tolerance).tolist()
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
            
            if not response.is_rotation_required:
                return 'step_ok'

            reference = np.arctan2(response.normal[1], response.normal[0]) 

        if self.task_name == 'alignment_before_rotation':
            latest_box = self.boxstate.data
            box_pose = np.array([latest_box.x, latest_box.y])
            difference = np.array([robot_x, robot_y]) - box_pose
            
            # Rotate the difference vector by +- 90 degrees
            rotated = response.direction * np.array([- difference[1], difference[0]])

            reference = np.arctan2(rotated[1], rotated[0])

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
                if self.task_name == 'iterator':
                    return 'alignment_ok'
                else:
                    return 'step_ok'

            # Wait for next clock
            self.clock.sleep()
        
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
        robot_state (Subscriber): Subscriber to robot odometry
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['point_reached'], input_keys=['goal'])
        
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

class BoxFineApproach(State):
    """State of the fsm in which the robot slowly and accurately approaches the box.

    Outcomes:
    fine_approach_ok: the robot approached the box
    
    Inputs:
    none

    Outputs:
    none
    """
    def __init__(self, robot_state, controller_index, set_control):
        """Initialize the state of the fsm.

        Arguments:
        robot_state (Subscriber): Subscriber to robot odometry
        controller_index (int): index of the robot
        set_control (function): function that publish a twist
        """
        State.__init__(self, outcomes=['step_ok'])

        self.robot_state = robot_state
        self.set_control = set_control
        self.controller_index = controller_index 
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
        self.tolerance = 3300

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
        # Let the other robots know that it's their turn
        msg = TaskState()
        msg.robot_id = self.controller_index
        msg.task_name = 'wait_for_turn'
        # Three pubs should suffice
        self.pub.publish(msg)
        self.pub.publish(msg)
        self.pub.publish(msg)

        rospy.wait_for_service('box_get_docking_point_rotate')
        docking = rospy.ServiceProxy('box_get_docking_point_rotate', BoxGetDockingPointRotate)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass
            
        if not response.is_rotation_required:
            return 'step_ok'

        # Next move the robot as close as possible to the box using IR
        linear_v = 0.01
        while self.max_ir_data() < self.tolerance:
            self.set_control(linear_v, 0)

        #Stop the robot
        self.set_control(0,0)
     
        return 'step_ok'

# class Synchronizer(smach.State):
#     """State of the fsm in which the robot synchronize with neighbors.

#     Outcomes:
#     synchronization_ok: robot completed synchronization
    
#     Inputs:
#     none

#     Outputs:
#     none
#     """
#     def __init__(self, controller_index):
#         """Initialize the state of the fsm.

#         Arguments:
#         controller_index (int): index of the robot
#         """
#         smach.State.__init__(self, outcomes=['synchronization_ok'])

#         self.controller_index = controller_index 
#         self.max_number_robots = 3
#         self.task_name = 'synchronization'

#         # Subscribe and publish to the topic 'robots_common'
#         rospy.Subscriber('robots_common', TaskState, self.callback)
#         self.robots_state = set()
#         self.lock = Lock()

#         # Synchronization message
#         self.msg = TaskState()
#         self.msg.robot_id = self.controller_index
#         self.msg.task_name = self.task_name
        
#         # State clock
#         # self.clock = rospy.Rate(10)
        
#     def callback(self, data):
#         """Update robots_state using data from wait topic.
        
#         Arguments:
#         data (TaskState): data from wait topic 
#         """
#         if data.task_name == self.task_name:
#             self.lock.acquire()
#             self.robots_state.add(data.robot_id)
#             self.lock.release()

#     def robots_ready(self):
#         """Return True if all the robots are ready."""
#         self.lock.acquire()
#         number = len(self.robots_state)
#         self.lock.release()
#         return number == self.max_number_robots        
    
#     def execute(self, userdata):
#         """Execute the main activity of the fsm state.

#         Arguments:
#         userdata: inputs and outputs of the fsm state.
#         """
#         pub = rospy.Publisher('robots_common', TaskState, queue_size=50)
#         # Wait until all robots are ready
#         while not self.robots_ready():
#             rospy.sleep(0.5)
#             pub.publish(self.msg)
#             #self.clock.sleep()

#         # Start the box state estimation service
#         rospy.wait_for_service('release_box_state')
#         start_box_estimation = rospy.ServiceProxy('release_box_state', Empty)
#         try:
#             start_box_estimation()
#         except rospy.ServiceException:
#             pass

#         pub.unregister()
#         return 'synchronization_ok'

# class Consensus(smach.State):
#     """State of the fsm in which the robot rotates in order to align
#     itself with the line of sight between its center and the goal and
#     start the box estimation service.
    
#     Outcomes:
#     consensus_ok: the robot completed consensus 

#     Inputs:
#     none

#     Outputs:
#     none
#     """
#     def __init__(self, controller_index, robots_state, boxstate, set_control):
#         """Initialize the state of the fsm.

#         Arguments:
#         controller_index (int): index of the robot
#         robots_state (Subscriber[]): list of Subscribers to robots odometry
#         boxstate (Subscriber): Subscriber to the box state estimation
#         set_control (function): function that publish a twist
#         """
#         smach.State.__init__(self, outcomes=['consensus_ok'])

#         self.boxstate  = boxstate
#         self.set_control = set_control
#         self.max_angular_v = float(rospy.get_param('max_angular_v'))
#         self.this_robot = robots_state[controller_index]
#         #self.neigh_robots = [robots_state[i] for i in range(len(robots_state)) if i != controller_index]

#         # Tuning
#         self.tolerance = 0.02
        
#         # State clock
#         self.clock = rospy.Rate(200)
        
#     def execute(self, userdata):
#         """Execute the main activity of the fsm state.

#         Arguments:
#         userdata: inputs and outputs of the fsm state.
#         """
#         # Set point
#         goal = rospy.get_param('box_goal')
#         goal_x = goal['x']
#         goal_y = goal['y']
#         latest_boxstate = self.boxstate.data
#         box_x = latest_boxstate.x
#         box_y = latest_boxstate.y
#         reference = np.arctan2(goal_y - box_y,  goal_x - box_x)

#         # Perform consensus
#         done = False
#         while not done:
#             # Check error
#             this_theta = utils.quaternion_to_yaw(self.this_robot.data.pose.pose.orientation)
#             error = utils.angle_normalization(reference - this_theta)
#             if abs(error) > self.tolerance:
#                 # Set the control
#                 #neigh_thetas = [utils.quaternion_to_yaw(robot_state.data.pose.pose.orientation) for robot_state in self.neigh_robots]
#                 #angular_v = consensus(this_theta, neigh_thetas, reference, self.max_angular_v)
#                 angular_v = proportional_control(2, reference, this_theta, self.max_angular_v, True)
#                 self.set_control(0, angular_v)
#             else:
#                 # Stop the robot
#                 self.set_control(0, 0)
#                 done = True

#             # Wait for next clock
#             self.clock.sleep()
          
#         return 'consensus_ok'
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
        omega_box = 0.1
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
                return 'step_ok'

            # Wait for next clock
            self.clock.sleep()

            

class PushBox(smach.State):
    """State of the fsm in which the robot pushes the box.
    
    Outcomes:
    box_at_goal: box at goal point
    box_drifted: box is drifting
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
        smach.State.__init__(self, outcomes=['box_at_goal','box_drifted'])
        self.controller_index = controller_index
        self.robot_state = robot_state
        self.boxstate  = boxstate
        self.set_control = set_control
        self.max_forward_v = float(rospy.get_param('max_forward_v'))

        # Box goal pose
        self.goal = rospy.get_param('box_goal')
        
        # Tuning
        self.distance_tolerance = 0.05
        self.drift_tolerance = 0.03
        
        # State Clock
        self.clock = rospy.Rate(200)
          
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
        return distance
  
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        # The desired trajectory of the box center is a line
        latest_boxstate = self.boxstate.data
        box_x = latest_boxstate.x
        box_y = latest_boxstate.y
        self.desired_traj = utils.Line([box_x, box_y],
                                       [self.goal['x'], self.goal['y']])

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

        # Push the box
        forward_v = 0.3
        while True:
            latest_boxstate = self.boxstate.data
            box_x = latest_boxstate.x
            box_y = latest_boxstate.y

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
