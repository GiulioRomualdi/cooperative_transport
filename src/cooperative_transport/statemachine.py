import rospy
import smach
import utils
import numpy as np

# State Machine
from smach import StateMachine, Iterator

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
from std_srvs.srv import Empty

def construct_sm(controller_index, robots_state, boxstate, set_control):
    """Construct the top level state machine for cooperative transport.

    Arguments:
    controller_index (int): index of the robot
    robots_state (Subscriber[]): list of Subscribers to robots odometry
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
                             transitions={'fine_approach_ok':'docking_ok'})
        #
        ##########################################################################################

        move_box = StateMachine(outcomes=['goal_reached'])

        with move_box:
        ##########################################################################################
        # MOVE BOX STATE MACHINE
        ##########################################################################################
        #


            consensus = Consensus(controller_index, robots_state, boxstate, set_control)
            StateMachine.add('CONSENSUS',\
                             consensus,\
                             transitions={'consensus_ok':'SYNCHRONIZER'})

            synchronizer = Synchronizer(controller_index)
            StateMachine.add('SYNCHRONIZER',synchronizer,\
                             transitions={'synchronization_ok':'PUSH_BOX'})


            push_box = PushBox(controller_index, robots_state[controller_index], boxstate, set_control)
            StateMachine.add('PUSH_BOX',\
                             push_box,\
                             transitions={'box_at_goal':'goal_reached',\
                                          'box_drifted':'CONSENSUS'})

        #
        ##########################################################################################
        StateMachine.add('BOX_DOCKING', box_docking,\
                         transitions={'docking_failed':'transport_failed',\
                                      'docking_ok':'MOVE_BOX'})

        StateMachine.add('MOVE_BOX', move_box,\
                         transitions={'goal_reached':'transport_ok'})
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

        # Subscribe and publish to the topic 'robots_common'
        rospy.Publisher('robots_common', TaskState, queue_size=50)
        rospy.Subscriber('robots_common', TaskState, self.callback)
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

        # Get docking point from service box_get_docking_point
        rospy.wait_for_service('box_get_docking_point')
        docking = rospy.ServiceProxy('box_get_docking_point', BoxGetDockingPointPush)
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
        robot_state (Subscriber): Subscriber to robot odometry
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['alignment_ok'], input_keys=['goal'])

        self.robot_state = robot_state
        self.set_control = set_control
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

        # Set point
        x = self.robot_state.data.pose.pose.position.x
        y = self.robot_state.data.pose.pose.position.y
        reference = np.arctan2(userdata.goal[1] - y, userdata.goal[0] - x)  

        # Perform alignment
        while True:
            # Check error
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            error = utils.angle_normalization(reference - theta)
            if abs(error) > self.tolerance:
                # Set control
                angular_v = proportional_control(self.kp, reference, theta, self.max_angular_v)
                self.set_control(0, angular_v)
            else:
                # Stop the robot
                self.set_control(0, 0)
                return 'alignment_ok'

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

class BoxFineApproach(smach.State):
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
        smach.State.__init__(self, outcomes=['fine_approach_ok'])

        self.robot_state = robot_state
        self.set_control = set_control
        self.controller_index = controller_index 
        self.max_forward_v = float(rospy.get_param('max_forward_v'))
        self.max_angular_v = float(rospy.get_param('max_angular_v'))

        # Subscribe to irbumper topic
        names = rospy.get_param('topics_names')[self.controller_index]
        rospy.Subscriber(names['irbumper'], RoombaIR, self.irsensors_callback)
        self.ir_data_lock = Lock()
        self.ir_data = {}

        # Tuning
        self.kp = 2
        self.linear_tolerance = 3300
        self.angular_tolerance = 0.02

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
        pub = rospy.Publisher('robots_common', TaskState, queue_size=50)
        msg = TaskState()
        msg.robot_id = self.controller_index
        msg.task_name = 'wait_for_turn'
        # Three pubs should suffice
        pub.publish(msg)
        pub.publish(msg)
        pub.publish(msg)

        # Get the normal direction pointing inward from service box_get_docking_point
        rospy.wait_for_service('box_get_docking_point')
        docking = rospy.ServiceProxy('box_get_docking_point', BoxGetDockingPointPush)
        try:
            response = docking(self.controller_index)
        except rospy.ServiceException:
            pass

        ################################
        # First align to the normal
        #
        # Set point
        reference = np.arctan2(response.normal[1], response.normal[0])
        # Perform alignment
        done = False
        while not done:
            # Check error
            theta = utils.quaternion_to_yaw(self.robot_state.data.pose.pose.orientation)
            error = utils.angle_normalization(reference - theta)
            if abs(error) > self.angular_tolerance:
                # Set control
                angular_v = proportional_control(self.kp, reference, theta, self.max_angular_v)
                self.set_control(0, angular_v)
            else:
                # Stop the robot
                self.set_control(0, 0)
                done = True
        #
        ################################
    
        ##############################################################
        # Next move the robot as close as possible to the box using IR
        linear_v = 0.01
        while self.max_ir_data() < self.linear_tolerance:
            self.set_control(linear_v, 0)

        #Stop the robot
        self.set_control(0,0)
        #
        ##############################################################

        return 'fine_approach_ok'

class Synchronizer(smach.State):
    """State of the fsm in which the robot synchronize with neighbors.

    Outcomes:
    synchronization_ok: robot completed synchronization
    
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
        self.max_number_robots = 3
        self.task_name = 'synchronization'

        # Subscribe and publish to the topic 'robots_common'
        rospy.Subscriber('robots_common', TaskState, self.callback)
        self.robots_state = set()
        self.lock = Lock()

        # Synchronization message
        self.msg = TaskState()
        self.msg.robot_id = self.controller_index
        self.msg.task_name = self.task_name
        
        # State clock
        # self.clock = rospy.Rate(10)
        
    def callback(self, data):
        """Update robots_state using data from wait topic.
        
        Arguments:
        data (TaskState): data from wait topic 
        """
        if data.task_name == self.task_name:
            self.lock.acquire()
            self.robots_state.add(data.robot_id)
            self.lock.release()

    def robots_ready(self):
        """Return True if all the robots are ready."""
        self.lock.acquire()
        number = len(self.robots_state)
        self.lock.release()
        return number == self.max_number_robots        
    
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        pub = rospy.Publisher('robots_common', TaskState, queue_size=50)
        # Wait until all robots are ready
        while not self.robots_ready():
            rospy.sleep(0.5)
            pub.publish(self.msg)
            #self.clock.sleep()

        # Start the box state estimation service
        rospy.wait_for_service('release_box_state')
        start_box_estimation = rospy.ServiceProxy('release_box_state', Empty)
        try:
            start_box_estimation()
        except rospy.ServiceException:
            pass

        pub.unregister()
        return 'synchronization_ok'

class Consensus(smach.State):
    """State of the fsm in which the robot rotates in order to align
    itself with the line of sight between its center and the goal and
    start the box estimation service.
    
    Outcomes:
    consensus_ok: the robot completed consensus 

    Inputs:
    none

    Outputs:
    none
    """
    def __init__(self, controller_index, robots_state, boxstate, set_control):
        """Initialize the state of the fsm.

        Arguments:
        controller_index (int): index of the robot
        robots_state (Subscriber[]): list of Subscribers to robots odometry
        boxstate (Subscriber): Subscriber to the box state estimation
        set_control (function): function that publish a twist
        """
        smach.State.__init__(self, outcomes=['consensus_ok'])

        self.boxstate  = boxstate
        self.set_control = set_control
        self.max_angular_v = float(rospy.get_param('max_angular_v'))
        self.this_robot = robots_state[controller_index]
        #self.neigh_robots = [robots_state[i] for i in range(len(robots_state)) if i != controller_index]

        # Tuning
        self.tolerance = 0.02
        
        # State clock
        self.clock = rospy.Rate(200)
        
    def execute(self, userdata):
        """Execute the main activity of the fsm state.

        Arguments:
        userdata: inputs and outputs of the fsm state.
        """
        # Set point
        goal = rospy.get_param('box_goal')
        goal_x = goal['x']
        goal_y = goal['y']
        latest_boxstate = self.boxstate.data
        box_x = latest_boxstate.x
        box_y = latest_boxstate.y
        reference = np.arctan2(goal_y - box_y,  goal_x - box_x)

        # Perform consensus
        done = False
        while not done:
            # Check error
            this_theta = utils.quaternion_to_yaw(self.this_robot.data.pose.pose.orientation)
            error = utils.angle_normalization(reference - this_theta)
            if abs(error) > self.tolerance:
                # Set the control
                #neigh_thetas = [utils.quaternion_to_yaw(robot_state.data.pose.pose.orientation) for robot_state in self.neigh_robots]
                #angular_v = consensus(this_theta, neigh_thetas, reference, self.max_angular_v)
                angular_v = proportional_control(2, reference, this_theta, self.max_angular_v)
                self.set_control(0, angular_v)
            else:
                # Stop the robot
                self.set_control(0, 0)
                done = True

            # Wait for next clock
            self.clock.sleep()
          
        return 'consensus_ok'

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
