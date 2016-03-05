import rospy
import numpy as np
from box import BoxGeometry
from std_srvs.srv import Empty
from cooperative_transport.srv import BoxGetDockingPoint
from cooperative_transport.srv import BoxGetDockingPointRequest
from cooperative_transport.srv import BoxGetDockingPointResponse
from cooperative_transport.srv import BoxSetGoal
from cooperative_transport.srv import BoxSetGoalRequest
from cooperative_transport.msg import BoxState
from subscriber import Subscriber
from cooperative_transport.utils import angle_normalization

def find_docking_points(box_current_pose, box_goal_pose, box_length, box_width):
    """Find the docking points on the perimeter of the box required
    by the robots in order to move the box to the goal position.

    Arguments:
        box_current_pose (float[]): current box pose
        box_goal_pose (float[]): goal box pose
        length (float): box length in meters
        width (float): box width in meters

    Returns:
        A list containing the docking points and the normal directions, one for each robot.
    """
    singularity = False
    box_pose = [box_current_pose[0], box_current_pose[1]]
    box_theta = box_current_pose[2]
                        
    # Create a box geometry object
    box_geometry = BoxGeometry(box_length, box_width, box_pose, box_theta)
                         
    # Test if (box_goal_pose - box_pose) is parallel to an edge of the box
    difference = np.array(box_goal_pose) - np.array(box_pose)
    angle_between = angle_normalization(np.arctan2(difference[1], difference[0]) - box_theta)
    tolerance = 0.1
    for angle in [0, + np.pi / 2, - np.pi / 2, np.pi]:
        if abs(angle_between - angle) < tolerance:
            singularity = True
            break

    # Calculate the distances between the goal and the vertices
    distances = [np.linalg.norm(np.array(box_goal_pose) - np.array(v)) for v in box_geometry.vertices()]
    # Take into account the indexes of the vertices
    dist_indexed = [(i, distances[i]) for i in range(4)]
    # Sort the distances
    dist_ordered =  sorted(dist_indexed, key = lambda item : item[1], reverse = True)

    # Indexes of interest
    argmax0 = dist_ordered[0][0]
    argmax1 = dist_ordered[1][0]
    argmax2 = dist_ordered[2][0]
    pair_01 = [argmax0, argmax1]
    pair_01.sort()
    pair_02 = [argmax0, argmax2]
    pair_02.sort()
    # In case of pair_0i[0] == 0 and pair__0i[1] == 3 they should be reversed
    if pair_01[0] == 0 and pair_01[1] == 3:
        pair_01.reverse()
    if pair_02[0] == 0 and pair_02[1] == 3:
        pair_02.reverse()

    points = []
    edges = []
    # Find docking points and normals
    if singularity == True:
        edges.append(box_geometry.edge(*pair_01))
        edges.append(box_geometry.edge((pair_01[0] - 1) % 4, (pair_01[1] - 1) % 4))
        edges.append(box_geometry.edge((pair_01[0] + 1) % 4, (pair_01[1] + 1) % 4))
        points = [1.0 / 2] * 3
    else:
        shorter_edge = box_geometry.edge(*pair_01)
        longer_edge = box_geometry.edge(*pair_02)
        if shorter_edge >= longer_edge:
            shorter_edge, longer_edge = longer_edge, shorter_edge
        edges.append(shorter_edge)
        edges.append(longer_edge)
        edges.append(longer_edge)
        points = [1.0 / 2, 1.0 / 4, 3.0 / 4]
        
    return [{'point' : edges[i].point(points[i]) , 'normal' : edges[i].normal() } for i in range(3)]

def find_docking_points_to_rotate(box_current_pose, box_goal_pose, box_length, box_width):
    """Find the docking points on the perimeter of the box required
    by the robots in order to rotate the box.

    Arguments:
        box_current_pose (float[]): current box pose
        box_goal_pose (float[]): goal box pose
        length (float): box length in meters
        width (float): box width in meters

    Returns:
        A list containing the docking points and the normal directions, one for each robot.
    """
    box_pose = [box_current_pose[0], box_current_pose[1]]
    box_theta = box_current_pose[2]
                        
    # Create a box geometry object
    box_geometry = BoxGeometry(box_length, box_width, box_pose, box_theta)
                         
    p0_pg = np.array(box_goal_pose) - np.array(box_pose)
    los_angle = np.arctan2(p0_pg[1], p0_pg[0])
    references = [angle_normalization(los_angle + np.pi / 4),\
                  angle_normalization(los_angle - np.pi / 4)]

    differences = [angle_normalization(reference - box_theta) for reference in references]

    for angle in [0, np.pi/2, -np.pi/2, np.pi]:
        if np.all_close(differences[0], angle):
            return False, 

    edges =[box_geometry.edge(0, 1), box_geometry.edge(2, 3), box_geometry(1,2)]
    normals = [edge.normal() for edge in edges]

    min_differences_index = 0
    if abs(differences[0]) > abs(differences[1]):
        min_differences_index = 1
    
    if differences[min_differences_index] < 0:
        # Cockwise rotation
        point_position = [1.0 / 4, 1.0 / 4, 1.0 / 2]
        points = [edges[i].point(point_position[i]) for i in range(3)]
    else:
        # Counterclockwise rotation
        point_position = [3.0 / 4, 3.0 / 4, 1.0 / 2]
        points = [edges[i].point(point_position[i]) for i in range(3)]
        
    results = [{'point' : points[i], 'normal' : normals[i],\
                'reference_theta' : references[min_differences_index]} for i in range(3)]

    return results.insert(0, True)
    
                
class BoxInformationServices():
    """Provide box information services."""

    def __init__(self):
        """Initialize the object."""
        rospy.init_node('box_information_services')
    
        # Subcrive to box state topic
        self.boxstate = Subscriber('box_state', BoxState)
        # Obtain box length and width
        self.box_length = rospy.get_param('box')['length']
        self.box_width = rospy.get_param('box')['width']

        # Provide 'box_get_docking_point' service
        rospy.Service('box_get_docking_point', BoxGetDockingPoint, self.box_get_docking_point)
        # Provide 'box_set_goal' service
        rospy.Service('box_set_goal', BoxSetGoal, self.box_set_goal)

        # Docking points and normals
        self.docking = []

    def box_get_docking_point(self, request):
        """Provide a robot with the docking point/normal on the perimeter of the box in its current position.

        Arguments:
            request (BoxGetDockingPointRequest): the request
        """
        # The response
        response = BoxGetDockingPointResponse()
        docking = self.docking[request.robot_id]
        response.point = docking['point']
        response.normal = docking['normal']
        
        return response

    def box_set_goal(self, request):
        """Set a new box goal position and update the docking points/normals accordingly.

        Arguments:
            request (BoxSetGoalRequest): the request
        """
        # Get the latest box state
        latest_boxstate = self.boxstate.data
        box_x = latest_boxstate.x
        box_y = latest_boxstate.y
        box_theta = latest_boxstate.theta
        
        # Update the docking points/normals
        self.docking = find_docking_points([box_x, box_y, box_theta],
                                                        [request.x_goal, request.y_goal],
                                                        self.box_length, self.box_width)
    def run(self):
        """Main activity of the node."""
        # Wait for boxstate to be ready
        while not self.boxstate.is_ready:
            rospy.sleep(1)
        
        # Get the initial box goal pose
        initial_boxgoal = rospy.get_param('box_goal')
        goal_x = initial_boxgoal['x']
        goal_y = initial_boxgoal['y']

        # Set the new goal 
        request = BoxSetGoalRequest(goal_x, goal_y, 0)
        self.box_set_goal(request)

        while not rospy.is_shutdown():
            #nothing to do
            rospy.sleep(1)

def main():
    """Initialize BoxInformationServices."""
    box_information_services = BoxInformationServices()
    try:
        box_information_services.run()
    except rospy.ROSInterruptException:
        pass
