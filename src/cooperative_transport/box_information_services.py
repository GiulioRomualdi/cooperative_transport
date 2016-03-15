
import rospy
import numpy as np
import threading
from box import BoxGeometry
from std_srvs.srv import Empty
from cooperative_transport.srv import BoxGetDockingPointPush
from cooperative_transport.srv import BoxGetDockingPointPushRequest
from cooperative_transport.srv import BoxGetDockingPointPushResponse
from cooperative_transport.srv import BoxGetDockingPointRotate
from cooperative_transport.srv import BoxGetDockingPointRotateRequest
from cooperative_transport.srv import BoxGetDockingPointRotateResponse
from cooperative_transport.srv import SetPoseUncertaintyArea
from cooperative_transport.srv import SetPoseUncertaintyAreaRequest
from cooperative_transport.srv import SetPoseUncertaintyAreaResponse
from cooperative_transport.srv import GetDockingPointUncertaintyArea
from cooperative_transport.srv import GetDockingPointUncertaintyAreaRequest
from cooperative_transport.srv import GetDockingPointUncertaintyAreaResponse
from std_srvs.srv import Empty
from cooperative_transport.msg import BoxState
from subscriber import Subscriber
from cooperative_transport.utils import angle_normalization

def find_docking_points_to_push(box_current_pose, box_goal_pose, box_length, box_width):
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
    # difference = np.array(box_goal_pose) - np.array(box_pose)
    # angle_between = angle_normalization(np.arctan2(difference[1], difference[0]) - box_theta)
    # tolerance = 0.1
    # for angle in [0, + np.pi / 2, - np.pi / 2, np.pi]:
    #     if abs(angle_between - angle) < tolerance:
    #         singularity = True
    #         break

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
        A list containing the docking points and the normal directions, one for each robot,
        and the best angular position of the box required to push it.
    """
    box_pose = [box_current_pose[0], box_current_pose[1]]
    box_theta = box_current_pose[2]
                        
    # Create a box geometry object
    box_geometry = BoxGeometry(box_length, box_width, box_pose, box_theta)
                         
    p0_pg = np.array(box_goal_pose) - np.array(box_pose)
    los_angle = np.arctan2(p0_pg[1], p0_pg[0])
    references = [angle_normalization(los_angle + np.pi / 4),\
                  angle_normalization(los_angle - np.pi / 4),
                  angle_normalization(los_angle + np.pi /4 + np.pi / 2),
                  angle_normalization(los_angle - np.pi /4 - np.pi / 2)]

    differences = [angle_normalization(reference - box_theta) for reference in references]
    
    edges = [box_geometry.edge(1,2), box_geometry.edge(0, 1), box_geometry.edge(2, 3)]
    normals = [edge.normal() for edge in edges]

    for angle in [0, np.pi/2, -np.pi/2, np.pi]:
        if np.allclose(differences[0], angle):
            results = [{'point' : 0, 'normal' : normals[i]} for i in range(3)]
            return False, 0, 0, results

    abs_differences = np.array([abs(difference) for difference in differences])
    argmin_differences = np.argmin(abs_differences)

    direction = 0
    if differences[argmin_differences] < 0:
        # Clockwise rotation
        point_position = [1.0 / 2, 1.0 / 4, 1.0 / 4]
        points = [edges[i].point(point_position[i]) for i in range(3)]
        direction = - 1
    else:
        # Counterclockwise rotation
        point_position = [1.0 / 2, 3.0 / 4, 3.0 / 4]
        points = [edges[i].point(point_position[i]) for i in range(3)]
        direction = + 1
        
    results = [{'point' : points[i], 'normal' : normals[i]} for i in range(3)]

    return True, references[argmin_differences], direction, results

def find_docking_points_in_uncertainty_area(uncertainty_area_pose, length, width):
    """Find the docking points on the perimeter of the uncertainty area.

    Arguments:
        uncertainty_area_pose (float[]): uncertainty area pose
        length (float): box length in meters
        width (float): box width in meters

    Returns:
        A list containing the docking points and the normal directions, one for each robot.
    """    
    max_length = max(length, width)
    min_length = min(length, width)

    uncertainty_geometry = BoxGeometry(2 * max_length,\
                                       max_length,\
                                       [uncertainty_area_pose[0],\
                                        uncertainty_area_pose[1]],\
                                       uncertainty_area_pose[2])

    print (uncertainty_geometry.vertices())
    edges = [uncertainty_geometry.edge(1,2), uncertainty_geometry.edge(3,0)]
    points = [edges[0].point((1.0/2 * min_length) / max_length), edges[1].point((max_length - min_length) / max_length + (1.0/2 * min_length) / max_length)]
    normals = [edge.normal() for edge in edges]
            
    results = [{'point' : points[i], 'normal' : normals[i]} for i in range(2)]

    return results

                
class BoxInformationServices():
    """Provide box information services."""

    def __init__(self):
        """Initialize the object."""
        rospy.init_node('box_information_services')
    
        # Subcrive to box state topic
        self.boxstate = Subscriber('box_state', BoxState)
        # Obtain box length and width
        self.box_length = float(rospy.get_param('box')['length'])
        self.box_width = float(rospy.get_param('box')['width'])

        # Provide 'box_get_docking_point' service
        rospy.Service('box_get_docking_point_push', BoxGetDockingPointPush, self.box_get_docking_point_push)
        rospy.Service('box_get_docking_point_rotate', BoxGetDockingPointRotate, self.box_get_docking_point_rotate)

        # Provide 'uncertainty_area' service
        rospy.Service('uncertainty_area_set_pose', SetPoseUncertaintyArea, self.set_pose_uncertainty_area)
        rospy.Service('uncertainty_area_get_docking_point', GetDockingPointUncertaintyArea, self.get_docking_point_uncertainty_area)
        self.service_ready = False
        self.first_robotid_rcvd = -1

        # Provide 'clear_docking_point' service
        self.update_docking_point = True
        rospy.Service('clear_docking_point', Empty, self.clear_docking_point)

        # Get the initial box goal pose
        initial_boxgoal = rospy.get_param('box_goal')
        self.goal_x = initial_boxgoal['x']
        self.goal_y = initial_boxgoal['y']

        # Latest docking points and normals
        self.docking = []

        # Latest outcome of find_docking_points_to_rotate
        self.rotation_required = False
        self.theta = 0
        self.direction = 0

        # Lock used to avoid concurrent access to update_docking_point
        self.update_docking_point_lock = threading.Lock()

        # Lock used to avoid concurrent access to service_ready
        self.service_ready_lock = threading.Lock()

    def box_get_docking_point_push(self, request):
        """Provide a robot with the docking point/normal on the perimeter of the box in its current position.

        Arguments:
            request (BoxGetDockingPointPushRequest): the request
        """
        self.update_docking_point_lock.acquire()
        flag = self.update_docking_point
        self.update_docking_point_lock.release()
        
        # Only the first time
        if flag:
            # Get the latest box state
            latest_boxstate = self.boxstate.data
            box_x = latest_boxstate.x
            box_y = latest_boxstate.y
            box_theta = latest_boxstate.theta
        
            # Update the docking points/normals
            self.docking = find_docking_points_to_push([box_x, box_y, box_theta],
                                                       [self.goal_x, self.goal_y],
                                                       self.box_length, self.box_width)
            self.update_docking_point_lock.acquire()
            self.update_docking_point = False
            self.update_docking_point_lock.release()

        # The response
        response = BoxGetDockingPointPushResponse()
        docking = self.docking[request.robot_id]
        response.point = docking['point']
        response.normal = docking['normal']
        
        return response

    def box_get_docking_point_rotate(self, request):
        """Provide a robot with the docking point/normal on the perimeter of the box in its current position.

        Arguments:
            request (BoxGetDockingPointPushRequest): the request
        """
        self.update_docking_point_lock.acquire()
        flag = self.update_docking_point
        self.update_docking_point_lock.release()
        
        # Only the first time
        if flag:
            # Get the latest box state
            latest_boxstate = self.boxstate.data
            box_x = latest_boxstate.x
            box_y = latest_boxstate.y
            box_theta = latest_boxstate.theta
        
            # Update the docking points/normals
            self.rotation_required, self.theta, self.direction, self.docking = find_docking_points_to_rotate([box_x, box_y, box_theta],
                                                                                                              [self.goal_x, self.goal_y],
                                                                                                              self.box_length, self.box_width)
    
            self.update_docking_point_lock.acquire()
            self.update_docking_point = False
            self.update_docking_point_lock.release()

        # The response
        response = BoxGetDockingPointRotateResponse()
        response.theta = self.theta
        response.is_rotation_required = self.rotation_required
        response.direction = self.direction

        if self.rotation_required:
            docking = self.docking[request.robot_id]
            response.point = docking['point']
            response.normal = docking['normal']

        return response

    def clear_docking_point(self, request):
        """Clear all docking point previusly evaluated

        Arguments:
            request (Empty): the request
        """
        self.update_docking_point_lock.acquire()
        self.update_docking_point = True
        self.update_docking_point_lock.release()

    def set_pose_uncertainty_area(self, request):
        """Set pose of the uncertainty area

        Arguments:
            request (SetPoseUncertaintyArea): the request
        """        
        self.uncertainty_area_pose = request.pose
        self.service_ready_lock.acquire()
        self.service_ready = True
        self.service_ready_lock.release()

    def get_docking_point_uncertainty_area(self,request):
        """Provide a robot with the docking point/normal on the perimeter of the uncertainty_area.

        Arguments:
            request (GetDockingPointUncertaintyArea): the request

        """
        self.service_ready_lock.acquire()
        service_ready = self.service_ready
        self.service_ready_lock.release()

        # Is service ready?
        if not service_ready:
            response = GetDockingPointUncertaintyAreaResponse()
            response.is_ready = False
            response.pose = []
            response.point = []
            response.normal = []
            return response
        
        self.update_docking_point_lock.acquire()
        flag = self.update_docking_point
        self.update_docking_point_lock.release()
        
        # Only the first time
        if flag:

            self.first_robotid_rcvd = request.robot_id
            # Update the docking points/normals
            self.docking = find_docking_points_in_uncertainty_area(self.uncertainty_area_pose, self.box_length, self.box_width)

            self.update_docking_point_lock.acquire()
            self.update_docking_point = False
            self.update_docking_point_lock.release()

        # The response
        response = GetDockingPointUncertaintyAreaResponse()
        if self.first_robotid_rcvd == request.robot_id:
            docking = self.docking[0]
        else:
            docking = self.docking[1]

        response.is_ready = True
        response.pose = self.uncertainty_area_pose
        response.point = docking['point']
        response.normal = docking['normal']

        return response

    def run(self):
        """Main activity of the node."""
        # Wait for boxstate to be ready
        while not self.boxstate.is_ready:
            rospy.sleep(1)
        
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
