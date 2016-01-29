from sympy.mpmath import *
import sys

class IrSensor():
    """Simulate the irobot create 2 ir light bumper sensor using laser range finder data.

    The sensor modelling is taken from the USA patent US2014/0188325.
    
    Attributes:
    alpha_o (float): ir emitter/detector outer spread off-center angle (rad)
    alpha_i (float): ir emitter/detector inner spread off-center angle (rad)
    beta (float): angle between emission and detection fields (rad)
    housing_length (float): emitter and detector housing length (meters)    
    laser_resolution (float): laser angular resolution (rad)
    epsilon_max (float): maximum positive angle of a laser ray intersecting the ir detection area (rad)
    robot_radius (float): irobot create radius (meters)
    bounds (dictionary): minimum and maximum lengths describing the ir detection area (meters)
    sweep_angles_indexes(int[]): list of indexes j such that a laser ray with angle 
                                 j*laser_resolution intersects the ir detection area

    Public methods:
    laser_to_ir(data, angle_0, ir_angle) : convert laser range finder data to ir sensor reading
    """

    def __init__(self, alpha_o, alpha_i, beta, housing_length, laser_resolution, robot_radius):
        """Initialize the object.

        Arguments:
        alpha_o (float): ir emitter/detector outer spread off-center angle (rad)
        alpha_i (float): ir emitter/detector inner spread off-center angle (rad)
        beta (float): angle between emission and detection fields (rad)
        housing_length (float): emitter and detector housing length (meters)    
        laser_resolution (float): laser angular resolution (rad)
        robot_radius (float): irobot create radius (meters)
        """
        self.alpha_o = alpha_o
        self.alpha_i = alpha_i
        self.beta = beta
        self.housing_length = housing_length
        self.laser_resolution = laser_resolution
        self.robot_radius = robot_radius
        self._epsilon_max = 0
        self.eval_epsilon_max()
        self.sweep_angles_indexes = []
        self.eval_sweep_angles_indexes()
        self.bounds = {}
        self.eval_bounds()

        
    @property
    def epsilon_max(self):
        """Return epsilon_max."""
        return self._epsilon_max

    def get_bounds(self):
        """Return self.bounds."""
        return self.bounds

    def eval_epsilon_max(self):
        """Evaluate the maximum positive angle of a laser ray intersecting the ir detection area (rad)."""
        self._epsilon_max = float(asin((self.housing_length * csc(self.alpha_i - self.alpha_o + self.beta) * sin(self.alpha_i + self.alpha_o))/\
                    (sqrt((2 * self.robot_radius + self.housing_length * cot(self.alpha_i + self.beta/2))**2 +\
                    2 * self.housing_length * cot(self.alpha_i + self.beta/2) * (2 * self.robot_radius + self.housing_length *\
                    cot(self.alpha_i + self.beta/2)) * csc(self.alpha_i - self.alpha_o + self.beta) * sin(self.alpha_i + self.alpha_o) +\
                    self.housing_length**2 * (csc(self.alpha_i + self.beta/2))**2 * (csc(self.alpha_i - self.alpha_o + self.beta))**2 *\
                    (sin(self.alpha_i+self.alpha_o))**2))))

    def eval_sweep_angles_indexes(self):
        """Evaluate the indexes relative to the laser ray sweep angles."""
        # how many laser rays inside the ir detection area?
        number = int(round(self._epsilon_max / self.laser_resolution))
   
        # discrete indexes relative to the angles of the laser rays intersecting
        # the ir detection area
        self.sweep_angles_indexes = [index for index in range(-number, number + 1)]

    def eval_lower_bound(self, index):
        """Evaluate the lower bound given the index of the laser ray sweep angle.

        Args:
        index (int): laseer ray sweep angle index
        """        
        angle = index * self.laser_resolution
        lower_bound = 1./2 * csc(self.alpha_i + self.beta/2 - abs(angle)) * (self.housing_length * cos(self.alpha_i + self.beta/2) +\
                    2 * self.robot_radius * sin(self.alpha_i + self.beta/2))

        return float(lower_bound)

    def eval_upper_bound(self, index):
        """Return the upper bound given the index of the laser ray sweep angle.

        Args:
        index (int): laser sweep angle index
        """        
        angle = index * laser_resolution
        upper_bound = 1./2 * sqrt(((sec(angle))**2 * (self.housing_length - 2 * self.robot_radius * tan(self.alpha_o -\
                    self.beta/2))**2)/(tan(1./2 * (-2 * self.alpha_o + self.beta)) + tan(abs(angle)))**2)

        return float(upper_bound)

    def eval_bounds(self):
        """Evaluate the bounds that describe the ir detection area."""        
        self.bounds = {index: [self.eval_lower_bound(index), self.eval_upper_bound(index)]\
                       for index in self.sweep_angles_indexes}

    def distance_from_housing(self, distance_from_laser, index):
        """Return the distance from the center of the emitter/detector housing.
        
        Args:
        distance_from_laser (float): distance from the center of laser range finder
        index (int): sweep angle index
        """
        angle = self.laser_resolution * index
        return float(distance_from_laser * cos(angle) - self.robot_radius)

    def laser_to_ir(self, data, angle_0, ir_angle):
        """Convert laser range finder data into IR sensor reading.

        Return an ir sensor reading between 0 and 4095.

        Args:
        data (float[]): laser range finder data
        angle_0 (int): angle corresponding to the first item of data (rad)
        ir_angle (float): angular position of the emitter/detector housing (-180, 180)
        """

        # ir light bumper sensor values range as reported in the
        # irobot create 2 open interface
        irobot_min_ir = 0
        irobot_max_ir = 4095

        # force ir_angle inside [0, 2pi]
        ir_angle = float(pi) / 180 * ir_angle
        if(ir_angle < 0):
            ir_angle = float(ir_angle + 2 * pi)
    
        # index relative to the angle ir_angle
        ir_angle_index = int(round((ir_angle - angle_0) / self.laser_resolution))

        # laser range relative to the laser ray with angle = ir_angle
        range_at_ir_angle = data[ir_angle_index + 0]
        
        # if range_at_ir_angle is less than the lower bound relative to 
        # the angle = ir_angle  then the object cannot be identified by the ir detector
        if range_at_ir_angle < self.bounds[0][0]:
            return irobot_min_ir

        # find all the admissible ranges, i.e, inside [lower_bound, upper_bound],
        # and evaluate them wrt to the center of the emitter/detector housing
        admissible_ranges = [self.distance_from_housing(data[ir_angle_index + index], index)\
                             for index in self.sweep_angles_indexes\
                             if (data[ir_angle_index + index] < self.bounds[index][1]\
                            and data[ir_angle_index + index] > self.bounds[index][0])]
        
        # if no adimssible ranges the object cannot be identified by the ir detector
        if not admissible_ranges:
            return irobot_min_ir

        # take the minimum 
        equivalent_range = min(admissible_ranges)

        # scale the value so that it adhere with the irobot create 2 open specification (0-4095)
        max_distance = self.bounds[0][1] - self.robot_radius
        ir_value = int(round((1 - equivalent_range/max_distance)*irobot_max_ir))
        
        return ir_value

if __name__ == "__main__":
    """ Some demos."""

    # sensor specs
    k = float(pi/180)
    alpha_o = 10 * k
    alpha_i = 25 * k
    beta = 50 * k
    housing_length = 0.034

    # laser specs
    angle_0 = float(pi)
    data_length = 360
    laser_resolution = data_length*k/360

    # robot specs
    robot_radius = 0.1696
    max_radius = robot_radius + 0.065

    # istance of IrSensor
    ir = IrSensor(alpha_o, alpha_i, beta, housing_length, laser_resolution, robot_radius)
    ir_angle = 0

    # some outputs
    min_radius = ir.get_bounds()[0][0]
    number_steps = 50
    step = (max_radius - min_radius) / number_steps

    print('epsilon_max', ir.epsilon_max * 180 / float(pi))
    print('bounds', ir.get_bounds())
    for i in range(0, number_steps):
        radius = min_radius + i*step
        fake_data = [radius for  j in range (data_length)]
        ir_reading = ir.laser_to_ir(fake_data, ir_angle, angle_0)
        print "constant radius: %f, ir: %f" % (radius, ir_reading)
