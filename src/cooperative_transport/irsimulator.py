from sympy.mpmath import *

class IrSensor():
    """Simulate an IR sensor behevior using Hoyuko laser data.

    Attributes:
    laser_resolution (float): laser resolution in radiants [2 * pi / (laser ray number).
    epsilon_max (float): laser ray maximum relative angle compatible with the IR sensor geometry.
    roomba_radius (float): roomba radius in meters.
    relative_indices(int[]): list of relative indices used to indicate a compatible laser ray.
    bounds (dictionary): dictionary with minimum and maximum value accepted for every laser ray.
    """
    def __init__(self, alpha_o, alpha_i, beta, sensor_length, roomba_radius, laser_resolution):
        """
        Args:
        alpha_o (float): 'external' angle, in radiants, used to describe IR sensor geometry.
        alpha_i (float): 'internal' angle, in radiants, used to describe IR sensor geometry.
        beta (float): angle, in radiants, used to describe IR sensor geometry.
        sensor_length (float): geometry length of IR sensor.
        roomba_radius (float): roomba radius in meters.
        laser_resolution (float): laser resolution in radiants [2 * pi / (laser ray number).
        """
        self.laser_resolution = laser_resolution
        self.epsilon_max = self.epsilon_max_evaluation(alpha_o, alpha_i, beta, sensor_length, self.roomba_radius)
        self.roomba_radius = roomba_radius

        acceptable_laser_number = int(round(self.epsilon_max / self.laser_resolution))
   
        self.relative_indices = [index for index in range(-acceptable_laser_number, acceptable_laser_number + 1)]

        self.bounds = {index: [self.generate_lower_bound(alpha_i, beta, sensor_length, self.roomba_radius, index, laser_resolution),\
                    self.generate_upper_bound(alpha_o, beta, sensor_length, self.roomba_radius, index, laser_resolution)]\
                       for index in self.relative_indices}

    def epsilon_max_evaluation(self, alpha_o, alpha_i, beta, sensor_length, roomba_radius):
        """
        Return epsilon max value, in radiants. 
        Args:
        alpha_o (float): 'external' angle, in radiants, used to describe IR sensor geometry.
        alpha_i (float): 'internal' angle, in radiants, used to describe IR sensor geometry.
        beta (float): angle, in radiants, used to describe IR sensor geometry.
        sensor_length (float): geometry length of IR sensor.
        roomba_radius (float): roomba radius in meters.
        """
        epsilon_max = asin((sensor_length * csc(alpha_i - alpha_o + beta) * sin(alpha_i + alpha_o))/\
                    (sqrt((2 * roomba_radius + sensor_length * cot(alpha_i + beta/2))**2 +\
                    2 * sensor_length * cot(alpha_i + beta/2) * (2 * roomba_length + seonsor_length *\
                    cot(alpha_i + beta/2)) * csc(alpha_i - alpha_o + beta) * sin(alpha_i + alpha_o) +\
                    sensor_length**2 * (csc(alpha_i + beta/2))**2 * (csc(alpha_i - alpha_o + beta))**2 *\
                    (sin(alpha_i+alpha_o))**2)))

        return float(epsilon_max)


    def generate_lower_bound(self, alpha_i, beta, sensor_length, roomba_radius, index, laser_resolution):
        """
        Return lower bound for the laser ray specified by index.
        Args:
        alpha_i (float): 'internal' angle, in radiants, used to describe IR sensor geometry.
        beta (float): angle, in radiants, used to describe IR sensor geometry.
        sensor_length (float): geometry length of IR sensor.
        roomba_radius (float): roomba radius in meters.
        index (int): relative index of laser whereof lower bound is evaluated.
        laser_resolution (float): laser resolution in radiants [2 * pi / (laser ray number).
        """        
        angle = index * laser_resolution
        lower_bound = 1/2 * csc(alpha_i + beta/2 - abs(angle)) * (sensor_length * cos(alpha_i + beta/2) +\
                    2 * roomba_radius * sin(alpha_i + beta/2))
        
        return float(lower_bound)


    def generate_upper_bound(self, alpha_o, beta, sensor_length, roomba_radius, index, laser_resolution):
        """
        Return upper bound for the laser ray specified by index.
        Args:
        alpha_o (float): 'external' angle, in radiants, used to describe IR sensor geometry.
        beta (float): angle, in radiants, used to describe IR sensor geometry.
        sensor_length (float): geometry length of IR sensor.
        roomba_radius (float): roomba radius in meters.
        index (int): relative index of laser whereof lower bound is evaluated.
        laser_resolution (float): laser resolution in radiants [2 * pi / (laser ray number).
        """
        angle = index * laser_resolution
        upper_bound = 1/2 * sqrt(((sec(angle))**2 * (sensor_length - 2 * roomba_length * tan(alpha_o -\
                    beta/2))**2)/(tan(1/2 * (-2 * alpha_o + beta)) + tan(abs(angle)))**2)

        return float(upper_bound)

    def translate_distance(self, data, index):
        """
        Return distance from the center of IR sensor.
        Args:
        data (float): distance from the center of Hokuyo laser.
        index (int): laser ray relative index.
        """
        angle = self.laser_resolution * index
        return data * cos(angle) - self.roomba_radius

    def data_converter(self, data, angle_0, ir_angle):
        """
        Convert laser data into IR sensor data (range from 4950 to 0).
        Args:
        data (float[]): Hokuyo laser data.
        angle_0 (int): angle, in radiants, corresponding to the first position in data list.
        ir_angle (float): angular position of IR sensor in roomba robot.
        """

        max_distance = 0
        min_distance = 4950

        #Keep angle from 0 to 2 * pi   
        angle = ir_angle
        if(angle < 0):
            angle = float(angle + 2 * pi)
    
        angle_index = int(round((angle - angle_0) / self.laser_resoultion))

        if data[angle_index] < self.bounds[0][0]:
            return min_distance

        acceptable_data = {relative_index: data[angle_index + relative_index]\
                        if (data[angle_index + relative_index] < self.bounds[relative_index][1]\
                        and data[angle_index + relative_index] > self.bounds[relative_index][0])  for relative_index in self.relative_indeces}
       
        if not bool(acceptable_data):
            return max_distance

        object_distace = min([self.data_convert(acceptable_data[relative_index], relative_index)])
        
        converted_object_distance = (1 - (object_distance - self.bounds[0][0])/ (self.bounds[0][1] - self.bounds[0][0])) * min_distance

        return converted_object_distance
