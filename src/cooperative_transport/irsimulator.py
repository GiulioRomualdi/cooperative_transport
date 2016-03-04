import rospy
import re
from sensor_msgs.msg import LaserScan
from irsensor import IrSensor
from irobotcreate2.msg import RoombaIR

class IrSimulator:
    """Simulate the ir readings from a laser_scan topic and publish them.

    Attributes:
        ir_sensor(IrSensor): the object containing the ir sensor model
        sensors_angles(int[]): angular positions of the sensors (rad)
        angle_0 (int): angle corresponding to the first item of the laser_scan data (rad)
    
    """

    def __init__(self, index, topic_name, ir_sensor, sensors_angles, angle_0):
        """Initialize the object.

        Arguments: 
            index(int): the index of the robot
            topic_name(string): the name of the laser_scan topic
            ir_sensor(IrSensor): the object containing the ir sensor model
            sensors_angles(int[]): angular positions of the sensors
            angle_0 (int): angle corresponding to the first item of data (rad)
        """

        self.sensors_angles = sensors_angles
        self.ir_sensor = ir_sensor
        self.angle_0 = angle_0

        # Subscribe to the laser_scan topic
        rospy.Subscriber(topic_name, LaserScan, self.laser_callback)

        # Publish to the ir_bumper_index topic
        self.irbumper_pub = rospy.Publisher('/ir_bumper_' + `index`, RoombaIR, queue_size=50)


    def laser_callback(self, data):
        """Publish a new irbumper message given the laser range finder data.
        
        Arguments:
            data(LaserScan): laser range finder data
        """
        irbumper = RoombaIR()
        irbumper.header.stamp = rospy.Time.now()

        for frame_id,angle in self.sensors_angles.iteritems():
            
            irbumper.header.frame_id = frame_id
            irbumper.signal = self.ir_sensor.laser_to_ir(data.ranges, self.angle_0, angle)
            irbumper.state = False
            if irbumper.signal != 0:
                irbumper.state = True

            if not rospy.is_shutdown():
                self.irbumper_pub.publish(irbumper)

def main():
    """Start the node that simulates the ir sensors readings."""
    # Initialize the node
    rospy.init_node('ir_simulator')
    clock = rospy.Rate(1)

    # Create one ir sensor only since the irobot create 2 has 6 identical ir sensors
    ir_sensor_specs = rospy.get_param("ir_simulator/ir_sensor_specs")
    ir_sensor_kwargs = dict(ir_sensor_specs, robot_radius=float(rospy.get_param("housing_radius")))
    ir_sensor = IrSensor(**ir_sensor_kwargs)

    # Start publishing ir readings for any '/iRobot_x/laser_scan' topic found
    sensors_angles = rospy.get_param("sensors_angles")
    angle_0 = float(rospy.get_param("ir_simulator/angle_0"))
    laser_topics = []

    try:
        while not rospy.is_shutdown():

            topics = rospy.get_published_topics()

            for topic in topics:
                topic_name = topic[0]
                if re.match("/iRobot(_[0-9])?/laser_scan",topic_name) and not topic_name in laser_topics:
                    laser_topics.append(topic_name)
                    index = 0
                    for i in range(10):
                        if str(i) in topic_name:
                            index = i + 1
                            break
                    IrSimulator(index, topic_name, ir_sensor, sensors_angles, angle_0)
                    rospy.loginfo("Found topic " + topic_name + ". Will publish on /ir_bumper_" + `index`)

            clock.sleep()
    except rospy.ROSInterruptException:
        pass
