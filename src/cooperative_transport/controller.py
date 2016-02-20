import rospy

class Controller:

    def __init__(self, controller_index):

        rospy.init_node('controller' + str(controller_index))
        topics_names = rospy.get_param('topics_names')

        print topics_names
        

def main(controller_index):
    try:
        controller = Controller(controller_index)
    except rospy.ROSInterruptException:
        pass
