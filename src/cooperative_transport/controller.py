import rospy

def main(node_name):
    rospy.init_node(node_name)

    topic_names = rospy.get_param(node_name + "/topic_names")

    rospy.loginfo(node_name + ": topics are {" + topic_names["cmdvel"] + ", " +
                  topic_names["odom"] + ", " + topic_names["irbumper"] + "}.")
    
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
