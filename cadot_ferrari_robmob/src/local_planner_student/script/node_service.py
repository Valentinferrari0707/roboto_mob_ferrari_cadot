#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point, PoseStamped, PointStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan


from std_msgs.msg import Bool
from math import fabs, sqrt, atan2, pi, fmod

from local_planner_student.srv import localGoal , Possible
from local_planner_student.srv import Path as PathToGoal 
from copy import copy, deepcopy



class Service_node :

    def __init__() : 
        rospy.init_node('node_service' , anonymous=True)
        rate=rospy.Rate(10)
        rospy.wait_for_service('/move_to/singleGoal')
        while not rospy.is_shutdown():
            self.possible=rospy.ServiceProxy("/move_to/singleGoal" , Possible)
            self.possible()
            


    
if __name__ == '__main__':
    try:

        sn=Service_node()
        


    except rospy.ROSInterruptException:
        pass
  