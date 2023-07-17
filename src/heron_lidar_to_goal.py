#!/home/kingfisher/venv/rl/bin/python


import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from visualization_msgs.msg import Marker


class LidarToGoal:


    def __init__(self):

        # Get parameters
       
        rospy.Subscriber("~scan", LaserScan, self.lidar_data, queue_size=1)
        goal_pub = rospy.Publisher("~goalFromLidar", PoseStamped, queue_size = 2)
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        
        self.goal_to_be_published = PoseStamped()

        self.goal_to_be_published.header.seq = 1
        self.goal_to_be_published.header.stamp = rospy.Time.now()
        self.goal_to_be_published.header.frame_id = "kingfisher/laser"
        
        self.marker = Marker()

        self.marker.header.frame_id = "kingfisher/laser"
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5

        self.goal = None
        self.current = None

        self.threshold = 850
        

        print("Publishing from lidar...")

        while not rospy.is_shutdown():
            marker_pub.publish(self.marker)
            goal_pub.publish(self.goal_to_be_published)
            


    def lidar_data(self, lidar_data):


        lidar_values = lidar_data.intensities
        lidar_ranges = lidar_data.ranges
        lidar_angle_increment = lidar_data.angle_increment
        lidar_angle_min = lidar_data.angle_min
        lidar_angle_max = lidar_data.angle_max

        goal_indice = self.find_target(lidar_values)

        if goal_indice != None:
            goal_range = lidar_ranges[goal_indice]
            goal_heading = lidar_angle_min + goal_indice * lidar_angle_increment

            #print(f"Got goal @ (heading, range) = ({goal_heading:.3f}, {goal_range:.3f})")  #python3
            #print("goal heading: %f range: %f " % (goal_heading, goal_range))

            #set the goal position
            self.goal_to_be_published.pose.position.x = math.cos(goal_heading) * goal_range
            self.goal_to_be_published.pose.position.y = math.sin(goal_heading) * goal_range
            self.goal_to_be_published.pose.position.z = 0.0

            # Set the pose of the marker
            self.marker.pose.position.x =  math.cos(goal_heading) * goal_range
            self.marker.pose.position.y =  math.sin(goal_heading) * goal_range
            self.marker.pose.position.z = 0.0

            #print("marker x: %f y: %f " % (self.marker.pose.position.x, self.marker.pose.position.y))

            return goal_heading, goal_range 
        else:
            return 

    def find_target(self, lidar_values):

        chosen_points_indices = self.get_brightest_points(lidar_values)
        
        if len(chosen_points_indices)!=0:
            closest_points = self.get_closest_points(chosen_points_indices)
        
        else:
            closest_points = []

        if len(closest_points) == 0:
            goal_indice=None

        else:
            half_size_points_array = len(closest_points) // 2
            goal_indice = closest_points[half_size_points_array]
        
        
        return goal_indice


    def get_closest_points(self, chosen_points_indices):

        closest_points_indices=[]
        for i in range(len(chosen_points_indices)-1):
            if chosen_points_indices[i+1] - chosen_points_indices[i] == 1 :
                closest_points_indices.append(chosen_points_indices[i])
        #print(closest_points_indices)
        return closest_points_indices  

    """ def get_closest_points(self, chosen_points_indices):

        aux=[]
        matrix=[]
        for i in range(len(chosen_points_indices)-1):
            aux.append(chosen_points_indices[i])
            if chosen_points_indices[i+1] - chosen_points_indices[i] == 1 :
                pass #do nothing
            else: 
                matrix.append(aux)
                aux=[]
        if len(matrix)!=0:
            return max(matrix,key=len)
        else:
            return [] """

        
    def get_brightest_points(self, lidar_values):

        chosen_points_indices=[]
        for i in range(len(lidar_values)):
            if lidar_values[i] > self.threshold:
                chosen_points_indices.append(i)
        
        return chosen_points_indices


if __name__ == '__main__':
    rospy.init_node('heron_lidar_to_goal')
    tufo = LidarToGoal()
    rospy.spin()

