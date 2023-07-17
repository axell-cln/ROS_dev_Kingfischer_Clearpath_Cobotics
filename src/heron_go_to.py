#!/home/kingfisher/venv/rl/bin/python

from squaternion import Quaternion
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import message_filters
import tf
# rl games et omnigymenvs librairies
from gym import spaces
import torch
import yaml
from rl_games.algos_torch.players import BasicPpoPlayerContinuous, BasicPpoPlayerDiscrete

class GoTo:


    def __init__(self):

        # Get parameters
       
        self.config_name = "/home/kingfisher/ros_ws/src/test_rl/config/TurtlebotPPO.yaml"
        self.policy_path="/home/kingfisher/ros_ws/src/test_rl/config/Turtlebot.pth"

        self.path = Path()
        self.dist_threshold = 0.15

        self.observation_space = spaces.Box(np.ones(2) * -np.Inf, np.ones(2) * np.Inf)
        self.act_space = spaces.Box(low=np.array([0.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        with open(self.config_name, 'r') as stream:
            self.cfg = yaml.safe_load(stream)
        
        self.player = BasicPpoPlayerContinuous(self.cfg, self.observation_space, self.act_space, clip_actions=True, deterministic=True)
        self.player.restore(self.policy_path)

        rospy.Subscriber("~goalFromLidar", PoseStamped, self.move_base_callback, queue_size=1)

        self.tf_listener = tf.TransformListener()
        #self.tf_listener.cache_time = rospy.Duration(10.0)

        # Attendez que la transformation entre les deux repères soit disponible
        self.tf_listener.waitForTransform('kingfisher/base', 'kingfisher/laser', rospy.Time(), rospy.Duration(1.0))
        """self.pos_sub = message_filters.Subscriber('/goal_from_lidar', PoseStamped)

        ts = message_filters.TimeSynchronizer([self.pos_sub, self.tf_listener_], 10)
        ts.registerCallback(self.move_base_callback) """

        self.twist_pub_ = rospy.Publisher("~cmd_boat", Twist, queue_size=1)
        self.path_pub_ = rospy.Publisher("~path", Path, queue_size=1)

        self.goal = None

        print("Waiting for goal...")


    def move_base_callback(self, pose):

        self.goal = pose
        print("goal received")
        self.odom_callback()
        #print("goal x: %f y: %f " % (self.goal.pose.position.x, self.goal.pose.position.y))


    def odom_callback(self):
        if not self.goal: return
        
        
        """ t = self.tf_listener_.getLatestCommonTime("kingfisher/base", "kingfisher/laser")
        self.goal_in_base_frame = self.tf_listener_.transformPose("kingfisher/base", self.goal)
        print("Position of the goal in the robot base:")
        print(self.goal_in_base_frame) """
        
        

        # Transformez le point dans le repère cible
        point_target = self.tf_listener.transformPose('kingfisher/base', self.goal)

        self.goal_in_base_frame = point_target

        # Affichez le point transformé    
        print("Point transformé : x = %f, y = %f, z = %f", point_target.pose.position.x, point_target.pose.position.y, point_target.pose.position.z)

        # Get velocity
        obs = self.get_observations()
        action = self.player.get_action(obs["obs"], is_deterministic=True)
        lin_vel=action[0]
        ang_vel=action[1]

        print("Action: ", action)
        
        self.publish_velocity(lin_vel, ang_vel)
        
        """ # Create path visualization
        self.path.header = odom.header
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.poses.append(pose)
        self.path_pub_.publish(self.path) """


    def compute_distance_to_goal(self):
        
        #norm 2 
        return math.sqrt(self.goal_in_base_frame.pose.position.x**2 + self.goal_in_base_frame.pose.position.y**2)


    def getRotation(self):
        
        x = self.goal_in_base_frame.pose.orientation.x
        y = self.goal_in_base_frame.pose.orientation.y
        z = self.goal_in_base_frame.pose.orientation.z
        w = self.goal_in_base_frame.pose.orientation.w

        q = Quaternion(w,x,y,z)
        theta  = q.to_euler(degrees=False)[2]

        print("")
        print("Theta: ", theta)

        return theta


    def get_observations(self):

        yaws = self.getRotation()
       
        goal_angles = torch.atan2(torch.tensor([self.goal_in_base_frame.pose.position.y]),torch.tensor([self.goal_in_base_frame.pose.position.x]))

        heading = goal_angles - yaws
        heading = torch.where(heading > math.pi, heading - 2 * math.pi, heading)
        heading = torch.where(heading < -math.pi, heading + 2 * math.pi, heading)

        heading=heading.item()

        # Compute distance and heading
        dist = self.compute_distance_to_goal()
       
        print('Heading: ',heading )
        print('Dist: ', dist )

        obs = dict({'obs':torch.tensor([heading,dist], dtype=torch.float32, device='cpu')})

        print("observation: ",obs)

        return obs
                
    def publish_velocity(self, lin_vel, ang_vel):
        msg = Twist()
        if self.compute_distance_to_goal() < self.dist_threshold:
            msg.linear.x = 0
            msg.angular.z = 0
            print("Reached goal")
            self.goal = None
        else:
            msg.linear.x = lin_vel 
            msg.angular.z = ang_vel

        print("Message sent to the robot:", msg)
        self.twist_pub_.publish(msg)


if __name__ == '__main__':
    rospy.init_node('goto_heron')
    tufo = GoTo()
    rospy.spin()

