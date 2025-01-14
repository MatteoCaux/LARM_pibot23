#!/usr/bin/python3
import rclpy
from  rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import scan_echo
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


# on transforme une posistion dans 'odom' en une posistion dans 'base_link'
class localGoal(Node):
    def __init__(self):
        super().__init__('localgoal')

        #Pubs
        self._pubLocalPose= rosNode.create_publisher(
            Twist, "/moveto/localgoal", 10
        )
        #Subs
        self._subToGoalPose= rosNode.create_subscription(
                Pose, '/moveto/globalgoal',
                self.global_goal_listen, 10
            )
        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Node Attribute:
        self.local_frame= 'base_link'

        self.create_timer(0.1, self.publish_goal)
    #Subs calls
    def global_goal_listen(self,msg):
        self.global_goal=Pose()
        self.global_goal=msg
    #Process funcs
    def publish_goal(self):
        currentTime= rclpy.time.Time()
        # Get Transformation
        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        self.local_frame,
                        'odom',
                        currentTime)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):TransformException as tex:
            self._node.get_logger().info( f'Could not transform the goal into {self.local_frame}: {tex}')
            return
        # Compute goal into local coordinates
        localGoal = tf2_geometry_msgs.do_transform_pose( self.global_goal, stampedTransform )


if __name__ == '__main__':
    globalnode=localGoal()