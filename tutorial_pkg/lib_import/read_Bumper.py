
import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import BumperEvent
#from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from move_robot import stop_mov, move_metre, move_degre
#test
def listen():
    # Initialize ROS node with ROS client
    #rclpy.init()
    aNode= Node( "listener" )
    listener= ROSBumperListener(aNode)
    # Start infinite loop
    rclpy.spin(aNode)
    # Clean everything and switch the light off
    aNode.destroy_node()
    #rclpy.shutdown()


class ROSBumperListener():
    #BumperEvent, '/events/bumper', for real
    #Twist, '/cmd_vel', for virtual testing 
    def __init__(self, rosNode):
        self._logger= rosNode.get_logger()
        self._subscription= rosNode.create_subscription(
            BumperEvent, '/events/bumper',
            self.listener_callback, 10
        )
        self._publisher= rosNode.create_publisher( Bool, '/com_signal/bumper', 10 )

    def listener_callback(self, msg):
        self._logger.info( 'I heard: ' + str(msg))
        print(msg.state)
        if msg.state==1:
            self._publisher.publish(1)
        else:
            self._publisher.publish(0)