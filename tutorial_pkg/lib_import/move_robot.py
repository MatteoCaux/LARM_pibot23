import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from time import time
from kobuki_ros_interfaces.msg import BumperEvent
from kobuki_ros_interfaces.msg import WheelDropEvent

###MOVE BASE FUNCTION
#utilize spin once and continue process until duration is achieved
#may have a problem of duration due to timer inside of CMD_ROBOT
def move2(vx,rz, duration):
    completed=True
    start = time()
    aNode= Node( "tempTalker" )
    #finish=rclpy.task.function
    talker= CMD_ROBOTV2(aNode,duration,vx,rz)
    counter=0
    while counter < duration and not talker._emergency_stop: 
        # Start one loop
        rclpy.spin_once(aNode)
        counter = time()-start
    # Clean everything and switch the light off
    aNode.destroy_node()
    if talker._emergency_stop:
        completed=False
    return(completed)

###PRETEMPLATED MOVE FUNCTIONS  
def move_degre(deg):# turn on himself the deg value (degres, full turn = 360), positive value : turn to the left
    deg_per_sec=0.5*180/3.14
    duration=abs(deg/deg_per_sec) - 0.25
    if deg<0:
        rotation_speed=-0.5
    else:
        rotation_speed=0.5
    return(move2(0.0,rotation_speed,duration))

def move_metre(m): # move m meter in front of himself
    duration=m/0.2
    return(move2(0.2,0.0,duration))

def stop_mov(duration): #stop the movements for a certain duration
    """
    Fonction qui arrête le robot
    """
    move2(0.0,0.0,duration)
### CLASS OF A MOV CMD TO THE ROBOT
class CMD_ROBOTV2: #for move2
    #publish once the command, no duration
    def __init__(self,rosNode,_duration,vx,rz): 
        self._emergency_stop=False
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._publisher= rosNode.create_publisher( Twist, '/multi/cmd_nav', 10 )
        self._subscription1= rosNode.create_subscription(
            BumperEvent, '/events/bumper',
            self.bumper_state, 10
        )
        self._subscription2= rosNode.create_subscription(
            WheelDropEvent, '/events/wheel_drop',
            self.bumper_state, 10
        )

        self._timer = rosNode.create_timer(0.1, self.timer_callback)  
    def bumper_state(self, msg):
        if msg.state==1:
            velocity=Twist()
            velocity.linear.x = 0.0 #m/s
            velocity.angular.z = 0.0 #rad/s
            self._publisher.publish(velocity)
            self._emergency_stop=True

    def timer_callback(self):
        if not self._emergency_stop:
            velocity=Twist()
            velocity.linear.x = self._vx #m/s
            velocity.angular.z = self._rz #rad/s
            self._publisher.publish(velocity)
