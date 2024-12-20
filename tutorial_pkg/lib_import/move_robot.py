import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from time import time
from kobuki_ros_interfaces.msg import BumperEvent

emergency_stop=False
###MOVE BASE FUNCTION
#utilize spin and stop the process when duration is achieved
def move1(vx,rz, duration):
    aNode= Node( "tempTalker" )
    #finish=rclpy.task.function
    talker= CMD_ROBOTV1(aNode,duration,vx,rz)
    completed=False
    # Start infinite loop
    rclpy.spin(aNode)#_until_future_complete
    # Clean everything and switch the light off
    aNode.destroy_node()

#utilize spin once and continue process until duration is achieved
#may have a problem of duration due to timer inside of CMD_ROBOT
def move2(vx,rz, duration):
    start = time()
    aNode= Node( "tempTalker" )
    #finish=rclpy.task.function
    talker= CMD_ROBOTV2(aNode,duration,vx,rz)
    counter=0
    while counter < duration and not emergency_stop: 
        # Start one loop
        rclpy.spin_once(aNode)
        counter = time()-start
    # Clean everything and switch the light off
    aNode.destroy_node()
    return()

###PRETEMPLATED MOVE FUNCTIONS  
def move_degre(deg):# turn on himself the deg value (degres, full turn = 360), positive value : turn to the left
    deg_per_sec=0.5*180/3.14
    duration=deg/deg_per_sec - 0.25
    move2(0.0,0.5,duration)

def move_metre(m): # move m meter in front of himself
    duration=m/0.5
    move2(0.5,0.0,duration)

def stop_mov(duration): #stop the movements for a certain duration
    """
    Fonction qui arrête le robot
    """
    move2(0.0,0.0,duration)
### CLASS OF A MOV CMD TO THE ROBOT
class CMD_ROBOTV2: #for move2
    #publish once the command, no duration
    def __init__(self,rosNode,_duration,vx,rz): 
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._publisher= rosNode.create_publisher( Twist, '/multi/cmd_nav', 10 )
        self._subscription1= rosNode.create_subscription(
            BumperEvent, '/events/bumper',
            self.bumper_state, 10
        )
        #self._timer = rosNode.create_timer(0.5, self.timer_callback)
        self.timer_callback
    def bumper_state(self, msg):
        if msg.state==1:
            velocity=Twist()
            velocity.linear.x = 0 #m/s
            velocity.angular.z = 0 #rad/s
            self._publisher.publish(velocity)
            emergency_stop=True
    def timer_callback(self):
        if not emergency_stop:
            velocity=Twist()
            velocity.linear.x = self._vx #m/s
            velocity.angular.z = self._rz #rad/s
            self._publisher.publish(velocity)

class CMD_ROBOTV1:#for move1
    #publish the command for the duration
    def __init__(self,rosNode,duration,vx,rz): 
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._duration=duration
        self._publisher= rosNode.create_publisher( Twist, '/multi/cmd_nav', 10 )
        self._timer = rosNode.create_timer(0.5, self.timer_callback)
        self._i=0

    def timer_callback(self):
        velocity=Twist()
        if self._i <=self._duration*2 :
            velocity.linear.x = self._vx #m/s
            velocity.angular.z = self._rz #rad/s
            self._publisher.publish(velocity)
        else :
            velocity.linear.x = 0.0 #m/s
            velocity.angular.z = 0.0 #rad/s
            self._publisher.publish(velocity) 
            rosNode.destroy_node()
        self._i+=1