import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import time

#utilize spin once and continue process until duration is achieved
#may have a problem of duration due to timer inside of CMD_ROBOT
def move2(vx,rz, duration):
    start = time()
    aNode= Node( "tempTalker" )
    #finish=rclpy.task.function
    talker= CMD_ROBOTV2(aNode,duration,vx,rz)
    counter=0
    while counter < duration : 
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
class CMD_ROBOTV2: #for move2
    #publish once the command, no duration
    def __init__(self,rosNode,_duration,vx,rz): 
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._publisher= rosNode.create_publisher( Twist, '/multi/cmd_nav', 10 )
        #self._timer = rosNode.create_timer(0.5, self.timer_callback)
        self.timer_callback

    def timer_callback(self):
        velocity=Twist()
        velocity.linear.x = self._vx #m/s
        velocity.angular.z = self._rz #rad/s
        self._publisher.publish(velocity)

# Execute the function.
if __name__ == "__main__":
    print("move_robot :: START...")
    path()
    print("move finished")
