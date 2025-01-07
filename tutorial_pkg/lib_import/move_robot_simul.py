import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from time import time

emergency_stop=False
###MOVE BASE FUNCTION
#utilize spin and stop the process when duration is achieved
def move1(vx,rz, duration):
    aNode= Node( "tempTalker" )
    #finish=rclpy.task.function
    print("create talker")
    talker= CMD_ROBOTV1(aNode,duration,vx,rz)
    # Start infinite loop
    print("start infinit loop")
    rclpy.spin(aNode)#_until_future_complete
    # Clean everything and switch the light off
    print("finished infinit loop")
    aNode.destroy_node()

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

#utilize spin once and continue process until duration is achieved
#try to accelerate progressively
def move3(vx : float ,rz : float, duration : float,distance: float):
    """
    vx : speed of the robot in m/s (can be negative)
    rz : angular speed of the robot in rad/s (can be negative)
    duration : duration of the movement in float
    distance : distance to travel in m (not negative, but negative speed to move forward)
    """
    aNode= Node( "movement" )
    #pente = vx/t1
    #nb_repe=int(vx/pente)
    temp_vitesse=0.1
    #for i in range(nb_repe):
    while(temp_vitesse<vx):
        move_robot= CMD_ROBOTV3(aNode,temp_vitesse,rz)
        print(f"commande de vitesse :{temp_vitesse}")
        rclpy.spin_once(aNode)
        temp_vitesse +=0.1
        print(temp_vitesse)
    for _ in range(10):
        move_robot= CMD_ROBOTV3(aNode,vx,rz)
        print(f"commande de vitesse :{temp_vitesse}")
        rclpy.spin_once(aNode)

    stop_mov()
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
        self._publisher= rosNode.create_publisher( Twist, '/cmd_vel', 10 )
        #self._subscription1= rosNode.create_subscription(BumperEvent, '/events/bumper',self.bumper_state, 10)
        self._timer = rosNode.create_timer(0.1, self.timer_callback)
        #self.timer_callback
    def bumper_state(self, msg):
        if msg.state==1:
            velocity=Twist()
            velocity.linear.x = 0 #m/s
            velocity.angular.z = 0 #rad/s
            self._publisher.publish(velocity)
            emergency_stop=True
    def timer_callback(self):
        if not emergency_stop:
            print("moving")
            velocity=Twist()
            velocity.linear.x = self._vx #m/s
            velocity.angular.z = self._rz #rad/s
            self._publisher.publish(velocity)

class CMD_ROBOTV1:#for move1
    #publish the command for the duration
    def __init__(self,rosNode,duration,vx,rz):
        self._i=0
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._duration=duration
        self._publisher= rosNode.create_publisher( Twist, '/multi/cmd_nav', 10 )
        print('create timer')
        self._timer = rosNode.create_timer(0.5, self.timer_callback)
        print('timer created')

    def timer_callback(self):
        velocity=Twist()
        if self._i <=self._duration*2 :
            print("moving")
            velocity.linear.x = self._vx #m/s
            velocity.angular.z = self._rz #rad/s
            self._publisher.publish(velocity)
        else :
            print("stopped moving")
            velocity.linear.x = 0.0 #m/s
            velocity.angular.z = 0.0 #rad/s
            self._publisher.publish(velocity) 
            #rosNode.destroy_node()

        self._i+=1

class CMD_ROBOTV3:#for move3
    #publish the command for the duration
    def __init__(self,rosNode,vx,rz):
        self._vx = vx #m/s
        self._rz = rz #rad/s
        self._publisher= rosNode.create_publisher( Twist, '/cmd_vel', 10 )
        self._timer = rosNode.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        velocity=Twist()
        velocity.linear.x = self._vx #m/s
        velocity.angular.z = self._rz #rad/s
        self._publisher.publish(velocity)

if __name__=="__main__":
    rclpy.init()
    move3(0.5,0,1.0,1.0)
    rclpy.shutdown()