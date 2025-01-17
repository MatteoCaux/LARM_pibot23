import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
import math
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

class LIDAR:
    def __init__(self, ros_node):
        self.scanMsg=None
        self.PointCloud = PointCloud()
        header = Header()
        # header.stamp = rclpy.time.now()
        header.frame_id = 'base_link'
        self.PointCloud.header = header
        print(PointCloud.header)
        ros_node.create_subscription( LaserScan, 'scan', self.listen_callback, 10)
        self._publisher= ros_node.create_publisher( PointCloud, '/com_signal/point_cloud', 10 )

    def listen_callback(self,scan):
        self.scanMsg=scan
        #print("ici")
        #print(scan)

    
    def scan_pointcloud(self):
        if self.scanMsg is not None : 
            angle= self.scanMsg.angle_min
            for aDistance in self.scanMsg.ranges :
                if 0.1 < aDistance and aDistance < 5.0 :
                    aPoint= Point32()
                    aPoint.x= (float)(math.cos(angle) * aDistance)
                    aPoint.y= (float)(math.sin( angle ) * aDistance)
                    aPoint.z= (float)(0)
                    self.PointCloud.points.append(aPoint)
                angle+= self.scanMsg.angle_increment
        else : 
            print("Scan not found, exit")
            #exit()
    
    def publish_PC(self):
        self.scan_pointcloud()
        self._publisher.publish(self.PointCloud)
        #print(lidar.PointCloud)

# def scan_callback(scanMsg):
#         global rosNode
#         lidar=LIDAR(scanMsg)
#         lidar.scan_pointcloud()
#         print(lidar.PointCloud)
#         print(type(lidar.PointCloud))

def scan_lidar():
    rclpy.init()
    aNode=Node("Scan_Lidar")
    lidar=LIDAR(aNode)

    while True :
        lidar.publish_PC()
        rclpy.spin_once( aNode )
    scanInterpret.destroy_node()
    rclpy.shutdown()
# def main():
#     print('Move move move !')

if __name__ == '__main__' :
    scan_lidar()
