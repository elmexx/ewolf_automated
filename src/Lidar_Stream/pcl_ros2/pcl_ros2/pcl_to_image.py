import rclpy
import ros2_numpy
import math
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2, Image

class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        #self.publisher_ = self.create_publisher(PointCloud2, 'lidar_topic', 10)
        self.publisher_ = self.create_publisher(Image, 'lidar_topic', 10)
        self.subscription = self.create_subscription(PointCloud2, 'scala_decoder_sdk_points', self.listener_callback, 10)
        self.subscription
        self.i = 0
        
        #Lidarsensor Auflösung
        #self.vbeamAngles = np.arange(-5.1,5.11,0.6)
        self.vbeamAngles = np.arange(-30,30,0.1)
        #self.hbeamAngles = np.concatenate((np.arange(-66.5,-15,0.25), np.arange(-15,15,0.125), np.arange(15,66.6,0.25)), axis=0)
        self.hbeamAngles = np.arange(-66.5,66.6,0.125)
        #self.vResolution = 18
        #self.hResolution = 653
        self.vResolution = len(self.vbeamAngles)+1
        self.hResolution = len(self.hbeamAngles)+1
        self.cmap = plt.get_cmap('rainbow')
        
    def listener_callback(self, msg):
    
        #Umwandlung der PointCloud in ein Image
        data_in = ros2_numpy.numpify(msg)    
                
        points=np.zeros((data_in.shape[0],4))
        points[:,0]=data_in['x']
        points[:,1]=data_in['y']
        points[:,2]=data_in['z']
        points[:,3]=data_in['intensity']
        
        data=np.array(points, dtype=np.float32)
        
        #self.get_logger().info('Data: "%s"' % data)
        #np.savetxt('data.csv', data_2, delimiter=',')
        
        #Calculate radial distance for every point
        r = np.sqrt(data[:,0]**2 + data[:,1]**2)
        r[r==0] = 1e-6
        
        #Calculate the pitch and yaw angles for each point
        pitch = [math.degrees(math.atan2(data[i,2], r[i])) for i in range(len(r))]
        yaw = [math.degrees(math.atan2(data[i,1], data[i,0])) for i in range(len(data[:,0]))]
        
        #Calculate the row indices for all points based on the bin in which the yaw angle for each point fall into
        rowIdx = np.digitize(pitch,self.vbeamAngles)
        #rowIdx[rowIdx==0] = np.nan
        #rowIdx = self.vResolution - rowIdx + 1
        
        #Calculate the column indices for all points based on the bin in which the yaw angle for each point fall into
        colIdx = np.digitize(yaw, self.hbeamAngles)
        #colIdx[colIdx==0] = np.nan
        
        #Create PseudoImage
        #pseudoImage = np.empty((self.vResolution, self.hResolution), dtype=[('x',np.float32),('y',np.float32), ('z',np.float32), ('r',np.float32), ('g',np.float32), ('b',np.float32)]) #('intensity',np.float32)])
        #pseudoImage['x'] = np.nan
        #pseudoImage['y'] = np.nan
        #pseudoImage['z'] = np.nan
        #pseudoImage['r'] = 0
        #pseudoImage['g'] = 0
        #pseudoImage['b'] = 0
        #pseudoImage['intensity'] = np.nan
        pseudoIntensity = np.zeros((self.vResolution, self.hResolution))
        ptDistance = np.zeros((self.vResolution, self.hResolution))
        
        for i in range(len(data)):
            if ~np.isnan(rowIdx[i]) and ~np.isnan(colIdx[i]):
                if pseudoIntensity[rowIdx[i],colIdx[i]] == 0: #np.isnan(pseudoImage[rowIdx[i],colIdx[i]]['x']):
                    #pseudoImage[rowIdx[i],colIdx[i]]['x'] = data[i,0]
                    #pseudoImage[rowIdx[i],colIdx[i]]['y'] = data[i,1]
                    #pseudoImage[rowIdx[i],colIdx[i]]['z'] = data[i,2]
                    #pseudoImage[rowIdx[i],colIdx[i]]['intensity'] = data[i,3]
                    pseudoIntensity[rowIdx[i],colIdx[i]]  = data[i,3]
                    ptDistance[rowIdx[i],colIdx[i]] = np.linalg.norm(np.array([data[i,0], data[i,1], data[i,2]]))
                else:
                    update_Distance = np.linalg.norm(np.array([data[i,0], data[i,1], data[i,2]]))
                    if update_Distance < ptDistance[rowIdx[i],colIdx[i]]:
                        #pseudoImage[rowIdx[i],colIdx[i]]['x'] = data[i,0]
                        #pseudoImage[rowIdx[i],colIdx[i]]['y'] = data[i,1]
                        #pseudoImage[rowIdx[i],colIdx[i]]['z'] = data[i,2]
                        #pseudoImage[rowIdx[i],colIdx[i]]['intensity'] = data[i,3]
                        pseudoIntensity[rowIdx[i],colIdx[i]]  = data[i,3]
                        ptDistance[rowIdx[i],colIdx[i]] = update_Distance
        
        
        rgba_img = self.cmap(ptDistance)
        
        #pseudoImage[:]['r'] = rgba_img[:,:,0]*255
        #pseudoImage[:]['g'] = rgba_img[:,:,1]*255
        #pseudoImage[:]['b'] = rgba_img[:,:,2]*255
        
        pseudoImage_2 = np.zeros((self.vResolution, self.hResolution,3), dtype=np.uint8)
        pseudoImage_2[:,:,0] = rgba_img[:,:,0]*255
        pseudoImage_2[:,:,1] = rgba_img[:,:,1]*255
        pseudoImage_2[:,:,2] = rgba_img[:,:,2]*255
        
        pseudoImage_2[:,:,:][ptDistance==0] = 0
        
        msg_2 = ros2_numpy.msgify(Image, pseudoImage_2, encoding='rgb8')

        #Send Image
        self.publisher_.publish(msg_2)
        self.get_logger().info('Publishing: Stream "%s"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    lidar_publisher = LidarPublisher()
    
    rclpy.spin(lidar_publisher)
    
    lidar_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
