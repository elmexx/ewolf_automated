import rclpy
import ros2_numpy
#import math
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
        #timer_period = 0.04
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        #Lidarsensor Auflösung
        self.in_sensor = np.empty((1), dtype=[('x', np.int8),('y', np.int8),('z', np.int8),('roll', np.int_), ('pitch', np.int_), ('yaw', np.int_)]) 
        self.in_sensor['x'] = -2 #-20
        self.in_sensor['y'] = 0
        self.in_sensor['z'] = 1.5 #15
        self.in_sensor['roll'] = 90
        self.in_sensor['pitch'] = -2.5
        self.in_sensor['yaw'] = 0
    
        self.FOV_horizontal = 66.5 #81.6540
        self.width_pixels = 1080
        self.height_pixels = 1080
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
        
        # Code ohne Schleife
        [outpoint_x,outpoint_y,outpoint_z] = self.InverseTransformPoint(data[:,0],data[:,1],data[:,2])
        point_y = np.round((-outpoint_z/outpoint_x)*(self.width_pixels/2/np.tan(self.FOV_horizontal*np.pi/180/2))+self.height_pixels/2).astype('int')
        point_x = np.round((outpoint_y/outpoint_x)*(self.width_pixels/2/np.tan(self.FOV_horizontal*np.pi/180/2))+self.width_pixels/2).astype('int')
    
        indec = np.where((point_y<self.height_pixels)&(point_x<self.width_pixels)&(point_y>1) & (point_x>1))
    
        new_Image_2 = np.zeros((self.height_pixels,self.width_pixels))
        new_Image_2[point_x[indec], point_y[indec]] =  data[indec,3]
    
        cmap = plt.get_cmap('rainbow')
        rgba_img = cmap(new_Image_2)
    
        pseudoImage_2 = np.zeros((self.height_pixels,self.width_pixels,3), dtype=np.uint8)
        pseudoImage_2[:,:,0:2] = rgba_img[:,:,0:2]*255
        #pseudoImage_2[:,:,1] = rgba_img[:,:,1]*255
        #pseudoImage_2[:,:,2] = rgba_img[:,:,2]*255
    
        pseudoImage_2[:,:,:][new_Image_2 == 0] = 0
        
        msg_2 = ros2_numpy.msgify(Image, pseudoImage_2, encoding='rgb8')

        #Send Image
        self.publisher_.publish(msg_2)
        self.get_logger().info('Publishing: Stream "%s"' % self.i)
        self.i += 1
        
    # Kamerabild nach DSA
    def InverseTransformPoint(self, point_x,point_y,point_z):
        zw_point_x = point_x - self.in_sensor['x']
        zw_point_y = point_y - self.in_sensor['y']
        zw_point_z = point_z - self.in_sensor['z']
        
        cy = np.cos(self.in_sensor['yaw']*np.pi/180)
        sy = np.sin(self.in_sensor['yaw']*np.pi/180)
        cr = np.cos(self.in_sensor['roll']*np.pi/180)
        sr = np.sin(self.in_sensor['roll']*np.pi/180)
        cp = np.cos(self.in_sensor['pitch']*np.pi/180)
        sp = np.sin(self.in_sensor['pitch']*np.pi/180)
        
        out_point_x = zw_point_x * (cp * cy) + zw_point_y * (cp * sy) + zw_point_z * (sp)
        out_point_y = zw_point_x * (cy * sp * sr - sy * cr) + zw_point_y * (sy * sp * sr + cy * cr) + zw_point_z * (-cp * sr)
        out_point_z = zw_point_x * (-cy * sp * cr - sy * sr) + zw_point_y * (-sy * sp * cr + cy * sr) + zw_point_z * (cp * cr)
    
        return out_point_x,out_point_y,out_point_z


def main(args=None):
    rclpy.init(args=args)
    
    lidar_publisher = LidarPublisher()
    
    rclpy.spin(lidar_publisher)
    
    lidar_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
