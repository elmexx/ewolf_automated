#define BOOST_BIND_NO_PLACEHOLDERS

#include <functional>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <pthread.h>
/*#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>
*/
using std::placeholders::_1;

class Lidar_Subscriber : public rclcpp::Node
{
public:
    Lidar_Subscriber()
        : Node("lidar_converter")
    {     
        in_sensor_vm_.x = -1.507-0.5; //-20
        in_sensor_vm_.y = 0.0;
        in_sensor_vm_.z = 1.255; //15
        in_sensor_vm_.roll = 0.0 *M_PI/180;
        in_sensor_vm_.pitch = -10.0 *M_PI/180;
        in_sensor_vm_.yaw = 0.0 *M_PI/180;
        
        in_sensor_vl_.x = 0.18; //-20
        in_sensor_vl_.y = -0.78;
        in_sensor_vl_.z = -0.103; //15
        in_sensor_vl_.roll = 0.0 *M_PI/180;
        in_sensor_vl_.pitch = 0.0 *M_PI/180;
        in_sensor_vl_.yaw = 45.0 *M_PI/180;
        
        in_sensor_vr_.x = 0.18; //-20
        in_sensor_vr_.y = 0.78;
        in_sensor_vr_.z = -0.103; //15
        in_sensor_vr_.roll = 0.0 *M_PI/180;
        in_sensor_vr_.pitch = 0.0 *M_PI/180;
        in_sensor_vr_.yaw = -45.0 *M_PI/180;
        
        /*sensor_transform_vm.header.stamp = this->get_clock()->now();
        sensor_transform_vm.header.frame_id = "Kamera";
        sensor_transform_vm.child_frame_id = "Lidar_mitte";        
        sensor_transform_vm.transform.translation.x = in_sensor_vm_.x;
        sensor_transform_vm.transform.translation.y = in_sensor_vm_.y;
        sensor_transform_vm.transform.translation.z = in_sensor_vm_.z;        
        tf2::Quaternion myQuaternion_vm;
        myQuaternion_vm.setRPY(in_sensor_vm_.roll, in_sensor_vm_.pitch, in_sensor_vm_.yaw);
        sensor_transform_vm.transform.rotation.x = myQuaternion_vm.x();
        sensor_transform_vm.transform.rotation.y = myQuaternion_vm.y();
        sensor_transform_vm.transform.rotation.z = myQuaternion_vm.z();
        sensor_transform_vm.transform.rotation.w = myQuaternion_vm.w();
        
        sensor_transform_vl.header.stamp = this->get_clock()->now();
        sensor_transform_vl.header.frame_id = "Lidar_mitte";
        sensor_transform_vl.child_frame_id = "Lidar_links";        
        sensor_transform_vl.transform.translation.x = in_sensor_vl_.x;
        sensor_transform_vl.transform.translation.y = in_sensor_vl_.y;
        sensor_transform_vl.transform.translation.z = in_sensor_vl_.z;        
        tf2::Quaternion myQuaternion_vl;
        myQuaternion_vl.setRPY(in_sensor_vl_.roll, in_sensor_vl_.pitch, in_sensor_vl_.yaw);
        sensor_transform_vl.transform.rotation.x = myQuaternion_vl.x();
        sensor_transform_vl.transform.rotation.y = myQuaternion_vl.y();
        sensor_transform_vl.transform.rotation.z = myQuaternion_vl.z();
        sensor_transform_vl.transform.rotation.w = myQuaternion_vl.w();
        
        sensor_transform_vr.header.stamp = this->get_clock()->now();
        sensor_transform_vr.header.frame_id = "Lidar_mitte";
        sensor_transform_vr.child_frame_id = "Lidar_rechts";        
        sensor_transform_vr.transform.translation.x = in_sensor_vr_.x;
        sensor_transform_vr.transform.translation.y = in_sensor_vr_.y;
        sensor_transform_vr.transform.translation.z = in_sensor_vr_.z;        
        tf2::Quaternion myQuaternion_vr;
        myQuaternion_vr.setRPY(in_sensor_vr_.roll, in_sensor_vr_.pitch, in_sensor_vr_.yaw);
        sensor_transform_vr.transform.rotation.x = myQuaternion_vr.x();
        sensor_transform_vr.transform.rotation.y = myQuaternion_vr.y();
        sensor_transform_vr.transform.rotation.z = myQuaternion_vr.z();
        sensor_transform_vr.transform.rotation.w = myQuaternion_vr.w();
        */
        pcl_frame_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scala_decoder_sdk_points_2", 10, std::bind(&Lidar_Subscriber::topic_callback, this, _1));
            
        pcl_frame_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scala_decoder_sdk_points", 10, std::bind(&Lidar_Subscriber::save_pcl_2, this, _1));
            
        pcl_frame_3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scala_decoder_sdk_points_3", 10, std::bind(&Lidar_Subscriber::save_pcl_3, this, _1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("lidar_topic", 10);
        
        pt_cloud_2_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>); 
        pt_cloud_3_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>); 
    }

private:
    struct Sensor{float x, y, z, roll, pitch, yaw;};

    pcl::PointXYZI transform_sensor_camera(Sensor sensor_input, pcl::PointXYZI pt_point_In) const
    {         
        double zw_point_x = pt_point_In.x; // - sensor_input.x;
        double zw_point_y = pt_point_In.y; // - sensor_input.y;
        double zw_point_z = pt_point_In.z; // - sensor_input.z;
    
        double cp = cos(sensor_input.pitch);
        double sp = sin(sensor_input.pitch);
            
        pcl::PointXYZI pt_point_Out;
            
        pt_point_Out.x = (zw_point_x * cp) + (zw_point_z * sp);
        pt_point_Out.y = zw_point_y;
        pt_point_Out.z = (zw_point_x * -sp) + (zw_point_z * cp);
        pt_point_Out.intensity = pt_point_In.intensity;  

        pt_point_Out.x =  pt_point_Out.x - sensor_input.x;
        pt_point_Out.y =  pt_point_Out.y - sensor_input.y;
        pt_point_Out.z =  pt_point_Out.z - sensor_input.z;

        return pt_point_Out;      
    }

    pcl::PointXYZI transform_sensor_sensor(Sensor sensor_input, pcl::PointXYZI pt_point_In) const
    {         
        double zw_point_x = pt_point_In.x; // - sensor_input.x;
        double zw_point_y = pt_point_In.y; // - sensor_input.y;
        double zw_point_z = pt_point_In.z; // - sensor_input.z;
    
        double cy = cos(sensor_input.yaw);
        double sy = sin(sensor_input.yaw);
            
        pcl::PointXYZI pt_point_Out;
            
        pt_point_Out.x = (cy * zw_point_x) - (sy * zw_point_y);
        pt_point_Out.y = (sy * zw_point_x) + (cy * zw_point_y);
        pt_point_Out.z = zw_point_z;
        pt_point_Out.intensity = pt_point_In.intensity;  

        pt_point_Out.x =  pt_point_Out.x - sensor_input.x;
        pt_point_Out.y =  pt_point_Out.y - sensor_input.y;
        pt_point_Out.z =  pt_point_Out.z - sensor_input.z;

        return pt_point_Out;      
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        //RCLCPP_INFO(this->get_logger(), "Publishing: Stream - Start");
        pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud (new pcl::PointCloud<pcl::PointXYZI>);   
        //pcl::PointCloud<pcl::PointXYZI> pt_cloud;
        pcl::fromROSMsg(*msg,*pt_cloud);
        
        new_Image_2 = cv::Mat::zeros(cv::Size(width_pixels, height_pixels), CV_8UC1);
        pseudoImage_2 = cv::Mat::zeros(cv::Size(width_pixels, height_pixels), CV_8UC3);
        
        // Merge die PointClouds
        if (pt_cloud_2_->width > 1)
        {
            pthread_mutex_lock(&mutex_2);
            *pt_cloud += *pt_cloud_2_;
            pthread_mutex_unlock(&mutex_2);
            //RCLCPP_INFO(this->get_logger(), "Add PCL 2");
        }
        if (pt_cloud_3_->width > 1)
        {
            pthread_mutex_lock(&mutex_3);
            *pt_cloud += *pt_cloud_3_;
            pthread_mutex_unlock(&mutex_3);
            //RCLCPP_INFO(this->get_logger(), "Add PCL 3");
        }       
        
        // Transformiere PCL_Cloud
        //sensor_msgs::msg::PointCloud2 msg_trans;
        //sensor_msgs::msg::PointCloud2 msg_all;
        
        //pcl::toROSMsg(*pt_cloud,msg_all);      
        //pcl_ros::transformPointCloud("Valeo_VM_trans", sensor_transform_vm, msg_all, msg_trans);
        //pcl::fromROSMsg(msg_trans,*pt_cloud);
        
        //RCLCPP_INFO(this->get_logger(), "Publishing: Stream - Start Loop");
        
        double minVal= DBL_MAX; 
        double maxVal= 0;
        for (size_t i = 0; i < pt_cloud->size(); ++i) 
        {
            if(pt_cloud->points[i].intensity > maxVal)
            {
                maxVal = pt_cloud->points[i].intensity;
            }
            if(pt_cloud->points[i].intensity<minVal)
            {
                minVal = pt_cloud->points[i].intensity;
            }
        }
        
        for (size_t i = 0; i < pt_cloud->size(); ++i) 
        {          
            // Aufruf der Transformations-Funktion
            pcl::PointXYZI pt_point = pt_cloud->points[i];
            
            pcl::PointXYZI pt_point_trans = transform_sensor_camera(in_sensor_vm_, pt_point);
        
            double out_point_x = pt_point_trans.x;
            double out_point_y = pt_point_trans.y;
            double out_point_z = pt_point_trans.z;
            double intens = pt_point_trans.intensity;

            /*double out_point_x = pt_cloud->points[i].x;
            double out_point_y = pt_cloud->points[i].y;
            double out_point_z = pt_cloud->points[i].z;
            double intens = pt_cloud->points[i].intensity;
            */

            //double temp = (-out_point_z / out_point_x) * (width_pixels / 2 / tan(FOV_horizontal * M_PI / 180 / 2)) + height_pixels / 2;
            double w_x = 1.0 / tan(FOV_horizontal * M_PI / 180 / 2);
            //double w_y = 1.0 / tan(FOV_horizontal * M_PI / 180 / 2) * width_pixels/height_pixels;
            //double temp = ((out_point_y/out_point_z) * height_pixels) / (2 * w_y) + (height_pixels / 2);
            double temp = ((-out_point_z / out_point_x) * width_pixels) / (2 * w_x) + (height_pixels / 2);
            int pixel_y = std::round(temp);
            
            //temp = (out_point_y / out_point_x) * (width_pixels / 2 / tan(FOV_horizontal * M_PI / 180 / 2)) + width_pixels / 2;
            //temp = ((out_point_x/out_point_z) * width_pixels) / (2 * w_x) + (width_pixels / 2);
            temp = ((-out_point_y / out_point_x) * width_pixels) / (2 * w_x) + (width_pixels / 2);
            int pixel_x = std::round(temp);
    
            if (pixel_y < height_pixels && pixel_x < width_pixels && pixel_y >= 0 && pixel_x >= 0) 
            {   
                int intens_2 =  ((intens-minVal)/(maxVal-minVal))*255;
                //printf( "Image %f %f %f %d \n", maxVal, minVal,intens, intens_2);
                if (new_Image_2.at<uchar>(pixel_y, pixel_x) < intens_2)
                {
                    new_Image_2.at<uchar>(pixel_y, pixel_x) = (unsigned char)intens_2;         
                }    
            }
        }
        
        //RCLCPP_INFO(this->get_logger(), "Publishing: Stream - End Loop");
        
        // Skalieren new_Image_2 auf 0-255
        /*double minVal, maxVal;
        cv::minMaxLoc(new_Image_2, &minVal, &maxVal);
        
        new_Image_2 = ((new_Image_2-minVal)/(maxVal-minVal))*255.0;
        new_Image_2 = (255.0-new_Image_2);
        
        cv::Mat new_Image_3(cv::Size(width_pixels, height_pixels), CV_8UC1);
        new_Image_2.convertTo(new_Image_3, CV_8UC1);*/
        
        // Cmap Umwandlung
        //cv::Mat pseudoImage_2(cv::Size(width_pixels, height_pixels), CV_8UC3);
        
        new_Image_2 = (255-new_Image_2);
        cv::applyColorMap(new_Image_2, pseudoImage_2, cv::COLORMAP_JET);
        
        for (int j=0; j<width_pixels; j++)
        {   
            for (int i=0; i<height_pixels; i++)
            {         
                if (new_Image_2.at<uchar>(i,j) > 254.0)
                {
                    pseudoImage_2.at<cv::Vec3b>(i, j)[0] = 0;
                    pseudoImage_2.at<cv::Vec3b>(i, j)[1] = 0;
                    pseudoImage_2.at<cv::Vec3b>(i, j)[2] = 0;
                }
            }
        }          
        //cv::cvtColor(pseudoImage_2, pseudoImage_2, CV_BGR2RGB);
        
        sensor_msgs::msg::Image::SharedPtr msg_out = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",pseudoImage_2)
                .toImageMsg();
        publisher_->publish(*msg_out.get()); 
        
        RCLCPP_INFO(this->get_logger(), "Publishing: Stream");
    }
    
    void save_pcl_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg_2) 
    {     
        //RCLCPP_INFO(this->get_logger(), "Start PCL_2");
        // Transformiere PCL_Cloud
        //sensor_msgs::msg::PointCloud2 msg_2_trans;
        //pcl_ros::transformPointCloud("Valeo_VL_trans", sensor_transform_vl, *msg_2, msg_2_trans);
        
        
        // Speichere die PCL
        pthread_mutex_lock(&mutex_2);
        pcl::fromROSMsg(*msg_2,*pt_cloud_2_);
        
        for (size_t i = 0; i < pt_cloud_2_->size(); ++i) 
        {
            pcl::PointXYZI pt_point = pt_cloud_2_->points[i];
            
            pcl::PointXYZI pt_point_trans = transform_sensor_sensor(in_sensor_vl_, pt_point);
        
            pt_cloud_2_->points[i] = pt_point_trans;
        }
        pthread_mutex_unlock(&mutex_2);
        //RCLCPP_INFO(this->get_logger(), "Ende PCL_2");
    }
    
    void save_pcl_3(const sensor_msgs::msg::PointCloud2::SharedPtr msg_3) 
    {     
        //RCLCPP_INFO(this->get_logger(), "Start PCL_3");
        // Transformiere PCL_Cloud
        //sensor_msgs::msg::PointCloud2 msg_3_trans;
        //pcl_ros::transformPointCloud("Valeo_VR_trans", sensor_transform_vr, *msg_3, msg_3_trans);
        
        
        // Speichere die PCL        
        pthread_mutex_lock(&mutex_3);
        pcl::fromROSMsg(*msg_3,*pt_cloud_3_);
        
        for (size_t i = 0; i < pt_cloud_3_->size(); ++i) 
        {
            pcl::PointXYZI pt_point = pt_cloud_3_->points[i];
            
            pcl::PointXYZI pt_point_trans = transform_sensor_sensor(in_sensor_vr_, pt_point);
        
            pt_cloud_3_->points[i] = pt_point_trans;
        }
        pthread_mutex_unlock(&mutex_3);
        //RCLCPP_INFO(this->get_logger(), "Ende PCL_3");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_frame_2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_frame_3_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud_2_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud_3_;
    
    pthread_mutex_t mutex_2 = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mutex_3 = PTHREAD_MUTEX_INITIALIZER;
    
    //Lidarsensor Auflösung
    Sensor in_sensor_vm_;
    Sensor in_sensor_vl_;
    Sensor in_sensor_vr_;
    
    //geometry_msgs::msg::TransformStamped sensor_transform_vm;
    //geometry_msgs::msg::TransformStamped sensor_transform_vl;
    //geometry_msgs::msg::TransformStamped sensor_transform_vr;
    
    float FOV_horizontal = 81.0; //66.5; //81.6540
    int width_pixels = 1280;
    int height_pixels = 720;
    
    cv::Mat new_Image_2;
    cv::Mat pseudoImage_2;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar_Subscriber>());
    rclcpp::shutdown();
    return 0;
}
