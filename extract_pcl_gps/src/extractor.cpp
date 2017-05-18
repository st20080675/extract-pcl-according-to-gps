#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int frame_counter = 0;
std::string save_path = "/home/sunting/Documents/program/3d_data/hkust_hanghao/2/bin_data/";
std::vector <float *> pcl_buffer;
std::vector <unsigned int> pcl_size_buffer;
std::vector <double> pcl_header_time_buffer;



void callback_pcl(const PointCloud::ConstPtr& msg)
{
  //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

  float * temp_data; 
  temp_data = new float[msg->size() * 4];
  int j = 0; 

  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  {
      temp_data[j++] = pt.x;
      temp_data[j++] = pt.y;
      temp_data[j++] = pt.z;
      temp_data[j++] = pt.data[3];
  }

  pcl_buffer.push_back(temp_data);
  pcl_size_buffer.push_back(msg->size());
  pcl_header_time_buffer.push_back(msg->header.stamp);
    
/*
  sensor_msgs::Image msg_image;
  cv::Mat image;
  pcl::toROSMsg(cloud, msg_image);
*/
   
}


void callback_gps(const sensor_msgs::NavSatFixConstPtr& fix) 
{
    // save file to binary
    std::string cur_frame_save_path = save_path + std::to_string(frame_counter/100000) + std::to_string((frame_counter%100000)/10000) + std::to_string((frame_counter%10000)/1000) + std::to_string((frame_counter%1000)/100) + std::to_string((frame_counter%100)/10) + std::to_string(frame_counter%10); 

   double cur_fix_time = fix->header.stamp.toSec();
   int i = pcl_header_time_buffer.size() - 1;
   
   while ((i > 0) && (pcl_header_time_buffer[i - 1] > cur_fix_time))
   {  
       i--;
   }   
   
    FILE *file = fopen(cur_frame_save_path.c_str(), "wb");
    fwrite(pcl_buffer[i], sizeof(float), pcl_size_buffer[i] * 4, file);
    fclose(file);

    std::cout << frame_counter << std::endl;
    frame_counter++;
    
    for (unsigned i = 0; i < pcl_buffer.size(); i++)
    {
       delete[] pcl_buffer[i];
    }

    pcl_buffer.clear();
    pcl_size_buffer.clear();
    pcl_header_time_buffer.clear();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl_gps");
  ros::NodeHandle nh;
  ros::Subscriber sub_pcl = nh.subscribe<PointCloud>("/velodyne_points", 1, callback_pcl);
  ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, callback_gps);
  ros::spin();
}
