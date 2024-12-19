#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <math.h>
#include <rosbag/bag.h>
#include <ceres/ceres.h>

#include "ba.hpp"
#include "tools.hpp"
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 100);
  ros::Publisher pub_debug = nh.advertise<sensor_msgs::PointCloud2>("/cloud_debug", 100);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);
  ros::Publisher pub_trajectory = nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 100);
  ros::Publisher pub_pose_number = nh.advertise<visualization_msgs::MarkerArray>("/pose_number", 100);

  string file_path;
  double downsample_size, marker_size;
  int pcd_name_fill_num;

  nh.getParam("file_path", file_path);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  nh.getParam("marker_size", marker_size);

  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<mypcl::pose> pose_vec;

  pose_vec = mypcl::read_pose(file_path + "pose.json");
  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;

  pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr pc_full(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_full(new pcl::PointCloud<pcl::PointXYZRGB>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "camera_init";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  //cout<<"push enter to view"<<endl;
  //getchar();
  for(size_t i = 0; i < pose_size; i++)
  {
    mypcl::loadPCD(file_path + "pcd/", pcd_name_fill_num, pc_surf, i);

    pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
    pc_filtered->resize(pc_surf->points.size());
    int cnt = 0;
    for(size_t j = 0; j < pc_surf->points.size(); j++)
    {
      pc_filtered->points[cnt] = pc_surf->points[j];
      cnt++;
    }
    pc_filtered->resize(cnt);
    
    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);
    
    for(size_t j = 0; j < pc_filtered->points.size(); j++)
    {
      pc_full->push_back(pc_filtered->points[j]);
    }

//    downsample_voxel(*pc_filtered, downsample_size);

  }

//    pcl::io::savePCDFile(file_path + "hba_map_before_voxel.pcd", *pc_full);
    downsample_voxel(*pc_full, downsample_size);
    pcl::io::savePCDFile(file_path + "hba_map_after_voxel.pcd", *pc_full);

}
