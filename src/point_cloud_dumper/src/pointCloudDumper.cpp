#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

string saveDataDir = ".";
string stateEstimationTopic = "/aft_mapped_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

double laserTime = 0;
double odomTime = 0;

FILE *point_cloud_file, *trajectory_file;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = ros::Time::now().toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;

  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    fprintf(trajectory_file, "%f %f %f %f %f %f %f\n", odom->pose.pose.position.z, odom->pose.pose.position.x, 
            odom->pose.pose.position.y, roll, pitch, yaw, odomTime);
  } else {
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    fprintf(trajectory_file, "%f %f %f %f %f %f %f\n", odom->pose.pose.position.x, odom->pose.pose.position.y, 
            odom->pose.pose.position.z, roll, pitch, yaw, odomTime);
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserTime = laserCloud2->header.stamp.toSec();

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);
  int laserCloudNum = laserCloud->points.size();

  for (int i = 0; i < laserCloudNum; i++) {
    if (laserCloud->points[i].x > -1000000.0 && laserCloud->points[i].x < 1000000.0 && 
        laserCloud->points[i].y > -1000000.0 && laserCloud->points[i].y < 1000000.0 &&
        laserCloud->points[i].z > -1000000.0 && laserCloud->points[i].z < 1000000.0) {
      if (flipRegisteredScan) {
        fprintf(point_cloud_file, "%f %f %f %f %f\n", laserCloud->points[i].z, laserCloud->points[i].x, laserCloud->points[i].y,
                laserCloud->points[i].intensity, laserTime);
      } else {
        fprintf(point_cloud_file, "%f %f %f %f %f\n", laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z,
                laserCloud->points[i].intensity, laserTime);
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointCloudDumper");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("saveDataDir", saveDataDir);
  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 2, laserCloudHandler);

  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  string point_cloud_dir = saveDataDir + "/pointcloud_" + timeString + ".txt";
  point_cloud_file = fopen(point_cloud_dir.c_str(), "w");
  string trajectory_dir = saveDataDir + "/trajectory_" + timeString + ".txt";
  trajectory_file = fopen(trajectory_dir.c_str(), "w");

  ros::spin();

  fclose(point_cloud_file);
  fclose(trajectory_file);

  printf("\nPoint cloud and vehicle trajectory are saved on desktop.\n\n");

  return 0;
}
