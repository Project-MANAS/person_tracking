//
// Created by naivehobo on 4/24/19.
//

#ifndef PERSON_TRACKING_PERSONTRACKER_H
#define PERSON_TRACKING_PERSONTRACKER_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <vector>

class PersonTracker {

 public:
  PersonTracker();

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  void trackingCallback(const std_msgs::Bool &msg);

  void keepPointsUpTo(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                      double distance, bool clipCloud);
  std::vector<pcl::PointXYZ> getClusterCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr);
  void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                         float in_max_height = 0.2,
                         float in_floor_max_angle = 0.1);
  pcl::PointXYZ getClosestPoint(pcl::PointXYZ& p, std::vector<pcl::PointXYZ>& points);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher goal_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cmd_pub_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber tracking_sub_;

  visualization_msgs::Marker marker_;
  visualization_msgs::Marker wp_marker_;

  pcl::PointXYZ object_position_;

  double keep_points_upto_;
  double distance_to_maintain_;
  double linear_threshold_;
  double angular_threshold_;
  double time_to_x_;
  double time_to_angle_;
  double max_angular_speed_;
  double min_angular_speed_;
  double max_linear_speed_;
  double min_linear_speed_;
  double clip_angle_min_;
  double clip_angle_max_;
  bool clip_cloud_;

  std::string lidar_topic_;
  std::string tracking_topic_;
  std::string cmd_topic_;

  bool start_tracking_;

  double euclideanDistance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
  }

  double euclideanDistance(pcl::PointXYZ pt) {
    return sqrt(pow(pt.x, 2) + pow(pt.y, 2));
  }

};

#endif //PERSON_TRACKING_PERSONTRACKER_H
