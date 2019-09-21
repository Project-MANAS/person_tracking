//
// Created by naivehobo on 4/24/19.
//

#include "person_tracking/PersonTracker.h"

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <string>
#include <vector>

PersonTracker::PersonTracker() : private_nh_("~") {
  private_nh_.param("keep_points_upto", keep_points_upto_, 10.0);
  private_nh_.param("distance_to_maintain", distance_to_maintain_, 1.5);
  private_nh_.param<std::string>("lidar_topic", lidar_topic_, "scan");
  private_nh_.param<std::string>("tracking_topic", tracking_topic_, "start_tracking");
  private_nh_.param<std::string>("cmd_vel_topic", cmd_topic_, "cmd_vel");
  private_nh_.param("linear_threshold", linear_threshold_, 0.1);
  private_nh_.param("angular_threshold", angular_threshold_, 0.1);
  private_nh_.param("max_angular_speed", max_angular_speed_, 1.0);
  private_nh_.param("min_angular_speed", min_angular_speed_, 0.0);
  private_nh_.param("max_linear_speed", max_linear_speed_, 0.5);
  private_nh_.param("min_linear_speed", min_linear_speed_, 0.1);
  private_nh_.param("time_to_x", time_to_x_, 1.0);
  private_nh_.param("time_to_angle", time_to_angle_, 1.0);
  private_nh_.param("clip_cloud", clip_cloud_, true);
  private_nh_.param("clip_angle_min", clip_angle_min_, -1.31);
  private_nh_.param("clip_angle_max", clip_angle_max_, 1.31);

  start_tracking_ = false;

  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.ns = "bezier";
  marker_.scale.x = 0.2;
  marker_.scale.y = 0.2;
  marker_.scale.z = 0.2;
  marker_.color.a = 1.0;
  marker_.color.r = 1.0;
  marker_.color.g = 0.0;
  marker_.color.b = 0.0;

  wp_marker_.id = 0;
  wp_marker_.type = visualization_msgs::Marker::SPHERE;
  wp_marker_.action = visualization_msgs::Marker::ADD;
  wp_marker_.ns = "bezier";
  wp_marker_.scale.x = 0.2;
  wp_marker_.scale.y = 0.2;
  wp_marker_.scale.z = 0.2;
  wp_marker_.color.a = 1.0;
  wp_marker_.color.r = 0.0;
  wp_marker_.color.g = 0.0;
  wp_marker_.color.b = 1.0;

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("clusters", 1);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_wp", 1);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

  cloud_sub_ = nh_.subscribe(lidar_topic_, 1, &PersonTracker::cloudCallback, this);
  tracking_sub_ = nh_.subscribe(tracking_topic_, 1, &PersonTracker::trackingCallback, this);
}

std::vector<pcl::PointXYZ> PersonTracker::getClusterCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_ptr, *cloud_2d);
  for (auto &point : cloud_2d->points)
    point.z = 0;

  if (!cloud_2d->points.empty())
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.3);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);

  std::vector<pcl::PointXYZ> cluster_centroids;
  for (auto &it : cluster_indices) {
    pcl::PointXYZ cluster_centroid;
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (auto &pit : it.indices)
      centroid.add(cloud_2d->points[pit]);
    centroid.get(cluster_centroid);
    cluster_centroids.push_back(cluster_centroid);
  }

  return cluster_centroids;
}

void PersonTracker::removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                                      float in_max_height,
                                      float in_floor_max_angle) {
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height);
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*out_cloud_ptr);
}

pcl::PointXYZ PersonTracker::getClosestPoint(pcl::PointXYZ &p, std::vector<pcl::PointXYZ> &points) {
  auto min_distance = DBL_MAX;
  double distance;
  pcl::PointXYZ closest_point;
  for (auto &i : points) {
    distance = euclideanDistance(p, i);
    if (distance < min_distance) {
      closest_point = i;
      min_distance = distance;
    }
  }
  return closest_point;
}

void PersonTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*cloud, *cloud_ptr);

  if (keep_points_upto_ > 3.0)
    removeGroundPlane(cloud_ptr, ground_filtered_ptr);
  else
    ground_filtered_ptr = cloud_ptr;

  if (clip_cloud_)
    keepPointsUpTo(ground_filtered_ptr, filtered_ptr, keep_points_upto_, !start_tracking_);
  else
    keepPointsUpTo(ground_filtered_ptr, filtered_ptr, keep_points_upto_, false);

  auto cluster_centroids = getClusterCentroid(filtered_ptr);

  marker_.header = cloud->header;
  marker_.points.clear();
  geometry_msgs::Point cp;

  auto min_distance = DBL_MAX;
  double dist;

  if (!start_tracking_) {
    ROS_INFO("Found %d objects!", (int) cluster_centroids.size());
    for (int i = 0; i < cluster_centroids.size(); i++) {
      cp.x = cluster_centroids[i].x;
      cp.y = cluster_centroids[i].y;
      cp.z = cluster_centroids[i].z;
      marker_.points.push_back(cp);
      dist = euclideanDistance(cluster_centroids[i]);
      if (dist < min_distance) {
        object_position_ = cluster_centroids[i];
        min_distance = dist;
      }
      ROS_INFO("Object %d found at (%lf, %lf, %lf)", i + 1, cp.x, cp.y, cp.z);
    }
    marker_pub_.publish(marker_);
  }

  geometry_msgs::Twist move_cmd;

  if (start_tracking_) {
    if (!cluster_centroids.empty()) {
      auto p = getClosestPoint(object_position_, cluster_centroids);
      if (euclideanDistance(p, object_position_) > 1.0) {
        ROS_WARN("Object being tracked was lost!");
        cmd_pub_.publish(move_cmd);
        return;
      }
      object_position_ = p;
      ROS_INFO("Tracking object at (%lf, %lf)", object_position_.x, object_position_.y);
      wp_marker_.pose.position.x = object_position_.x;
      wp_marker_.pose.position.y = object_position_.y;
      wp_marker_.pose.position.z = 0.0;
      wp_marker_.header = cloud->header;
      goal_pub_.publish(wp_marker_);

      auto disp = euclideanDistance(object_position_);
      auto theta = atan2(object_position_.y, object_position_.x);

      if ((abs(disp - distance_to_maintain_) > linear_threshold_)
          || (abs(theta) > angular_threshold_)) {
        auto linear_speed = (disp - distance_to_maintain_) / time_to_x_;
        auto angular_speed = theta / time_to_angle_;

        linear_speed = copysign(fmax(min_linear_speed_,
                                     fmin(max_linear_speed_, abs(linear_speed))), linear_speed);
        angular_speed = copysign(fmax(min_angular_speed_,
                                      fmin(max_angular_speed_, abs(angular_speed))), angular_speed);

        move_cmd.linear.x = linear_speed;
        move_cmd.angular.z = angular_speed;
      }
      cmd_pub_.publish(move_cmd);
    } else
      ROS_INFO("No object to track!");
  }

}

void PersonTracker::trackingCallback(const std_msgs::Bool &msg) {
  start_tracking_ = msg.data;
}

void PersonTracker::keepPointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, double distance, bool clipCloud) {
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    if (clipCloud) {
      auto a = atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x);
      if (a <= clip_angle_min_ || a >= clip_angle_max_)
        continue;
    }
    if (euclideanDistance(in_cloud_ptr->points[i]) < distance)
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
  }
}
