//
// Created by naivehobo on 4/24/19.
//

#include <ros/ros.h>

#include <person_tracking/PersonTracker.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "person_tracker");

  auto tracker = PersonTracker();

  ros::spin();

  return 0;
}