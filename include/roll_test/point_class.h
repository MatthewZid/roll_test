#ifndef POINT_CLASS_H
#define POINT_CLASS_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <regex>
#include <fstream>

namespace roll_test
{

typedef struct pointClass
{
  std::string name;
  ros::Time stamp;
  std::string topic;
  std::string type;
  std::vector<geometry_msgs::Point> points;
}PointClass;

std::vector<PointClass> readcsv();
size_t binarySearch(const std::vector<PointClass>& pcvec, size_t l, size_t r, double x);
size_t cluster_search(const std::vector<PointClass>& pcvec, const ros::Time x);

}

#endif