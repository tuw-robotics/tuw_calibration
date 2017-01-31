#ifndef LASER2CORNER_NODE_H
#define LASER2CORNER_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>

#include <tuw_geometry/linesegment2d.h>
#include <tuw_geometry_msgs/LineSegments.h>


class Laser2CornerNode
{
public:
  Laser2CornerNode(); // Constructor
private:
  ros::NodeHandle nh_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::vector<tuw::LineSegment2D> linesegments_;
  
  ros::Subscriber sub_segments_;
  
  void callbackSegments(const tuw_geometry_msgs::LineSegments &_segments_msg);
};

#endif //LASER2CORNER_NODE_H