#ifndef LASER2CORNER_NODE_H
#define LASER2CORNER_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>

#include <tuw_geometry/linesegment2d.h>
#include <tuw_geometry_msgs/LineSegments.h>

/**
 * @class Laser2CornerNode
 * @brief Create tf from laser to a corner using detected line segments in the 2D scan
 */
class Laser2CornerNode
{
public:
  /**
   * Constructor
   */
  Laser2CornerNode();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::vector<tuw::LineSegment2D> linesegments_;

  double corner_point_tolerance_;  /// max. difference between two points to form a corner
  double corner_point_x_;          /// heuristic ref. point to find line segments forming a corner
  double corner_point_y_;          /// heuristic ref. point to find line segments forming a corner

  ros::Subscriber sub_segments_;
  ros::Publisher pub_marker_;

  /**
    * @brief Callback for received LineSegments msgs
    *
    * @param _segments_msg line segments from line segment detector node
    */
  void callbackSegments(const tuw_geometry_msgs::LineSegments &_segments_msg);
};

#endif  // LASER2CORNER_NODE_H
