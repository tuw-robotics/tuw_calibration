#include "laser2corner_node.h"

#include <geometry_msgs/Pose2D.h>

using std::vector;
using std::string;

Laser2CornerNode::Laser2CornerNode()
{
  tf_listener_ = std::make_shared<tf::TransformListener>();
  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  sub_segments_ = nh_.subscribe("line_segments", 1000, &Laser2CornerNode::callbackSegments, this);
}

void Laser2CornerNode::callbackSegments(const tuw_geometry_msgs::LineSegments& _segments_msg)
{
  // assume corner lies near (1, -1, 0) wrt laser_base
  tuw::Point2D pc(1, -1);

  linesegments_.resize(_segments_msg.segments.size());

  if (linesegments_.size() < 2)
  {
    ROS_WARN("need at least two lines for corner");
    return;
  }

  for (int i = 0; i < linesegments_.size(); i++)
  {
    linesegments_[i].set(_segments_msg.segments[i].p0.x, _segments_msg.segments[i].p0.y, _segments_msg.segments[i].p1.x,
                         _segments_msg.segments[i].p1.y);
  }

  // search 2 lines nearest to this point and copy lines to member variable
  double closest_dist_1 = linesegments_[0].distanceTo(pc);
  double closest_dist_2 = linesegments_[1].distanceTo(pc);
  int closest_idx_1 = 0;
  int closest_idx_2 = 1;

  if (closest_dist_1 > closest_dist_2)
  {
    closest_dist_1 = linesegments_[1].distanceTo(pc);
    closest_dist_2 = linesegments_[0].distanceTo(pc);
    closest_idx_1 = 1;
    closest_idx_2 = 0;
  }

  double tmp_dist;
  for (int i = 2; i < linesegments_.size(); i++)
  {
    tmp_dist = linesegments_[i].distanceTo(pc);
    if (tmp_dist < closest_dist_1)
    {
      closest_dist_1 = tmp_dist;
      closest_idx_1 = i;
    }
    else if (tmp_dist < closest_dist_2)
    {
      closest_dist_2 = tmp_dist;
      closest_idx_2 = i;
    }
  }
  
  ROS_INFO("linesegment[%d] closest 1, p0 = (%f, %f), p1 = (%f, %f)", closest_idx_1, linesegments_[closest_idx_1].p0().x(),
       linesegments_[closest_idx_1].p0().y(),
             linesegments_[closest_idx_1].p1().x(), linesegments_[closest_idx_1].p1().y());
  
  ROS_INFO("linesegment[%d] closest 2, p0 = (%f, %f), p1 = (%f, %f)", closest_idx_2, linesegments_[closest_idx_2].p0().x(),
       linesegments_[closest_idx_2].p0().y(),
             linesegments_[closest_idx_2].p1().x(), linesegments_[closest_idx_2].p1().y());

  // use middle of line segments to check which is upfront and which on the side
  tuw::Point2D middle_1(std::abs(linesegments_[closest_idx_1].p0().x() - linesegments_[closest_idx_1].p1().x()) / 2,
                        std::abs(linesegments_[closest_idx_1].p0().y() - linesegments_[closest_idx_1].p1().y()) / 2);
  tuw::Point2D middle_2(std::abs(linesegments_[closest_idx_2].p0().x() - linesegments_[closest_idx_2].p1().x()) / 2,
                        std::abs(linesegments_[closest_idx_2].p0().y() - linesegments_[closest_idx_2].p1().y()) / 2);
  
  ROS_INFO("middle_1 = (%f, %f), middle_2 = (%f, %f)", middle_1.x(), middle_1.y(), middle_2.x(), middle_2.y());
  
  if(middle_1.x() > middle_2.x())
  {
    ROS_INFO("middle_1 is upfront");
    if(middle_2.y() < 0) 
      ROS_INFO("middle_2 is right");
    else
      ROS_INFO("middle_2 is left");
  }
  else
  {
    ROS_INFO("middle_2 is upfront");
    if(middle_1.y() < 0)
      ROS_INFO("middle_1 is right");
    else
      ROS_INFO("middle_1 is left");
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser2corner");

  Laser2CornerNode laser2corner_node;

  ros::spin();

  return 0;
}