#include "laser2corner_node.h"

#include <geometry_msgs/Pose2D.h>

using std::vector;
using std::string;

Laser2CornerNode::Laser2CornerNode() : nh_private_("~")
{
  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  sub_segments_ = nh_.subscribe("line_segments", 1000, &Laser2CornerNode::callbackSegments, this);

  nh_private_.param("laser_frame", laser_frame_, std::string("laser_frame"));
  nh_private_.param("corner_point_tolerance", corner_point_tolerance_, 0.005);
  nh_private_.param("corner_point_x", corner_point_x_, 1.0);
  nh_private_.param("corner_point_y", corner_point_y_, -1.0);
}

void Laser2CornerNode::callbackSegments(const tuw_geometry_msgs::LineSegments& _segments_msg)
{
  // assume corner lies near (corner_point_x_, corner_point_y_, 0) wrt laser_base
  tuw::Point2D pc(corner_point_x_, corner_point_y_);

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

  // find corner point (where linesegments meet)
  tuw::Point2D corner_point;
  if (linesegments_[closest_idx_1].p0().equal(linesegments_[closest_idx_2].p0(), corner_point_tolerance_) ||
      linesegments_[closest_idx_1].p0().equal(linesegments_[closest_idx_2].p1(), corner_point_tolerance_))
  {
    corner_point = linesegments_[closest_idx_1].p0();
  }
  else if (linesegments_[closest_idx_1].p1().equal(linesegments_[closest_idx_2].p0(), corner_point_tolerance_) ||
           linesegments_[closest_idx_1].p1().equal(linesegments_[closest_idx_2].p1(), corner_point_tolerance_))
  {
    corner_point = linesegments_[closest_idx_1].p1();
  }

  double corner_yaw;  // rotation around z axis

  // use middle of line segments to check which is upfront and which on the side
  tuw::Point2D middle_1 = linesegments_[closest_idx_1].pc();
  tuw::Point2D middle_2 = linesegments_[closest_idx_2].pc();

  // use angle from line on the side
  if (middle_1.x() > middle_2.x())
  {
    corner_yaw = linesegments_[closest_idx_2].angle();
  }
  else
  {
    corner_yaw = linesegments_[closest_idx_1].angle();
  }

  tf::Vector3 T = tf::Vector3(corner_point.x(), corner_point.y(), 0);
  tf::Quaternion Q;
  Q.setRPY(0, 0, corner_yaw);

  tf::Transform laser_to_corner(Q, T);
  tf_broadcaster_->sendTransform(tf::StampedTransform(laser_to_corner, ros::Time::now(), laser_frame_, "corner"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_laser2corner");

  Laser2CornerNode laser2corner_node;

  ros::spin();

  return 0;
}