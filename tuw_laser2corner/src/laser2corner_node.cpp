#include "laser2corner_node.h"

#include <geometry_msgs/Pose2D.h>

using std::vector;
using std::string;

Laser2CornerNode::Laser2CornerNode()
{
  tf_listener_ = std::make_shared<tf::TransformListener>();
  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  sub_laser_ = nh_.subscribe("scan", 1000, &Laser2CornerNode::callbackLaser, this);
}
void Laser2CornerNode::callbackLaser(const sensor_msgs::LaserScan& _laser)
{
  int nr = (_laser.angle_max - _laser.angle_min) / _laser.angle_increment;
  measurement_laser_->range_max() = _laser.range_max;
  measurement_laser_->range_min() = _laser.range_min;
  measurement_laser_->resize (nr);
  measurement_laser_->stamp() = _laser.header.stamp.toBoost();
  for (int i = 0; i < nr; i++) 
  {
      MeasurementLaser::Beam &beam = measurement_laser_->operator[](i);
      beam.length = _laser.ranges[i];
      beam.angle = _laser.angle_min + (_laser.angle_increment * i);
      beam.end_point.x() = cos (beam.angle) * beam.length;
      beam.end_point.y() = sin (beam.angle) * beam.length;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser2corner");
  
  Laser2CornerNode laser2corner_node;
  
  ros::spin();
  
  return 0;
}