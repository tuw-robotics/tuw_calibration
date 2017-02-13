#ifndef CALIB_BASE2CAM_NODE_H
#define CALIB_BASE2CAM_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class Base2CamNode
{
public:
  ros::NodeHandle nh_;
  
  Base2CamNode(); // Constructor
  void getBase2CamTf();
private:
  ros::NodeHandle nh_private_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::string camera_link_;
  std::string base_link_;
  std::string checkerboard_frame_;
  std::string corner_frame_;
  
  double laser_height_;
  double checker_height_;
  double checker_y_;
  double checker_z_;
};

#endif //CALIB_BASE2CAM_NODE_H