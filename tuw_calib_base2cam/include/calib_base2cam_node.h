#ifndef CALIB_BASE2CAM_NODE_H
#define CALIB_BASE2CAM_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

/**
 * @brief Create base_link to camera_link tf from intermediate tf's
 * @class Base2CamNode
 */
class Base2CamNode
{
public:
  ros::NodeHandle nh_;

  Base2CamNode();  /// Constructor

  /**
   * @brief Listen to intermediate transform to create direct base_link --> camera_link tf
   */
  void getBase2CamTf();

private:
  ros::NodeHandle nh_private_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::string camera_link_;         /// camera_link frame id
  std::string base_link_;           /// base_link frame id
  std::string checkerboard_frame_;  /// checkerboard frame id
  std::string corner_frame_;        /// corner frame id

  double laser_height_;    /// height of the laser scanner from the ground
  double checker_height_;  /// height of the checkerboard origin from the ground
  double checker_y_;       /// distance from the corner to the checkerboard origin
  double checker_z_;       /// checker_height_ - laser_height_
  
  bool publish_all_tf_;
};

#endif  // CALIB_BASE2CAM_NODE_H
