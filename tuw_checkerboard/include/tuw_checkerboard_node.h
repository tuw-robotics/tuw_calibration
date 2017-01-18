#ifndef TUW_CHECKERBOARD_NODE_H
#define TUW_CHECKERBOARD_NODE_H

#include <ros/ros.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>


class CheckerboardNode
{
public:
  CheckerboardNode(); // Constructor
private:
  
  ros::NodeHandle nh_;
  ros::Publisher pub_pose_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  image_geometry::PinholeCameraModel cam_model_;
  image_transport::CameraSubscriber sub_cam_;
  image_transport::Publisher pub_image_;
  
  void callbackCamera(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
};

#endif //TUW_CHECKERBOARD_NODE_H