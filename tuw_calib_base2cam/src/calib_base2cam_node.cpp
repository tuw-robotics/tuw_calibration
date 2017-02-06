#include "calib_base2cam_node.h"

Base2CamNode::Base2CamNode() : nh_private_("~")
{
  tf_listener_ = std::make_shared<tf::TransformListener>();
  // tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  nh_private_.param("camera_link", camera_link_, std::string("camera_link"));
  nh_private_.param("base_link", base_link_, std::string("base_link"));
}

void Base2CamNode::getBase2CamTf()
{
  tf::StampedTransform base2cam;
  try
  {
    tf_listener_->lookupTransform(base_link_, camera_link_, ros::Time(0), base2cam);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  tf::Vector3 origin = base2cam.getOrigin();
  tf::Quaternion rotation = base2cam.getRotation();

  // print as static transform publisher
  // TODO output only once, maybe using an average or most probable tf since checkerboard detection and line detection
  // are no always stable
  ROS_INFO("%f %f %f %f %f %f %f %s %s", origin.getX(), origin.getY(), origin.getZ(), rotation.x(), rotation.y(),
           rotation.z(), rotation.w(), base_link_.c_str(), camera_link_.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_calib_base2cam");

  Base2CamNode base2cam_node;
  ros::Rate rate(10.0);
  while (base2cam_node.nh_.ok())
  {
    base2cam_node.getBase2CamTf();
  }

  return 0;
}