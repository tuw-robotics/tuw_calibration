/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include "calib_base2cam_node.h"

Base2CamNode::Base2CamNode() : nh_private_("~")
{
  tf_listener_ = std::make_shared<tf::TransformListener>();
  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  nh_private_.param("camera_link", camera_link_, std::string("/camera_link"));
  nh_private_.param("base_link", base_link_, std::string("/base_link"));
  nh_private_.param("checkerboard_frame", checkerboard_frame_, std::string("/checkerboard"));
  nh_private_.param("corner_frame", corner_frame_, std::string("/corner"));
  nh_private_.param("laser_height", laser_height_, 0.3);
  nh_private_.param("checker_height", checker_height_, 1.295);
  nh_private_.param("checker_y", checker_y_, 0.475);
  checker_z_ = checker_height_ - laser_height_;
  nh_private_.param("publish_all_tf", publish_all_tf_, true);
}

void Base2CamNode::getBase2CamTf()
{
  // create corner to checkerboard tf
  tf::Transform corner2checker;
  corner2checker.setOrigin(tf::Vector3(0, checker_y_, checker_z_));
  tf::Quaternion q;
  q.setRPY(0, 1.5708, 2 * 1.5708);
  corner2checker.setRotation(q);

if(publish_all_tf_)
  tf_broadcaster_->sendTransform(tf::StampedTransform(corner2checker, ros::Time::now(), corner_frame_, "checkerboardVI"
                                                                                                       "S"));

  tf::StampedTransform camlink2checker;
  try
  {
    tf_listener_->lookupTransform(checkerboard_frame_, camera_link_, ros::Time(0), camlink2checker);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

if(publish_all_tf_)
  tf_broadcaster_->sendTransform(tf::StampedTransform(camlink2checker, ros::Time::now(), "checkerboardVIS", "camlinkVI"
                                                                                                            "S"));

  tf::StampedTransform corner2base;

  try
  {
    tf_listener_->lookupTransform(base_link_, corner_frame_, ros::Time(0), corner2base);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

if(publish_all_tf_)
  tf_broadcaster_->sendTransform(tf::StampedTransform(corner2base, ros::Time::now(), base_link_, "cornerVIS"));

  tf::Transform base2cam = corner2base * corner2checker * camlink2checker;

if(publish_all_tf_)
  tf_broadcaster_->sendTransform(tf::StampedTransform(base2cam, ros::Time::now(), base_link_, "camVIS"));

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
