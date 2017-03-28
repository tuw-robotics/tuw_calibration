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
  
  bool rotate_180_;        /// whether to rotate the camera tf by 180°
  bool publish_all_tf_;
};

#endif  // CALIB_BASE2CAM_NODE_H
