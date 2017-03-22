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
