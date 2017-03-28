/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                         Markus Bader <markus.bader@tuwien.ac.at>        *
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

#ifndef LINESEGMENT2D_DETECTOR_NODE_H
#define LINESEGMENT2D_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/measurement_laser.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_linedetection/Linesegment2DDetectorConfig.h>

namespace tuw
{
/**
 * @brief ROS wrapper node for LineSegment2DDetector
 * @class Linesegment2DDetectorNode
 */
class Linesegment2DDetectorNode : public LineSegment2DDetector
{
public:
  Linesegment2DDetectorNode();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_laser_;  /// Subscriber to the laser measurements
  ros::Publisher line_pub_;
  MeasurementLaserPtr measurement_laser_;                /// laser measurements
  std::vector<Point2D> measurement_local_scanpoints_;    /// laser beam endpoints for line detection
  std::vector<LineSegment2D> measurement_linesegments_;  /// detected line segments in sensor coordinates

  /// parameter server for dynamic detector configuration
  dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig> reconfigure_server_;

  /// parameter server callback
  dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig>::CallbackType reconfigure_fnc_;

  /**
   * @brief callback function on incoming parameter changes
   * @param config the configuration message
   * @param level not used here, but required for dynamic reconfigure callbacks
   */
  void callbackConfig(tuw_geometry::Linesegment2DDetectorConfig &config, uint32_t level);
  /**
   * @brief callback function for incoming laser scans
   * @param _laser laser scan message
   */
  void callbackLaser(const sensor_msgs::LaserScan &_laser);
};
};

#endif  // LINESEGMENT2D_DETECTOR_NODE_H
