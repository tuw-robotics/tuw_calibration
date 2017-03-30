/***************************************************************************
 * Copyright (c) 2017
 * Florian Beck <florian.beck@tuwien.ac.at>
 * Markus Bader <markus.bader@tuwien.ac.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the TU-Wien.
 * 4. Neither the name of the TU-Wien nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Markus Bader ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************/


#ifndef TUW_CAMERA_CALIBRATION_NODE_H
#define TUW_CAMERA_CALIBRATION_NODE_H

#include <ros/ros.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_camera_calibration/CameraCalibrationConfig.h>


class CameraCalibrationNode
{
public:
    CameraCalibrationNode(); // Constructor
    bool use_current_image_;
    bool calibrate_;
    bool reset_;
private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    image_transport::Subscriber sub_img_;
    sensor_msgs::CameraInfo cameraInfo_;
    cv::Mat image_grey_;
    cv::Mat image_rgb_;
    cv::Mat image_used_;
    tuw_camera_calibration::CameraCalibrationConfig config_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string checkerboard_frame_id_;
    cv::Mat_<float> cameraMatrix_;
    cv::Mat_<float> projectionMatrix_;
    cv::Mat_<float>  distCoeffs_;
    std::vector<cv::Mat> rvecs_;
    std::vector<cv::Mat> tvecs_;
    std::vector<cv::Point2f> image_corners_;
    std::vector<cv::Point3f> object_corners_;
    std::vector<std::vector<cv::Point2f> > image_calibration_corners_;
    std::vector<std::vector<cv::Point3f> > object_calibration_corners_;
    std::string wnd_detection_;
    std::string wnd_images_;
    std::vector<cv::Mat> images_gray_;
    std::vector<cv::Mat> images_rgb_;

    void calibrate();
    void reset();
    void callbackImage ( const sensor_msgs::ImageConstPtr& image_msg );
    dynamic_reconfigure::Server<tuw_camera_calibration::CameraCalibrationConfig>* reconfigureServer_; ///< parameter server stuff
    dynamic_reconfigure::Server<tuw_camera_calibration::CameraCalibrationConfig>::CallbackType reconfigureFnc_;///< parameter server stuff
    void callbackConfig ( tuw_camera_calibration::CameraCalibrationConfig &_config, uint32_t _level ); ///< callback function on incoming parameter changes
};

#endif //TUW_CAMERA_CALIBRATION_NODE_H
