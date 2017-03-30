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

#include "tuw_camera_calibration_node.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using std::vector;
using std::string;

void callbackButtonUseImage ( int i, void* param ) {
    CameraCalibrationNode *node = ( CameraCalibrationNode * ) param;
    node->calibrate_ = true;
}

void callbackButtonCalibrate ( int i, void *node ) {
    CameraCalibrationNode *camera_calibartion = ( CameraCalibrationNode* ) node;
    camera_calibartion->use_current_image_ = true;
}

CameraCalibrationNode::CameraCalibrationNode() : nh_private_ ( "~" ) {
    image_transport::ImageTransport it_ ( nh_ );
    use_current_image_ = false;
    calibrate_ = false;

    nh_.param<std::string> ( "frame_id", checkerboard_frame_id_, "checkerboard" );

    reconfigureServer_ = new dynamic_reconfigure::Server<tuw_camera_calibration::CameraCalibrationConfig> ( ros::NodeHandle ( "~" ) );
    reconfigureFnc_ = boost::bind ( &CameraCalibrationNode::callbackConfig, this,  _1, _2 );
    reconfigureServer_->setCallback ( reconfigureFnc_ );

    sub_img_ = it_.subscribe ( "image", 1, &CameraCalibrationNode::callbackImage, this );
    wnd_detection_ = "Detection - " + nh_private_.getNamespace();
    cv::namedWindow ( wnd_detection_ );
    cvCreateButton ( "Use Image",callbackButtonUseImage, this, CV_PUSH_BUTTON ,0 );
    cvCreateButton ( "Calibrate",callbackButtonCalibrate, this, CV_PUSH_BUTTON ,0 );

}

void CameraCalibrationNode::calibrate() {
    
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    int flags = CV_CALIB_USE_INTRINSIC_GUESS;
    cv::TermCriteria criteria( TermCriteria::COUNT + TermCriteria::EPS, 30, double(2.22044604925031308085e-16L) );
    calibrateCamera ( object_calibration_corners_, image_calibration_corners_, image_grey_.size(), cameraMatrix, distCoeffs, rvecs, tvecs, flags, criteria );
    std::cout << cameraMatrix;
}

void CameraCalibrationNode::callbackConfig ( tuw_camera_calibration::CameraCalibrationConfig &_config, uint32_t _level ) {
    config_ = _config;

    object_corners_.clear();
    for ( int i = 0; i < config_.checkerboard_rows; i++ ) {
        for ( int j = 0; j < config_.checkerboard_columns; j++ ) {
            object_corners_.push_back ( Point3f ( float ( i * config_.checkerboard_square_size ), float ( j * config_.checkerboard_square_size ), 0.f ) );
        }
    }
}
/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CameraCalibrationNode::callbackImage ( const sensor_msgs::ImageConstPtr& image_msg ) {

    Size patternsize ( config_.checkerboard_columns, config_.checkerboard_rows );
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy ( image_msg, sensor_msgs::image_encodings::MONO8 );
        if ( config_.rotate_camera_image_180 ) {
            cv::flip ( input_bridge->image, image_grey_, -1 );
        } else {
            image_grey_ = input_bridge->image;
        }
        cvtColor ( image_grey_, image_rgb_, CV_GRAY2BGR, 0 );

    } catch ( cv_bridge::Exception& ex ) {
        ROS_ERROR ( "[camera_tf_node] Failed to convert image" );
        return;
    }

    int flags = 0;
    if ( config_.adaptive_thresh ) flags += CV_CALIB_CB_ADAPTIVE_THRESH;
    if ( config_.normalize_image ) flags += CV_CALIB_CB_NORMALIZE_IMAGE;
    if ( config_.filter_quads ) flags += CV_CALIB_CB_FILTER_QUADS;
    if ( config_.fast_check ) flags += CALIB_CB_FAST_CHECK;
    bool patternfound = findChessboardCorners ( image_grey_, patternsize, image_corners_, flags );

    if ( patternfound ) {
        if ( config_.subpixelfit ) {

            int winSize = config_.subpixelfit_window_size;
            cornerSubPix ( image_grey_, image_corners_, Size ( winSize, winSize ), Size ( -1, -1 ), TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
        }

        if ( use_current_image_ ) {
            use_current_image_ = false;
            image_calibration_corners_.push_back ( image_corners_ );
            object_calibration_corners_.push_back ( object_corners_ );
        }
    }

    if ( calibrate_ ) this->calibrate();


    drawChessboardCorners ( image_rgb_, patternsize, Mat ( image_corners_ ), patternfound );
    cv::imshow ( wnd_detection_, image_rgb_ );
    cv::waitKey ( config_.show_camera_image_waitkey );

}


int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "tuw_camera_calibartion" );

    CameraCalibrationNode checkerboard_node;

    ros::spin();

    return 0;
}




