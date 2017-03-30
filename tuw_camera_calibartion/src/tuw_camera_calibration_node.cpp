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
#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>


void callbackMouseImage ( int event, int x, int y, int flags, void* ptr ) {
    CameraCalibrationNode *camera_calibartion = ( CameraCalibrationNode* ) ptr;
    if ( event == CV_EVENT_LBUTTONDOWN ) {
        camera_calibartion->use_current_image_ = true;
    }
    //if ( event == CV_EVENT_LBUTTONUP )
    //if ( event == CV_EVENT_MBUTTONUP )
}

void callbackButtonUseImage ( int state, void* ptr ) {
    CameraCalibrationNode *camera_calibartion = ( CameraCalibrationNode* ) ptr;
    camera_calibartion->use_current_image_ = true;
}

void callbackButtonCalibrate ( int i, void *ptr ) {
    CameraCalibrationNode *camera_calibartion = ( CameraCalibrationNode* ) ptr;
    camera_calibartion->calibrate_ = true;
}
void callbackButtonReset ( int i, void *ptr ) {
    CameraCalibrationNode *camera_calibartion = ( CameraCalibrationNode* ) ptr;
    camera_calibartion->reset_ = true;
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
    wnd_images_ = "Images - " + nh_private_.getNamespace();
    cv::namedWindow ( wnd_detection_ );
    cv::namedWindow ( wnd_images_ );
    cv::moveWindow ( wnd_images_, config_.thumb_width*config_.thumb_cols*1.1, 0 );
    cvCreateButton ( "Use Image",callbackButtonUseImage, this, CV_PUSH_BUTTON ,0 );
    cvCreateButton ( "Calibrate",callbackButtonCalibrate, this, CV_PUSH_BUTTON ,0 );
    cvCreateButton ( "Reset",callbackButtonReset, this, CV_PUSH_BUTTON ,0 );
    cv::setMouseCallback ( wnd_detection_, callbackMouseImage, this );
    

}


void CameraCalibrationNode::reset() {
  image_corners_.clear();
  image_calibration_corners_.clear();
  object_calibration_corners_.clear();
  images_gray_.clear();
  images_rgb_.clear();
  image_used_.create(config_.thumb_width*3/4, config_.thumb_width*config_.thumb_cols, CV_8UC3);
  image_used_.setTo(0xFF);
}
void CameraCalibrationNode::calibrate() {
    calibrate_ = false;
    if ( image_corners_.empty() ) {
        return;
    }
    int flags = 0;
    cv::TermCriteria criteria ( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, double ( 2.22044604925031308085e-16L ) );
    cv::calibrateCamera ( object_calibration_corners_, image_calibration_corners_, image_grey_.size(), cameraMatrix_, distCoeffs_, rvecs_, tvecs_, flags, criteria );
    projectionMatrix_ = cv::getOptimalNewCameraMatrix ( cameraMatrix_, distCoeffs_, image_grey_.size(), config_.projection_alpha );

    cameraInfo_.height = image_grey_.cols;
    cameraInfo_.width = image_grey_.rows;

    cameraInfo_.D.resize ( distCoeffs_.cols );
    for ( size_t c = 0; c < distCoeffs_.cols; c++ ) {
        cameraInfo_.D[c] = distCoeffs_ ( 0, c );
    }

    for ( size_t i = 0; i < 9; i++ ) {
        cameraInfo_.K[i] = cameraMatrix_ ( i );
    }

    for ( size_t i = 0; i < 12; i++ ) {
        cameraInfo_.P[i] = projectionMatrix_ ( i );
    }


    printf ( "\n" );
    printf ( " cameraMatrix\n" );
    for ( int r = 0; r < cameraMatrix_.rows; r++ ) {
        printf ( " %-12.4f, %-12.4f, %-12.4f\n", cameraMatrix_ ( r, 0 ), cameraMatrix_ ( r, 1 ), cameraMatrix_ ( r, 2 ) );
    }
    printf ( " distCoeffs\n" );
    for ( int c = 0; c < distCoeffs_.cols; c++ ) {
        printf ( " %-12.4f%s", distCoeffs_ ( 0, c ), ( c!=distCoeffs_.cols-1 ) ?" ":"\n" );
    }
    printf ( " projectionMatrix\n" );
    for ( int r = 0; r < projectionMatrix_.rows; r++ ) {
        printf ( " %-12.4f, %-12.4f, %-12.4f, %-12.4f\n", projectionMatrix_ ( r, 0 ), projectionMatrix_ ( r, 1 ), projectionMatrix_ ( r, 2 ), projectionMatrix_ ( r, 3 ) );
    }

}

void CameraCalibrationNode::callbackConfig ( tuw_camera_calibration::CameraCalibrationConfig &_config, uint32_t _level ) {
    config_ = _config;

    object_corners_.clear();
    for ( int i = 0; i < config_.checkerboard_rows; i++ ) {
        for ( int j = 0; j < config_.checkerboard_columns; j++ ) {
            object_corners_.push_back ( cv::Point3f ( float ( i * config_.checkerboard_square_size ), float ( j * config_.checkerboard_square_size ), 0.f ) );
        }
    }
}
/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CameraCalibrationNode::callbackImage ( const sensor_msgs::ImageConstPtr& image_msg ) {


    if ( reset_ ) {
        reset_ = false;
        reset();
    }
    cv::Size patternsize ( config_.checkerboard_columns, config_.checkerboard_rows );
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy ( image_msg, sensor_msgs::image_encodings::MONO8 );
        image_grey_ = input_bridge->image;

        cvtColor ( image_grey_, image_rgb_, CV_GRAY2BGR, 0 );

    } catch ( cv_bridge::Exception& ex ) {
        ROS_ERROR ( "[camera_tf_node] Failed to convert image" );
        return;
    }

    int flags = 0;
    if ( config_.adaptive_thresh ) {
        flags += CV_CALIB_CB_ADAPTIVE_THRESH;
    }
    if ( config_.normalize_image ) {
        flags += CV_CALIB_CB_NORMALIZE_IMAGE;
    }
    if ( config_.filter_quads ) {
        flags += CV_CALIB_CB_FILTER_QUADS;
    }
    if ( config_.fast_check ) {
        flags += CV_CALIB_CB_FAST_CHECK;
    }
    bool patternfound = findChessboardCorners ( image_grey_, patternsize, image_corners_, flags );

    if ( patternfound ) {
        if ( config_.subpixelfit ) {

            int winSize = config_.subpixelfit_window_size;
            cv::TermCriteria criteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 );
            cv::cornerSubPix ( image_grey_, image_corners_, cv::Size ( winSize, winSize ), cv::Size ( -1, -1 ),  criteria );
        }


        cv::drawChessboardCorners ( image_rgb_, patternsize, cv::Mat ( image_corners_ ), patternfound );
        if ( use_current_image_ ) {
            use_current_image_ = false;
            images_gray_.push_back ( image_grey_.clone() );
            images_rgb_.push_back ( image_rgb_.clone() );

            image_calibration_corners_.push_back ( image_corners_ );
            object_calibration_corners_.push_back ( object_corners_ );
            double scale = ( ( double ) config_.thumb_width ) / ( double ) image_grey_.cols;
            cv::Size size ( config_.thumb_width, image_grey_.rows * scale );
            image_used_.create ( size.height* ( images_gray_.size() /config_.thumb_cols+1 ), size.width*config_.thumb_cols, CV_8UC3 );
	    image_used_.setTo(0xFF);
            for ( int i = 0; i < images_gray_.size(); i++ ) {
                cv::Rect rect ( 0,0,size.width, size.height );
                rect.x = size.width * ( i%config_.thumb_cols );
                rect.y = size.height * ( i/config_.thumb_cols );
                cv::Mat img = image_used_ ( rect );
                cv::resize ( images_rgb_[i], img, size );
            }
        }
    }

    if ( calibrate_ ) {
        this->calibrate();
    }

    cv::imshow ( wnd_detection_, image_rgb_ );
    if ( !image_used_.empty() ) {
        cv::imshow ( wnd_images_, image_used_ );
    }
    cv::waitKey ( config_.show_camera_image_waitkey );

}


int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "tuw_camera_calibartion" );

    CameraCalibrationNode checkerboard_node;

    ros::spin();

    return 0;
}




