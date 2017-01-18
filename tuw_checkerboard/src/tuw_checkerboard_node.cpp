#include "tuw_checkerboard_node.h"

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;
using std::vector;
using std::string;


CheckerboardNode::CheckerboardNode()
{
    string image_topic = nh_.resolveName("image");
    image_transport::ImageTransport it_(nh_);
    sub_cam_ = it_.subscribeCamera(image_topic, 1, &CheckerboardNode::callbackCamera, this);
    
    pub_image_ = it_.advertise("image_out", 1);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("checkerboard_pose", 1);
    
    tf_listener_ = std::make_shared<tf::TransformListener>();
    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
}

/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CheckerboardNode::callbackCamera(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{        
    Mat image;
    Mat image_grey;
    vector<Point2f> image_corners;
    Size patternsize(7, 5);
    cv_bridge::CvImagePtr input_bridge;
    try 
    {
	input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	image = input_bridge->image;
	cvtColor(image, image_grey, CV_BGR2GRAY, 0);
    }
    catch(cv_bridge::Exception& ex) 
    {
	ROS_ERROR("[camera_tf_node] Failed to convert image");
	return;
    }
    
    bool patternfound = findChessboardCorners(image, patternsize, image_corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    
    if(patternfound) 
    {
      cornerSubPix(image_grey, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
      // generate object points
      //float square_size = 30.0f; // chessboard square size in mm - output in mm
      float square_size = 0.03f; // chessboard square size in m - output in meters
      vector<Point3f> object_corners;
      
      for(int i = 0; i < patternsize.width; i++)
      {
	for(int j = 0; j < patternsize.height; j++)
	{
	  object_corners.push_back(Point3f(i*square_size, j*square_size, 0.0f));
	}
      }
      
      // solvePnP -- camera matrix, distortion coefficients
      cam_model_.fromCameraInfo(info_msg);
      Mat camera_matrix = Mat(cam_model_.intrinsicMatrix());
      Mat dist_coeff = cam_model_.distortionCoeffs();
      Vec3d rotation_vec;
      Vec3d translation_vec;
      
      solvePnP(object_corners, image_corners, camera_matrix, dist_coeff, rotation_vec, translation_vec, false, SOLVEPNP_ITERATIVE);

      // generate rotation matrix from vector
      Mat rotation_mat;
      Rodrigues(rotation_vec, rotation_mat, noArray());

      /*
      std::cout << "rotation_mat [m] = " << std::endl << rotation_mat << std::endl;
      std::cout << "translation_mat [m] = " << std::endl << Mat(translation_vec) << std::endl;
      */
      
      // generate tf model to camera
      tf::Matrix3x3 R(rotation_mat.at<double>(0, 0), rotation_mat.at<double>(0, 1), rotation_mat.at<double>(0, 2), \
		      rotation_mat.at<double>(1, 0), rotation_mat.at<double>(1, 1), rotation_mat.at<double>(1, 2), \
		      rotation_mat.at<double>(2, 0), rotation_mat.at<double>(2, 1), rotation_mat.at<double>(2, 2));
      
      tf::Vector3 T = tf::Vector3(translation_vec(0), translation_vec(1), translation_vec(2));   
      
      tf::Transform chess_to_cam(R, T);
      
      tf::Transform cam_to_chess = chess_to_cam.inverse();
      
      
      // transform correction for ros coordinate system
      // ROS: x - forward, y - left, z - up
      // opencv: x - right, y - down, z - forward   
      tf::Quaternion q;
      q.setRPY(1.5708, 0, 1.5708);
      q *= cam_to_chess.getRotation();
      cam_to_chess.setRotation(q);
      cam_to_chess.setOrigin(tf::Vector3(T.getZ(), -T.getX(), -T.getY()));
      
      tf_broadcaster_->sendTransform(tf::StampedTransform(cam_to_chess, ros::Time::now(), "camera_rgb_frame", "cam_to_chess"));
      
      // also publish pose
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "camera_rgb_frame";
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.pose.orientation.x = cam_to_chess.getRotation().getX();
      pose_stamped.pose.orientation.y = cam_to_chess.getRotation().getY();
      pose_stamped.pose.orientation.z = cam_to_chess.getRotation().getZ();
      pose_stamped.pose.orientation.w = cam_to_chess.getRotation().getW();

      pose_stamped.pose.position.x = cam_to_chess.getOrigin().getX();
      pose_stamped.pose.position.y = cam_to_chess.getOrigin().getY();
      pose_stamped.pose.position.z = cam_to_chess.getOrigin().getZ();
      
      pub_pose_.publish(pose_stamped);
 
    }
    
    drawChessboardCorners(image, patternsize, Mat(image_corners), patternfound);
    
    pub_image_.publish(input_bridge->toImageMsg());
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tuw_checkerboard");
    
    CheckerboardNode checkerboard_node;
    
    ros::spin();
    
    return 0;
}
