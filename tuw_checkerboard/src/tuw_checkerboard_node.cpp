#include "tuw_checkerboard_node.h"

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;
using std::vector;
using std::string;

CheckerboardNode::CheckerboardNode() : nh_private_("~")
{
  string image_topic = nh_.resolveName("image");
  image_transport::ImageTransport it_(nh_);
  sub_cam_ = it_.subscribeCamera(image_topic, 1, &CheckerboardNode::callbackCamera, this);

  pub_image_ = it_.advertise("image_out", 1);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("checkerboard_pose", 1);

  tf_listener_ = std::make_shared<tf::TransformListener>();
  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();


  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  nh_private_.param("checkerboard_size_col", checkerboard_size_col_, 7);
  nh_private_.param("checkerboard_size_row", checkerboard_size_row_, 5);
  nh_private_.param("checkerboard_square_size", checkerboard_square_size_, 0.03);
}

/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CheckerboardNode::callbackCamera(const sensor_msgs::ImageConstPtr& image_msg,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  Mat image;
  Mat image_grey;
  vector<Point2f> image_corners;
  Size patternsize(checkerboard_size_col_, checkerboard_size_row_);
  cv_bridge::CvImagePtr input_bridge;
  try
  {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    // rotate image by 180 deg
    // cv::flip(input_bridge->image, image, -1);
    image = input_bridge->image;
    // input_bridge->image = image;
    cvtColor(image, image_grey, CV_BGR2GRAY, 0);
  }
  catch (cv_bridge::Exception& ex)
  {
    ROS_ERROR("[camera_tf_node] Failed to convert image");
    return;
  }

  bool patternfound = findChessboardCorners(image, patternsize, image_corners, CALIB_CB_FAST_CHECK);
  // CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +

  if (patternfound)
  {
    cornerSubPix(image_grey, image_corners, Size(11, 11), Size(-1, -1),
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    // generate object points
    // float square_size = 30.0f; // chessboard square size in mm - output in mm
    float square_size = float(checkerboard_square_size_);  // chessboard square size in m - output in meters
    vector<Point3f> object_corners;

    for (int i = 0; i < patternsize.height; i++)
    {
      for (int j = 0; j < patternsize.width; j++)
      {
        object_corners.push_back(Point3f(float(i * square_size), float(j * square_size), 0.f));
      }
    }

    // solvePnP -- camera matrix, distortion coefficients
    cam_model_.fromCameraInfo(info_msg);
    Mat camera_matrix = Mat(cam_model_.intrinsicMatrix());
    Mat dist_coeff = cam_model_.distortionCoeffs();
    Vec3d rotation_vec;
    Vec3d translation_vec;

    solvePnP(object_corners, image_corners, camera_matrix, dist_coeff, rotation_vec, translation_vec);
    // , false, SOLVEPNP_ITERATIVE

    for (int i = 0; i < image_corners.size(); i++)
    {
      // ROS_INFO("image_corners(%d) = (%f, %f)", i, image_corners[i].x, image_corners[i].y);
    }

    // generate rotation matrix from vector
    Mat rotation_mat;
    Rodrigues(rotation_vec, rotation_mat, noArray());

    /*
    std::cout << "rotation_mat [m] = " << std::endl << rotation_mat << std::endl;
    std::cout << "translation_mat [m] = " << std::endl << Mat(translation_vec) << std::endl;
    */

    // generate tf model to camera
    tf::Matrix3x3 R(rotation_mat.at<double>(0, 0), rotation_mat.at<double>(0, 1), rotation_mat.at<double>(0, 2),
                    rotation_mat.at<double>(1, 0), rotation_mat.at<double>(1, 1), rotation_mat.at<double>(1, 2),
                    rotation_mat.at<double>(2, 0), rotation_mat.at<double>(2, 1), rotation_mat.at<double>(2, 2));

    tf::Vector3 T = tf::Vector3(translation_vec(0), translation_vec(1), translation_vec(2));

    tf::Transform chess_to_cam(R, T);

    // tf::Transform cam_to_chess = chess_to_cam.inverse();

    // transform correction for ros coordinate system
    // ROS: x - forward, y - left, z - up
    // opencv: x - right, y - down, z - forward
    tf::Quaternion q;
    q.setRPY(1.5708, 0, 1.5708);
    q *= chess_to_cam.getRotation();
    chess_to_cam.setRotation(q);
    chess_to_cam.setOrigin(tf::Vector3(T.getZ(), -T.getX(), -T.getY()));

    tf_broadcaster_->sendTransform(tf::StampedTransform(chess_to_cam, ros::Time::now(), "camera_rgb_frame", "chess_to_"
                                                                                                            "cam"));

    vector<Point2f> image_points;
    projectPoints(object_corners, rotation_vec, translation_vec, camera_matrix, dist_coeff, image_points);

    for (int i = 0; i < image_points.size(); i++)
    {
      circle(image, image_points[i], 2, Scalar(0, 0, 255), -1, 8, 0);
    }

    // also publish pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "camera_rgb_frame";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.orientation.x = chess_to_cam.getRotation().getX();
    pose_stamped.pose.orientation.y = chess_to_cam.getRotation().getY();
    pose_stamped.pose.orientation.z = chess_to_cam.getRotation().getZ();
    pose_stamped.pose.orientation.w = chess_to_cam.getRotation().getW();

    pose_stamped.pose.position.x = chess_to_cam.getOrigin().getX();
    pose_stamped.pose.position.y = chess_to_cam.getOrigin().getY();
    pose_stamped.pose.position.z = chess_to_cam.getOrigin().getZ();

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
