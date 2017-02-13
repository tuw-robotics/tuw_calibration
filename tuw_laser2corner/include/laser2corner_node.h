#ifndef LASER2CORNER_NODE_H
#define LASER2CORNER_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>

#include <tuw_geometry/linesegment2d.h>
#include <tuw_geometry_msgs/LineSegments.h>


/**
 * @brief ...
 * @class 
 * @author
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

    std::string laser_frame_;
    double corner_point_tolerance_;
    double corner_point_x_;
    double corner_point_y_;

    ros::Subscriber sub_segments_;

    /**
     * @brief ...
     * 
     * @param _segments_msg ...
     * @return void
     */
    void callbackSegments ( const tuw_geometry_msgs::LineSegments &_segments_msg );
};

#endif //LASER2CORNER_NODE_H
