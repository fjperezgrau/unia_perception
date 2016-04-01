/**
 * @file img_proc_full.cpp
 * @brief This file includes the full ROS node implementation.
 * @author Francisco J. Perez Grau (fjperezgrau@gmail.com)
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/transform_broadcaster.h>

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Subscriber caminfo_sub_;
  image_transport::Publisher img_hsv_pub_;
  image_transport::Publisher img_thres_pub_;
  image_transport::Publisher img_morph_pub_;
  image_transport::Publisher img_contour_pub_;
  image_transport::Publisher img_det_pub_;
  tf::TransformBroadcaster tf_;

  cv::Scalar threshold_low_;
  cv::Scalar threshold_high_;
  int morph_size_;
  float target_radius_;

  cv::Mat hsv_image_;
  cv::Mat thres_image_;
  cv::Mat morph_image_;
  cv::Mat contour_image_;
  cv::Mat det_image_;
  bool calib_init_;
  cv::Mat camera_matrix_;
  std::string camera_frame_;
  bool found_;

  std::vector<std::vector<cv::Point> > det_contours_;
  std::vector<cv::Vec3f> det_circles_;

public:
  ImageProcessor(ros::NodeHandle& n):
    nh_(n),
    it_(nh_),
    calib_init_(false),
    found_(false)
  {
    //Read input parameters
    std::string image_topic, caminfo_topic;
    int h_min, h_max, s_min, s_max, v_min, v_max;
    nh_.param<std::string>("image_topic", image_topic, "/input_image");
    nh_.param<std::string>("caminfo_topic", caminfo_topic, "/input_caminfo");
    nh_.param("h_min", h_min, int(0));
    nh_.param("h_max", h_max, int(255));
    nh_.param("s_min", s_min, int(0));
    nh_.param("s_max", s_max, int(255));
    nh_.param("v_min", v_min, int(0));
    nh_.param("v_max", v_max, int(255));
    threshold_low_  = cv::Scalar(h_min, s_min, v_min);
    threshold_high_ = cv::Scalar(h_max, s_max, v_max);
    nh_.param("morph_size", morph_size_, int(20));
    nh_.param("target_radius", target_radius_, float(0.085));

    ROS_INFO_STREAM("Subscribed to " << image_topic);
    ROS_INFO_STREAM("Thresholds: " << threshold_low_ << " - " << threshold_high_);
    ROS_INFO_STREAM("Size for morphological operations: " << morph_size_);
    ROS_INFO_STREAM("Target radius: " << target_radius_);

    img_sub_ = it_.subscribe(image_topic, 1, &ImageProcessor::image_callback, this);
    caminfo_sub_ = nh_.subscribe(caminfo_topic, 1, &ImageProcessor::caminfo_callback, this);
    img_hsv_pub_    = it_.advertise("/image_processor/hsv_image", 1);
    img_thres_pub_  = it_.advertise("/image_processor/thres_image", 1);
    img_morph_pub_  = it_.advertise("/image_processor/morph_image", 1);
    img_contour_pub_= it_.advertise("/image_processor/contour_image", 1);
    img_det_pub_    = it_.advertise("/image_processor/det_image", 1);
  }

  ~ImageProcessor() {}

  void caminfo_callback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    if(!calib_init_)
    {
      calib_init_ = true;
      // Store camera parameters
      camera_matrix_ = cv::Mat(3, 3, CV_64FC1, (void *)msg->K.elems).clone();
      camera_frame_ = msg->header.frame_id;
    }
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(calib_init_)
    {
      found_ = false;

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat img = cv_ptr->image;

      det_contours_.clear();
      det_circles_.clear();
      detectColor(img, det_image_);

      //Locate first circle
      cv::Point3d circle_location;
      if(det_circles_.size() > 0)
      {
        locateTarget(det_circles_[0], circle_location);
      }

      //Publish images
      cv_bridge::CvImage send_hsv     (cv_ptr->header, cv_ptr->encoding, hsv_image_);
      cv_bridge::CvImage send_thres   (cv_ptr->header, cv_ptr->encoding, thres_image_);
      cv_bridge::CvImage send_morph   (cv_ptr->header, cv_ptr->encoding, morph_image_);
      cv_bridge::CvImage send_contour (cv_ptr->header, cv_ptr->encoding, contour_image_);
      cv_bridge::CvImage send_det     (cv_ptr->header, cv_ptr->encoding, det_image_);
      img_hsv_pub_.publish(send_hsv.toImageMsg());
      img_thres_pub_.publish(send_thres.toImageMsg());
      img_morph_pub_.publish(send_morph.toImageMsg());
      img_contour_pub_.publish(send_contour.toImageMsg());
      img_det_pub_.publish(send_det.toImageMsg());

      //Publish target location
      if(found_)
      {
        tf::Transform target_tf;
        target_tf.setOrigin(tf::Vector3(circle_location.x, circle_location.y, circle_location.z));
        tf::Quaternion q;
        q.setRPY(0,0,0);
        target_tf.setRotation(q);
        tf_.sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), camera_frame_, "target"));
        ROS_INFO_STREAM("Target detected at " << circle_location);
      }
    }
  }

  void detectColor(cv::Mat src, cv::Mat& dst)
  {
    cv::cvtColor(src, hsv_image_, cv::COLOR_BGR2HSV);

    cv::Mat thres_img;
    cv::inRange(hsv_image_, threshold_low_, threshold_high_, thres_img);
    cv::cvtColor(thres_img, thres_image_, CV_GRAY2BGR);

    cv::Mat morph_img;
    morph(thres_img, morph_img);
    cv::cvtColor(morph_img, morph_image_, CV_GRAY2BGR);

    cv::findContours(morph_img, det_contours_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours(src, det_contours_, contour_image_);

    detectRoundedContours(det_contours_, det_circles_);
    drawRoundedContours(src, det_circles_, det_image_);

  }

  void morph(cv::Mat src, cv::Mat& dst)
  {
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_size_, morph_size_));
    cv::Mat colorEroded, colorDilated;
    cv::erode(src, colorEroded, element);
    cv::dilate(colorEroded, colorDilated, element);
    cv::dilate(colorDilated, colorDilated, element);
    cv::erode(colorDilated, dst, element);
  }

  void drawContours(cv::Mat src, std::vector<std::vector<cv::Point> >& contours, cv::Mat& dst)
  {
    dst = src.clone();
    cv::RNG rng(12345);
    for(int i=0; i<contours.size(); i++)
     {
       cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
       cv::drawContours(dst, contours, i, color, 5, 8);
     }
  }

  void detectRoundedContours(std::vector<std::vector<cv::Point> > contours,
                             std::vector<cv::Vec3f>& circles)
  {
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Point2f> center( contours.size() );
    std::vector<float> radius( contours.size() );
    for(int i=0; i<contours.size(); i++)
    {
      cv:approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
      cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );

      cv::Vec3f c(center[i].x, center[i].y, radius[i]);
      circles.push_back(c);
    }
  }

  void drawRoundedContours(cv::Mat src, std::vector<cv::Vec3f>& circles, cv::Mat& dst)
  {
    dst = src.clone();
    for(int i=0; i<circles.size(); i++)
    {
      cv::circle(dst, cv::Point(circles[i][0], circles[i][1]), 3, CV_RGB(0,255,0), -1, 8, 0 );
      cv::circle(dst, cv::Point(circles[i][0], circles[i][1]), circles[i][2], CV_RGB(255,0,0), 2);
    }

  }

  void locateTarget(cv::Vec3f circle, cv::Point3d& target_location)
  {
    // Generate rays for center +/- radius
    cv::Point target_center (circle[0], circle[1]);
    float radius = circle[2];

    std::vector<cv::Point> corners;
    corners.push_back(target_center + cv::Point(0, -radius));
    corners.push_back(target_center + cv::Point(radius, 0));
    corners.push_back(target_center + cv::Point(0, radius));
    corners.push_back(target_center + cv::Point(-radius, 0));

    std::vector<cv::Vec3f> rays, world_points;
    for(size_t i=0; i<corners.size(); i++)
    {
      cv::Vec3f ray;
      getRay(corners[i], ray);
      rays.push_back(ray);
    }

    getTruePoints(rays, world_points);

    //Get mean location
    float mean_x=0., mean_y=0., mean_z=0.;
    for(size_t i=0; i<world_points.size(); i++)
    {
        mean_x += world_points[i][0];
        mean_y += world_points[i][1];
        mean_z += world_points[i][2];
    }
    mean_x /= world_points.size();
    mean_y /= world_points.size();
    mean_z /= world_points.size();

    target_location = cv::Point3f(mean_x, mean_y, mean_z);
    found_ = true;
  }

  void getRay(cv::Point pixel, cv::Vec3f& ray)
  {
    ray = cv::Vec3f ( (pixel.x - camera_matrix_.at<double>(0,2))/camera_matrix_.at<double>(0,0), 
                      (pixel.y - camera_matrix_.at<double>(1,2))/camera_matrix_.at<double>(1,1), 
                      1.0 );
  }

  void getTruePoints(std::vector<cv::Vec3f> rays, std::vector<cv::Vec3f>& targets)
  {
    double lambda=0.;
    double step = 0.1;
    bool size_reached = false;

    while(!size_reached)
    {
      std::vector<cv::Point3d> pInRays;
      for(size_t i=0; i<rays.size(); i++)
        pInRays.push_back(cv::Point3d(lambda*rays[i]));
      size_reached = checkDistance(pInRays);
      lambda += step;
    }

    for(size_t i=0; i<rays.size(); i++)
    {
      targets.push_back(lambda*rays[i]);
    }

  }

  bool checkDistance(std::vector<cv::Point3d> points)
  {
    float d1=0., d2=0.;
    d1 = sqrt( (points[0].x - points[2].x)*(points[0].x - points[2].x) +
               (points[0].y - points[2].y)*(points[0].y - points[2].y) +
               (points[0].z - points[2].z)*(points[0].z - points[2].z) );
    d2 = sqrt( (points[1].x - points[3].x)*(points[1].x - points[3].x) +
               (points[1].y - points[3].y)*(points[1].y - points[3].y) +
               (points[1].z - points[3].z)*(points[1].z - points[3].z) );
    float dmean = (d1 + d2)/2;
    if(dmean > 2*target_radius_)
      return true;
    else
      return false;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_proc_full");
  ros::NodeHandle n("~");
  ImageProcessor im(n);
  ros::spin();
  return 0;
}
