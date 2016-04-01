/**
 * @file img_proc_base.cpp
 * @brief This file includes the initial ROS node implementation.
 * @author Francisco J. Perez Grau (fjperezgrau@gmail.com)
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;

public:
  ImageProcessor(ros::NodeHandle& n):
    nh_(n),
    it_(nh_)
  {
    img_sub_ = it_.subscribe("input_image", 1, &ImageProcessor::image_callback, this);
    img_pub_ = it_.advertise("output_image", 1);
  }

  ~ImageProcessor() {}

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // Do something with img and store the result in send

    //Publish image
    cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, img);
    img_pub_.publish(send.toImageMsg());

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_proc_base");
  ros::NodeHandle n("~");
  ImageProcessor im(n);
  ros::spin();
  return 0;
}
