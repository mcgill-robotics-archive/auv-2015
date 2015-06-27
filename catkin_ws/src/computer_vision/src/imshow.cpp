#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/legacy/compat.hpp>
#include <list>
#include <computer_vision/histogram.hpp>
#include <computer_vision/threshold.hpp>
#include <computer_vision/buoy.hpp>

std::string IMAGE_WINDOW = "Live Feed";
std::string feed_name = "/camera_front_left/camera/image_rect_color";

class ImageFeed
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
public:
  ImageFeed():it_(nh_)
  {
    image_sub_ = it_.subscribe(feed_name, 1, 
      &ImageFeed::imageCb, this);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr img_ptr;
    try
    {
      img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
      cv::imshow(IMAGE_WINDOW, img_ptr->image);
      cv::namedWindow(IMAGE_WINDOW);
      cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  if (argc != 1 ) {
    feed_name = argv[1];
    IMAGE_WINDOW = "Live Feed of " + feed_name;
    std::cout << feed_name << std::endl;
  }
  ImageFeed _if;
  ros::spin();
  return 0;
}
