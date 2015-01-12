#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter():it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Take the input image and store it locally
    cv_bridge::CvImagePtr img_ptr;
    cv::Mat img_rgb;
    try
    {
      img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_rgb = img_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert from BGR to HSV
    cv::Mat img_hsv, img_threshold;
    cvtColor(img_rgb, img_hsv, CV_BGR2HSV);

    img_threshold = img_hsv.clone();

    // Hue value we are comparing to
    unsigned int ho = 165;
    // How close the hues need to be to be seen as the same
    unsigned int hTolerance = 10;
    // Hue value we are comparing to
    unsigned int sMin = 40;
    // Hue value we are comparing to
    unsigned int vMin = 24;

    // Iterates over each pixel in the image
    for (int y = 0; y < img_hsv.rows; y++)
    {
      for (int x = 0; x < img_hsv.cols; x++)
      {
        // Gets the hsv values from a pixel h[0,180] s[0, 255] v[0, 255]
        const cv::Vec3b& hsv = img_hsv.at<cv::Vec3b>(y, x);
        unsigned int h = (unsigned int)hsv.val[0];
        unsigned int s = (unsigned int)hsv.val[1];
        unsigned int v = (unsigned int)hsv.val[2];

        // If the pixel is not very saturated/too dark, return black, otherwise compare hue
        if (s > sMin && v > vMin) 
        {
          // The difference between the image and comparisson hue
          int dh = ho - h;
          // Compares the magnitudes of the difference at a period offset of -180, 0, and 180
          unsigned int minDifference = std::min(std::abs(dh), std::min(std::abs(180 - dh), (180 + dh)));
          
          // If the colors do now match return black
          if (minDifference <= hTolerance) {
            img_threshold.at<cv::Vec3b>(y, x) = hsv;
          } else {
            img_threshold.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);
          }
        } 
        else 
        {
            img_threshold.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);
        }
      }
    }

    // Update GUI Window
    cvtColor(img_threshold, img_threshold, CV_HSV2BGR);
    cv::imshow(OPENCV_WINDOW, img_threshold);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(img_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
