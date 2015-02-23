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
#include <computer_vision/object_finder.hpp>

// GUI Options
static const bool SHOW_HISTOGRAM = true;
static const bool SHOW_THRESHOLDS = true;
static const bool SHOW_BOUNDING_BOXS = true;
static const float HISTOGRAM_EXPONENT_SCALE = 0.25f; // changes the exponent that changes the histogram scale [0,1]

// Filters (TEMPORARY)
static const bool USE_GUASSIAN_BLUR = false;
static const int GUASSIAN_BLUR_SIZE = 15; // size of the guassian blur, should be odd
static const bool USE_MEDIAN_BLUR = true;
static const int MEDIAN_BLUR_SIZE = 13; // size of the median blur, should be odd

// Filtering Options
static const float MIN_PEAK_THRESHOLD = 0.005f; // the minimum fraction of the image a single hue must contain to be recognised as a peak [0,1]
static const float MAX_PEAK_THRESHOLD = 0.20f; // the maximum fraction of the image a single hue can contain to be recognised as a peak (for ignoring a background color) [0,1]
static const float MIN_PEAKINESS = 0.25f; // the minimum prominence of a hue peak required to be thresholded [0,1]
static const int S_MIN = 135; // minimum percieved saturation for colors [0,255]
static const int V_MIN = 64; // minimum percieved value for colors (brightness) [0.255]
static const float MIN_BOUNDING_BOX = 250.0f; // if an object's bounding box has less area than this, we ignore the object

class ImageConverter
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  Histogram hist;
  Threshold threshold;
  ObjectFinder objectFinder;

public:
  ImageConverter():it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr img_ptr;

    try
    {
      img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      processImageColor(img_ptr->image.clone());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  /*
  * Takes in a rgb image from the pipeline and runs color based analysis on it.
  * 
  * Currently has no return, but eventually it should return the origional image
  * back into the pipe, along with a stuct containing data about visible objects.
  */
  void processImageColor(cv::Mat img_rgb) 
  {
    //---------------------MOVE THESE FILTERS INTO PIPELINE------------------
    if (true)
    {
      cv::imshow("Input Image", img_rgb);
      cv::namedWindow("Input Image");
      cv::waitKey(3);
    }
    // clean image for analysis
    cv::normalize(img_rgb, img_rgb, 0, 255, cv::NORM_MINMAX, CV_8UC1); // maximizes the contrast
    
    if (USE_GUASSIAN_BLUR) 
    {
      cv::GaussianBlur(img_rgb, img_rgb, cv::Size(GUASSIAN_BLUR_SIZE, GUASSIAN_BLUR_SIZE), 0, 0, cv::BORDER_DEFAULT); // Reduces noise
    } 
    if (USE_MEDIAN_BLUR) 
    {
      cv::medianBlur(img_rgb, img_rgb, MEDIAN_BLUR_SIZE); // Reduces noise
    }
    //-----------------------------------------------------------------------

    cv::Mat img_hsv; 
    cvtColor(img_rgb, img_hsv, CV_BGR2HSV); // Convert from BGR to HSV

    // Analyze hue histogram
    hist.createHistogram(img_hsv);
    hist.findPeakHues(MIN_PEAK_THRESHOLD, MAX_PEAK_THRESHOLD, MIN_PEAKINESS, img_hsv.rows * img_hsv.cols);
    std::list<HuePeak> hues = hist.getHuePeaks();
    
    std::list<cv::Mat> imgs_threshold = threshold.threshold(img_hsv, hues, S_MIN, V_MIN); // Threshold images

    // Find the different objects of each color
    objectFinder.findObjects(imgs_threshold, hues, MIN_AREA, MIN_LINE_LENGTH_RATIO);
    std::list<VisibleObject> visibleObjects = objectFinder.getVisibleObjects();

    // Update GUI Windows
    if (SHOW_HISTOGRAM) {
      hist.drawHistogram(HISTOGRAM_EXPONENT_SCALE);
    }

    if (SHOW_THRESHOLDS) {
      threshold.drawThreshold(imgs_threshold, hues);
    }
    
    if (SHOW_BOUNDING_BOXS) {
      objectFinder.drawContours(img_rgb);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
