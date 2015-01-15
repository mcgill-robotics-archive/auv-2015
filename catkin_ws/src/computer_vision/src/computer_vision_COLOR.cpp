#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/legacy/compat.hpp>

static const std::string OPENCV_WINDOW = "Image window";
unsigned int ho = 165;

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

    // Normalize the image (maximize the contrast)
    cv::normalize(img_rgb, img_rgb, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // Noise Reduction
    //cv::fastNlMeansDenoisingColored(img_rgb, img_rgb, 15.0f, 15.0f, 5, 5);
    cv::GaussianBlur(img_rgb, img_rgb, cv::Size(15, 15), 0, 0, cv::BORDER_DEFAULT);

    // Convert from BGR to HSV
    cv::Mat img_hsv, img_threshold;
    cvtColor(img_rgb, img_hsv, CV_BGR2HSV);

    img_threshold = img_hsv.clone();

    // Create hue histogram
    int hbins = 180;
    int histSize[] = { hbins };
    float hranges[] = { 0, 180 };
    const float* ranges[] = { hranges };
    cv::MatND h_hist;
    int channels[] = {0};

    cv::calcHist(&img_hsv, 1, channels, cv::Mat(), h_hist, 1, histSize, ranges, true, false);
    
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/hbins );
    cv::Mat img_hist (hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0));

    cv::normalize(h_hist, h_hist, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
    for (int i = 1; i < hbins; i++)
    {
        cv::line( img_hist, cv::Point( bin_w*(i-1), hist_h - cvRound(h_hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(h_hist.at<float>(i)) ),
                         cv::Scalar( 255, 255, 255), 2, 8, 0  );
    }
    
    // Hue value we are comparing to
    //unsigned int ho = 172;
    ho = (ho + 1) % 180;
    // How close the hues need to be to be seen as the same
    unsigned int h_tolerance = 7;
    // minimum percieved saturation
    unsigned int s_min = 90;
    // minimum percieved value (brightness)
    unsigned int v_min = 64;

    // Iterates over each pixel in the image
    for (int row = 0; row < img_hsv.rows; row++)
    {
      for (int col = 0; col < img_hsv.cols; col++)
      {
        // Gets the hsv values from a pixel h[0,180] s[0, 255] v[0, 255]
        const cv::Vec3b& hsv = img_hsv.at<cv::Vec3b>(row, col);
        unsigned int h = (unsigned int)hsv.val[0];
        unsigned int s = (unsigned int)hsv.val[1];
        unsigned int v = (unsigned int)hsv.val[2];

        // If the pixel is not very saturated/too dark, return black, otherwise compare hue
        if (s > s_min && v > v_min) 
        {
          // The difference between the image and comparisson hue
          int dh = ho - h;
          // Compares the magnitudes of the difference at a period offset of -180, 0, and 180
          unsigned int min_difference = std::min(std::abs(dh), std::min(std::abs(180 - dh), (180 + dh)));
          
          // If the colors do now match return black
          if (min_difference <= h_tolerance) 
          {
            img_threshold.at<cv::Vec3b>(row, col) = hsv;
          } else {
            img_threshold.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
          }
        } 
        else 
        {
          img_threshold.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
        }
      }
    }

    // imshow outputs rgb images, so we need to convert it
    cvtColor(img_threshold, img_threshold, CV_HSV2BGR);
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img_threshold);
    cv::namedWindow("Histogram", 1);
    cv::imshow("Histogram", img_hist);
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
