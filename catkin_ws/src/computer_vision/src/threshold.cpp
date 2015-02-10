#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <computer_vision/histogram.hpp>
#include <computer_vision/threshold.hpp>

  // img is the image to take the threshold of
  // hues is the list of peaks we are using to determine each threshold.
  // s_min is the minimum percieved saturation [0,255]
  // v_min is the minimum percieved value (brightness) [0.255]
  std::list<cv::Mat> Threshold::threshold(cv::Mat& img, std::list<HuePeak> hues, unsigned int s_min, unsigned int v_min )
  {
    std::list<cv::Mat> images_threshold;
    std::list<HuePeak>::iterator it = hues.begin();

    // Thresholds the image for each hue in the list
    for (int i = 0; i < hues.size(); i++) 
    {
      HuePeak peak = *it;
      it++;
      int ho = peak._peak;

      // Finds the thresholds for a hue based on the adjacent local minimums for each side
      int left_threshold = getMinHueDifference(ho, peak._l_valley);
      int right_threshold = getMinHueDifference(peak._r_valley, ho);

      // Creates a black image to draw the thresholded elements on
      cv::Mat img_threshold (img.rows, img.cols, CV_8UC3, cv::Scalar(0,0,0) );

      // Iterates over each pixel in the image
      for (int row = 0; row < img.rows; row++)
      {
        for (int col = 0; col < img.cols; col++)
        {
          // Gets the hsv values from a pixel
          const cv::Vec3b& hsv = img.at<cv::Vec3b>(row, col);
          unsigned int h = (unsigned int)hsv.val[0];
          unsigned int s = (unsigned int)hsv.val[1];
          unsigned int v = (unsigned int)hsv.val[2];
          
          // If the pixel is not very saturated/too dark, continue on, otherwise compare hue  
          if (s > s_min && v > v_min) 
          {
            int min_difference = getMinHueDifference (ho, h);
            
            // If the colors match return white
            if ((min_difference <= 0 && std::abs(min_difference) <= left_threshold) ||
                (min_difference > 0 && std::abs(min_difference) <= right_threshold) ) 
            {
              img_threshold.at<cv::Vec3b>(row, col) = cv::Vec3b(255,255,255);
            }
          }
        }
      }

      cvtColor(img_threshold, img_threshold, CV_BGR2GRAY);

      // adds the result the the returned list
      images_threshold.push_back(img_threshold);
    }

    return images_threshold;
  }

  void Threshold::drawThreshold(std::list<cv::Mat> images, std::list<HuePeak> hues)
  {
    std::list<cv::Mat>::iterator it_imgs = images.begin();
    std::list<HuePeak>::iterator it_hues = hues.begin();

    // Thresholds the image for each hue in the list
    for (int i = 0; i < images.size(); i++) 
    {
      cv::Mat img = *it_imgs;
      HuePeak peak = *it_hues;
      it_imgs++;
      it_hues++;
      int ho = peak._peak;

      // Show each threshold result
      std::ostringstream s;
      s << "Threshold hue = " << ho;
      std::string windowName = s.str();
      //cv::namedWindow(windowName);
      //cv::imshow(windowName, img);
      //cv::waitKey(3);
    }
  }

  // returns the difference between the image hue (h) and comparisson hue (ho), accounding for periodicity
  int Threshold::getMinHueDifference(int ho, int h)
  {
    int dh_m_offset = ho - h; // middle - no period offset
    int dh_l_offset = 180 - dh_m_offset; // left - period offset left
    int dh_r_offset = 180 + dh_m_offset; // right - period offset right

    if (std::abs(dh_m_offset) < std::abs(dh_l_offset) && std::abs(dh_m_offset) < std::abs(dh_r_offset)) 
    {
      return dh_m_offset;
    } 
    else if (std::abs(dh_l_offset) < std::abs(dh_m_offset) && std::abs(dh_l_offset) < std::abs(dh_r_offset)) 
    {
      return dh_l_offset;
    }
    else 
    {
      return dh_r_offset;
    }
  }
