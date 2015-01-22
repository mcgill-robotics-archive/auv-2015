#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <math.h>
#include <ros/console.h>
#include <computer_vision/histogram.hpp>

  static const std::string HISTOGRAM_WINDOW = "Histogram";
  
  HuePeak::HuePeak(int l_valley, int peak, int r_valley)
  {
    _l_valley = l_valley;
    _peak = peak;
    _r_valley = r_valley;
  }

  std::list<HuePeak> Histogram::getHuePeaks()
  {
    return huePeaks;
  }

  // Takes in an hsv image and makes a histogram of the hues
  void Histogram::createHistogram (cv::Mat& img_hsv) 
  {
    int histSize[] = { 180 }; // hue is [0,179], so there are 180 bins needed
    float hranges[] = { 0, 180 };
    const float* _ranges[] = { hranges };
    int channels[] = {0};

    cv::calcHist(&img_hsv, 1, channels, cv::Mat(), h_hist, 1, histSize, _ranges, true, false);
  }

  // minPeakTheshold [0,1] is the minimum fraction of the image a peak must contain to be recognised
  // maxPeakTheshold [0,1] is the maximum fraction of the image a peak can contain and be recognised
  // minPeakiness [0,1] determines how distinct of a peak needs to be to be recognised
  // resolution area is the width x height of an image used to find the fraction of pixels are a specific color
  void Histogram::findPeakHues (float minPeakTheshold, float maxPeakTheshold, float minPeakiness, int resolutionArea)
  {
    std::list<HuePeak> peaks;

    for (int i = 0; i < 180; i++) 
    {
      float hv = h_hist.at<float>(i); // hue left min value

      // if the hue value is a minimum, find the next relative max and min and calculate peakiness
      if ((hv - h_hist.at<float>((i+179) % 180)) <= 0 && (hv - h_hist.at<float>((i+1) % 180)) < 0)
      {
        float hSum = hv;
        int hDiff = 1;

        // Finds the next relative maximum
        float hpv; // hue peak value
        int hpi; // hue peak index
        for (int k = i + 1; k < (i + 180); k++) 
        {
          hpv = h_hist.at<float>(k);
          hSum += hpv;
          hDiff++;
          if ( (hpv - h_hist.at<float>((k+1) % 180)) > 0 )
          {
            hpi = k;
            break;
          }
        }

        // Finds the next relative minimum
        float hrmv; // hue right min value
        int hpmi; // huel right min index
        for (int k = hpi + 1; k < (i + 180); k++) 
        {
          hrmv = h_hist.at<float>(k);
          hSum += hrmv;
          hDiff++;
          if ( (hrmv - h_hist.at<float>((k+1) % 180)) <= 0 )
          {
            hpmi = k;
            break;
          }
        }

        // http://www.cs.ucf.edu/courses/cap6411/cap5415/spring02/Lecture-9-h.pdf (page 5)
        float peakiness = (1 - (hv + hrmv) / (2 * hpv)) * (1 - (hSum / (hDiff * hpv)));

        // if it enough of a peak, and the color takes up over a specific fraction of the image, count it as a peak
        if (peakiness >= minPeakiness && (hpv / resolutionArea) >= minPeakTheshold && (hpv / resolutionArea) <= maxPeakTheshold ) {
          HuePeak peak (i, hpi, hpmi);
          peaks.push_back(peak);
        }
      }
    }

    huePeaks = peaks;
  }

  // displays the hue histogram with a exponential scale
  void Histogram::drawHistogram(float exponent) 
  { 
    // Size of the histogram image
    int img_w = 720; int img_h = 640;
    int bin_w = cvRound( (double) img_w/180 );

    // Creates a black image and copy of the histogram
    cv::Mat img_hist (img_h, img_w, CV_8UC3, cv::Scalar(0,0,0) );
    cv::MatND h_hist_exp = h_hist.clone();

    // Makes the scale of the histogram exponential
    for (int i = 0; i < 180; i++) {
      h_hist_exp.at<float>(i) = pow(h_hist_exp.at<float>(i), exponent);
    }

    // Scales the histogram so the drawing spans the entire vertical heigh of the image
    cv::normalize(h_hist_exp, h_hist_exp, (img_h * 0.005f) , img_h * 0.9f, cv::NORM_MINMAX, -1, cv::Mat());

    // Draws lines at the hue peaks in the histogram
    for (std::list<HuePeak>::iterator it = huePeaks.begin(); it != huePeaks.end(); ++it) 
    {
      HuePeak peak = *it;
      int h = peak._peak;
      int hl = peak._l_valley;
      int hr = peak._r_valley;

      // Draw line at center of peak
      cv::line( img_hist, cv::Point( bin_w*(h), img_h ) ,
                         cv::Point( bin_w*(h), 0 ),
                         cv::Scalar(200,200,200), 1, 8, 0  );

      // Draw line at left threshold boundary
      cv::line( img_hist, cv::Point( bin_w*(hl), img_h ) ,
                         cv::Point( bin_w*(hl), 0 ),
                         cv::Scalar(64,64,64), 1, 8, 0  );

      // Draw line at right threshold boundary
      cv::line( img_hist, cv::Point( bin_w*(hr), img_h ) ,
                         cv::Point( bin_w*(hr), 0 ),
                         cv::Scalar(64,64,64), 1, 8, 0  );
    }

    // Draws each hue as a vertical line
    for (int i = 0; i < 180; i++)
    {
      // Gets RGB color from HSV based on i=hue
      cv::Mat color (1, 1, CV_8UC3, cv::Scalar(i,255,255) );
      cvtColor(color, color, CV_HSV2BGR);
      cv::Vec3b& rgb = color.at<cv::Vec3b>(0,0);
      cv::Scalar drawColor ( (unsigned int)rgb.val[0], 
                             (unsigned int)rgb.val[1], 
                             (unsigned int)rgb.val[2] );

      cv::line( img_hist, cv::Point( bin_w*(i), img_h - cvRound( (h_hist_exp.at<float>(i))) ) ,
                         cv::Point( bin_w*(i), img_h ),
                         drawColor, 2, 8, 0  );
    }

    // Show Histogram
    cv::namedWindow(HISTOGRAM_WINDOW);
    cv::imshow(HISTOGRAM_WINDOW, img_hist);
    cv::waitKey(3);
  }