#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <computer_vision/histogram.hpp>
#include <computer_vision/buoy.hpp>
#include <iostream>

  static const std::string CONTOURS_WINDOW = "Contours";

  std::list<BuoyCircle> Buoy::getBuoyCircles()
  {
    return buoyCircles;
  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
  // hues contains the colors that were thresholded
  // minArea is the smallest size of rectange we'll count as an object
  void Buoy::createContours(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea, int minThresh[], int maxThresh[])
  {
    std::list<BuoyCircle> foundObjects;
    std::list<cv::Mat>::iterator it_imgs = images.begin();
    std::list<HuePeak>::iterator it_hues = hues.begin();

    // Iterate over each threshold image result
    for ( int h = 0; h < images.size(); h++)
    {
      cv::Mat img = *it_imgs;
      HuePeak peak = *it_hues;
      it_imgs++;
      it_hues++;
      // If the object doesn't fall within our hue ranges then we should continue to the next image.
      bool proceed = false;
      for ( int i = 0; i < 3; i++)
      {
        proceed |= testPeak(peak, minThresh[i], maxThresh[i]);
      }
      if (!proceed)
      {
        continue;
      }
      cv::Mat img_contours = img.clone();
      int hue = peak._peak;

      // Gets A list of points approximating each shape 
      contours = std::vector<std::vector<cv::Point> >();
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(img_contours, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      std::vector<cv::Point2f> center ( contours.size() );
      std::vector<float> radii ( contours.size() );
      contours_poly = std::vector<std::vector<cv::Point> > ( contours.size() );
      std::vector<cv::RotatedRect> boundRect ( contours.size());

      // Finds a bounding box for each of those shapes
      for( int i = 0; i < contours.size(); i++ )
      { 
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::minAreaRect( cv::Mat(contours_poly[i]) );
        cv::minEnclosingCircle( contours_poly[i], center[i], radii[i] );
      }
      for( int i = 0; i < contours.size(); i++ )
      {
	double contourArea = cv::contourArea( contours[i] ); // Area of the object
	if (contourArea > minArea)
	{
	  double circleArea = M_PI * std::pow(radii[i], 2.0f); // Area of the best approximating circle
	  // We want only contours which would have been circles.
	  if ( (circleArea / contourArea) < (boundRect[i].size.area() / contourArea) )
	  {
	    BuoyCircle buoyCircle = BuoyCircle(hue, center[i], radii[i]);
	    foundObjects.push_back(buoyCircle);

	    // Prints out the x,y position of a circle and its hue.
	    std::cout << "Circle found at " << center[i].x << ", " << center[i].y << " with hue " << hue << "\n";
	  }
        }
      }
    }
    buoyCircles = foundObjects;
  }

// Tests to see if our peak is between "low" and "high".

  bool Buoy::testPeak(HuePeak peak, int low, int high)
  {
      int peakLow = peak._l_valley;
      int peakHigh = peak._r_valley;
      int threshWidth = (high - low) % 180;
      return (peakLow > low && peakHigh < high)
	|| (peakLow < low && peakHigh < high && high - peakHigh < threshWidth)
        || (peakLow > low && peakHigh > high && peakLow - low < threshWidth);
  }

 // drawing is the image we are drawng the boxes over

  void Buoy::drawContours(cv::Mat drawing)
  {
    for (std::list<BuoyCircle>::iterator it = buoyCircles.begin(); it != buoyCircles.end(); ++it) 
    {
      BuoyCircle buoyCircle = *it;
      // Gets an RGB color from HSV based on hue
      cv::Mat color (1, 1, CV_8UC3, cv::Scalar(buoyCircle._hue,255,255) );
      cvtColor(color, color, CV_HSV2BGR);
      cv::Vec3b& rgb = color.at<cv::Vec3b>(0,0);
      cv::Scalar drawColor ( (unsigned int)rgb.val[0], 
                             (unsigned int)rgb.val[1], 
                             (unsigned int)rgb.val[2] );
      cv::circle( drawing, buoyCircle._circle_center, buoyCircle._circle_radius, drawColor, 2, 8, 0 );
    }
    
    //cv::namedWindow(CONTOURS_WINDOW);
    cv::imshow(CONTOURS_WINDOW, drawing);
    cv::waitKey(3);
  }
  BuoyCircle::BuoyCircle(unsigned int hue, cv::Point2f circle_center, float circle_radius)
  {
    	_hue = hue;
        _circle_center = circle_center;
        _circle_radius = circle_radius;
  }
