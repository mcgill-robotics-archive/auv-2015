#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <computer_vision/histogram.hpp>
#include <computer_vision/object_finder.hpp>

  static const std::string CONTOURS_WINDOW = "Contours";

  std::list<VisibleObject> ObjectFinder::getVisibleObjects()
  {
    return visibleObjects;
  }

  // hues contains the colors that were thresholded
  // minArea is the smallest size of rectange we'll count as an object
  void ObjectFinder::createContours(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea)
  {
    std::list<VisibleObject> foundObjects;

    std::list<cv::Mat>::iterator it_imgs = images.begin();
    std::list<HuePeak>::iterator it_hues = hues.begin();

    // Iterate over each threshold image result
    for ( int h = 0; h < images.size(); h++)
    {
      cv::Mat img = *it_imgs;
      HuePeak peak = *it_hues;
      it_imgs++;
      it_hues++;

      cv::Mat img_contours = img.clone();
      int hue = peak._peak;

      // Gets A list of points approximating each shape 
      contours = std::vector<std::vector<cv::Point> >();
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(img_contours, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      contours_poly = std::vector<std::vector<cv::Point> > ( contours.size() );
      std::vector<cv::Rect> boundRect ( contours.size() );

      // Finds a bounding box for each of those shapes
      for( int i = 0; i < contours.size(); i++ )
      { 
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      }

      for( int i = 0; i < contours.size(); i++ )
      { 
        if (boundRect[i].area() > minArea && !isRectContained(boundRect, boundRect[i]))
        {
          VisibleObject visibleObject(hue, boundRect[i]);
          foundObjects.push_back(visibleObject);
        }
      }
    }

    visibleObjects = foundObjects;
  }
  
  // drawing is the image we are drawng the boxes over
  void ObjectFinder::drawContours(cv::Mat drawing)
  {
    for (std::list<VisibleObject>::iterator it = visibleObjects.begin(); it != visibleObjects.end(); ++it) 
    {
      VisibleObject visibleObject = *it;
      // Gets an RGB color from HSV based on hue
      cv::Mat color (1, 1, CV_8UC3, cv::Scalar(visibleObject._hue,255,255) );
      cvtColor(color, color, CV_HSV2BGR);
      cv::Vec3b& rgb = color.at<cv::Vec3b>(0,0);
      cv::Scalar drawColor ( (unsigned int)rgb.val[0], 
                             (unsigned int)rgb.val[1], 
                             (unsigned int)rgb.val[2] );

      // Draws the bounding box
      cv::rectangle( drawing, visibleObject._rect.tl(), visibleObject._rect.br(), drawColor, 2, 8, 0 );
      // Draws the center of the object
      cv::circle( drawing, visibleObject._center, 5, drawColor, 2, 8, 0 );
    }
    
    cv::namedWindow(CONTOURS_WINDOW);
    cv::imshow(CONTOURS_WINDOW, drawing);
    cv::waitKey(3);
  }

  // Returns whether one rectangle is inside another one
  bool ObjectFinder::isRectContained(std::vector<cv::Rect> rects, cv::Rect rect) 
  {
    for( int i = 0; i < rects.size(); i++ ) 
    {
      if( rects[i].contains(rect.tl()) && rects[i].contains(rect.br()) ) {
        return true;
      }
    }
    return false;
  }

  VisibleObject::VisibleObject(unsigned int hue, cv::Rect rect)
  {
    _hue = hue;
    _rect = rect;

    int center_x, center_y;
    center_x = (rect.tl().x + rect.br().x)/2;
    center_y = (rect.tl().y + rect.br().y)/2;
    _center = cv::Point(center_x, center_y);
  }
