#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <math.h>
#include <computer_vision/histogram.hpp>
#include <computer_vision/object_finder.hpp>

  static const std::string CONTOURS_WINDOW = "Contours";

  std::list<VisibleObject> ObjectFinder::getVisibleObjects()
  {
    return visibleObjects;
  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
  // hues contains the colors that were thresholded
  // minArea is the smallest size of rectange we'll count as an object
  void ObjectFinder::findObjects(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea, float minLineLenghtRatio)
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
      cv::findContours(img_contours, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      contours_poly = std::vector<std::vector<cv::Point> > ( contours.size() );
      std::vector<cv::RotatedRect> boundRect ( contours.size() ); // Best bounding rectangels
      std::vector<cv::Point2f> center ( contours.size() ); // Best bounding circles' centers
      std::vector<float> radii ( contours.size() ); // Best bounding circles' radii
      std::vector<cv::Vec4f> lines ( contours.size() ); // Line along the longest axis of an object

      // Finds a bounding box for each of those shapes
      for( int i = 0; i < contours.size(); i++ )
      {
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::minAreaRect( contours_poly[i] ); // Gets bounding rect
        cv::minEnclosingCircle( contours_poly[i], center[i], radii[i] ); // Gets bounding circle
        cv::fitLine( contours_poly[i], lines[i], CV_DIST_L2, 0, 0.01, 0.01 );
      }

      // If the object is large enough to likely be relevant, make an object for it and determine the type
      for( int i = 0; i < contours.size(); i++ )
      {
        double contourArea = cv::contourArea( contours[i] ); // Area of the object

        if (contourArea > minArea)
        {
          double circleArea = M_PI * std::pow(radii[i], 2.0f); // Area of the best approximating circle
          ObjectType objectType;

          // if the circle is a closer approximation of the objects area than a rectangle, call it a circle
          if ( (circleArea / contourArea) < (boundRect[i].size.area() / contourArea) ) 
          {
            objectType = Circle;
          } 
          else 
          {
            // get the lenght of the longer and shorter side
            float lenght = std::max(boundRect[i].size.width, boundRect[i].size.height);
            float width = std::min(boundRect[i].size.width, boundRect[i].size.height);

            // if the longer side is over certain number of times the lengh of the short side, we call it a line
            if ( lenght / width > minLineLenghtRatio) 
            {
              objectType = Line;
            }
            else
            {
              objectType = Rect;
            }
          } 

          VisibleObject visibleObject(objectType, hue, boundRect[i], center[i], radii[i], lines[i]);
          foundObjects.push_back(visibleObject);
        }
      }
    }

    visibleObjects = foundObjects;
  }
  
  // drawing is the image we are drawng the boxes over
  void ObjectFinder::drawObjects(cv::Mat drawing)
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

      // Draws the appropriate representation of the object
      if ( visibleObject._objectType == Rect ) 
      {
        cv::Point2f vertices[4];
        visibleObject._rect.points(vertices);
        for (int i = 0; i < 4; i++) 
        {
          cv::line(drawing, vertices[i], vertices[(i+1)%4], drawColor, 2, 8, 0 );
        }
      }
      else if ( visibleObject._objectType == Circle )
      {
        cv::circle( drawing, visibleObject._circle_center, visibleObject._circle_radius, drawColor, 2, 8, 0 );
      }
      else if ( visibleObject._objectType == Line ) 
      {
        float vx = visibleObject._line[0];
        float vy = visibleObject._line[1];
        float x = visibleObject._line[2];
        float y = visibleObject._line[3];

        cv::Point2f vertices[2];
        vertices[0] = cv::Point2f (0, (-x*vy/vx) + y);
        vertices[1] = cv::Point2f ((drawing.cols-1), ((drawing.cols-x)*vy/vx)+y);
        
        cv::line(drawing, vertices[0], vertices[1], drawColor, 2, 8, 0 );
      }
    }
    
    cv::namedWindow(CONTOURS_WINDOW);
    cv::imshow(CONTOURS_WINDOW, drawing);
    cv::waitKey(3);
  }

  VisibleObject::VisibleObject(ObjectType objectType, unsigned int hue, cv::RotatedRect rect, cv::Point2f circle_center, float circle_radius, cv::Vec4f line)
  {
    _objectType = objectType;
    _hue = hue;
    _rect = rect;
    _circle_center = circle_center;
    _circle_radius = circle_radius;
    _line = line;
  }
