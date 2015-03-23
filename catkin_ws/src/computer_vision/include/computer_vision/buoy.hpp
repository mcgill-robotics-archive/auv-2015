#ifndef BUOY_H 
#define BUOY_H 

// Stores information about any distinct objects we find in the image

class BuoyCircle
{
public:
	unsigned int _hue;
        cv::Point2f _circle_center;
        float _circle_radius;
        BuoyCircle(unsigned int hue, cv::Point2f circle_center, float circle_radius);
};

class Buoy
{
private:
	std::list<BuoyCircle> buoyCircles;
	std::vector<std::vector<cv::Point> > contours;
        std::vector<std::vector<cv::Point> > contours_poly;

public:
	bool testPeak(HuePeak peak, int low, int high);
	std::list<BuoyCircle> getBuoyCircles();
	void createContours(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea, int minThresh[], int maxThresh[]);
	void drawContours(cv::Mat drawing);
};

#endif  
