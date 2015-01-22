#ifndef OBJECT_FINDER_H 
#define OBJECT_FINDER_H 

// Stores information about any distinct objects we find in the image
class VisibleObject
{
public:
	unsigned int _hue;
	cv::Rect _rect;
	cv::Point _center;

	VisibleObject(unsigned int hue, cv::Rect rect);
};

class ObjectFinder
{
private:
	std::list<VisibleObject> visibleObjects;
	std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contours_poly;
	bool isRectContained(std::vector<cv::Rect> rects, cv::Rect rect);

public:
	std::list<VisibleObject> getVisibleObjects();
	void createContours(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea);
	void drawContours(cv::Mat drawing);
};

#endif  
