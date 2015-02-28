#ifndef OBJECT_FINDER_H 
#define OBJECT_FINDER_H 

// Represents each of the shapes we need to detect
enum ObjectType { Line, Rect, Circle };

// Stores information about any distinct objects we find in the image
class VisibleObject
{
public:
	ObjectType _objectType;
	unsigned int _hue;
	cv::RotatedRect _rect;
	cv::Point2f _circle_center;
	float _circle_radius;
	cv::Vec4f _line;

	VisibleObject(ObjectType objectType, unsigned int hue, cv::RotatedRect rect, cv::Point2f circle_center, float circle_radius, cv::Vec4f line);
};

class ObjectFinder
{
private:
	std::list<VisibleObject> visibleObjects;
	std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contours_poly;

public:
	std::list<VisibleObject> getVisibleObjects();
	void findObjects(std::list<cv::Mat> images, std::list<HuePeak> hues, float minArea, float minLineLenghtRatio);
	void drawObjects(cv::Mat drawing);
};

#endif  
