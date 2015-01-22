#ifndef THRESHOLD_H 
#define THRESHOLD_H 

class Threshold
{
private:
	int getMinHueDifference(int ho, int h);

public:
	std::list<cv::Mat> threshold(cv::Mat& img, std::list<HuePeak> hues, unsigned int s_min, unsigned int v_min );
	void drawThreshold(std::list<cv::Mat> images, std::list<HuePeak> hues);
};

#endif  
