#ifndef HISTOGRAM_H 
#define HISTOGRAM_H 

// class that contains a peak hue and it's adjacent minimums
class HuePeak
{
public:
	int _l_valley;
	int _peak;
	int _r_valley;
	HuePeak(int l_valley, int peak, int r_valley);
};

class Histogram
{
private:
	cv::MatND h_hist;
    std::list<HuePeak> huePeaks;

public:
  	std::list<HuePeak> getHuePeaks();
	void createHistogram(cv::Mat& img_hsv);
  	void findPeakHues (float minPeakTheshold, float maxPeakTheshold, float minPeakiness, int resolutionArea);
	void drawHistogram(float exponent);
};

#endif  
