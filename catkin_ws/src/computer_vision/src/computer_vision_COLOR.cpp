// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/CvBridge.h>

// class MyVisionNode
// {
//   	ros::NodeHandle nh_;
// 	image_transport::ImageTransport it_;
// 	image_transport::CameraSubscriber sub_;
// 	image_transport::Publisher pub_;
// 	sensor_msgs::CvBridge bridge_;

// 	MyVisionNode:
//   	public()
//     	: it_(nh_)
//    		{
//      		sub_ = it_.subscribeCamera("camera/image_rect_color", 1, &MyVisionNode::imageCb, this);
//      		pub_ = it_.advertise("camera/image_out_vitaly", 1);
//    		}
 
//    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
//                 const sensor_msgs::CameraInfoConstPtr& info_msg)
//    		{
// 	   		IplImage *cv_image = NULL;
// 	     	try {
// 	       		cv_image = bridge_.imgMsgToCv(image_msg, "bgr8");
// 	     	}
// 	     	catch (sensor_msgs::CvBridgeException& error) {
// 	       	ROS_ERROR("Couldn't convert image with encoding %s",
// 	        image_msg­->encoding.c_str());
// 	    	return;
// 	    }

// 	    image = image_msg;

// 	   	pub_.publish(bridge_.cvToImgMsg(image, "bgr8"));
//    	}
// };

// void main(int argc, char **argv){
// 	//ros::init(argc, argv, "Vitaly_Vision"); // Initiate ROS node
//   	MyVisionNode vitalijs(); // Create new grid mapper object
//   	vitalijs.spin(); // Execute FSM loop
// }


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //
    cv::Mat hsvIm, threshImage;
    cvtColor(cv_ptr->image,hsvIm,CV_BGR2HSV);
    inRange(hsvIm,cv::Scalar(80,0,0),cv::Scalar(110,255,255),threshImage);

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, threshImage);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}