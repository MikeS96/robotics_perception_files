#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection";

class Edge_Detector //Creo la clase Edge detector
{
  ros::NodeHandle nh_; //Creo objeto nh_ para ROS.
  image_transport::ImageTransport it_; //Estos son los atributos de la clase
  image_transport::Subscriber image_sub_; //Publisher de ROS
  image_transport::Publisher image_pub_; //Suscriber de ROS
  
public: 
  Edge_Detector() //Constructor de la clase, 
    : it_(nh_) //Inicializo el atributo it_ con nh_ de ROS
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &Edge_Detector::imageCb, this); //Suscribo el parametro image_sub a el topico /usb_cam/image_raw, pasandole la referencia de la clase Edge_detector y estara revisando el metodo imageCB
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1); //Publico el parametro image_pub a el topico edge_detector/raw_image, pasandole la referencia de la clase Edge_detector
    cv::namedWindow(OPENCV_WINDOW); //Inicio el objeto namedWindow con el nombre Raw Image window

  }

  ~Edge_Detector() //Destructor de la clase edte_detector
  {
    cv::destroyWindow(OPENCV_WINDOW); //destruyo el objeto namedWindow
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) //Metodo imageCB
  {

    cv_bridge::CvImagePtr cv_ptr; //Creo un objeto del tipo CvImagePyr perteneciente a el paquete cv_bridge
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){

	detect_edges(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }
  void detect_edges(cv::Mat img)
  {

   	cv::Mat src, src_gray;
	cv::Mat dst, detected_edges;

	int edgeThresh = 1;
	int lowThreshold = 200;
	int highThreshold =300;
	int kernel_size = 5;

	img.copyTo(src);

	cv::cvtColor( img, src_gray, CV_BGR2GRAY );
        cv::blur( src_gray, detected_edges, cv::Size(5,5) );
	cv::Canny( detected_edges, detected_edges, lowThreshold, highThreshold, kernel_size );

  	dst = cv::Scalar::all(0);
  	img.copyTo( dst, detected_edges);
	dst.copyTo(img);

    	cv::imshow(OPENCV_WINDOW, src);
    	cv::imshow(OPENCV_WINDOW_1, dst);
    	cv::waitKey(3);

  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
