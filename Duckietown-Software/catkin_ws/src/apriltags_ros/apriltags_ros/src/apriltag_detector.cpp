#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <duckietown_msgs/AprilTagDetection.h>
#include <duckietown_msgs/AprilTagDetectionArray.h>
#include <XmlRpcException.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <zbar.h>
#include <stdlib.h>

/*cv::VideoCapture cap(0);
						 
	if (!cap.isOpened())  
	{
		std::cout << "Cannot open the video cam" << std::endl;
		return -1;
	}
	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	cv::Mat frame;

	bool bSuccess = cap.read(frame); 
	if (!bSuccess) 
	{
		std::cout << "Cannot read a frame from video stream" << std::endl;		
	}
	cv::Mat grey;
	cv::cvtColor(frame, grey, CV_BGR2GRAY);
	int width = frame.cols;
	int height = frame.rows;
	uchar *raw = (uchar *)grey.data;
		
	zbar::Image image(width, height, "Y800", raw, width * height);
	// scan the image for barcodes   
	int n = scanner.scan(image);
	//example
	bool qr_exists = qr_found(n);
	for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
	{
		std::vector<cv::Point> vp;
		
		std::cout << symbol->get_data() << std::endl;   
		int n = symbol->get_location_size();
		for (int i = 0; i<n; i++)
		{
			vp.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}
		symbol->get_location_size();
	}	 
	system("pause");
return 0;*/

namespace apriltags_ros{

  AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){
    /*XmlRpc::XmlRpcValue april_tag_descriptions;
    if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
      ROS_WARN("No april tags specified");
    }
    else{
      try{
	descriptions_ = parse_tag_descriptions(april_tag_descriptions);
      } catch(XmlRpc::XmlRpcException e){
	ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
      }
    }*/
    
    if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
      sensor_frame_id_ = "";
    }
    
    //AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;
    tag_detector_= boost::shared_ptr<zbar::ImageScanner>(new zbar::ImageScanner);
    tag_detector_->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
    switch_sub_ = nh.subscribe("switch",1,&AprilTagDetector::switchCB, this);
    image_pub_ = it_.advertise("tag_detections_image", 1);
    detections_pub_ = nh.advertise<duckietown_msgs::AprilTagDetectionArray>("tag_detections", 1);
    on_switch=true;
  }
  AprilTagDetector::~AprilTagDetector(){
    image_sub_.shutdown();
  }

  void AprilTagDetector::switchCB(const duckietown_msgs::BoolStamped::ConstPtr& switch_msg){
    on_switch=switch_msg->data;
  }
  
  void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat grey;
    cv::cvtColor(cv_ptr->image, grey, CV_BGR2GRAY);
    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;
    uchar *raw = (uchar *)grey.data;
		
    zbar::Image image(width, height, "Y800", raw, width * height); 
    int n = tag_detector_->scan(image);
    ROS_DEBUG("%d tag detected", n);
    
    if(!sensor_frame_id_.empty())
      cv_ptr->header.frame_id = sensor_frame_id_;
    
    duckietown_msgs::AprilTagDetectionArray tag_detection_array;
    geometry_msgs::PoseArray tag_pose_array;
    tag_pose_array.header = cv_ptr->header;
    
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      double tag_size = 0.065;
      geometry_msgs::PoseStamped tag_pose;
      tag_pose.pose.position.x = 0;
      tag_pose.pose.position.y = 0;
      tag_pose.pose.position.z = 0;
      tag_pose.pose.orientation.x = 0;
      tag_pose.pose.orientation.y = 0;
      tag_pose.pose.orientation.z = 0;
      tag_pose.pose.orientation.w = 1;
      tag_pose.header = cv_ptr->header;

      duckietown_msgs::AprilTagDetection tag_detection;
      tag_detection.pose = tag_pose;
      tag_detection.id = atoi(symbol->get_data().c_str());
      tag_detection.size = tag_size;
      tag_detection_array.detections.push_back(tag_detection);
      tag_pose_array.poses.push_back(tag_pose.pose);
    }
    detections_pub_.publish(tag_detection_array);
    image_pub_.publish(cv_ptr->toImageMsg());
  }


  std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
    std::map<int, AprilTagDescription> descriptions;
    ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
      XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
      ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      int id = (int)tag_description["id"];
      double size = (double)tag_description["size"];

      std::string frame_name;
      if(tag_description.hasMember("frame_id")){
	ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
	frame_name = (std::string)tag_description["frame_id"];
      }
      else{
	std::stringstream frame_name_stream;
	frame_name_stream << "tag_" << id;
	frame_name = frame_name_stream.str();
      }
      AprilTagDescription description(id, size, frame_name);
      ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
      descriptions.insert(std::make_pair(id, description));
    }
    return descriptions;
  }


}
