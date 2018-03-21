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
#include <iostream>
#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include <utility>
#include <vector>
namespace apriltags_ros{
using namespace cv;
using namespace std;
using namespace zbar;

const int CV_QR_NORTH = 0;
const int CV_QR_EAST = 1;
const int CV_QR_SOUTH = 2;
const int CV_QR_WEST = 3;

float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1, Point2f v2);

Mat detect_qr(Mat & image, vector<Point2f>& src)
{
	// Creation of Intermediate 'Image' Objects required later
	Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image
	Mat edges(image.size(), CV_MAKETYPE(image.depth(), 1));			// To hold Grayscale Image
	Mat traces(image.size(), CV_8UC3);								// For Debug Visuals
	Mat qr, qr_raw, qr_gray, qr_thres;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> pointsseq;    //used to save the approximated sides of each contour

	int mark, A, B, C, top, right, bottom, median1, median2, outlier;
	float AB, BC, CA, dist, slope, areat, arear, areab, large, padding;

	int align, orientation;

	traces = Scalar(0, 0, 0);
	qr_raw = Mat::zeros(100, 100, CV_8UC3);
	qr = Mat::zeros(100, 100, CV_8UC3);
	qr_gray = Mat::zeros(100, 100, CV_8UC1);
	qr_thres = Mat::zeros(100, 100, CV_8UC1);

	cvtColor(image, gray, CV_RGB2GRAY);		// Convert Image captured from Image Input to GrayScale	
	Canny(gray, edges, 100, 200, 3);		// Apply Canny edge detection on the gray image

	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

	mark = 0;								// Reset all detected marker count for this frame

											// Get Moments for all Contours and the mass centers
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}


	// Start processing the contour data

	// Find Three repeatedly enclosed contours A,B,C
	// NOTE: 1. Contour enclosing other contours is assumed to be the three Alignment markings of the QR code.
	// 2. Alternately, the Ratio of areas of the "concentric" squares can also be used for identifying base Alignment markers.
	// The below demonstrates the first method

	for (int i = 0; i < contours.size(); i++)
	{
		//Find the approximated polygon of the contour we are examining
		approxPolyDP(contours[i], pointsseq, arcLength(contours[i], true)*0.02, true);
		if (pointsseq.size() == 4)      // only quadrilaterals contours are examined
		{
			int k = i;
			int c = 0;

			while (hierarchy[k][2] != -1)
			{
				k = hierarchy[k][2];
				c = c + 1;
			}
			if (hierarchy[k][2] != -1)
				c = c + 1;

			if (c >= 5)
			{
				if (mark == 0)		A = i;
				else if (mark == 1)	B = i;		// i.e., A is already found, assign current contour to B
				else if (mark == 2)	C = i;		// i.e., A and B are already found, assign current contour to C
				mark = mark + 1;
			}
		}
	}


	if (mark >= 3)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
	{
		// We have found the 3 markers for the QR code; Now we need to determine which of them are 'top', 'right' and 'bottom' markers

		// Determining the 'top' marker
		// Vertex of the triangle NOT involved in the longest side is the 'outlier'

		AB = cv_distance(mc[A], mc[B]);
		BC = cv_distance(mc[B], mc[C]);
		CA = cv_distance(mc[C], mc[A]);

		if (AB > BC && AB > CA)
		{
			outlier = C; median1 = A; median2 = B;
		}
		else if (CA > AB && CA > BC)
		{
			outlier = B; median1 = A; median2 = C;
		}
		else if (BC > AB && BC > CA)
		{
			outlier = A;  median1 = B; median2 = C;
		}

		top = outlier;							// The obvious choice

		dist = cv_lineEquation(mc[median1], mc[median2], mc[outlier]);	// Get the Perpendicular distance of the outlier from the longest side			
		slope = cv_lineSlope(mc[median1], mc[median2], align);		// Also calculate the slope of the longest side

																	// Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
																	// Determine the 'right' and 'bottom' markers

		if (align == 0)
		{
			bottom = median1;
			right = median2;
		}
		else if (slope < 0 && dist < 0)		// Orientation - North
		{
			bottom = median1;
			right = median2;
			orientation = CV_QR_NORTH;
		}
		else if (slope > 0 && dist < 0)		// Orientation - East
		{
			right = median1;
			bottom = median2;
			orientation = CV_QR_EAST;
		}
		else if (slope < 0 && dist > 0)		// Orientation - South			
		{
			right = median1;
			bottom = median2;
			orientation = CV_QR_SOUTH;
		}

		else if (slope > 0 && dist > 0)		// Orientation - West
		{
			bottom = median1;
			right = median2;
			orientation = CV_QR_WEST;
		}


		// To ensure any unintended values do not sneak up when QR code is not present
		float area_top, area_right, area_bottom;

		if (top < contours.size() && right < contours.size() && bottom < contours.size() && contourArea(contours[top]) > 10 && contourArea(contours[right]) > 10 && contourArea(contours[bottom]) > 10)
		{

			vector<Point2f> L, M, O, tempL, tempM, tempO;
			Point2f N;

			vector<Point2f> dst;		// src - Source Points basically the 4 end co-ordinates of the overlay image
											// dst - Destination Points to transform overlay image	

			Mat warp_matrix;

			cv_getVertices(contours, top, slope, tempL);
			cv_getVertices(contours, right, slope, tempM);
			cv_getVertices(contours, bottom, slope, tempO);

			cv_updateCornerOr(orientation, tempL, L); 			// Re-arrange marker corners w.r.t orientation of the QR code
			cv_updateCornerOr(orientation, tempM, M); 			// Re-arrange marker corners w.r.t orientation of the QR code
			cv_updateCornerOr(orientation, tempO, O); 			// Re-arrange marker corners w.r.t orientation of the QR code

			int iflag = getIntersectionPoint(M[1], M[2], O[3], O[2], N);


			src.push_back(L[0]);
			src.push_back(M[1]);
			src.push_back(N);
			src.push_back(O[3]);

			dst.push_back(Point2f(0, 0));
			dst.push_back(Point2f(qr.cols, 0));
			dst.push_back(Point2f(qr.cols, qr.rows));
			dst.push_back(Point2f(0, qr.rows));

			if (src.size() == 4 && dst.size() == 4)			// Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
			{
				warp_matrix = getPerspectiveTransform(src, dst);
				warpPerspective(image, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
				copyMakeBorder(qr_raw, qr, 10, 10, 10, 10, BORDER_CONSTANT, Scalar(255, 255, 255));

				cvtColor(qr, qr_gray, CV_RGB2GRAY);
				threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);

				//threshold(qr_gray, qr_thres, 0, 255, CV_THRESH_OTSU);
				//for( int d=0 ; d < 4 ; d++){	src.pop_back(); dst.pop_back(); }
			}
		}
	}
	return qr_raw;
}

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

float cv_distance(Point2f P, Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//	  calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f L, Point2f M, Point2f J)
{
	float a, b, c, pdist;

	a = -((M.y - L.y) / (M.x - L.x));
	b = 1.0;
	c = (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y;

	// Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

	pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
	return pdist;
}

// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
//	  indicates the line is vertical and the slope is infinity.

float cv_lineSlope(Point2f L, Point2f M, int& alignement)
{
	float dx, dy;
	dx = M.x - L.x;
	dy = M.y - L.y;

	if (dy != 0)
	{
		alignement = 1;
		return (dy / dx);
	}
	else				// Make sure we are not dividing by zero; so use 'alignement' flag
	{
		alignement = 0;
		return 0.0;
	}
}



// Function: Routine to calculate 4 Corners of the Marker in Image Space using Region partitioning
// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
//	The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
//	exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
//	4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
//	every contour point in that region, the farthest point is deemed as the vertex of that region. Calculating
//	for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
{
	Rect box;
	box = boundingRect(contours[c_id]);

	Point2f M0, M1, M2, M3;
	Point2f A, B, C, D, W, X, Y, Z;

	A = box.tl();
	B.x = box.br().x;
	B.y = box.tl().y;
	C = box.br();
	D.x = box.tl().x;
	D.y = box.br().y;


	W.x = (A.x + B.x) / 2;
	W.y = A.y;

	X.x = B.x;
	X.y = (B.y + C.y) / 2;

	Y.x = (C.x + D.x) / 2;
	Y.y = C.y;

	Z.x = D.x;
	Z.y = (D.y + A.y) / 2;

	float dmax[4];
	dmax[0] = 0.0;
	dmax[1] = 0.0;
	dmax[2] = 0.0;
	dmax[3] = 0.0;

	float pd1 = 0.0;
	float pd2 = 0.0;

	if (slope > 5 || slope < -5)
	{

		for (int i = 0; i < contours[c_id].size(); i++)
		{
			pd1 = cv_lineEquation(C, A, contours[c_id][i]);	// Position of point w.r.t the diagonal AC 
			pd2 = cv_lineEquation(B, D, contours[c_id][i]);	// Position of point w.r.t the diagonal BD

			if ((pd1 >= 0.0) && (pd2 > 0.0))
			{
				cv_updateCorner(contours[c_id][i], W, dmax[1], M1);
			}
			else if ((pd1 > 0.0) && (pd2 <= 0.0))
			{
				cv_updateCorner(contours[c_id][i], X, dmax[2], M2);
			}
			else if ((pd1 <= 0.0) && (pd2 < 0.0))
			{
				cv_updateCorner(contours[c_id][i], Y, dmax[3], M3);
			}
			else if ((pd1 < 0.0) && (pd2 >= 0.0))
			{
				cv_updateCorner(contours[c_id][i], Z, dmax[0], M0);
			}
			else
				continue;
		}
	}
	else
	{
		int halfx = (A.x + B.x) / 2;
		int halfy = (A.y + D.y) / 2;

		for (int i = 0; i < contours[c_id].size(); i++)
		{
			if ((contours[c_id][i].x < halfx) && (contours[c_id][i].y <= halfy))
			{
				cv_updateCorner(contours[c_id][i], C, dmax[2], M0);
			}
			else if ((contours[c_id][i].x >= halfx) && (contours[c_id][i].y < halfy))
			{
				cv_updateCorner(contours[c_id][i], D, dmax[3], M1);
			}
			else if ((contours[c_id][i].x > halfx) && (contours[c_id][i].y >= halfy))
			{
				cv_updateCorner(contours[c_id][i], A, dmax[0], M2);
			}
			else if ((contours[c_id][i].x <= halfx) && (contours[c_id][i].y > halfy))
			{
				cv_updateCorner(contours[c_id][i], B, dmax[1], M3);
			}
		}
	}

	quad.push_back(M0);
	quad.push_back(M1);
	quad.push_back(M2);
	quad.push_back(M3);

}

// Function: Compare a point if it more far than previously recorded farthest distance
// Description: Farthest Point detection using reference point and baseline distance
void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner)
{
	float temp_dist;
	temp_dist = cv_distance(P, ref);

	if (temp_dist > baseline)
	{
		baseline = temp_dist;			// The farthest distance is the new baseline
		corner = P;						// P is now the farthest point
	}

}

// Function: Sequence the Corners wrt to the orientation of the QR Code
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT)
{
	Point2f M0, M1, M2, M3;
	if (orientation == CV_QR_NORTH)
	{
		M0 = IN[0];
		M1 = IN[1];
		M2 = IN[2];
		M3 = IN[3];
	}
	else if (orientation == CV_QR_EAST)
	{
		M0 = IN[1];
		M1 = IN[2];
		M2 = IN[3];
		M3 = IN[0];
	}
	else if (orientation == CV_QR_SOUTH)
	{
		M0 = IN[2];
		M1 = IN[3];
		M2 = IN[0];
		M3 = IN[1];
	}
	else if (orientation == CV_QR_WEST)
	{
		M0 = IN[3];
		M1 = IN[0];
		M2 = IN[1];
		M3 = IN[2];
	}

	OUT.push_back(M0);
	OUT.push_back(M1);
	OUT.push_back(M2);
	OUT.push_back(M3);
}

// Function: Get the Intersection Point of the lines formed by sets of two points
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
{
	Point2f p = a1;
	Point2f q = b1;
	Point2f r(a2 - a1);
	Point2f s(b2 - b1);

	if (cross(r, s) == 0) { return false; }

	float t = cross(q - p, s) / cross(r, s);

	intersection = p + t*r;
	return true;
}

float cross(Point2f v1, Point2f v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}

Eigen::Matrix4d getRelativeTransform(std::pair<float, float>* p, double tag_size, double fx, double fy, double px, double py) {
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

  std::pair<float, float> p1 = p[0];
  std::pair<float, float> p2 = p[1];
  std::pair<float, float> p3 = p[2];
  std::pair<float, float> p4 = p[3];
  imgPts.push_back(cv::Point2f(p1.first, p1.second));
  imgPts.push_back(cv::Point2f(p2.first, p2.second));
  imgPts.push_back(cv::Point2f(p3.first, p3.second));
  imgPts.push_back(cv::Point2f(p4.first, p4.second));

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           fx, 0, px,
                           0, fy, py,
                           0,  0,  1);
  cv::Vec4f distParam(0,0,0,0); // all 0?
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Matrix4d T; 
  T.topLeftCorner(3,3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;

  return T;
}

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
    std::vector<cv::Point2f> src;
    cv::Mat qr_raw = detect_qr(cv_ptr->image,src);
    cv::Mat grey;
    cv::cvtColor(qr_raw, grey, CV_BGR2GRAY);
    int width = qr_raw.cols;
    int height = qr_raw.rows;
    uchar *raw = (uchar *)grey.data;
		
    zbar::Image image(width, height, "Y800", raw, width * height); 
    int n = tag_detector_->scan(image);
    ROS_DEBUG("%d tag detected", n);

    double fx = cam_info->K[0];
    double fy = cam_info->K[4];
    double px = cam_info->K[2];
    double py = cam_info->K[5];
    
    if(!sensor_frame_id_.empty())
      cv_ptr->header.frame_id = sensor_frame_id_;
    
    duckietown_msgs::AprilTagDetectionArray tag_detection_array;
    geometry_msgs::PoseArray tag_pose_array;
    tag_pose_array.header = cv_ptr->header;
    
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      double tag_size = 0.065;
      std::pair<float,float> p[4];

      for (int i = 0; i < 4; i++) {
        p[i].first = src[i].x;
        p[i].second = src[i].y;
      }


      Eigen::Matrix4d transform = getRelativeTransform(p,tag_size, fx, fy, px, py);
      Eigen::Matrix3d rot = transform.block(0,0,3,3);
      Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

      geometry_msgs::PoseStamped tag_pose;
      tag_pose.pose.position.x = transform(0,3);
      tag_pose.pose.position.y = transform(1,3);
      tag_pose.pose.position.z = transform(2,3);
      tag_pose.pose.orientation.x = rot_quaternion.x();
      tag_pose.pose.orientation.y = rot_quaternion.y();
      tag_pose.pose.orientation.z = rot_quaternion.z();
      tag_pose.pose.orientation.w = rot_quaternion.w();
      tag_pose.header = cv_ptr->header;

      duckietown_msgs::AprilTagDetection tag_detection;
      tag_detection.pose = tag_pose;
      tag_detection.id = atoi(symbol->get_data().c_str());
      cv::circle(cv_ptr->image,cv::Point(symbol->get_location_x(0), symbol->get_location_y(0)), 10,cv::Scalar( 255, 0, 0 ),8);
      int mask = 1024*1024;
      while (mask>0) {
        std::cout << int(bool(mask&tag_detection.id));
        mask = mask >> 1;
      }
      std::cout << std::endl;
      tag_detection.size = tag_size;
      tag_detection_array.detections.push_back(tag_detection);
      tag_pose_array.poses.push_back(tag_pose.pose);
    }
    detections_pub_.publish(tag_detection_array);
    image_pub_.publish(cv_ptr->toImageMsg());
  }


  std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
    std::map<int, AprilTagDescription> descriptions;
    /*ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
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
    }*/
    return descriptions;
  }


}
