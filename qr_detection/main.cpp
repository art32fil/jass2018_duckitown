#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <zbar.h>  
#include <iostream>  

bool qr_found(int n)
{
	if (n == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

int main(int argc, char* argv[])
{
	cv::VideoCapture cap(0);
						 
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
	return 0;
}

