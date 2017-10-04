#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	cout << "Press ESC to quit" << endl;

	VideoCapture cap(0); //capture the video from web cam
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	
	int iLowH = 0;	//0
	int iHighH = 11;	//179

	int iLowS = 158;	//0
	int iHighS = 255;	//255

	int iLowV = 140;	//0
	int iHighV = 255;	//255
	
	//namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	/*
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179);	//Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255);	//Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255);	//Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	*/

	do
	{	
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}


		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		/*cout << "Width : " << imgThresholded.cols << endl;
		cout << "Height : " << imgThresholded.rows << endl;	*/

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0);	//Blur Effect

		vector <vector<Point> > contours;
		vector <Vec4i> hierarchy;

		Canny(imgThresholded, imgThresholded, 50, 120, 3);
		findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		// Get the moments of image
		vector <Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		// Get the mass centres
		vector <Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			cout << "Mass Centre : " << mc[i] << endl;
		}
		
		// Draw contours
		Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);
		
		for (size_t i = 0; i< contours.size(); i++)
		{
			drawContours(drawing, contours, (int)i, Scalar(0, 0, 255), 1, 8, hierarchy, 0, Point());
			circle(drawing, mc[i], 4, Scalar(0, 255, 0), -1, 8, 0);
		}
		
		//namedWindow("Contours", CV_WINDOW_AUTOSIZE);
		//imshow("Contours", drawing);

		//imshow("Thresholded Image", imgThresholded); //show the thresholded image
		//imshow("Original", imgOriginal); //show the original image


	} while (waitKey(10) != 27);

	return 0;

}