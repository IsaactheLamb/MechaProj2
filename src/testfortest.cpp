{
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
	

	while (true)
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

		X_prev = X;

		// Get the mass centres
		vector <Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			X = mc[i].x;
			delay(loop_del);
		}
	}
}

//================================================================

if (X != X_prev)
{
	if (X <= 256)	// target on the right
	{
		force_dir_L = 1;
		force_dir_R = 0;
	}


	else if	(X >= 384) // target on the left
	{
		force_dir_L = 0;
		force_dir_R = 1;
	}
}


else
{	// X =  256 <= X <384 : go forward
	//cout << " RUNNING ELSE CASE &%&%&%&%&%&%&%&%" << endl;
	force_dir_L = 1;
	force_dir_R = 1;
}


/*	If X < 128 : turnright fast
	If 128 <= X <256 : turn right slow
		0-256: right
	If 256 <= X <384 : forward
		256-384: straight
	If 384 <= X < 512 : left slow
	If X > 512 : left fast
		384-640: left
*/