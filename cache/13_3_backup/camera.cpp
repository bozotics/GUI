#include "camera.h"

void picam::process(){	
	vector<vector<Point> > contours;
	Rect boundRect;
	Mat kernel = Mat::ones(3, 3, CV_8UC1);
	int prevType = 1, prevValue = 0, showFrame = 0, showFrame2 = 3, trackType;

	int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

	//unsigned int end = 0, begin = 0;
	double ms = 0;
    double pixelDistanceX, pixelDistanceY;
    double predX, predY;
 
    unsigned int vtype = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, vtype);
 
    cv::Mat state(stateSize, 1, vtype);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, vtype);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
 
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);
 
    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, vtype);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;
 
    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
 
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

	bool found = false;
  	int notFoundCount = 0;

	auto begin = chrono::high_resolution_clock::now();

    while (!stopped) {
		//cout << val << " " << type<< endl;
		if(type != prevType || val != prevValue){
			prevType = type;
			prevValue = val;
			if(type == 0) stopped = true;
            if(type <= 24){
                tempThres[type-1] = val;
			    if((type-1)%6 <= 2 && (type-1)%6 >= 0){	
			    	if(val > tempThres[type+2]){
                        thres[type-1] = tempThres[type-1];
                        thres[type+2] = tempThres[type+2];
			        }   
                }
			    else if((type-1)%6 <=5 && (type-1)%6 >= 3){
			    	if(val < tempThres[type-4]){
                        thres[type-1] = tempThres[type-1];
                        thres[type-4] = tempThres[type-4];
			        }
                }
                for(int i = 0; i < 24; ++i) cout << thres[i]<< endl;
                cout << endl;
            }
            else if(type == 25){
                configVal[0] = val;
                Camera.setAWB_RB((float)configVal[0]/10.0, (float)configVal[1]/10.0);
            }
            else if(type == 26){
                configVal[1] = val;
                Camera.setAWB_RB((float)configVal[0]/10.0, (float)configVal[1]/10.0);
            }
            else if(type == 27) Camera.setExposureCompensation(val);
            else if(type == 28) Camera.setBrightness(val);
            else if(type == 29) Camera.setSaturation(val);
            else if(type == 30) Camera.setShutterSpeed(val);
            else if(type == 31) Camera.setISO(val);	
            else if(type == 32) showFrame = val;
			else if(type == 33) showFrame2 = val;
            else if(type == 34) trackType = val;
		}
		readFrame(processImages[0]);
		processImages[5] = processImages[0].clone();
		processImages[6] = processImages[0].clone();
		processImages[7] = processImages[0].clone();
        // Convert from BGR to HSV colorspace
        cvtColor(processImages[0], processImages[1], COLOR_BGR2HSV);
		//cvtColor(imageProcess, imageClr, COLOR_BGR2Lab);
		medianBlur(processImages[1], processImages[2], 3);
		inRange(processImages[2], Scalar(thres[3], thres[4], thres[5]), Scalar(thres[0], thres[1], thres[2]), processImages[8]);
		cvtColor(processImages[8], processImages[3], COLOR_GRAY2BGR);
		dilate(processImages[8], processImages[8], kernel, Point(-1, -1), 3);
		erode(processImages[8], processImages[8], kernel, Point(-1, -1), 3);

		cvtColor(processImages[8], processImages[4], COLOR_GRAY2BGR);
		findContours(processImages[8], contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
		if(contours.size() > 0){
			for (vector<Point> contour : contours){
    				drawContours(processImages[5], vector<vector<Point> >(1,contour), -1, (255, 0, 0), 1, 8);
  				}
			sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
    		return contourArea(c1, false) < contourArea(c2, false);
			});
			boundRect = boundingRect(contours[contours.size()-1]);
			rectangle(processImages[6], boundRect.tl(), boundRect.br(), Scalar(0, 255, 0), 2, 8, 0 );
		}

		auto end = chrono::high_resolution_clock::now();
		ms = ((float)chrono::duration_cast<std::chrono::milliseconds>(end-begin).count())/1000;
		if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = ms;
            kf.transitionMatrix.at<float>(9) = ms;
            // <<<< Matrix A
 
            //cout << "dT:" << endl << ms << endl;
 
            state = kf.predict();
            //cout << "State post:" << endl << state << endl;

            predX = abs(state.at<float>(0));
            predX = 2.74486 + (1.33896) * predX + (0.00618538) * pow(predX, 2) + (-0.00013955) * pow(predX, 3) + (0.00000069493) * pow(predX, 4) + (-0.0000000011522) * pow(predX, 5);
            if(state.at<float>(0) < 0) predX *= -1.0;
            predX += configVal[7];
            predY = abs(state.at<float>(1));
            predY = 2.74486 + (1.33896) * predY + (0.00618538) * pow(predY, 2) + (-0.00013955) * pow(predY, 3) + (0.00000069493) * pow(predY, 4) + (-0.0000000011522) * pow(predY, 5);
            if(state.at<float>(1) < 0) predY *= -1.0;
            predY += configVal[8];

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = predX - predRect.width / 2;
            predRect.y = predY - predRect.height / 2;
 
            cv::Point center;
            center.x = predX;
            center.y = predY;

            cv::circle(processImages[6], center, 2, CV_RGB(255,0,0), -1);
            cv::rectangle(processImages[6], predRect, CV_RGB(255,0,0), 2);
            //cout << state.at<float>(2) << " " << state.at<float>(3) << endl;
            predX = abs(state.at<float>(0) + state.at<float>(2) * 1);
            predX = 2.74486 + (1.33896) * predX + (0.00618538) * pow(predX, 2) + (-0.00013955) * pow(predX, 3) + (0.00000069493) * pow(predX, 4) + (-0.0000000011522) * pow(predX, 5);
            if(state.at<float>(0) + state.at<float>(2) * 0.1< 0) predX *= -1.0;
            predX += configVal[7];
            predY = abs(state.at<float>(1) + state.at<float>(3) * 1);
            predY = 2.74486 + (1.33896) * predY + (0.00618538) * pow(predY, 2) + (-0.00013955) * pow(predY, 3) + (0.00000069493) * pow(predY, 4) + (-0.0000000011522) * pow(predY, 5);
            if(state.at<float>(1) + state.at<float>(3) * 0.1< 0) predY *= -1.0;
           // cout << state.at<float>(0) << " " << state.at<float>(1) << " " << predX << " " << predY << endl;
            predY += configVal[8];

            cv::circle(processImages[6], Point(predX, predY), 2, CV_RGB(0,0,255), -1);
        }
		begin = chrono::high_resolution_clock::now();
		 if (contours.size() == 0)
        {
            notFoundCount++;
            //cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                kf.statePost = state;*/
        }
        else
        {
            notFoundCount = 0;
            
            pixelDistanceX = abs((boundRect.x + boundRect.width / 2) - configVal[7]);
            realDistanceX = -1.08267 + 1.21493 * pixelDistanceX + (-0.0402618) * pow(pixelDistanceX, 2) + (0.000964692) * pow(pixelDistanceX, 3) + (-0.00000913121) * pow(pixelDistanceX, 4) + (0.000000030743) * pow(pixelDistanceX, 5);
            if((boundRect.x + boundRect.width / 2) < configVal[7]) realDistanceX *= -1.0;
            pixelDistanceY = abs((boundRect.y + boundRect.height / 2) - configVal[8]);
            realDistanceY = -1.08267 + 1.21493 * pixelDistanceY + (-0.0402618) * pow(pixelDistanceY, 2) + (0.000964692) * pow(pixelDistanceY, 3) + (-0.00000913121) * pow(pixelDistanceY, 4) + (0.000000030743) * pow(pixelDistanceY, 5);
            if((boundRect.y + boundRect.height / 2) < configVal[8]) realDistanceY *= -1.0;
            meas.at<float>(0) = realDistanceX;
            meas.at<float>(1) = realDistanceY;
            meas.at<float>(2) = (float)boundRect.width;
            meas.at<float>(3) = (float)boundRect.height;

           // cout << meas.at<float>(0) << " " << meas.at<float>(1) << " " << realDistanceX << " " << pixelDistance << " " << ms << endl;

            if (!found) // First detection!
            {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px
 
                state.at<float>(0) = realDistanceX;
                state.at<float>(1) = realDistanceY;
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization
 
                kf.statePost = state;
                
                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction
 
            //cout << "Measure matrix:" << endl << meas << endl;
        }
        circle(processImages[0], Point(configVal[7], configVal[8]), 151, CV_RGB(0, 255, 0), 1);
        circle(processImages[0], Point(configVal[7], configVal[8]), 1, Scalar(0, 255, 0), 1);
        if(trackType == 5){
            writeShow(showMtx, processImages[showFrame], processImages[showFrame2], ref(showImage));
        }
		else writeShow(showMtx, processImages[showFrame], processImages[showFrame2], ref(showImage));
		//cout << ms << "\n";
	}
}

void picam::processGoal(){
    Mat frameGoal, maskGoalY, maskGoalB, maskField;
    vector<vector<Point> > contours;
	RotatedRect boundRect;
    Point2f points[12];
	Mat kernel = Mat::ones(3, 3, CV_8UC1);
    Mat bigKernel = Mat::ones(5, 5, CV_8UC1);
    while(!stopped){
        readFrame(frameGoal);
        cvtColor(frameGoal, frameGoal, COLOR_BGR2HSV);
	    medianBlur(frameGoal, frameGoal, 3);
	    inRange(frameGoal, Scalar(thres[9], thres[10], thres[11]), Scalar(thres[6], thres[7], thres[8]), maskGoalB);
        inRange(frameGoal, Scalar(thres[15], thres[16], thres[17]), Scalar(thres[12], thres[13], thres[14]), maskGoalY);
        inRange(frameGoal, Scalar(thres[21], thres[22], thres[23]), Scalar(thres[18], thres[19], thres[20]), maskField);
	    dilate(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
	    erode(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
        dilate(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
	    erode(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
        dilate(maskField, maskField, bigKernel, Point(-1, -1), 3);
	    erode(maskGoalB, maskField, bigKernel, Point(-1, -1), 3);
	    findContours(maskGoalB, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    if(contours.size() > 0){
	    	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        	return contourArea(c1, false) < contourArea(c2, false);
	    	});
	    	boundRect = minAreaRect(contours[contours.size()-1]);
            boundRect.points(points);
	    }
        findContours(maskGoalY, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    if(contours.size() > 0){
	    	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        	return contourArea(c1, false) < contourArea(c2, false);
	    	});
	    	boundRect = minAreaRect(contours[contours.size()-1]);
            boundRect.points(points+4);
	    }
        findContours(maskField, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    if(contours.size() > 0){
	    	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        	return contourArea(c1, false) < contourArea(c2, false);
	    	});
	    	boundRect = minAreaRect(contours[contours.size()-1]);
            boundRect.points(points+8);
	    }
        writePoints(points);
    }
}

void picam::streaming(){
	if (!Camera.open()){
		cerr << "Can't open camera\n";
		return;
	}
	db = new debug(showImage);
	thread thread_display(&debug::socketDisplay, db, ref(showMtx), ref(showImage), ref(stopped), ref(dualMode));
	thread thread_commands(&debug::socketCommands, db, ref(type), ref(val), ref(stopped));
	
	thread thread_process(&picam::process, this);

		auto begin = chrono::high_resolution_clock::now();
	while(!stopped){
		frameCnt++;

		Camera.grab();
		writeFrame();

        auto end = chrono::high_resolution_clock::now();
		auto ms = chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
		//cout << "time_real: " << ms  << "\n";
		//cout << "frame_real: " << frameCnt << "\n";
		//cout << "fps_processing: " << ((float)frameCnt/(float)ms*1000000.0) << "\n";	
	}	
	thread_process.join();
	thread_display.join();	
    thread_commands.join();
	close(db -> socket_fdesc);
	Camera.release();
}


void picam::writeFrame()
{
	unique_lock<shared_mutex> lock(mtx);
	Camera.retrieve(image);
}

void picam::readFrame(Mat &clone)
{
	shared_lock<shared_mutex> lock(mtx);
	resize(image, clone, Size(320, 240), 0, 0, INTER_NEAREST);
}

void picam::writeShow(shared_mutex &showMtx, Mat &image1, Mat &image2, Mat &show){
	unique_lock<shared_mutex> lock(showMtx);
	if (dualMode > 1) return;
	else if(dualMode == 0) show = image1.clone();
	else show = image2.clone();
	dualMode += 2;
}

void picam::writePoints(Point2f (&points)[12])
{
    unique_lock<shared_mutex> lock(goalMtx);
    for(int i = 0; i < 12; ++i) drawPoints[i] = points[i]; 
}

Mat picam::readPoints(Mat frame)
{
    shared_lock<shared_mutex> lock(goalMtx);
    for (int i = 0; i < 4; i++) line(frame, drawPoints[i], drawPoints[(i+1)%4], Scalar(255, 0, 0), 2);
    for (int i = 0; i < 4; i++) line(frame, drawPoints[i+4], drawPoints[(i+1)%4+4], Scalar(0, 255, 255), 2);
    for (int i = 0; i < 4; i++) line(frame, drawPoints[i+8], drawPoints[(i+1)%4+8], Scalar(0, 0, 255), 2);
    return frame;
}

void picam::readConfig(){
	ifstream cFile ("/home/alarm/project/cpp/camera2/config.txt");
    if (cFile.is_open())
    {
        string line;
        while(getline(cFile, line)){
            line.erase(remove_if(line.begin(), line.end(), [](unsigned char x){return isspace(x);}),
                                 line.end());
            if(line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
    		string name = line.substr(0, delimiterPos);
            string value = line.substr(delimiterPos + 1);
            cout << name << " " << value << '\n';
			if(name == "width") width = stoi(value);
			else if(name == "height") height = stoi(value);
			else if(name == "1-hi")	thres[0] = stoi(value);
			else if(name == "2-hi")	thres[1] = stoi(value);
			else if(name == "3-hi") thres[2] = stoi(value);
			else if(name == "1-lo") thres[3] = stoi(value);
			else if(name == "2-lo") thres[4] = stoi(value);
			else if(name == "3-lo") thres[5] = stoi(value);
            else if(name == "b1-hi") thres[6] = stoi(value);
			else if(name == "b2-hi") thres[7] = stoi(value);
			else if(name == "b3-hi") thres[8] = stoi(value);
			else if(name == "b1-lo") thres[9] = stoi(value);
			else if(name == "b2-lo") thres[10] = stoi(value);
			else if(name == "b3-lo") thres[11] = stoi(value);
			else if(name == "y1-hi") thres[12] = stoi(value);
			else if(name == "y2-hi") thres[13] = stoi(value);
			else if(name == "y3-hi") thres[14] = stoi(value);
			else if(name == "y1-lo") thres[15] = stoi(value);
			else if(name == "y2-lo") thres[16] = stoi(value);
			else if(name == "y3-lo") thres[17] = stoi(value);
			else if(name == "f1-hi") thres[18] = stoi(value);
			else if(name == "f2-hi") thres[19] = stoi(value);
			else if(name == "f3-hi") thres[20] = stoi(value);
			else if(name == "f1-lo") thres[21] = stoi(value);
			else if(name == "f2-lo") thres[22] = stoi(value);
			else if(name == "f3-lo") thres[23] = stoi(value);
			else if(name == "fps") fps = stoi(value);
            else if(name == "awb_r") configVal[0] = stoi(value);
			else if(name == "awb_b") configVal[1] = stoi(value);
			else if(name == "exposure") configVal[2] = stoi(value);
			else if(name == "brightness") configVal[3] = stoi(value);
			else if(name == "saturation") configVal[4] = stoi(value);
			else if(name == "ss") configVal[5] = stoi(value);
			else if(name == "ISO") configVal[6] = stoi(value);
            else if(name == "centre_x") configVal[7] = stoi(value);
            else if(name == "centre_y") configVal[8] = stoi(value);
			for(int i=0; i<24; i++) tempThres[i] = thres[i];
        }
    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
}

picam::picam()
{
	readConfig();
	Camera.setWidth(640);
	Camera.setHeight(480);
	Camera.setFrameRate(fps);
	Camera.setSensorMode(7);
	showImage = Mat::zeros(height, width, CV_8UC3);
	//if(exposure > -1) Camera.set(CAP_PROP_EXPOSURE, exposure);
    Camera.setAWB_RB((float)configVal[0]/10.0, (float)configVal[1]/10.0);	
	Camera.setBrightness(configVal[3]);	
	Camera.setSaturation(configVal[4]);	
	Camera.setAWB(raspicam::RASPICAM_AWB_OFF);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_OFF);
    Camera.setExposureCompensation(configVal[2]);
    Camera.setShutterSpeed(configVal[5]);
    Camera.setISO(configVal[6]);
	processImages[0] = Mat::zeros(height, width, CV_8UC3);
	image.create(480, 640, CV_8UC3);
}
//test2
