#include "camera.h"

void picam::run()
{
    debugCheck = false;
    thread threadBall(&picam::processBall, this);
    thread threadGoal(&picam::processGoal, this);
    thread treadField(&picam::processField, this);
	while(!stopped.load(memory_order_relaxed))
	{
		
	}
}

void picam::processBall()
{
    vector<vector<Point> > contours;

    const int stateSize = 6;
    const int measSize = 4;
    const int contrSize = 0;

	double ms = 0;
 
    unsigned int vtype = CV_32F;

    Point center;
    Rect predRect;

    KalmanFilter kf(stateSize, measSize, contrSize, vtype);
 
    Mat state(stateSize, 1, vtype);  // [x,y,v_x,v_y,w,h]
    Mat meas(measSize, 1, vtype); 

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);
 
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, vtype);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;
 
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

    Mat frameBall, maskBall, temp;
    unsigned int frameNum = 0;

    Rect boundRect;
    auto begin = chrono::high_resolution_clock::now();
    auto end = chrono::high_resolution_clock::now();

    coordinates pixelCoor;
    ///////////////////////////
    ///////////LOOP////////////
    ///////////////////////////
    while(!stopped.load(memory_order_relaxed))
    {
       begin = chrono::high_resolution_clock::now();
        if(readFrame(frameNum, ref(frameBall))) 
		{		
            cvtColor(frameBall, frameBall, COLOR_BGR2HSV);
            
            //medianBlur(frameBall, frameBall, 3);
            if(debugCheck) read.lock();
		    inRangeHSV(0, frameBall, maskBall, temp);
            if(debugCheck) read.unlock();
            
		    dilate(maskBall, maskBall, kernel, Point(-1, -1), 2);
		    erode(maskBall, maskBall, kernel, Point(-1, -1), 2);
            
		    findContours(maskBall, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
		    
		    if(contours.size() > 0){
		    
			    sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
    		    return contourArea(c1, false) < contourArea(c2, false);
			    });
			    boundRect = boundingRect(contours[contours.size()-1]);
		    }
            end = chrono::high_resolution_clock::now();
            ms = ((float)chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() + 14)/1000;
			
            if (found)
            {
                kf.transitionMatrix.at<float>(2) = ms;
                kf.transitionMatrix.at<float>(9) = ms;
                //cout << "dT:" << endl << ms << endl;
 
                state = kf.predict();
                //cout << "State post:" << endl << state << endl;

                predRect.width = state.at<float>(4);
                predRect.height = state.at<float>(5);
                predRect.x = state.at<float>(0);
                predRect.y = state.at<float>(1);
            }   
            else predRect.width = 0;
		    if(contours.size() == 0)
            {   
                notFoundCount++;
                boundRect.width = 0;
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

                pixelCoor.x = boundRect.x; // + boundRect.width / 2;
                boundRect.x = distanceMapper(pixelCoor.x, prop.x); 
                pixelCoor.y = boundRect.y; // + boundRect.height /2;
                boundRect.y = distanceMapper(pixelCoor.y, prop.y); 
                meas.at<float>(0) = boundRect.x;
                meas.at<float>(1) = boundRect.y;
                meas.at<float>(2) = (float)boundRect.width;
                meas.at<float>(3) = (float)boundRect.height;

                // cout << meas.at<float>(0) << " " << meas.at<float>(1) << " " << real.x << " " << pixelDistance << " " << ms << endl;

                if (!found) // First detection!
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(7) = 1; // px
                    kf.errorCovPre.at<float>(14) = 1;
                    kf.errorCovPre.at<float>(21) = 1;
                    kf.errorCovPre.at<float>(28) = 1; // px
                    kf.errorCovPre.at<float>(35) = 1; // px
    
                    state.at<float>(0) = boundRect.x;
                    state.at<float>(1) = boundRect.y;
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
            if(found) writeBall(boundRect, predRect);
            end = chrono::high_resolution_clock::now();
			//cout << "ballTime: " << ((float)chrono::duration_cast<std::chrono::microseconds>(end-begin).count()) << "\n";
        }
		else usleep(500);
    }
	cout << "ball\n";
}

void picam::processGoal()
{
    Mat frameGoal, maskGoalY, maskGoalB, temp;
    unsigned int frameNum = 0;
    vector<vector<Point> > bContours;
    vector<vector<Point> > yContours;
	RotatedRect yRect, bRect;
    RotatedRect empty(Point2f(500, 500), Size2f(1000, 1000), 0);
    while(!stopped.load(memory_order_relaxed)){
        auto begin = chrono::high_resolution_clock::now();
        if(readFrame(frameNum, ref(frameGoal)))
        {
        	cvtColor(frameGoal, frameGoal, COLOR_BGR2HSV);
	    	medianBlur(frameGoal, frameGoal, 3);
        	if(debugCheck) read.lock();
            inRangeHSV(1, frameGoal, maskGoalB, temp);
            inRangeHSV(2, frameGoal, maskGoalY, temp);
	    	//inRange(frameGoal, Scalar(thres[9], thres[10], thres[11]), Scalar(thres[6], thres[7], thres[8]), maskGoalB);
        	//inRange(frameGoal, Scalar(thres[15], thres[16], thres[17]), Scalar(thres[12], thres[13], thres[14]), maskGoalY);
	    	if(debugCheck) read.unlock();
        	dilate(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
	    	erode(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
        	dilate(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
	    	erode(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
	    	findContours(maskGoalB, bContours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    	if(bContours.size() > 0)
	    	{
	    		sort(bContours.begin(), bContours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        			return contourArea(c1, false) < contourArea(c2, false);
	    				});
	    		bRect = minAreaRect(bContours[bContours.size()-1]);
	    	}
        	else bRect = empty;
        	findContours(maskGoalY, yContours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    	if(yContours.size() > 0)
	    	{
	    		sort(yContours.begin(), yContours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        			return contourArea(c1, false) < contourArea(c2, false);
	    				});
	    		yRect = minAreaRect(yContours[yContours.size()-1]);
	    	}
        	else yRect = empty;
        	if(yContours.size() > 0 || bContours.size() > 0) writeGoal(bRect, yRect);

            auto end = chrono::high_resolution_clock::now();
            //cout << "goalTime: " << ((float)chrono::duration_cast<std::chrono::microseconds>(end-begin).count()) << "\n";
		}
		else usleep(1000);
    }
    cout << "goal\n";
}

void picam::processField()
{
    Mat frameField, maskField, temp;
    unsigned int frameNum = 0;
    vector<vector<Point> > contours;
	RotatedRect fRect;
    Mat bigKernel = Mat::ones(5, 5, CV_8UC1);
    RotatedRect empty(Point2f(500, 500), Size2f(1000, 1000), 0);
    while(!stopped.load(memory_order_relaxed)){
        auto begin = chrono::high_resolution_clock::now();
        if(readFrame(frameNum, ref(frameField)))
        {
            cvtColor(frameField, frameField, COLOR_BGR2HSV);
	        medianBlur(frameField, frameField, 3);
            if(debugCheck) read.lock();
            inRangeHSV(3, frameField, maskField, temp);
            //inRange(frameField, Scalar(thres[21], thres[22], thres[23]), Scalar(thres[18], thres[19], thres[20]), maskField);
	        if(debugCheck) read.unlock();
            dilate(maskField, maskField, kernel, Point(-1, -1), 3);
	        erode(maskField, maskField, kernel, Point(-1, -1), 3);
	        findContours(maskField, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	        if(contours.size() > 0){
	        	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
            	return contourArea(c1, false) < contourArea(c2, false);
	        	});
	        	fRect = minAreaRect(contours[contours.size()-1]);
	        }
            else fRect = empty;
            if(contours.size() > 0) writeField(fRect);
            auto end = chrono::high_resolution_clock::now();
            //cout << "fieldTime: " << ((float)chrono::duration_cast<std::chrono::microseconds>(end-begin).count()) << "\n";
        }
        else usleep(1000);
    }
    cout << "field\n";
}

double picam::distanceMapper(double pixel, int &centre)
{
	pixel -= centre;
    if (pixel > 0) return (-1.08267 + 1.21493 * pixel + (-0.0402618) * pow(pixel, 2) + (0.000964692) * pow(pixel, 3) + (-0.00000913121) * pow(pixel, 4) + (0.000000030743) * pow(pixel, 5));
    else return (-1.08267 + 1.21493 * abs(pixel) + (-0.0402618) * pow(abs(pixel), 2) + (0.000964692) * pow(abs(pixel), 3) + (-0.00000913121) * pow(abs(pixel), 4) + (0.000000030743) * pow(abs(pixel), 5)) * (-1.0);
}

double picam::distanceUnmapper(double const &dist, int &centre)
{
    if (dist > 0) return (2.74486 + (1.33896) * dist + (0.00618538) * pow(dist, 2) + (-0.00013955) * pow(dist, 3) + (0.00000069493) * pow(dist, 4) + (-0.0000000011522) * pow(dist, 5)) + centre;
    else return (2.74486 + (1.33896) * abs(dist) + (0.00618538) * pow(abs(dist), 2) + (-0.00013955) * pow(abs(dist), 3) + (0.00000069493) * pow(abs(dist), 4) + (-0.0000000011522) * pow(abs(dist), 5)) * (-1.0) + centre;
}

void picam::getFrame()
{
float cnt = 0;
    if(debugCheck){
    
        while(!stopped.load(memory_order_relaxed))
        {
        auto begin = chrono::high_resolution_clock::now();
        cnt+=1;
            
            
	    	Camera.grab();
	    	
            
	    	writeFrame();
            auto end = chrono::high_resolution_clock::now();
            cout << "readTime: " << ((float)chrono::duration_cast<std::chrono::microseconds>(end-begin).count()) << "\n";
        }
       
	cout << "getFrame\n";
    }
    else{
        while(!stopped.load(memory_order_relaxed))
        {
	    	Camera.grab();
	    	writeFrame();
        }
    }
    
}

unsigned int picam::readFrame(unsigned int &frameNum, Mat &frame)
{
    shared_lock<shared_mutex> lock(imgMtx);
    if(imgCnt!=frameNum)
    {
    resize(image[bufferPosition], image2, Size(320, 240), 0, 0, INTER_NEAREST);
        frame = image2.clone();
        frameNum = imgCnt;
        return 1;
    }
    else return 0;
}

void picam::writeFrame()
{
	bufferPosition = Camera.retrieve();
	
    imgCnt++;
}

void picam::writeBall(Rect real, Rect pred)
{
    unique_lock<shared_mutex> lock(ballMtx);
    ++ballCnt;
    rBall = real;
    pBall = pred;
}

unsigned int picam::readBall(unsigned int &frameNum, Rect &realClone, Rect &predClone)
{
    shared_lock<shared_mutex> lock(ballMtx);
    if(ballCnt!=frameNum)
    {
        realClone = rBall;
        predClone = pBall;
		frameNum = ballCnt;
        return 1;
    }
    else return 0;
}

void picam::writeGoal(RotatedRect bRect, RotatedRect yRect)
{
    unique_lock<shared_mutex> lock(goalMtx);
    ++goalCnt;
    bGoal = bRect;
    yGoal = yRect;
}

unsigned int picam::readGoal(unsigned int &frameNum, RotatedRect &bGoalClone, RotatedRect &yGoalClone)
{
    shared_lock<shared_mutex> lock(goalMtx);
    if(goalCnt!=frameNum)
    {
        bGoalClone = bGoal;
        yGoalClone = yGoal;
		frameNum = goalCnt;
        return 1;
    }
    else return 0;
}

void picam::writeField(RotatedRect fRect)
{
    unique_lock<shared_mutex> lock(fieldMtx);
    ++fieldCnt;
    field = fRect;
}

unsigned int picam::readField(unsigned int &frameNum, RotatedRect &fieldClone)
{
    shared_lock<shared_mutex> lock(fieldMtx);
    if(fieldCnt!=frameNum)
    {
        fieldClone = field;
		frameNum = fieldCnt;
        return 1;
    }
    else return 0;
}

void picam::inRangeHSV(int type, Mat &input, Mat &output, Mat &temp)
{
    if(thres[type*6] <= 180)
        inRange(input, Scalar(thres[type*6+3], thres[type*6+4], thres[type*6+5]), Scalar(thres[type*6], thres[type*6+1], thres[type*6+2]), output);
    else
    {
        inRange(input, Scalar(thres[type*6+3], thres[type*6+4], thres[type*6+5]), Scalar(180, thres[type*6+1], thres[type*6+2]), temp);
        inRange(input, Scalar(0, thres[type*6+4], thres[type*6+5]), Scalar(thres[type*6] - 180, thres[type*6+1], thres[type*6+2]), output);
        bitwise_or(temp, output, output);
    }
}

void picam::readConfig()
{
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
			if(name == "width") prop.width = stoi(value);
			else if(name == "height") prop.height = stoi(value);
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
			else if(name == "fps") prop.fps = stoi(value);
            else if(name == "awb_r") prop.configVal[0] = stoi(value);
			else if(name == "awb_b") prop.configVal[1] = stoi(value);
			else if(name == "exposure") prop.configVal[2] = stoi(value);
			else if(name == "brightness") prop.configVal[3] = stoi(value);
			else if(name == "saturation") prop.configVal[4] = stoi(value);
			else if(name == "ss") prop.configVal[5] = stoi(value);
			else if(name == "ISO") prop.configVal[6] = stoi(value);
            else if(name == "centre_x") prop.x = stoi(value);
            else if(name == "centre_y") prop.y = stoi(value);
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
	Camera.setSensorMode(7);
    Camera.setFrameRate(prop.fps);
    Camera.setAWB_RB((float)prop.configVal[0]/10.0, (float)prop.configVal[1]/10.0);	
	Camera.setBrightness(prop.configVal[3]);	
	Camera.setSaturation(prop.configVal[4]);	
	Camera.setAWB(raspicam::RASPICAM_AWB_OFF);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_OFF);
    Camera.setExposureCompensation(prop.configVal[2]);
    Camera.setShutterSpeed(prop.configVal[5]);
    Camera.setISO(prop.configVal[6]);
	//image.create(480, 640, CV_8UC3); //important else program will crash
	image2.create(240, 320, CV_8UC3);
}
