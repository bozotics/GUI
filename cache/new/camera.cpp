#include "camera.h"

void picam::run()
{
    debugCheck = false;
    thread threadBall(&picam::processBall, this);
    thread threadGoal(&picam::processGoal, this);
    thread treadField(&picam::processField, this);
	while(!stopped.load(memory_order_acq_rel))
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

    Mat frameBall, maskBall, tempMat;
    unsigned int tempInt;

    Rect boundRect;
    auto begin = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    coordinates pixelCoor;
    ///////////////////////////
    ///////////LOOP////////////
    ///////////////////////////
    while(!stopped.load(memory_order_acq_rel))
    {
        if(ballReq.load(memory_order_acq_rel) > 0) waitForBall();
        ballReq.store(prop.PrioSkip[ballPriority.load(memory_order_acq_rel)-1], memory_order_acq_rel);
        cvtColor(image[bufferPosition.load(memory_order_acq_rel);], frameBall, COLOR_BGR2HSV);
        //medianBlur(frameBall, frameBall, 3);
		inRangeHSV(0, frameBall, maskBall, tempMat);
		dilate(maskBall, maskBall, kernel, Point(-1, -1), 2);
		erode(maskBall, maskBall, kernel, Point(-1, -1), 2);
		findContours(maskBall, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
		if(contours.size() > 0){
		    sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
    	    return contourArea(c1, false) < contourArea(c2, false);
		    });
		    boundRect = boundingRect(contours[contours.size()-1]);
		}
		
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
		if(contours.size() == 0)
        {   
            notFoundCount++;
            boundRect.width = 0;
            //cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
                predRect.width = 0;
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
        tempInt = (ballCnt.load(memory_order_acq_rel)+1)%3;
        rBall[tempInt] = boundRect;
        pBall[tempInt] = predRect;
        ballCnt.fetch_add(1, memory_order_acq_rel);
    }
    auto end = chrono::steady_clock::now();
    cout << "ball FPS: " << ((float)ballCnt/chrono::duration<double, milli>(end-begin).count()) << "\n";
}

void picam::processGoal()
{
    Mat frameGoal, maskGoalY, maskGoalB, tempMat;
    unsigned int tempInt;
    vector<vector<Point> > bContours;
    vector<vector<Point> > yContours;
    RotatedRect empty(Point2f(500, 500), Size2f(1000, 1000), 0);
    auto begin = chrono::steady_clock::now();
    unique_lock<shared_mutex> lock(goalMtx);
    while(!stopped.load(memory_order_acq_rel)){
        if(goalReq.load(memory_order_acq_rel) > 0) waitForGoal();
        goalReq.store(prop.PrioSkip[goalPriority.load(memory_order_acq_rel)-1], memory_order_acq_rel);
        cvtColor(image[bufferPosition.load(memory_order_acq_rel)], frameGoal, COLOR_BGR2HSV);
	    //medianBlur(frameGoal, frameGoal, 3);
        inRangeHSV(1, frameGoal, maskGoalB, tempMat);
        inRangeHSV(2, frameGoal, maskGoalY, tempMat);
        dilate(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
	    erode(maskGoalB, maskGoalB, kernel, Point(-1, -1), 3);
        dilate(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
	    erode(maskGoalY, maskGoalY, kernel, Point(-1, -1), 3);
	    findContours(maskGoalB, bContours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        tempInt = (goalCnt.load(memory_order_acq_rel) + 1)%3;
	    if(bContours.size() > 0)
	    {
	    	sort(bContours.begin(), bContours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        		return contourArea(c1, false) < contourArea(c2, false);
	    			});
	    	bGoal[tempInt] = minAreaRect(bContours[bContours.size()-1]);
	    }
        else bGoal[tempInt] = empty;
        findContours(maskGoalY, yContours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	    if(yContours.size() > 0)
	    {
	    	sort(yContours.begin(), yContours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        		return contourArea(c1, false) < contourArea(c2, false);
	    			});
	    	yGoal[tempInt] = minAreaRect(yContours[yContours.size()-1]);
	    }
        else yGoal[tempInt] = empty;
        ballCnt.fetch_add(1, memory_order_acq_rel);
    }
    auto end = chrono::steady_clock::now();
    cout << "goal FPS: " << ((float)goalCnt/chrono::duration<double, milli>(end-begin).count()) << "\n";
}

void picam::processField()
{
    Mat frameField, maskField, tempMat;
    unsigned int tempInt;
    vector<vector<Point> > contours;
    Mat bigKernel = Mat::ones(5, 5, CV_8UC1);
    RotatedRect empty(Point2f(500, 500), Size2f(1000, 1000), 0);
    auto begin = chrono::steady_clock::now();
    while(!stopped.load(memory_order_acq_rel)){
        if(fieldReq.load(memory_order_acq_rel) > 0) waitForField();
        fieldReq.store(prop.PrioSkip[fieldPriority.load(memory_order_acq_rel)-1], memory_order_acq_rel);
        cvtColor(image[bufferPosition.load(memory_order_acq_rel)], frameField, COLOR_BGR2HSV);
	    //medianBlur(frameField, frameField, 3);
        inRangeHSV(3, frameField, maskField, tempMat);
        dilate(maskField, maskField, kernel, Point(-1, -1), 3);
	    erode(maskField, maskField, kernel, Point(-1, -1), 3);
	    findContours(maskField, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        tempInt = (fieldCnt.load(memory_order_acq_rel) + 1)%3;
	    if(contours.size() > 0){
	    	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        	return contourArea(c1, false) < contourArea(c2, false);
	    	});
	    	field[tempInt] = minAreaRect(contours[contours.size()-1]);
	    }
        else field[tempInt] = empty;
        fieldCnt.fetch_add(1, memory_order_acq_rel);
    }
    auto end = chrono::steady_clock::now();
    cout << "field FPS: " << ((float)fieldCnt/chrono::duration_cast<double, milli>(end-begin).count()) << "\n";
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
    auto begin = chrono::steady_clock::now();
    while(!stopped.load(memory_order_acq_rel))
    {
        imgCnt++;
		Camera.grab();
		bufferPosition.store(Camera.retrieve(), memory_order_acq_rel);
        ballReq.fetch_sub(1, memory_order_seq_cst)
        goalReq.fetch_sub(1, memory_order_seq_cst);
        fieldReq.fetch_sub(1, memory_order_seq_cst);
        if(ballReq.load(memory_order_acq_rel) <= 0) ballCond.notify_all();
        if(goalReq.load(memory_order_acq_rel) <= 0) goalCond.notify_all();
        if(fieldReq.load(memory_order_acq_rel) <= 0) fieldCond.notify_all();
    }
    auto end = chrono::steady_clock::now();
    cout << "Frame FPS: " << ((float)imgCnt/chrono::duration<double, milli>(end-begin).count()) << "\n";
}

void picam::waitForBall()
{
    unique_lock<shared_mutex> lock(ballMtx);
    ballCond.wait(lock);
}

void picam::waitForGoal()
{
    unique_lock<shared_mutex> lock(goalMtx);
    goalCond.wait(lock);
}

void picam::waitForField()
{
    unique_lock<shared_mutex> lock(fieldMtx);
    fieldCond.wait(lock);
}

/*void picam::writeBall(Rect real, Rect pred)
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
}*/

void picam::inRangeHSV(int type, Mat &input, Mat &output, Mat &temp)
{
    if(debugCheck) read.lock();
    if(thres[type*6] <= 180)
        inRange(input, Scalar(thres[type*6+3], thres[type*6+4], thres[type*6+5]), Scalar(thres[type*6], thres[type*6+1], thres[type*6+2]), output);
    else
    {
        inRange(input, Scalar(thres[type*6+3], thres[type*6+4], thres[type*6+5]), Scalar(180, thres[type*6+1], thres[type*6+2]), temp);
        inRange(input, Scalar(0, thres[type*6+4], thres[type*6+5]), Scalar(thres[type*6] - 180, thres[type*6+1], thres[type*6+2]), output);
        bitwise_or(temp, output, output);
    }
    if(debugCheck) read.unlock();
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
            else if(name == "P1Skip") prop.PrioSkip[0] = stoi(value);
            else if(name == "P2Skip") prop.PrioSkip[1] = stoi(value);
            else if(name == "P3Skip") prop.PrioSkip[2] = stoi(value);
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
