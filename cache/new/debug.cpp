#include "camera.h"

void picam::debug()
{
    debugCheck = true;
    debugImages[0] = Mat::zeros(prop.height, prop.width, CV_8UC3);
    Mat imageToSend, temp;
    int imageSize, dualMode = 0;
    unsigned int frameNum = 0, localBallCnt = 0, localGoalCnt = 0, localFieldCnt = 0;
    int16_t conv;
    vector<vector<Point> > contours;
    Rect pBound, rBound;
    RotatedRect bBound, yBound, fBound;
    Point2f bBoundPoints[4], yBoundPoints[4], fBoundPoints[4];
	socketConnect();
	sendImageDims();
	thread threadCommands(&picam::receiveCommands, this);
	thread threadFrame(&picam::getFrame, this);
	//thread threadBall(&picam::processBall, this);
    //thread threadGoal(&picam::processGoal, this);
    //thread threadField(&picam::processField, this);
    while(!stopped.load(memory_order_relaxed))
    {
        if(readFrame(ref(frameNum), ref(debugImages[0])) != 0)
        {
            cvtColor(debugImages[0], debugImages[1], COLOR_BGR2HSV);
            debugImages[5] = debugImages[0].clone();
		    debugImages[6] = debugImages[0].clone();
		    debugImages[7] = debugImages[0].clone();
	        medianBlur(debugImages[1], debugImages[2], 3);
	        //read.lock();
	        if(trackType < 4)
			    //inRange(debugImages[2], Scalar(thres[trackType*6+3], thres[trackType*6+4], thres[trackType*6+5]), 	Scalar(thres[trackType*6+0], thres[trackType*6+1], thres[trackType*6+2]), debugImages[8]);
                inRangeHSV(trackType, debugImages[2], debugImages[8], temp);
			else //inRange(debugImages[2], Scalar(thres[3], thres[4], thres[5]), 	Scalar(thres[0], thres[1], thres[2]), debugImages[8]);
                inRangeHSV(0, debugImages[2], debugImages[8], temp);
	        //read.unlock();
			cvtColor(debugImages[8], debugImages[3], COLOR_GRAY2BGR);
			dilate(debugImages[8], debugImages[8], kernel, Point(-1, -1), 3);
			erode(debugImages[8], debugImages[8], kernel, Point(-1, -1), 3);
	        cvtColor(debugImages[8], debugImages[4], COLOR_GRAY2BGR);
			findContours(debugImages[8], contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		    if(contours.size() > 0)
		    {
		    	for (vector<Point> contour : contours){
    	    			drawContours(debugImages[5], vector<vector<Point> >(1,contour), -1, CV_RGB(0, 0, 0), 1, 8);
  		    		}
		    	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
    	    		return contourArea(c1, false) < contourArea(c2, false);
		    			});
		    }
            //read.lock();
            if(trackType == 0 || trackType == 4){
                if(readBall(localBallCnt, rBound, pBound) != 0)
                {
                    if(rBound.width != 0)
                    {
                        rBound.x = distanceUnmapper((double)rBound.x, prop.x);
                        rBound.y = distanceUnmapper((double)rBound.y, prop.y);
                        if((dualMode==0 && showFrame==6) || (dualMode==1 && showFrame2==6) || trackType == 4) 
                            rectangle(debugImages[(!dualMode) ? showFrame : showFrame2], rBound, CV_RGB(255,165,0), 2);
                    }
                    if(pBound.width != 0)
                    {
                        pBound.x = distanceUnmapper((double)pBound.x, prop.x);
                        pBound.y = distanceUnmapper((double)pBound.y, prop.y);
                        if((dualMode==0 && showFrame==6) || (dualMode==1 && showFrame2==6) || trackType == 4) 
                            rectangle(debugImages[(!dualMode) ? showFrame : showFrame2], pBound, CV_RGB(255,69,0), 2);
                    }
                }
            }
            if(trackType == 1 || trackType == 2 || trackType == 4){
                if(readGoal(localGoalCnt, bBound, yBound))
                {
                    if(bBound.size.width != 1000)
                    {
                        bBound.points(bBoundPoints);
                        if((dualMode==0 && showFrame==6) || (dualMode==1 && showFrame2==6) || trackType == 4)
                            for (int i = 0; i < 4; i++) 
                                line(debugImages[(!dualMode) ? showFrame : showFrame2], bBoundPoints[i], bBoundPoints[(i+1)%4], CV_RGB(0, 0, 255), 2);
                    }
                    if(yBound.size.width != 1000)
                    {
                        yBound.points(yBoundPoints);
                        if((dualMode==0 && showFrame==6) || (dualMode==1 && showFrame2==6) || trackType == 4)
                            for (int i = 0; i < 4; i++) 
                                line(debugImages[(!dualMode) ? showFrame : showFrame2], yBoundPoints[i], yBoundPoints[(i+1)%4], CV_RGB(255, 255, 0), 2);
                    }
                }   
            }
            if(trackType == 3 || trackType == 4){
                if(readField(localFieldCnt ,fBound))
                {
                    if(fBound.size.width != 1000)
                    {
                        fBound.points(fBoundPoints);
                        if((dualMode==0 && showFrame==6) || (dualMode==1 && showFrame2==6) || trackType == 4)
                            for (int i = 0; i < 4; i++) 
                                line(debugImages[(!dualMode) ? showFrame : showFrame2], fBoundPoints[i], fBoundPoints[(i+1)%4], CV_RGB(0, 255, 0), 2);
                    }
                }
                
            }
            imageToSend = debugImages[(!dualMode) ? showFrame : showFrame2].reshape(0,1);
            //read.unlock();
	        imageSize = imageToSend.total() * imageToSend.elemSize();
	        conv = htons(dualMode);
	        send(socketIdentity, (char*)&conv, sizeof(uint16_t), 0);
  	        send(socketIdentity, imageToSend.data, imageSize, 0);
            dualMode = (dualMode+1)%2;
        }
		else
		{
			usleep(1000);
		}
    }
	cout << "debug\n";
	threadCommands.join();
	threadFrame.join();
	//threadBall.join();
	//threadGoal.join();
	//threadField.join();
	close(socketIdentity);
	Camera.release();
}

void picam::receiveCommands()
{
    int type, val, prevType = 0, prevVal = 1;
	while(!stopped.load(memory_order_relaxed)){	
		recv(socketIdentity, (char*)&type, sizeof(uint16_t), 0);
		recv(socketIdentity, (char*)&val, sizeof(uint16_t), 0);
		type = ntohs(type);
		val = ntohs(val);
        if(type != prevType || val != prevVal)
        {
            read.lock();
			prevType = type;
			prevVal = val;
			if(type == 0) stopped.store(true, memory_order_relaxed);
            else if(type <= 24)
            {
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
            }
            else if(type == 25)
            {
                prop.configVal[0] = val;
                Camera.setAWB_RB((float)prop.configVal[0]/10.0, (float)prop.configVal[1]/10.0);
            }
            else if(type == 26)
            {
                prop.configVal[1] = val;
                Camera.setAWB_RB((float)prop.configVal[0]/10.0, (float)prop.configVal[1]/10.0);
            }
            else if(type == 27) Camera.setExposureCompensation(val);
            else if(type == 28) Camera.setBrightness(val);
            else if(type == 29) Camera.setSaturation(val);
            else if(type == 30) Camera.setShutterSpeed(val);
            else if(type == 31) Camera.setISO(val);	
            else if(type == 32) showFrame = val;
			else if(type == 33) showFrame2 = val;
            else if(type == 34) trackType = val;
            read.unlock();
		}	
	}
	cout << "commands\n";
}

void picam::socketConnect()
{
    struct addrinfo addrinfo_hints;
	struct addrinfo* addrinfo_resp;
    const char* hostname = "192.168.7.17";
	int port = 12345;
    // Specify criteria for address structs to be returned by getAddrinfo
  	memset(&addrinfo_hints, 0, sizeof(addrinfo_hints));
  	addrinfo_hints.ai_socktype = SOCK_STREAM;
  	addrinfo_hints.ai_family = AF_INET;

    // Populate addr_info_resp with address responses matching hints
  	if (getaddrinfo(hostname, to_string(port).c_str(),
        &addrinfo_hints, &addrinfo_resp) != 0) {
    	perror("Couldn't connect to host!");
    	exit(1);
  	}

    // Create socket file descriptor for server
  	socketIdentity = socket(addrinfo_resp->ai_family, addrinfo_resp->ai_socktype, addrinfo_resp->ai_protocol);
  	if (socketIdentity == -1) {
    	perror("Error opening socket");
    	exit(1);
  	}

    // Connect to server specified in address struct, assign process to server
    // file descriptor
  	if (connect(socketIdentity, addrinfo_resp->ai_addr,addrinfo_resp->ai_addrlen) == -1) {
    	perror("Error connecting to address");
    	exit(1);
  	}

  	free(addrinfo_resp);
}

void picam::sendImageDims()
{
    // Send number of rows to server
    if (send(socketIdentity, (char*)&prop.width, sizeof(prop.width), 0) == -1) {
        perror("Error sending rows");
        exit(1);
    }

    // Send number of cols to server
    if (send(socketIdentity, (char*)&prop.height, sizeof(prop.height), 0) == -1) {
       perror("Error sending cols");
       exit(1);
    }
}

