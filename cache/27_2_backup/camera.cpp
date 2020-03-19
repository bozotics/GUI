#include "camera.h"

int debug::socket_connect(){
	hostname = "192.168.7.17";
	port = 12345;
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
  	socket_fdesc = socket(addrinfo_resp->ai_family, addrinfo_resp->ai_socktype, addrinfo_resp->ai_protocol);
  	if (socket_fdesc == -1) {
    	perror("Error opening socket");
    	exit(1);
  	}

  // Connect to server specified in address struct, assign process to server
  // file descriptor
  	if (connect(socket_fdesc, addrinfo_resp->ai_addr,addrinfo_resp->ai_addrlen) == -1) {
    	perror("Error connecting to address");
    	exit(1);
  	}

  	free(addrinfo_resp);
	return socket_fdesc;
}

void debug::sendImageDims(int dest, int cols, int rows) {
  // Send number of rows to server
  if (send(socket_fdesc, (char*)&cols, sizeof(cols), 0) == -1) {
    perror("Error sending rows");
    exit(1);
  }

  // Send number of cols to server
  if (send(socket_fdesc, (char*)&rows, sizeof(rows), 0) == -1) {
    perror("Error sending cols");
    exit(1);
  }
}

debug::debug(Mat& image){
	socket_fdesc = socket_connect();
	sendImageDims(socket_fdesc, image.cols, image.rows);
}

void debug::socketDisplay(shared_mutex& mtx, Mat& image, bool &stopped){
	while(!stopped){	
		readShow(ref(mtx), ref(image), imageToSend);
		imageToSend = imageToSend.reshape(0,1);	
		int image_size = imageToSend.total() * imageToSend.elemSize();
  		int num_bytes = send(socket_fdesc, imageToSend.data, image_size, 0);
	}
}

void debug::socketCommands(int &type, int &val, bool &stopped){
	int temp1 = 0, temp2 = 0;
	while(!stopped){	
		usleep(10000);
		recv(socket_fdesc, (char*)&temp1, sizeof(uint16_t), 0);
		recv(socket_fdesc, (char*)&temp2, sizeof(uint16_t), 0);
		type = ntohs(temp1);
		val = ntohs(temp2);
		cout << type << " " << val << endl;
	}
}


void debug::display(shared_mutex& mtx, Mat& image){

	namedWindow("Frame", WINDOW_AUTOSIZE);

	auto begin = chrono::high_resolution_clock::now();

	while(1){
		outputCnt++;	
		if (waitKey(30) >= 0){
			break;
		}
	}
	auto end = chrono::high_resolution_clock::now();
	auto ms = chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
	cout << "fps_processing: " << ((float)outputCnt/(float)ms*1000.0) << "\n";

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

void debug::readShow(shared_mutex &showMtx, Mat &image, Mat &clone){
	shared_lock<shared_mutex> lock(showMtx);
	clone = image.clone();
}

void picam::writeShow(shared_mutex &showMtx, Mat &image, Mat &show){
	unique_lock<shared_mutex> lock(showMtx);
	show = image.clone();
}

void picam::process(){	
	vector<vector<Point> > contours;
	Rect boundRect;
	Mat kernel = Mat::ones(3, 3, CV_8UC1);
	int prevType = 1, prevValue = 0, showFrame = 0;
	float range0_[] = {0, 180}, range1_[] = {0, 255}, range2_[] = {0, 255};
    const float* range[] = {range0_, range1_, range2_};
    Mat roi_hist;
    int histSize[] = {180, 255, 255};
    int channels[] = {0, 1, 2};
	bool camshift = false;
	RotatedRect trackBox;
	Rect track_window(0, 0, width, height);
    while (!stopped) {
		//cout << val << " " << type<< endl;
		if(type != prevType || val != prevValue){
			prevType = type;
			prevValue = val;
			if(type == 0) stopped = true;
			else if(type <= 3 && type >= 1){	
				if(val > thres[type+2]) thres[type-1] = val;
			}
			else if(type <=6 && type >= 4){
				if(val < thres[type-4]) thres[type-1] = val;
			}
			else if(type == 7){
				showFrame = val;
				if(val == 7) camshift = true;
			}
		}
		auto begin = chrono::high_resolution_clock::now();
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
		/*if(!camshift){
			Mat bounding = Mat::zeros(height, width, CV_8UC1);
			rectangle(bounding, boundRect.tl() + Point(-10, -10), boundRect.br() + Point(10, 10), 255, -1);
			calcHist(&processImages[2], 1, channels, bounding, roi_hist, 3, histSize, range, true, false);
			normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);
			Rect track_window(boundRect.tl() + Point(-20, -20), boundRect.br() + Point(20, 20));
		}
		else{
			Mat dst;
			calcBackProject(&processImages[2], 1, channels, roi_hist, dst, range);
			//dst &= processImages[4];
    		trackBox = CamShift(dst, track_window,
                    	cv::TermCriteria( TermCriteria::EPS | TermCriteria::MAX_ITER, 10, 1 ));
			Point2f points[4];
        	trackBox.points(points);
        	for (int i = 0; i < 4; i++)
            	line(processImages[7], points[i], points[(i+1)%4], 255, 2);
			//processImages[7] = dst;
			//cvtColor(processImages[7], processImages[7], COLOR_GRAY2BGR);

		}*/
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
		auto ms = chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
		writeShow(showMtx, processImages[showFrame], showImage);
		//cout << ms << "\n";
	}
}

void picam::streaming(){
	if (!Camera.open()){
		cerr << "Can't open camera\n";
		return;
	}
	db = new debug(showImage);
	thread thread_display(&debug::socketDisplay, db, ref(showMtx), ref(showImage), ref(stopped));
	thread thread_commands(&debug::socketCommands, db, ref(type), ref(val), ref(stopped));
	
	thread thread_process(&picam::process, this);
	while(!stopped){
		frameCnt++;
		Camera.grab(); //new
		auto begin = chrono::high_resolution_clock::now();
		writeFrame();
		auto end = chrono::high_resolution_clock::now();
		auto ms = chrono::duration_cast<std::chrono::microseconds>(end-begin).count();

		//cout << "time_real: " << ms  << "\n";
		//cout << "frame_real: " << frameCnt << "\n";
		//cout << "fps_processing: " << ((float)frameCnt/(float)ms*1000.0) << "\n";	
	}	
	thread_process.join();
	thread_commands.join();
	close(db -> socket_fdesc);
	Camera.release();
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
			else if(name == "exposure") exposure = stoi(value);
			else if(name == "fps") fps = stoi(value);
			//for(int i=0; i<6; i++) cout << thres[i] << " ";
			//cout << endl;
        }
    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
	for(int i=0; i<6; i++) cout << thres[i] << " ";
	cout << endl;
}

picam::picam()
{
	readConfig();
	Camera.set(CAP_PROP_FRAME_WIDTH, 800);
	Camera.set(CAP_PROP_FRAME_HEIGHT, 608);
	Camera.set(CAP_PROP_FPS, fps);
	Camera.set(CAP_PROP_MODE, 4);
	cout << Camera.get(CAP_PROP_FRAME_HEIGHT) << " " << Camera.get(CAP_PROP_FRAME_WIDTH) << "\n";
	showImage = Mat::zeros(height, width, CV_8UC3);
	//if(exposure > -1) Camera.set(CAP_PROP_EXPOSURE, exposure);	
	//Camera.set(CAP_PROP_BRIGHTNESS, 30);	
	//Camera.set(CAP_PROP_CONTRAST, 30);	
	//Camera.setAWB(7);
	processImages[0] = Mat::zeros(height, width, CV_8UC3);
	image = Mat::zeros(height, width, CV_8UC3);
}
//test2