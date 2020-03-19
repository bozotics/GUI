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

void debug::socketDisplay(shared_mutex& mtx, Mat& image){
	while(1){	
		readFrame(ref(mtx), ref(image), imageToSend);
		imageToSend = imageToSend.reshape(0,1);	
		int image_size = imageToSend.total() * imageToSend.elemSize();
  		int num_bytes = send(socket_fdesc, imageToSend.data, image_size, 0);
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
	Camera.grab();
	unique_lock<shared_mutex> lock(mtx);
	Camera.retrieve(image);
}

void picam::readFrame(Mat &clone)
{
	shared_lock<shared_mutex> lock(mtx);
	clone = image.clone();
}

void debug::readFrame(shared_mutex &mtx, Mat &image, Mat &clone){
	shared_lock<shared_mutex> lock(mtx);
	clone = image.clone();
}

void picam::process(){	
	vector<vector<Point> > contours;
	Rect boundRect;
	Mat kernel = Mat::ones(3, 3, CV_8UC1);
cout << "processing\n";
    while (1) {
		auto begin = chrono::high_resolution_clock::now();
		readFrame(imageProcess);
        // Convert from BGR to HSV colorspace
        cvtColor(imageProcess, imageClr, COLOR_BGR2HSV);
		//cvtColor(imageProcess, imageClr, COLOR_BGR2Lab);
		GaussianBlur(imageClr, imageBlur, Size(5, 5), 0);
		inRange(imageBlur, Scalar(lo_1, lo_2, lo_3), Scalar(hi_1, hi_2, hi_3), imageRange);
		erode(imageRange, imageED, kernel, Point(-1, -1), 3);
		dilate(imageED, imageED, kernel, Point(-1, -1), 3);
		findContours(imageED, contours, RETR_CCOMP, CHAIN_APPROX_TC89_L1);
		//boundRect = boundingRect(Mat(contours[0]));
		auto end = chrono::high_resolution_clock::now();
		auto ms = chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
		cout << ms << "\n";
		///rectangle(, boundRect.tl(), boundRect.br(), Scalar(255, 0, 0), 2, 8, 0 );
	}
}

void picam::streaming(){

	db = new debug(image);
	thread thread_display(&debug::socketDisplay, db, ref(mtx), ref(image));
	cout << "open camera\n";
	if (!Camera.open()) cerr << "Can't open camera\n";
	cout << "opened camera\n";
	//auto begin = chrono::high_resolution_clock::now();
	thread thread_process(&picam::process, this);
	while(1){
		frameCnt++;
		writeFrame();
	}	
	thread_process.join();
	//auto end = chrono::high_resolution_clock::now();
	//auto ms = chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();

	//cout << "time_real: " << ms  << "\n";
	//cout << "frame_real: " << frame_count << "\n";
	//cout << "fps_processing: " << ((float)frame_count/(float)ms*1000.0) << "\n";	
	
	Camera.release();
}

void picam::readConfig(){
	ifstream cFile ("config.txt");
    if (cFile.is_open())
    {
        string line;
        while(getline(cFile, line)){
            line.erase(remove_if(line.begin(), line.end(), isspace),
                                 line.end());
            if(line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
    		const char *name = line.substr(0, delimiterPos);
            const char *value = line.substr(delimiterPos + 1);
            cout << name << " " << value << '\n';
			if(name == "width") width = stoi(value);
			else if(name == "height") height = stoi(value);
			else if(name == "hi_1")	hi_1 = stoi(value);
			else if(name == "hi_2")	hi_2 = stoi(value);
			else if(name == "hi_3") hi_3 = stoi(value);
			else if(name == "lo_1") lo_1 = stoi(value);
			else if(name == "lo_2") lo_2 = stoi(value);
			else if(name == "lo_3") lo_3 = stoi(value);
			else if(name == "exposure") exposure = stoi(value);
			else if(name == "fps") fps = stoi(value);
        }
    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
}

picam::picam()
{
	readConfig();
	Camera.set(CAP_PROP_FRAME_WIDTH, width);
	Camera.set(CAP_PROP_FRAME_HEIGHT, height);
	Camera.set(CAP_PROP_FPS, fps);
	if(exposure > -1) Camera.set(CAP_PROP_EXPOSURE, 30);	
	//Camera.set(CAP_PROP_BRIGHTNESS, 30);	
	//Camera.set(CAP_PROP_CONTRAST, 30);	
	//Camera.setAWB(7);
	image = Mat::zeros(height, width, CV_8UC3);
	imageProcess = Mat::zeros(height, height, CV_8UC3);		
	imageClr = Mat::zeros(height, height, CV_8UC3);
	imageBlur = Mat::zeros(height, height, CV_8UC3);
	imageRange = Mat::zeros(height, height, CV_8UC3);
	imageED = Mat::zeros(height, height, CV_8UC3);
	imageContours = Mat::zeros(height, height, CV_8UC3);
	processedImage = Mat::zeros(height, height, CV_8UC3);
}