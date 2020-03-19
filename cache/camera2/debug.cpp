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

void debug::socketDisplay(shared_mutex& mtx, Mat& image, bool &stopped, int &dualMode){
	int image_size, num_bytes, current = -1;
	int16_t conv;
	while(!stopped){	
		current = readShow(ref(mtx), ref(image), ref(imageToSend), ref(dualMode));
		if (current < 0) continue;
		imageToSend = imageToSend.reshape(0,1);
		image_size = imageToSend.total() * imageToSend.elemSize();
		conv = htons(current);
		send(socket_fdesc, (char*)&conv, sizeof(uint16_t), 0);
  		num_bytes = send(socket_fdesc, imageToSend.data, image_size, 0);
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
		if(!type) stopped = true;
		//cout << type << " " << val << endl;
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

int debug::readShow(shared_mutex &showMtx, Mat &image, Mat &clone, int &dualMode){
	unique_lock<shared_mutex> lock(showMtx);
	if(dualMode < 2) return -1;
	else{
		clone = image.clone();
		++dualMode;
		dualMode %= 2;
		return (dualMode + 1)%2;
	}
}
