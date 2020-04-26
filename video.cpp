#include<mainwindow.h>

videoStream::videoStream(QObject *parent) : QObject(parent)
{
	clrValue = Mat::zeros(Size(480, 640), CV_8UC3);
}

sshComm::sshComm(QObject * parent) :QObject(parent)
{
	session = ssh_new();
	int vbs = SSH_LOG_RARE;
	ssh_options_set(session, SSH_OPTIONS_HOST, "alarm@192.168.7.18");
	ssh_options_set(session, SSH_OPTIONS_PORT_STR, "22");
   	//ssh_options_set(session, SSH_OPTIONS_LOG_VERBOSITY, &vbs);
    ssh_connect(session);
	ssh_userauth_publickey_auto(session, NULL, NULL);
	channel = ssh_channel_new(session);
}

void sshComm::startCamera()
{
	ssh_channel_open_session(channel);
	ssh_channel_request_pty(channel);
  	ssh_channel_change_pty_size(channel, 80, 100);
 	ssh_channel_request_shell(channel);

	int nbytes, written, makeFirst = 0;
	char bufferRead[256] = {'0'};
	char cmake[] = "[100%] Built";
	//char bufferWrite[] = "\n";

	while (ssh_channel_is_open(channel) && !ssh_channel_is_eof(channel) && !stopped){
  		nbytes = ssh_channel_read_nonblocking(channel, bufferRead, sizeof(bufferRead), 0);
    	if (nbytes > 0) written = write(1, bufferRead, nbytes);
		else if (nbytes == 0){
			if(cnt == 0){
				char bufferWrite[] = "scp /home/alarm/project/cpp/camera4/config.txt justin@192.168.7.17:/home/justin/programs/cpp/GUI/cache/\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			else if(cnt == 1){
				char bufferWrite[] = "scp /home/alarm/project/cpp/camera4/main.cpp justin@192.168.7.17:/home/justin/programs/cpp/GUI/cache/current\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			else if(cnt == 2){
				char bufferWrite[] = "scp /home/alarm/project/cpp/camera4/camera.h justin@192.168.7.17:/home/justin/programs/cpp/GUI/cache/current\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			else if(cnt == 3){
				char bufferWrite[] = "scp /home/alarm/project/cpp/camera4/camera.cpp justin@192.168.7.17:/home/justin/programs/cpp/GUI/cache/current\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			else if(cnt == 4){
				char bufferWrite[] = "scp /home/alarm/project/cpp/camera4/debug.cpp justin@192.168.7.17:/home/justin/programs/cpp/GUI/cache/current\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			else if(cnt == 5){
				usleep(100000);
				char bufferWrite[] = "sudo /home/alarm/project/cpp/camera4/camera -\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				++cnt;
			}
			usleep(50000);
		}
		if (make){
			if(makeFirst == 0){
				char bufferWrite[] = "make -j4 -C /home/alarm/project/cpp/camera4/\n";
				nbytes = ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
				makeFirst = 1;
			}
			for(int j = 0; j < 12; j++){
				if(cmake[j] != bufferRead[j]) break;
					if(j == 11){
						make = false;
						int input;
    					input = system("gnome-terminal -e '/home/justin/programs/cpp/relaunch'");
						emit afterMake();
					}
				}
			}
			usleep(50000);
		}
	char bufferWrite[] = "exit\n";
	ssh_channel_write(channel, bufferWrite, sizeof(bufferWrite));
	ssh_channel_close(channel);
  	ssh_channel_send_eof(channel);
  	ssh_channel_free(channel);
}


void videoStream::startVideo()
{
	if(mode==1){	
		Mat frame;
		int screen = 0;
		if(stopped){
			listen(server_ptr -> sock_init, 1);
			server_ptr -> sock_connect = accept(server_ptr -> sock_init, (struct sockaddr*) &(server_ptr -> client_addr), &(server_ptr -> client_len));
			stopped = false;
		}
		else server_ptr = new socketServer();
		server_ptr->connectToNetwork();
		server_ptr->receiveImageDims();
		long long int count = 0;
		//auto begin = chrono::high_resolution_clock::now();
		while(!stopped){
			//count++;
			screen = server_ptr->receiveImage(frame);
			//qDebug() << screen;
			resize(frame, frame, Size(640, 480), 0, 0, INTER_CUBIC);
			if(!screen) clrValue = frame.clone();
			if (drawRect){
				if(point1.x() < point2.x()) rectangle(frame, Rect(Point(point1.x(), point1.y()), Point(point2.x(), point2.y())), Scalar(0, 255, 0));
				else rectangle(frame, Rect(Point(point2.x(), point2.y()), Point(point1.x(), point1.y())), Scalar(0, 255, 0));
			}
			if(screen == 0)
			{
				emit display(
            	        	QPixmap::fromImage(
            	            	QImage(
            	            	    frame.data,
            	            	    frame.cols,
            	            	    frame.rows,
            	            	    frame.step,
            	            	    QImage::Format_RGB888)
            	            	.rgbSwapped()));
			}
			else{
				
				emit display2(
            	        	QPixmap::fromImage(
            	            	QImage(
            	            	    frame.data,
            	            	    frame.cols,
            	            	    frame.rows,
            	            	    frame.step,
            	            	    QImage::Format_RGB888)
            	            	.rgbSwapped()));
			}
			//auto end = chrono::high_resolution_clock::now();
			//auto ms = chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
			//cout << ((float)count/(float)ms*1000.0) << "\n";
		}
	}
    else{
		VideoCapture camera(0);
	    Mat frame;
    	stopped = false;
    	while(camera.isOpened() && !stopped)
    	{
    	    camera >> frame;
    	    if(frame.empty())
    	        continue;
			clrValue = frame.clone();
			if (drawRect){
				if(point1.x() < point2.x()) rectangle(frame, Rect(Point(point1.x(), point1.y()), Point(point2.x(), point2.y())), Scalar(0, 255, 0));
				else rectangle(frame, Rect(Point(point2.x(), point2.y()), Point(point1.x(), point1.y())), Scalar(0, 255, 0));
			}
    	    emit display(
    	                QPixmap::fromImage(
    	                    QImage(
    	                        frame.data,
    	                        frame.cols,
    	                        frame.rows,
    	                        frame.step,
    	                        QImage::Format_RGB888)
    	                    .rgbSwapped()));
			emit display2(
    	                QPixmap::fromImage(
    	                    QImage(
    	                        frame.data,
    	                        frame.cols,
    	                        frame.rows,
    	                        frame.step,
    	                        QImage::Format_RGB888)
    	                    .rgbSwapped()));
		}
	}
}

void sshComm::stopCamera()
{
	stopped = true;
}

void videoStream::stopVideo()
{
    stopped = true;
}

socketServer::socketServer(QObject * parent) : 
	QObject(parent),
	image_dims(Size2i(0, 0)),
    client_len(0),
    server_addr_size(sizeof(server_addr)),
    port(12345),
    sock_init(0),
    sock_connect(0) { client_len = server_addr_size;}

void socketServer::connectToNetwork(){
	sock_init = socket(AF_INET, SOCK_STREAM, 0);
	if(sock_init == -1){
		close(sock_init);
		//perror("Couldn't create socket!\n");
		exit(1);
	}
	
	memset((char*)&server_addr, 0, server_addr_size);
	
	server_addr.sin_family = AF_INET;
  	server_addr.sin_addr.s_addr = INADDR_ANY;
  	server_addr.sin_port = htons(port);
	
	if (bind(sock_init, (struct sockaddr*) &server_addr, server_addr_size) == -1) {
    	perror("Couldn't bind initial socket file descriptor!");
      	close(sock_init);
      	exit(1);
    }
	
  	listen(sock_init, 1);

	sock_connect = accept(sock_init, (struct sockaddr*) &client_addr, &client_len);

  	if (sock_connect == -1) {
    perror("ERROR! Client couldn't connect!");
    exit(1);
	}
}

void socketServer::receiveImageDims(){
  	ssize_t bytes_sent = 0;
  	size_t dims_size = 0;

  	int cols = 0;
  	int rows = 0;

  	size_t sizeof_dims = sizeof(image_dims.height);

  	if (bytes_sent = recv(sock_connect, (char*)&cols, sizeof_dims, 0) == -1) {
    	printf("ERROR!: recv failed\n"
           "sock_connect: %d\n"
           "image_size: %zu\n"
           "bytes_sent: %zu\n", sock_connect, dims_size, bytes_sent);
    		exit(1);
  	}

  	if (bytes_sent = recv(sock_connect, (char*)&rows, sizeof_dims, 0) == -1) {
    	printf("ERROR!: recv failed\n"
          	"sock_connect: %d\n"
           	"image_size: %zu\n"
           	"bytes_sent: %zu\n", sock_connect, dims_size, bytes_sent);
    	exit(1);
  	}
  	image_dims = Size2i(cols, rows);
  	printf("Image dimensions: [%dx%d]\n", cols, rows);
	
}

int socketServer::receiveImage(Mat& image) 
{
	int temp1;
	recv(sock_connect, (char*)&temp1, sizeof(uint16_t), 0);
  	int num_bytes = 0;
  	int image_ptr = 0;
  	int image_size = 0;

  	image = Mat::zeros(image_dims, CV_8UC3);

  	image_size = image.total() * image.elemSize();

  	uchar sock_data[image_size];

  	for (int i = 0; i < image_size; i += num_bytes) {
    	num_bytes = recv(sock_connect, sock_data + i, image_size - i, 0);
    	if (num_bytes == -1) {
     		printf("ERROR!: recv failed\n"
             	"i: %d\n"
             	"sock_fdesc: %d\n"
             	"image_size: %d\n"
             	"num_bytes: %d\n", i, sock_connect, image_size, num_bytes);
      		exit(1);
    	}
  	}

  // Write image data to cv::Mat
  	for (int i = 0;  i < image_dims.height; ++i) {
    	for (int j = 0; j < image_dims.width; ++j) {
      	image.at<Vec3b>(i,j) = Vec3b(sock_data[image_ptr+0],
                                     sock_data[image_ptr+1],
                                     sock_data[image_ptr+2]);
      	image_ptr += 3;
    	}
  	}
	return ntohs(temp1);
}

void socketServer::sendCommands(int type, int value){ //exit: 0  thres: (1, 2, 3, 4, 5, 6) == (hi, hi, hi, lo, lo, lo), value mode: 7 ,(0, 1, 2, 3, 4, 5) == (normal, cvtColor, blur, er dl, contour)
	conv = htons(type);
	send(sock_connect, (char*)&conv, sizeof(uint16_t), 0);
	conv = htons(value);
	send(sock_connect, (char*)&conv, sizeof(uint16_t), 0);
}
