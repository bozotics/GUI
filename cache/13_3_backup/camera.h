#ifndef CAMERA_H
#define CAMERA_H
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <raspicam/raspicam.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <shared_mutex>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <netdb.h>
#include <memory>
#include <cstdio>
#include <vector>
#include <fstream>
#include <algorithm>

using namespace cv;
using namespace std;

class debug{
	public:
		void socketDisplay(shared_mutex& mtx, Mat& image, bool &stopped, int &dualMode);
		void display(shared_mutex& mtx, Mat& image);
		debug(Mat &image);
		void socketCommands(int &type, int &value, bool &stopped);

	private:
		int socket_connect();
		void sendImageDims(int dest, int cols, int rows);
		int readShow(shared_mutex &mtx, Mat &image, Mat &clone, int &dualMode);

	public:
		int socket_fdesc;

	private:
		const char* hostname;
		int port;
		struct addrinfo addrinfo_hints;
		struct addrinfo* addrinfo_resp;
		Mat cloneImg;
		Mat imageToSend = Mat::zeros(320, 480, CV_8UC3);
		unsigned int outputCnt = 0;
};

//uncomment

class picam{
	public:
		void streaming();
		picam();
	
	private:
		void writeFrame();
		void readFrame(Mat &clone);
		void writeShow(shared_mutex &mtx, Mat &image1, Mat &image2, Mat &show);
		void process();
		void readConfig();
		void processGoal();
		void processField();
		void writePoints(Point2f (&points)[12]);
		Mat readPoints(Mat frame);

	public:

	private:
		int width;
		int height;
		int tempThres[24];
		int thres[24];
		int configVal[9];
		int exposure;
		int fps;
		unsigned int frameCnt = 0;
		unsigned int processCnt = 0;
		debug* db;
		mutable shared_mutex mtx;
		mutable shared_mutex showMtx;
		mutable shared_mutex goalMtx;
		raspicam::RaspiCam Camera; 
		Mat image;
		Mat processImages[9];
		Mat showImage;
		int type = 1;
		int val = 0;
		bool stopped = false;
		float realDistanceX;
		float realDistanceY;
		float angle;
		int dualMode = 0;
		Point2f drawPoints[12];
};
#endif
