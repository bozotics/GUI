#ifndef CAMERA_H
#define CAMERA_H
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <raspicam/raspicam_cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
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
		void socketDisplay(shared_mutex& mtx, Mat& image);
		void display(shared_mutex& mtx, Mat& image);
		debug(Mat &image);

	private:
		int socket_connect();
		void sendImageDims(int dest, int cols, int rows);
		void readFrame(shared_mutex &mtx, Mat &image, Mat &clone);

	private:
		const char* hostname;
		int port;
		int socket_fdesc;
		struct addrinfo addrinfo_hints;
		struct addrinfo* addrinfo_resp;
		Mat cloneImg;
		Mat imageToSend = Mat::zeros(320, 480, CV_8UC3);
		unsigned int outputCnt = 0;
};

class picam{
	public:
		void streaming();
		void process();
		picam();
	
	private:
		void writeFrame();
		void readFrame(Mat &clone);
		void readConfig();
	
	public:

	private:
		int width;
		int height;
		int hi_1;
		int hi_2;
		int hi_3;
		int lo_1;
		int lo_2;
		int lo_3;
		int exposure;
		int fps;
		unsigned int frameCnt = 0;
		unsigned int processCnt = 0;
		debug* db;
		mutable shared_mutex mtx;
		raspicam::RaspiCam_Cv Camera; 
		Mat image;
		Mat imageProcess;
		Mat imageClr;
		Mat imageBlur;
		Mat imageRange;
		Mat imageED;
		Mat imageContours;
		Mat processedImage;
};
#endif
