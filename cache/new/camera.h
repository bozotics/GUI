#ifndef CAMERA_H
#define CAMERA_H
#include <raspicam/raspicam.h>  //image processing
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <chrono>   //time
#include <iostream> //IO
#include <stdio.h>
#include <pigpio.h>
#include <thread>   //Threading
#include <shared_mutex>
#include <unistd.h>
#include <atomic>
#include <sys/socket.h> //debug communication
#include <netinet/in.h>
#include <netdb.h>   
#include <sys/types.h>
#include <vector>   //config read
#include <fstream>
#include <stdio.h> 

using namespace std;
using namespace cv;

class picam{
    //FUNCTIONS
    public:
        picam();
        void run();
        void debug();
        raspicam::RaspiCam Camera; 
        Mat kernel, image[10];

    private:
        struct coordinates{
            double x;
            double y;
        };
        struct camProperties{
            int width;
            int height;
            int fps;
            int x;
            int y;
            int configVal[7]; //[awb_r, awb_b, exposure, brightness, saturation, ss, ISO]
        };

    private:    //camera
        void readConfig();
        void getFrame();
        void writeFrame();
        unsigned int readFrame(unsigned int &frameNum, Mat &frame);
        void processBall();
        void writeBall(Rect real, Rect pred);
        unsigned int readBall(unsigned int &frameNum, Rect &realClone, Rect &predClone);
        void processGoal();
        void writeGoal(RotatedRect bGoal, RotatedRect yGoal);
        unsigned int readGoal(unsigned int &frameNum, RotatedRect &bGoalClone, RotatedRect &yGoalClone);
        void processField();
        void writeField(RotatedRect field);
        unsigned int readField(unsigned int &frameNum, RotatedRect &fieldClone);
        double distanceMapper(double pixel, int &centre);
        double distanceUnmapper(double const &dist, int &centre);
        void inRangeHSV(int type,  Mat &input, Mat &output, Mat &temp);

    private:    //debug
        void socketConnect();
        void sendImageDims();
        void receiveCommands();
    //VARIABLES
    private:    //camera
        Mat image2;
        atomic_int bufferPosition = ATOMIC_VAR_INIT(0);
        mutable shared_mutex imgMtx;
        mutable shared_mutex ballMtx;
        mutable shared_mutex goalMtx;
        mutable shared_mutex fieldMtx;
        unsigned int imgCnt = 0, ballCnt = 0, goalCnt = 0, fieldCnt = 0;
        Rect pBall, rBall;
        RotatedRect bGoal, yGoal;
        RotatedRect field;
        atomic_bool stopped = ATOMIC_VAR_INIT(false);
        camProperties prop;
        int thres[24];

    private:    //debug
        bool debugCheck;
        int socketIdentity;
        mutex read;
        Mat debugImages[9];
        int tempThres[24];
        int showFrame = 0, showFrame2 = 3;
        int trackType = 0;
};
#endif
