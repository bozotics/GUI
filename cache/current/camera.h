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
#include <condition_variable>
#include <unistd.h>
#include <atomic>
#include <sys/socket.h> //debug communication
#include <netinet/in.h>
#include <netdb.h>   
#include <sys/types.h>
#include <vector>   //config read
#include <fstream>
#include <stdio.h> 
#include <math.h>

using namespace std;
using namespace cv;

#define PI 3.14159265

//bool fuckoff=false;

class picam{
    //FUNCTIONS
    public:
        picam();
        void run();
        void debug();
        raspicam::RaspiCam Camera; 
        Mat kernel, image[10];
        int zero = 0;
        atomic_bool stopped = ATOMIC_VAR_INIT(false);

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
            int PrioSkip[3];
            int maskRadius;
        };

    private:    //camera
        void readConfig();
        void getFrame();
        void processBall();
        void processGoal();
        void processField();
        void waitForBall();
        void wakeBall();
        void waitForGoal();
        void wakeGoal();
        void waitForField();
        void wakeField();
        void waitForSend();
        void wakeSend(int category);
        bool updateSend(int category);
        double distanceMapper(double pixel, int &centre);
        double distanceUnmapper(double const &dist, int &centre);
        void inRangeHSV(int type,  Mat &input, Mat &output, Mat &temp);
        void inttochar(const int integer, char *character, int &arrayPos);

    private:    //debug
        void socketConnect();
        void sendImageDims();
        void receiveCommands();
    //VARIABLES
    private:    //camera
        Mat image2, circleMask;
        bool sendDataFlag[3] = {false, false, false};
        bool wakeBallFlag = false;
        bool wakeGoalFlag = false;
        bool wakeFieldFlag = false;

        atomic_uint bufferPosition = ATOMIC_VAR_INIT(0);
        atomic_uint ballPriority = ATOMIC_VAR_INIT(1);
        atomic_uint goalPriority = ATOMIC_VAR_INIT(2);
        atomic_uint fieldPriority = ATOMIC_VAR_INIT(3);
        
        atomic_int ballReq = ATOMIC_VAR_INIT(1);
        atomic_int goalReq = ATOMIC_VAR_INIT(5);
        atomic_int fieldReq = ATOMIC_VAR_INIT(9);
        condition_variable ballCond;
        condition_variable goalCond;
        condition_variable fieldCond;
        condition_variable sendCond;
        //shared_mutex imgMtx;
        mutex ballMtx;
        mutex goalMtx;
        mutex fieldMtx;
        mutex sendMtx;
        unsigned int imgCnt = 0;
        atomic_uint ballCnt = ATOMIC_VAR_INIT(0);
        atomic_uint goalCnt = ATOMIC_VAR_INIT(0);
        atomic_uint fieldCnt = ATOMIC_VAR_INIT(0);
        Rect pBall[3], rBall[3];
        RotatedRect bGoal[3], yGoal[3];
        RotatedRect field[3];
        camProperties prop;
        int thres[24];
        int serialTX, serialRX;

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