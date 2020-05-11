#include "camera.h"

int main(int argc, char **argv){
	//pi = pigpio_start(0, 0);
	//if(pi < 0) printf("Can't connect to the pigpio daemon\n");
	
	picam* camera = new picam();	//Create camera object
	printf("Initiate stream\n");
    if (!(camera -> Camera.open(camera -> image))){	//open camera and pass the address of the circular buffer into the function
		cerr << "Can't open camera\n";
		return 0;
	}
    camera -> kernel = Mat::ones(3, 3, CV_8UC1);
	if(argc==1) camera -> run();
    else{
        printf("Debug Mode...\n");
        camera -> debug();
    }
	delete camera;	//delete camera object
	printf("Terminated program\n");
	return 0;
}
