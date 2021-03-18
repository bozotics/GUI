#include "camera.h"
#include <sys/types.h>
#include <sys/wait.h>

//extern bool fuckoff;

void s1_changed(int gpio, int level, uint32_t tick, void *userdata)
{
    cout << "S1 FUCK " << gpioRead(gpio) << endl;
	atomic_bool *tmp = (atomic_bool*) userdata;
    if (!gpioRead(gpio)) {   //if triggered low
        tmp -> store(true, memory_order_acq_rel);
    }
}

void dummy(int gpio, int level, uint32_t tick)
{
    cout << "debug dummy " << endl;
}

int main(int argc, char **argv){
	//pi = pigpio_start(0, 0);
	//if(pi < 0) printf("Can't connect to the pigpio daemon\n");
	if(gpioInitialise() < 0) cout << "GPIO Init failed\n";
	gpioSetMode(23, PI_INPUT);
	gpioSetMode(24, PI_INPUT);
	gpioGlitchFilter(23, 200000);
    gpioGlitchFilter(24, 200000);

	picam* camera = new picam();	//Create camera object

		
	printf("Initiate stream\n");
    if (!(camera -> Camera.open(camera -> image))){	//open camera and pass the address of the circular buffer into the function
		cerr << "Can't open camera\n";
		return 0;
	}
    camera -> kernel = Mat::ones(3, 3, CV_8UC1);
	if(argc==1) {
		gpioSetAlertFuncEx(24, s1_changed, &(camera -> stopped));
		camera -> run();
		pid_t pid = fork();
		if (pid==0) {
			execl("/usr/bin/sudo", "/usr/bin/sudo", "/home/alarm/project/cpp/masterbait/masterbait", (char *)NULL);
		} else {
			// int status;
			// waitpid(pid, &status, 0);
		}
		cout << "LEAVE RUN" << endl;
	}

    else{
        printf("Debug Mode...\n");
		gpioSetAlertFunc(24, dummy);
        camera -> debug();
		gpioTerminate();
    }
	delete camera;	//delete camera object
	printf("Terminated program\n");
	return 0;
}
