#include <stdio.h>
#include <pigpio.h>
#include <chrono>   //time
#include <iostream> //IO
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
using namespace std;

int fuckoff = 0;

void s1_changed(int gpio, int level, uint32_t tick)
{
    cout << "S1 FUCK " << gpioRead(gpio) << endl;
    if (gpioRead(gpio)) {   //if triggered high
        fuckoff=1;
    }
}

void s2_changed(int gpio, int level, uint32_t tick)
{
    cout << "S2 FUCK " << gpioRead(gpio) << endl;
    if (gpioRead(gpio)) {   //if triggered high
        fuckoff=2;
    }
}

int main() {
    gpioInitialise();
    usleep(1000);
    gpioSetMode(23, PI_INPUT);
	gpioSetMode(24, PI_INPUT);
    gpioGlitchFilter(23, 200000);
    gpioGlitchFilter(24, 200000);

    while (gpioRead(23)) ;      //starts off low
    while (gpioRead(24)) ;      //starts off low
    usleep(500000);
    gpioSetAlertFunc(24, s1_changed);
    gpioSetAlertFunc(23, s2_changed);
    cout << "READY" << endl;
    auto begin = std::chrono::steady_clock::now();
    while(!fuckoff) {
        if(chrono::steady_clock::now() - begin > chrono::seconds(6000)) break;
        usleep(100000);
    }
    if (fuckoff == 1) {
        cout << "fuck1" << endl;
        gpioTerminate();
        cout << "fuck2" << endl;
        pid_t pid = fork();
        if (pid==0) {
            execl("/usr/bin/sudo", "/usr/bin/sudo", "/home/alarm/project/cpp/camera4/camera", (char *)NULL);
        } else {
            // int status;
            // waitpid(pid, &status, 0);
        }
        cout << "fuck3" << endl;
    } else if (fuckoff == 2) {
        cout << "shutting down";
        system("sudo shutdown -h now");
    }
    
    return 0;
}