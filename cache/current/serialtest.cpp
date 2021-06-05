#include "unistd.h"
#include <pigpio.h>
#include <iostream>

using namespace std;

int main() {
    if(gpioInitialise() < 0) cout << "GPIO Init failed\n";
    char dest[] = "/dev/ttyAMA0";
    int serialTX = serOpen(dest, 1000000, 0);
    char buf[20], data[20] = "fuck u understand";
    
    for (int i=0; i<255; i++) {
        serWriteByte(serialTX,i);
        while(!serDataAvailable(serialTX)) sleep(0.2);
        cout << serReadByte(serialTX) << "\n";
        sleep(0.2);
    }
    

    sleep(0.2);
    
    
    serClose(serialTX);
    return(0);
}