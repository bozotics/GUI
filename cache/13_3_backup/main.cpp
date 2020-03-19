#include "camera.h"

int main(int argc, char **argv){
	picam* camera = new picam();
	cout << "Initiate stream\n";
	camera -> streaming();
	cout << "Terminated program\n";
	return 0;
}
