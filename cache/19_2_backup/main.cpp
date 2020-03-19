#include "camera.h"

int main(int argc, char **argv){
	picam* camera = new picam();
	cout << "Initiate stream\n";
	thread thread_stream(&picam::streaming, camera);
	thread_stream.join();
	return 0;
}
