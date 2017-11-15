#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sl/Camera.hpp>
#include <opencv2/core/utility.hpp>

int main(int argc, char** argv){

	//TODO customize parameteres
	//sl::InitParameters init_param;
	sl::Camera zed;
	sl::String filename;
	filename.set("test.png");
	if (zed.open() != sl::SUCCESS){
		std::cout << "open failed\n";
		return 1;
	}
	while (zed.isOpened()){
		sl::saveDepthAs(zed, sl::DEPTH_FORMAT_PNG, filename);
		sl::sleep_ms(100);
	}
	zed.close();
	return 0;
}
