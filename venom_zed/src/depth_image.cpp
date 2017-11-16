#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sl/Camera.hpp>

#include <iostream>
#include <thread>
#include <limits>
#include <signal.h>

typedef struct SaveParamStruct {
  sl::POINT_CLOUD_FORMAT PC_Format;
  sl::DEPTH_FORMAT Depth_Format;
  sl::String saveName;
  bool askSavePC;
  bool askSaveDepth;
  bool stop_signal;
} SaveParam;

sl::Camera *zed_ptr;
SaveParam *param;

std::string getFormatNamePC(sl::POINT_CLOUD_FORMAT f) {
  std::string str_;
  switch (f) {
    case sl::POINT_CLOUD_FORMAT_XYZ_ASCII:
      str_ = "XYZ";
      break;
    case sl::POINT_CLOUD_FORMAT_PCD_ASCII:
      str_ = "PCD";
      break;
    case sl::POINT_CLOUD_FORMAT_PLY_ASCII:
      str_ = "PLY";
      break;
    case sl::POINT_CLOUD_FORMAT_VTK_ASCII:
      str_ = "VTK";
      break;
    default:
      break;
  }
  return str_;
}

std::string getFormatNameD(sl::DEPTH_FORMAT f) {
  std::string str_;
  switch (f) {
    case sl::DEPTH_FORMAT_PNG:
      str_ = "PNG";
      break;
    case sl::DEPTH_FORMAT_PFM:
      str_ = "PFM";
      break;
    case sl::DEPTH_FORMAT_PGM:
      str_ = "PGM";
      break;
    default:
      break;
  }
  return str_;
}

void nix_exit_handler(int s) {
  printf("\nQuitting...\n");
  delete zed_ptr;
  exit(1);
}

void saveProcess() {
    while (!param->stop_signal) {

        if (param->askSaveDepth) {
            float max_value = std::numeric_limits<unsigned short int>::max();
            float scale_factor = max_value / zed_ptr->getDepthMaxRangeValue();

            std::cout << "Saving Depth Map " << param->saveName << " in " << getFormatNameD(param->Depth_Format) << " ..." << std::flush;
            sl::saveDepthAs(*zed_ptr, param->Depth_Format, param->saveName, scale_factor);
            std::cout << "done" << std::endl;
            param->askSaveDepth = false;
        }

        if (param->askSavePC) {
            std::cout << "Saving Point Cloud " << param->saveName << " in " << getFormatNamePC(param->PC_Format) << " ..." << std::flush;
            sl::savePointCloudAs(*zed_ptr, param->PC_Format, param->saveName, true, false);
            std::cout << "done" << std::endl;
            param->askSavePC = false;
        }

        sl::sleep_ms(1);
    }
}


int main(int argc, char** argv){
  sl::Camera zed;
  zed_ptr = &zed;
  sl::String filename("depth.png");
  sl::InitParameters parameters;
  parameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
  parameters.camera_resolution = static_cast<sl::RESOLUTION> (2); // 720p
  parameters.coordinate_units = sl::UNIT_MILLIMETER;
  parameters.sdk_verbose = 1;
  parameters.sdk_gpu_id = -1;

  sl::ERROR_CODE err = zed.open(parameters);
  std::cout << errorCode2str(err) << std::endl;

  if (err != sl::SUCCESS) {
    std::cout<<"error: camera open failed\n";
    return 1;
  }

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = nix_exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  sl::Mat depthDisplay;

  while ( zed.isOpened() ){
    zed.grab(sl::SENSING_MODE_STANDARD);
    zed.retrieveImage(depthDisplay, sl::VIEW_DEPTH);
    float max_value = std::numeric_limits<unsigned short int>::max();
    float scale_factor = max_value / zed.getDepthMaxRangeValue();
    std::cout << "scale_factor: " << scale_factor << std::endl;
    bool saveResult = sl::saveDepthAs(zed, sl::DEPTH_FORMAT_PNG, filename, scale_factor);
    if (!saveResult)
      std::cout<<"warning: save depth failed. Keep trying...\n";
    sl::sleep_ms(100);
  }
  zed.close();
  return 0;
}
