#ifndef VENOM_ZED_H

#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sl/Camera.hpp>
// defined in zed/utils/GlobalDefines.hpp --> Detect if we are running under
// a Jetson TK1 or TX1
#ifndef _SL_JETSON_
#include <opencv2/core/utility.hpp>
#endif

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>

namespace venom {

class VenomZed {

public:
  VenomZed (int argc, char **argv);
  ~VenomZed ();
  struct SaveParamStruct {
    sl::POINT_CLOUD_FORMAT pc_format;
    sl::DEPTH_FORMAT depth_format;
    sl::String save_name;
    bool ask_save_pc;
    bool ask_save_depth;
    bool stop_signal;
  } save_param_;

  static std::string getFormatNamePC(sl::POINT_CLOUD_FORMAT f);
  static std::string getFormatNameD(sl::DEPTH_FORMAT f);
  void Spin();

private:
#ifdef _WIN32
  static BOOL ctrlHandler(DWORD fdwCtrlType);
#else
  // TODO: signal handler function has to be static
  static void exitHandler(int s);
#endif

  void saveProcess();
  void saveSideBySideImage(std::string filename);
  int init(int argc, char **argv);
  int nbFrames_ = 0;
  sl::DEPTH_MODE depth_mode_ = sl::DEPTH_MODE_PERFORMANCE;
  sl::Camera zed_;
  sl::Pose camera_pose_;
  std::string path_;
  std::thread save_thread_;
  int mode_pc_;
  int mode_depth_;
  bool quit_;
  sl::Mat depthDisplay_;
  cv::Mat depth_cv_;

};// class VenomZed

VenomZed::VenomZed(int argc, char **argv) {
  //save_thread_ = new std::thread(this->saveProcess);
  init(argc, argv);
  save_thread_ = std::thread(&VenomZed::saveProcess, this);
}

VenomZed::~VenomZed() {
  zed_.close();
  save_thread_.join();
}

std::string VenomZed::getFormatNamePC(sl::POINT_CLOUD_FORMAT f) {
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

std::string VenomZed::getFormatNameD(sl::DEPTH_FORMAT f) {
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

// Save function called in a thread

void VenomZed::saveProcess() {
  while (!save_param_.stop_signal) {
    if (save_param_.ask_save_depth) {
      float max_value = std::numeric_limits<unsigned short int>::max();
      float scale_factor = max_value / zed_.getDepthMaxRangeValue();

      std::cout << "Saving Depth Map " << save_param_.save_name << " in "
	<< VenomZed::getFormatNameD(save_param_.depth_format)
	<< " ..." << std::flush;
      sl::saveDepthAs(zed_, save_param_.depth_format,
	  save_param_.save_name, scale_factor);
      std::cout << "done" << std::endl;
      save_param_.ask_save_depth = false;
    }

    if (save_param_.ask_save_pc) {
      std::cout << "Saving Point Cloud " << save_param_.save_name << " in "
	<< VenomZed::getFormatNamePC(save_param_.pc_format)
	<< " ..." << std::flush;
      sl::savePointCloudAs(zed_, save_param_.pc_format,
	  save_param_.save_name, true, false);
      std::cout << "done" << std::endl;
      save_param_.ask_save_pc = false;
    }

  sl::sleep_ms(1);
  }
}

// Save function using opencv

void VenomZed::saveSideBySideImage(std::string filename) {
  sl::Mat sbs_sl;
  zed_.retrieveImage(sbs_sl, sl::VIEW_SIDE_BY_SIDE);
  sbs_sl.write(filename.c_str());
  std::cout << "Image saved !" << std::endl;
}

#ifdef _WIN32

BOOL VenomZed::ctrlHandler(DWORD fdwCtrlType) {
  switch (fdwCtrlType) {
    //Handle the CTRL-C signal.
    case CTRL_C_EVENT:
      printf("\nQuitting...\n");
      //save_thread_.join();
      //zed_.close();
      exit(0);
    default:
      return FALSE;
  }
}
#else

void VenomZed::exitHandler(int s) {
    printf("\nQuitting...\n");
    //save_thread_.join();
    //zed_.close();
    exit(1);
}
#endif

int VenomZed::init(int argc, char **argv) {
  nbFrames_ = 0;
  depth_mode_ = sl::DEPTH_MODE_PERFORMANCE;
#ifdef _SL_JETSON_
  const cv::String keys = {
    "{ h | help      |       | print help message }"
    "{ f | filename  |       | path to SVO filename}"
    "{ r | resolution|   2   |ZED Camera resolution, ENUM 0: HD2K   1: HD1080   2: HD720   3: VGA}"
    "{ m | mode      |   2   |Disparity Map mode, ENUM 1: PERFORMANCE  2: MEDIUM   3: QUALITY}"
    "{ p | path      |   ./  |Output path (can include output filename prefix)}"
    "{ d | device    |   -1  |CUDA device ID }"
  };

  cv::CommandLineParser parser(argc, argv, keys.c_str());
#else

  const cv::String keys =
    "{help h usage ? || print this message}"
    "{filename f||SVO filename (ex : -f=test.svo  or --filename=test.svo) }"
    "{resolution r|2|ZED Camera resolution, ENUM 0: HD2K   1: HD1080   2: HD720   3: VGA (ex : -r=1  or --resolution=1 for HD1080)}"
    "{mode m|2|Disparity Map mode, ENUM 1: PERFORMANCE  2: MEDIUM   3: QUALITY  (ex : -m=1  or --mode=1)}"
    "{path p|./|Output path (can include output filename prefix) (ex : -p=./../ or --path=./../)}"
    "{device d|-1|CUDA device (ex : -d=0 or --device=0) }";

  cv::CommandLineParser parser(argc, argv, keys);
  //about is not available under OpenCV2.4
  parser.about("Sample from ZED SDK" + std::string(sl::Camera::getSDKVersion())); 
#endif
#ifdef _SL_JETSON_
  if (parser.get<bool>("help")) {
    parser.printParams();
    return 0;
  }
#else
  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }
#endif

  sl::InitParameters parameters;
  cv::String filename = parser.get<cv::String>("filename");
  if (filename.empty()) {
    std::cout << "Saving depth from ZED camera (LIVE)" << std::endl;
    int resolution = parser.get<int>("resolution");
    switch (resolution) {
      case 0: std::cout << "Resolution set to HD2K" << std::endl;
	break;
      case 1: std::cout << "Resolution set to HD1080" << std::endl;
	break;
      case 2: std::cout << "Resolution set to HD720" << std::endl;
	break;
      case 3: std::cout << "Resolution set to VGA" << std::endl;
	break;
      default: std::cout << "Invalid Resolution " << resolution << std::endl;
	break;
    }
    parameters.camera_resolution = static_cast<sl::RESOLUTION> (resolution);
  } else {
    std::cout << "Saving depth from SVO : " << filename << std::endl;
    parameters.svo_input_filename = filename.c_str();
    nbFrames_ = zed_.getSVONumberOfFrames();
    std::cout << "SVO number of frames : " << nbFrames_ << std::endl;
  }

  int mode = parser.get<int>("mode");
  switch (mode) {
    case 1:
      std::cout << "Mode set to PERFORMANCE" << std::endl;
      depth_mode_ = sl::DEPTH_MODE_PERFORMANCE;
      break;
    case 2:
      std::cout << "Mode set to MEDIUM" << std::endl;
      depth_mode_ = sl::DEPTH_MODE_MEDIUM;
      break;
    case 3:
      std::cout << "Mode set to QUALITY" << std::endl;
      depth_mode_ = sl::DEPTH_MODE_QUALITY;
      break;
    default:
      std::cout << "Invalid depth quality " << mode << std::endl;
      break;
  }
  depth_mode_ = static_cast<sl::DEPTH_MODE> (mode);
  path_ = parser.get<std::string>("path");
  int device = parser.get<int>("device");

#ifndef _SL_JETSON_
  //this check is available on 3.1 but not on OpenCV4Tegra
  if (!parser.check()) {
    parser.printErrors();
    return 0;
  }
#else
  // No check can be performed easily with OpenCV2.4
  // Check that parameters are set correctly or it will result to a SegFault
#endif
  parameters.depth_mode = depth_mode_;
  parameters.coordinate_units = sl::UNIT_MILLIMETER;
  parameters.sdk_verbose = 1;
  parameters.sdk_gpu_id = device;

  sl::ERROR_CODE err = zed_.open(parameters);
  std::cout << errorCode2str(err) << std::endl;

  //Quit if an error occurred
  if (err != sl::SUCCESS) {
      return 1;
  }

#ifdef _WIN32
  SetConsolectrlHandler((PHANDLER_ROUTINE) ctrlHandler, TRUE);
#else // unix
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = VenomZed::exitHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
#endif


  int depth_clamp = 5000;
  zed_.setDepthMaxRangeValue(depth_clamp);

  mode_pc_ = 0;
  mode_depth_ = 0;

  save_param_.ask_save_pc = false;
  save_param_.ask_save_depth = false;
  save_param_.stop_signal = false;
  save_param_.pc_format = static_cast<sl::POINT_CLOUD_FORMAT> (mode_pc_);
  save_param_.depth_format = static_cast<sl::DEPTH_FORMAT> (mode_depth_);

  quit_ = false;

  return 0;
} // init()

void VenomZed::Spin() {
  char key = ' '; // key pressed
  bool print_help = false;
  std::string help_string = "[d] save Depth, [P] Save Point Cloud, [m] change\
			    format PC, [n] change format Depth, [q] quit";
  std::string prefix_pc = "PC_"; //Default output file prefix
  std::string prefix_depth = "Depth_"; //Default output file prefix
  int count;
  while (!quit_ && (zed_.getSVOPosition() <= nbFrames_)) {

    zed_.grab(sl::SENSING_MODE_STANDARD);
    zed_.retrieveImage(depthDisplay_, sl::VIEW_DEPTH);

    depth_cv_ = cv::Mat(depthDisplay_.getHeight(), depthDisplay_.getWidth(),
	CV_8UC4, depthDisplay_.getPtr<sl::uchar1>(sl::MEM_CPU));

    if (print_help) // Write help text on the image if needed
      cv::putText(depth_cv_, help_string, cv::Point(20, 20),
	  CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(111, 111, 111, 255), 2);

    cv::imshow("Depth", depth_cv_);
    key = cv::waitKey(5);

    switch (key) {
      case 'p':
      case 'P':
	save_param_.save_name = std::string(path_ + prefix_pc + \
	    std::to_string(count)).c_str();
	save_param_.ask_save_pc = true;
	break;

      case 'd':
      case 'D':
	save_param_.save_name =  std::string(path_ + prefix_depth + \
	    std::to_string(count)).c_str();
	save_param_.ask_save_depth = true;
	break;

      case 'm': // point cloud format
      case 'M':
      {
	mode_pc_++;
	save_param_.pc_format = static_cast<sl::POINT_CLOUD_FORMAT> (mode_pc_ % 4);
	std::cout << "Format Point Cloud " \
	  << VenomZed::getFormatNamePC(save_param_.pc_format) << std::endl;
      }
	break;

      case 'n': // depth format
      case 'N':
      {
	mode_depth_++;
	save_param_.depth_format = static_cast<sl::DEPTH_FORMAT> (mode_depth_ % 3);
	std::cout << "Format Depth " \
	  << VenomZed::getFormatNameD(save_param_.depth_format) << std::endl;
      }
	break;

      case 'h': // print help
      case 'H':
	print_help = !print_help;
	std::cout << help_string << std::endl;
	break;

      case 's': // save side by side images
	saveSideBySideImage(std::string("ZEDImage") + std::to_string(count) + \
	    std::string(".png"));
	break;
      case 'q': // quit
      case 'Q':
      case 27:
	quit_ = true;
	break;
      count++;
    } // switch(key)

    save_param_.stop_signal = true;
  }
} // spin()

}// namespace venom

#endif // VENOM_ZED_H
