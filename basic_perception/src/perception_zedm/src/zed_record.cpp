/*
 * Record camera stream. Adapted from zed tutorial.
 */

#include <iostream>
#include <sl/Camera.hpp>

#include <unistd.h>
#include <signal.h>

using namespace sl;
using namespace std;

static bool exit_app = false;

inline uint64_t GetTimeStampMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void nix_exit_handler(int s) {
  exit_app = true;
}

void SetCtrlHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = nix_exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char **argv) {
  Camera zed;

  InitParameters init_parameters;
  init_parameters.camera_resolution = RESOLUTION::HD720;
  init_parameters.depth_mode = DEPTH_MODE::NONE;
  init_parameters.coordinate_units = sl::UNIT::METER;
  init_parameters.camera_fps = 15;

  auto returned_state = zed.open(init_parameters);
  if (returned_state != ERROR_CODE::SUCCESS) {
    cout << "Camera Open " << returned_state<< " Exit program." << endl;
    return -1;
  }

  // Enable recording with the filename specified in argument
  String path_output = "myRecording.svo";
  returned_state = zed.enableRecording(RecordingParameters(path_output, SVO_COMPRESSION_MODE::H264_LOSSLESS));
  if (returned_state != ERROR_CODE::SUCCESS) {
    cout << "Recording ZED : " << returned_state << endl;
    zed.close();
    return -1;
  }

  // Start recording SVO, stop with Ctrl-C command
  cout << "SVO is Recording, use Ctrl-C to stop." << endl;
  SetCtrlHandler();
  int frames_recorded = 0;

  uint64_t start_ts = GetTimeStampMs();
  sl::RecordingStatus rec_status;
  while (!exit_app) {
    if (zed.grab() == ERROR_CODE::SUCCESS) {
      // Each new frame is added to the SVO file
      rec_status = zed.getRecordingStatus();
      if (rec_status.status)
        frames_recorded++;
    }
  }

  uint64_t end_ts = GetTimeStampMs();
  double duration = double(end_ts - start_ts) / 1000;

  cout << "Frame count: " << to_string(frames_recorded) << " frame rate: " << frames_recorded / duration << endl;

  zed.disableRecording();
  zed.close();
  return EXIT_SUCCESS;
}
