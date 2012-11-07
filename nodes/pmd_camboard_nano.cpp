#include <ros/ros.h>
#include <ros/console.h>
#include <pmdsdk2.h>

class PMDCamboardNanoNode
{

public:

  PMDCamboardNanoNode()
  : handle_(0)
  { }

  ~PMDCamboardNanoNode()
  {
    if (handle_)
      pmdClose(handle_);
  }

  bool init()
  {
    int result;
    char error[128];
    result = pmdOpen(&handle_, PMD_PLUGIN_DIR "camboardnano", "", PMD_PLUGIN_DIR "camboardnanoproc", "");
    if (result != PMD_OK)
    {
      pmdGetLastError(0, error, 128);
      if (result == PMD_FILE_NOT_FOUND)
        ROS_ERROR_STREAM("Failed to load PMD plugins: " << error);
      else
        ROS_ERROR_STREAM("Failed to connect to the camera: " << error);
      return false;
    }

    char loaded[8];
    pmdSourceCommand(handle_, loaded, 8, "IsCalibrationDataLoaded");
    ROS_INFO_STREAM("Calibration loaded: " << loaded);

    return true;
  }

  void run()
  {
    ros::NodeHandle nh;
    while (nh.ok())
    {
      ros::spinOnce();
    }
  }

private:

  PMDHandle handle_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pmd_camboard_nano");
  PMDCamboardNanoNode pcnn;
  if (pcnn.init())
  {
    ROS_INFO("Initialized camboard node.");
    pcnn.run();
    return 0;
  }
  else
  {
    return 1;
  }
}
