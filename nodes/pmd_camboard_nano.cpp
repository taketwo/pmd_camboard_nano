/******************************************************************************
 * Copyright (c) 2012 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

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
