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

#ifndef PMD_CAMBOARD_NANO_H
#define PMD_CAMBOARD_NANO_H

#include <string>

#include <ros/time.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <pmdsdk2.h>

#ifndef PMD_PLUGIN_DIR
  #error PMD_PLUGIN_DIR should be defined
#endif

namespace pmd_camboard_nano
{

class PMDCamboardNano
{

public:

  typedef boost::shared_ptr<PMDCamboardNano> Ptr;

  /** Try to open a PMD camera with a given serial number.
    *
    * If the serial number is an empty string, then the first available camera
    * will be opened. The PMD SDK driver tends to open any available camera if
    * the camera with the specified serial number is not present. If this
    * happens, the constructor will close the opened camera and assume that the
    * requested camera is not available.
    *
    * An exception will be thrown if it is not possible to open the camera. */
  PMDCamboardNano(const std::string& device_serial = "");

  virtual ~PMDCamboardNano();

  /** Download the most recent data from the device.
    *
    * This function should be called before getDepthImage(),
    * getAmplitudeImage(),  or getCameraInfo(), otherwise they will return the
    * old data.
    *
    * An expection will be thrown if the interaction with the device failed. */
  void update();

  /** Get the depth map packaged in a ROS image message.
    *
    * Two subsequent calls to this function will return the same images with
    * the same timestapms unless update() is called in between.
    *
    * This function may treat the invalid pixels differently, see
    * setRemoveInvalidPixels().
    *
    * Note: the output message has a blanc frame_id field. */
  sensor_msgs::ImagePtr getDepthImage();

  /** Get the signal strength of active illumination packaged in a ROS image
    * message.
    *
    * Two subsequent calls to this function will return the same images with
    * the same timestapms unless update() is called in between.
    *
    * This function may treat the invalid pixels differently, see
    * setRemoveInvalidPixels().
    *
    * Note: the output message has a blanc frame_id field. */
  sensor_msgs::ImagePtr getAmplitudeImage();

  // TODO: cache camera info?
  sensor_msgs::CameraInfoPtr getCameraInfo();

  std::string getSerialNumber();

  /** Set an internal flag which controls whether the invalid pixels are
    * are filtered out from the output depth/amplitude images.
    *
    * Setting this flag affects the behavior of getDepthImage() and
    * getAmplitudeImage(). If the flag is not set, the depth/aplitude images
    * returned by those functions will contain all of the data received from the
    * camera. If the flag is set, the camera will be additionaly queried for the
    * "flags" array that describes which of the pixels are invalid. These pixels
    * will be replaced with quiet NaNs in the output depth/amplitude images.
    *
    * By default the flag is set to true. */
  void setRemoveInvalidPixels(bool remove);

  unsigned int getIntegrationTime();

  /** Set integration time.
    *
    * Not all integration times are valid settings. This function will first
    * query the camera to find the closest valid value for integration time and
    * then set it.
    *
    * @return coerced integration time */
  unsigned int setIntegrationTime(unsigned int time);

private:

  sensor_msgs::ImagePtr createImageMessage();

  void removeInvalidPixels(float* data);

  void throwExceptionIfFailed(int result);

  PMDHandle handle_;

  unsigned int num_rows_;
  unsigned int num_columns_;
  unsigned int num_pixels_;

  bool remove_invalid_pixels_;

  ros::Time last_update_time_;

};

}

#endif /* PMD_CAMBOARD_NANO_H */

