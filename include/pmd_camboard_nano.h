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

#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <pmdsdk2.h>

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

  std::string getSerialNumber();

  // TODO: cache camera info?
  sensor_msgs::CameraInfoPtr getCameraInfo();

  /** Set an internal flag which controls whether the invalid pixels are
    * are filtered out from the output depth images.
    *
    * Setting this flag affects the behavior of getDepthImage(). If the flag is
    * not set, the depth images returned by that function will contain all of
    * the data received from the camera. If the flag is set, the camera will be
    * additionaly queried for the "flags" array that describes which of the
    * pixels are invalid. These pixels will be replaced with quiet NaNs in the
    * output depth images.
    *
    * By default the flag is set to true. */
  void setRemoveInvalidPixels(bool remove);

  sensor_msgs::ImagePtr getDepthImage();

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

  void throwExceptionIfFailed(int result);

  PMDHandle handle_;

  unsigned int num_rows_;
  unsigned int num_columns_;
  unsigned int num_pixels_;

  bool remove_invalid_pixels_;

};

}

#endif /* PMD_CAMBOARD_NANO_H */

