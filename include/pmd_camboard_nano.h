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

class PMDCamboardNano
{

public:

  typedef boost::shared_ptr<PMDCamboardNano> Ptr;

  PMDCamboardNano(const std::string& device_serial);

  virtual ~PMDCamboardNano();

  std::string getSerialNumber();

  // TODO: cache camera info?
  sensor_msgs::CameraInfoPtr getCameraInfo();

  sensor_msgs::ImagePtr getDepthImage();

private:

  void throwExceptionIfFailed(int result);

  PMDHandle handle_;

  unsigned int num_rows_;
  unsigned int num_columns_;

};

#endif /* PMD_CAMBOARD_NANO_H */

