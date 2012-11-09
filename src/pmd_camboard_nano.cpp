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

#include <cstdio>

#include <boost/make_shared.hpp>

#include <sensor_msgs/image_encodings.h>

#include "pmd_camboard_nano.h"
#include "pmd_exceptions.h"

namespace pmd_camboard_nano
{

PMDCamboardNano::PMDCamboardNano(const std::string& device_serial)
: handle_(0)
, num_rows_(0)
, num_columns_(0)
, remove_invalid_pixels_(true)
{
  throwExceptionIfFailed(pmdOpen(&handle_, PMD_PLUGIN_DIR "camboardnano", device_serial.c_str(), PMD_PLUGIN_DIR "camboardnanoproc", ""));
  if (device_serial != "" && device_serial != getSerialNumber())
  {
    pmdClose(handle_);
    throw PMDCameraNotOpenedException("Opened a wrong camera (serial number does not match the requested).");
  }
}

PMDCamboardNano::~PMDCamboardNano()
{
  if (handle_)
    pmdClose(handle_);
}

std::string PMDCamboardNano::getSerialNumber()
{
  char serial[128];
  throwExceptionIfFailed(pmdSourceCommand(handle_, serial, sizeof(serial), "GetSerialNumber"));
  return std::string(serial);
}

sensor_msgs::CameraInfoPtr PMDCamboardNano::getCameraInfo()
{
  PMDDataDescription desc;
  throwExceptionIfFailed(pmdUpdate(handle_));
  throwExceptionIfFailed(pmdGetSourceDataDescription(handle_, &desc));
  num_rows_ = desc.img.numRows;
  num_columns_ = desc.img.numColumns;
  num_pixels_ = num_rows_ * num_columns_;
  char lens[128];
  throwExceptionIfFailed(pmdSourceCommand(handle_, lens, 128, "GetLensParameters"));
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  info->width = num_columns_;
  info->height = num_rows_;
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->D.resize(5, 0.0);
  info->K.assign(0.0);
  info->P.assign(0.0);
  info->R.assign(0.0);
  double fx, fy, cx, cy, k1, k2, p1, p2, k3;
  if (sscanf(lens, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &fx, &fy, &cx, &cy, &k1, &k2, &p1, &p2, &k3) == 9)
  {
    info->D[0] = k1;
    info->D[1] = k2;
    info->D[2] = p1;
    info->D[3] = p2;
    info->D[4] = k3;
    info->K[0] = info->P[0] = fx;
    info->K[2] = info->P[2] = cx;
    info->K[4] = info->P[5] = fy;
    info->K[5] = info->P[6] = cy;
    info->K[8] = info->P[10] = 1.0;
    info->R[0] = info->R[4] = info->R[8] = 1.0;
  }
  else
  {
    // TODO: are there any sensible defaults?
  }
  return info;
}

void PMDCamboardNano::setRemoveInvalidPixels(bool remove)
{
  remove_invalid_pixels_ = remove;
}

sensor_msgs::ImagePtr PMDCamboardNano::getDepthImage()
{
  throwExceptionIfFailed(pmdUpdate(handle_));
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
  depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->height = num_rows_;
  depth_msg->width = num_columns_;
  depth_msg->step = depth_msg->width * sizeof(float);
  size_t size = depth_msg->height * depth_msg->step;
  depth_msg->data.resize(size);
  float* data = reinterpret_cast<float*>(&depth_msg->data[0]);
  throwExceptionIfFailed(pmdGetDistances(handle_, data, size));
  if (remove_invalid_pixels_)
  {
    unsigned int flags[num_pixels_];
    throwExceptionIfFailed(pmdGetFlags(handle_, flags, sizeof(flags)));
    for (size_t i = 0; i < num_pixels_; i++)
    {
      const unsigned int INVALID = PMD_FLAG_INVALID | PMD_FLAG_LOW_SIGNAL | PMD_FLAG_INCONSISTENT;
      if (flags[i] & INVALID)
        data[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  return depth_msg;
}

unsigned int PMDCamboardNano::getIntegrationTime()
{
  unsigned int i;
  throwExceptionIfFailed(pmdGetIntegrationTime(handle_, &i, 0));
  return i;
}

unsigned int PMDCamboardNano::setIntegrationTime(unsigned int time)
{
  unsigned int valid;
  throwExceptionIfFailed(pmdGetValidIntegrationTime(handle_, &valid, 0, CloseTo, time));
  throwExceptionIfFailed(pmdSetIntegrationTime(handle_, 0, valid));
  return valid;
}

void PMDCamboardNano::throwExceptionIfFailed(int result)
{
  if (result != PMD_OK)
  {
    char error[128];
    pmdGetLastError(handle_, error, 128);
    switch (result)
    {
      case PMD_FILE_NOT_FOUND: throw PMDPluginNotFoundException(error);
      case PMD_COULD_NOT_OPEN: throw PMDCameraNotOpenedException(error);
      default: throw PMDException(result, error);
    }
  }
}

}

