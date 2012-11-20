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

void PMDCamboardNano::update()
{
  throwExceptionIfFailed(pmdUpdate(handle_));
  last_update_time_ = ros::Time::now();
}

sensor_msgs::ImagePtr PMDCamboardNano::getDepthImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetDistances(handle_, data, msg->height * msg->step));
  if (remove_invalid_pixels_)
    removeInvalidPixels(data);
  return msg;
}

sensor_msgs::ImagePtr PMDCamboardNano::getAmplitudeImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetAmplitudes(handle_, data, msg->height * msg->step));
  if (remove_invalid_pixels_)
    removeInvalidPixels(data);
  return msg;
}

sensor_msgs::PointCloud2Ptr PMDCamboardNano::getPointCloud()
{
  sensor_msgs::PointCloud2Ptr msg = createPointCloud2Message(false);
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGet3DCoordinates(handle_, data, msg->height * msg->row_step));
  if (remove_invalid_pixels_)
    removeInvalidPixels(data + 2, 3);
  return msg;
}

sensor_msgs::PointCloud2Ptr PMDCamboardNano::getPointCloudWithAmplitudes()
{
  sensor_msgs::PointCloud2Ptr msg = createPointCloud2Message(true);
  float* msg_data = reinterpret_cast<float*>(&msg->data[0]);
  size_t size = num_rows_ * num_columns_;
  float* pts_data = new float[size * 3];
  float* amp_data = new float[size];
  throwExceptionIfFailed(pmdGet3DCoordinates(handle_, pts_data, size * 3 * sizeof(float)));
  throwExceptionIfFailed(pmdGetAmplitudes(handle_, amp_data, size * sizeof(float)));
  if (remove_invalid_pixels_)
    removeInvalidPixels(pts_data + 2, 3);

  for (size_t i = 0; i < num_rows_ * num_columns_; i++)
  {
    msg_data[0] = pts_data[0];
    msg_data[1] = pts_data[1];
    msg_data[2] = pts_data[2];
    msg_data[3] = amp_data[0];
    pts_data += 3;
    amp_data += 1;
    msg_data += 4;
  }
  return msg;
}

sensor_msgs::CameraInfoPtr PMDCamboardNano::getCameraInfo()
{
  PMDDataDescription desc;
  throwExceptionIfFailed(pmdGetSourceDataDescription(handle_, &desc));
  num_rows_ = desc.img.numRows;
  num_columns_ = desc.img.numColumns;
  num_pixels_ = num_rows_ * num_columns_;
  char lens[128];
  throwExceptionIfFailed(pmdSourceCommand(handle_, lens, 128, "GetLensParameters"));
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  info->header.stamp = last_update_time_;
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
    // These numbers come from a forum post at www.cayim.com
    // https://www.cayim.com/forum/index.php?/topic/33-intrinsics-and-calibration/#entry125
    // Seems like most (all?) cameras are shipped with these calibration parameters.
    info->D[0] = -0.222609;
    info->D[1] = 0.063022;
    info->D[2] = 0.002865;
    info->D[3] = -0.001446;
    info->D[4] = 0;
    info->K[0] = info->P[0] = 104.119;
    info->K[2] = info->P[2] = 81.9494;
    info->K[4] = info->P[5] = 103.588;
    info->K[5] = info->P[6] = 59.4392;
    info->K[8] = info->P[10] = 1.0;
    info->R[0] = info->R[4] = info->R[8] = 1.0;
  }
  return info;
}

bool PMDCamboardNano::loadCalibrationData(const std::string& filename)
{
  char result[128];
  std::string cmd("LoadCalibrationData ");
  cmd.append(filename);
  throwExceptionIfFailed(pmdSourceCommand(handle_, result, sizeof(result), cmd.c_str()));
  return std::string("OK").compare(result) == 0;
}

bool PMDCamboardNano::isCalibrationDataLoaded()
{
  char loaded[8];
  throwExceptionIfFailed(pmdSourceCommand(handle_, loaded, sizeof(loaded), "IsCalibrationDataLoaded"));
  return std::string("Yes").compare(loaded) == 0;
}

std::string PMDCamboardNano::getSerialNumber()
{
  char serial[128];
  throwExceptionIfFailed(pmdSourceCommand(handle_, serial, sizeof(serial), "GetSerialNumber"));
  return std::string(serial);
}

void PMDCamboardNano::setRemoveInvalidPixels(bool remove)
{
  remove_invalid_pixels_ = remove;
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

unsigned int PMDCamboardNano::getAveragingFrames()
{
  char buffer[64];
  // First check whether averaging is enabled
  throwExceptionIfFailed(pmdProcessingCommand(handle_, buffer, 8, "GetAveraging"));
  if (std::string("On").compare(buffer) != 0)
    return 0;
  // Next get the actual number of frames in the averaging window
  throwExceptionIfFailed(pmdProcessingCommand(handle_, buffer, 8, "GetAveragingFrames"));
  unsigned int frames = 0;
  sscanf(buffer, "%u", &frames);
  return frames;
}

void PMDCamboardNano::setAveragingFrames(unsigned int frames)
{
  char cmd[64];
  // First enable/disable averaging
  sprintf(cmd, "SetAveraging %s", (frames != 0 ? "On" : "Off"));
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd));
  // Next set the number of frames (if non-zero)
  if (frames != 0)
  {
    sprintf(cmd, "SetAveragingFrames %u", frames);
    throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd));
  }
}

void PMDCamboardNano::setBilateralFilter(bool enable)
{
  char cmd[64];
  sprintf(cmd, "SetBilateralFilter %s", (enable ? "on" : "off"));
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd));
}

sensor_msgs::ImagePtr PMDCamboardNano::createImageMessage()
{
  sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
  msg->header.stamp = last_update_time_;
  msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg->height = num_rows_;
  msg->width = num_columns_;
  msg->step = msg->width * sizeof(float);
  msg->data.resize(msg->height * msg->step);
  return msg;
}

sensor_msgs::PointCloud2Ptr PMDCamboardNano::createPointCloud2Message(bool with_amplitude)
{
  sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  msg->header.stamp = last_update_time_;
  msg->is_dense = false;
  msg->is_bigendian = false;
  msg->height = num_rows_;
  msg->width = num_columns_;
  msg->fields.resize(with_amplitude ? 4 : 3);
  msg->fields[0].name = "x";
  msg->fields[0].offset = 0;
  msg->fields[0].datatype = 7;
  msg->fields[0].count = 1;
  msg->fields[1].name = "y";
  msg->fields[1].offset = 4;
  msg->fields[1].datatype = 7;
  msg->fields[1].count = 1;
  msg->fields[2].name = "z";
  msg->fields[2].offset = 8;
  msg->fields[2].datatype = 7;
  msg->fields[2].count = 1;
  if (with_amplitude)
  {
    msg->fields[3].name = "amp";
    msg->fields[3].offset = 12;
    msg->fields[3].datatype = 7;
    msg->fields[3].count = 1;
  }
  msg->point_step = sizeof(float) * msg->fields.size();
  msg->row_step = msg->width * msg->point_step;
  msg->data.resize(msg->height * msg->row_step);
  return msg;
}

void PMDCamboardNano::removeInvalidPixels(float* data, size_t step)
{
  unsigned int flags[num_pixels_];
  throwExceptionIfFailed(pmdGetFlags(handle_, flags, sizeof(flags)));
  for (size_t i = 0; i < num_pixels_; i++)
  {
    const unsigned int INVALID = PMD_FLAG_INVALID | PMD_FLAG_LOW_SIGNAL | PMD_FLAG_INCONSISTENT;
    if (flags[i] & INVALID)
      *data = std::numeric_limits<float>::quiet_NaN();
    data += step;
  }
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

