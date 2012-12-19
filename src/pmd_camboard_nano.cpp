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
#include <sstream>

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
, flip_vertical_(true)
{
  throwExceptionIfFailed(pmdOpen(&handle_, PMD_PLUGIN_DIR "camboardnano", device_serial.c_str(), PMD_PLUGIN_DIR "camboardnanoproc", ""));
  if (device_serial != "" && device_serial != getSerialNumber())
  {
    pmdClose(handle_);
    throw PMDCameraNotOpenedException("Opened a wrong camera (serial number does not match the requested).");
  }
  getDirectionVectors();
  // Enable intensity images (experimental feature)
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, "AllowIntensityImage"));
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

sensor_msgs::ImagePtr PMDCamboardNano::getDistanceImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetDistances(handle_, data, msg->height * msg->step));
  processData(data, 1, false, remove_invalid_pixels_, flip_vertical_);
  return msg;
}

sensor_msgs::ImagePtr PMDCamboardNano::getDepthImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetDistances(handle_, data, msg->height * msg->step));
  processData(data, 1, true, remove_invalid_pixels_, flip_vertical_);
  return msg;
}

sensor_msgs::ImagePtr PMDCamboardNano::getAmplitudeImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetAmplitudes(handle_, data, msg->height * msg->step));
  processData(data, 1, false, remove_invalid_pixels_, flip_vertical_);
  return msg;
}

sensor_msgs::ImagePtr PMDCamboardNano::getIntensityImage()
{
  sensor_msgs::ImagePtr msg = createImageMessage();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGetIntensities(handle_, data, msg->height * msg->step));
  // Ignore remove_invalid_pixels_ setting since it does not affect intensity image
  processData(data, 1, false, false, flip_vertical_);
  return msg;
}

sensor_msgs::PointCloud2Ptr PMDCamboardNano::getPointCloud()
{
  sensor_msgs::PointCloud2Ptr msg = createPointCloud2Message();
  float* data = reinterpret_cast<float*>(&msg->data[0]);
  throwExceptionIfFailed(pmdGet3DCoordinates(handle_, data, msg->height * msg->row_step));
  processData(data + 2, 3, false, remove_invalid_pixels_, flip_vertical_);
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
  if (getProcessingPluginSetting<std::string>("GetAveraging").compare("On") == 0)
    return getProcessingPluginSetting<unsigned int>("GetAveragingFrames");
  else
    return 0;
}

void PMDCamboardNano::setAveragingFrames(unsigned int frames)
{
  if (frames != 0)
  {
    setProcessingPluginSetting("SetAveraging", true);
    setProcessingPluginSetting("SetAveragingFrames", frames);
  }
  else
  {
    setProcessingPluginSetting("SetAveraging", false);
  }
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

sensor_msgs::PointCloud2Ptr PMDCamboardNano::createPointCloud2Message()
{
  sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  msg->header.stamp = last_update_time_;
  msg->is_dense = false;
  msg->is_bigendian = false;
  msg->height = num_rows_;
  msg->width = num_columns_;
  msg->fields.resize(3);
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
  msg->point_step = sizeof(float) * msg->fields.size();
  msg->row_step = msg->width * msg->point_step;
  msg->data.resize(msg->height * msg->row_step);
  return msg;
}

void PMDCamboardNano::processData(float* data, size_t step, bool apply_direction_vectors, bool remove_invalid_pixels, bool flip_vertical)
{
  // If no processing is requested, return
  if (!apply_direction_vectors && !remove_invalid_pixels && !flip_vertical)
    return;

  // If invalid pixels removal is requested, get the flag array
  unsigned int flags[num_pixels_];
  const unsigned int INVALID = PMD_FLAG_INVALID | PMD_FLAG_LOW_SIGNAL | PMD_FLAG_INCONSISTENT;
  if (remove_invalid_pixels)
    throwExceptionIfFailed(pmdGetFlags(handle_, flags, sizeof(flags)));

  // Iterate over the whole data array row-wise with two indices going from top and from bottom
  for (size_t r1 = 0, r2 = (num_rows_ - 1) * num_columns_; r1 < r2; r1 += num_columns_, r2 -= num_columns_)
  {
    // Iterate along the upper and lower rows
    for (size_t c = 0, i1 = r1, i2 = r2; c < num_columns_; c++, i1++, i2++)
    {
      float v1;
      float v2;
      float* d1 = data + i1 * step;
      float* d2 = data + i2 * step;
      if (remove_invalid_pixels && (flags[i1] & INVALID))
        v1 = std::numeric_limits<float>::quiet_NaN();
      else if (apply_direction_vectors)
        v1 = *d1 / direction_vectors_.get()[i1];
      else
        v1 = *d1;
      if (remove_invalid_pixels && (flags[i2] & INVALID))
        v2 = std::numeric_limits<float>::quiet_NaN();
      else if (apply_direction_vectors)
        v2 = *d2 / direction_vectors_.get()[i2];
      else
        v2 = *d2;
      *d1 = flip_vertical ? v2 : v1;
      *d2 = flip_vertical ? v1 : v2;
    }
  }
}

void PMDCamboardNano::getDirectionVectors()
{
  update();
  // Get the description of raw source data
  size_t size;
  PMDDataDescription desc;
  throwExceptionIfFailed(pmdGetSourceDataSize(handle_, &size));
  throwExceptionIfFailed(pmdGetSourceDataDescription(handle_, &desc));
  // Allocate memory for the source data and processed data
  size_t num_pixels = desc.img.numRows * desc.img.numColumns;
  boost::scoped_ptr<char> data(new char[size]);
  boost::scoped_ptr<float> distances(new float[num_pixels]);
  boost::scoped_ptr<float> points(new float[num_pixels * 3]);
  // Get source data and process it
  throwExceptionIfFailed(pmdGetSourceData(handle_, data.get(), size));
  throwExceptionIfFailed(pmdCalcDistances(handle_, distances.get(), num_pixels * sizeof(float), desc, data.get()));
  throwExceptionIfFailed(pmdCalc3DCoordinates(handle_, points.get(), num_pixels * sizeof(float) * 3, desc, data.get()));
  // Allocate memory and compute direction vectors
  direction_vectors_.reset(new double[num_pixels]);
  for (size_t i = 0; i < num_pixels; i++)
    direction_vectors_.get()[i] = distances.get()[i] / points.get()[i * 3 + 2];
}

void PMDCamboardNano::setProcessingPluginSetting(const std::string& command, bool value)
{
  std::stringstream cmd;
  cmd << command << ' ' << (value ? "On" : "Off");
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd.str().c_str()));
}

void PMDCamboardNano::setProcessingPluginSetting(const std::string& command, unsigned int value)
{
  std::stringstream cmd;
  cmd << command << ' ' << value;
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd.str().c_str()));
}

void PMDCamboardNano::setProcessingPluginSetting(const std::string& command, double value)
{
  std::stringstream cmd;
  cmd << command << ' ' << std::fixed << std::setprecision(2) << value;
  throwExceptionIfFailed(pmdProcessingCommand(handle_, 0, 0, cmd.str().c_str()));
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

