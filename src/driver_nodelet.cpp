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

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>

#include <pmd_camboard_nano/PMDConfig.h>
#include "pmd_camboard_nano.h"
#include "pmd_exceptions.h"

namespace pmd_camboard_nano
{

class DriverNodelet : public nodelet::Nodelet
{

public:

  virtual ~DriverNodelet()
  {
    // Make sure we interrupt initialization (if it happened to still execute).
    init_thread_.interrupt();
    init_thread_.join();
  }

private:

  virtual void onInit()
  {
    // We will be retrying to open camera until it is open, which may block the
    // thread. Nodelet::onInit() should not block, hence spawning a new thread
    // to do initialization.
    init_thread_ = boost::thread(boost::bind(&DriverNodelet::onInitImpl, this));
  }

  void onInitImpl()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pn = getPrivateNodeHandle();

    // Retrieve parameters from server
    std::string device_serial;
    double open_camera_retry_period;
    double update_rate;
    pn.param<std::string>("depth_frame_id", depth_frame_id_, "/camera_depth_optical_frame");
    pn.param<std::string>("device_serial", device_serial, "");
    pn.param<double>("open_camera_retry_period", open_camera_retry_period, 3);
    pn.param<double>("update_rate", update_rate, 30);

    // Open camera
    while (!camera_)
    {
      try
      {
        camera_ = boost::make_shared<PMDCamboardNano>(device_serial);
        NODELET_INFO("Opened PMD camera with serial number \"%s\"", camera_->getSerialNumber().c_str());
        if (camera_->isCalibrationDataLoaded())
          NODELET_INFO("Calibration data loaded.");
        else
          NODELET_WARN("Calibration data was not loaded.");
        camera_->update();
        camera_info_ = camera_->getCameraInfo();
      }
      catch (PMDCameraNotOpenedException& e)
      {
        if (device_serial != "")
          NODELET_INFO("Unable to open PMD camera with serial number %s...", device_serial.c_str());
        else
          NODELET_INFO("Unable to open PMD camera...");
      }
      boost::this_thread::sleep(boost::posix_time::seconds(open_camera_retry_period));
    }

    // Advertise topics
    ros::NodeHandle depth_nh(nh, "depth");
    image_transport::ImageTransport depth_it(depth_nh);
    depth_publisher_ = depth_it.advertiseCamera("image", 1);
    ros::NodeHandle amplitude_nh(nh, "amplitude");
    image_transport::ImageTransport amplitude_it(amplitude_nh);
    amplitude_publisher_ = amplitude_it.advertiseCamera("image", 1);

    // Setup periodic callback to get new data from the camera
    update_timer_ = nh.createTimer(ros::Rate(30).expectedCycleTime(), &DriverNodelet::updateCallback, this);

    // Setup dynamic reconfigure server
    reconfigure_server_.reset(new ReconfigureServer(pn));
    reconfigure_server_->setCallback(boost::bind(&DriverNodelet::reconfigureCallback, this, _1, _2));
  }

  void updateCallback(const ros::TimerEvent& event)
  {
    // Download the most recent data from the device
    camera_->update();
    // Get new depth and amplitude data
    sensor_msgs::ImagePtr depth = camera_->getDepthImage();
    sensor_msgs::ImagePtr amplitude = camera_->getAmplitudeImage();
    // Fill in frame ids and update timestamp in camera info
    camera_info_->header.frame_id = depth_frame_id_;
    depth->header.frame_id = depth_frame_id_;
    amplitude->header.frame_id = depth_frame_id_;
    camera_info_->header.stamp = depth->header.stamp;
    // Publish both
    depth_publisher_.publish(depth, camera_info_);
    amplitude_publisher_.publish(amplitude, camera_info_);
  }

  void reconfigureCallback(pmd_camboard_nano::PMDConfig &config, uint32_t level)
  {
    camera_->setRemoveInvalidPixels(config.remove_invalid_pixels);
    config.integration_time = camera_->setIntegrationTime(config.integration_time);
    camera_->setAveragingFrames(config.averaging_frames);
    camera_->setBilateralFilter(config.bilateral_filter);
    config_ = config;
  }

private:

  PMDCamboardNano::Ptr camera_;
  boost::thread init_thread_;
  ros::Timer update_timer_;
  image_transport::CameraPublisher depth_publisher_;
  image_transport::CameraPublisher amplitude_publisher_;
  std::string depth_frame_id_;
  typedef dynamic_reconfigure::Server<pmd_camboard_nano::PMDConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  pmd_camboard_nano::PMDConfig config_;
  sensor_msgs::CameraInfoPtr camera_info_;

};

}

// Register as a nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (pmd_camboard_nano, driver, pmd_camboard_nano::DriverNodelet, nodelet::Nodelet);

