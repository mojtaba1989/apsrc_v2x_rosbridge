#ifndef APSRC_V2X_ROSBRIDGENL_H
#define APSRC_V2X_ROSBRIDGEnL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <network_interface/udp_server.h>
#include <thread>
#include <mutex>

namespace apsrc_v2x_rosbridge
{
class ApsrcV2xRosBridgeNl : public nodelet::Nodelet
{
public:
  ApsrcV2xRosBridgeNl();
  ~ApsrcV2xRosBridgeNl();

private:
  // Init
  virtual void onInit();

};
}
#endif // APSRC_V2X_ROSBRIDGENL_H