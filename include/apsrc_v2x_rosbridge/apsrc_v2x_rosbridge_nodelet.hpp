#ifndef APSRC_V2X_ROSBRIDGENL_H
#define APSRC_V2X_ROSBRIDGEnL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <network_interface/udp_server.h>
#include <thread>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>

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
  void loadParams();

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // UDP server callbacks
  std::vector<uint8_t> handleServerResponse(const std::vector<uint8_t>& received_payload);

  // Publisher
  ros::Publisher bsm_pub_;
  ros::Publisher spat_pub_;
  ros::Publisher map_pub_;

  // Util
  bool startServer();

  // Internal State
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  bool udp_server_running_ = false;

  // Params
  std::string server_ip_;
  int server_port_;
  
};
}
#endif // APSRC_V2X_ROSBRIDGENL_H