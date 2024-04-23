#ifndef APSRC_V2X_ROSBRIDGENL_H
#define APSRC_V2X_ROSBRIDGENL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <network_interface/udp_server.h>
#include <thread>
#include <mutex>

#include "MessageFrame.h"
#include "Ieee1609Dot2Data.h"


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

  // Message handlers
  bool BasicSafetyMessagePublisher(const MessageFrame_t *j2735_data);
  bool SPaTPublisher(const MessageFrame_t *j2735_data);
  bool MapPublisher(const MessageFrame_t *j2735_data);

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
  Ieee1609Dot2Data_t *ieee1609_data_ = 0;
  const MessageFrame_t *j2735_data_ = 0;
  asn_dec_rval_t ieee1609_rval_t_, j2735_rval_t_;
  size_t size_;
  char buffer_[1024];

  // Params
  std::string server_ip_;
  int server_port_;

  // Plugins
  std::string bufferToHex(const unsigned char* buffer, std::size_t size)
  {
    std::ostringstream oss;
    oss << std::hex; // Set the stream to output in hexadecimal
    for (std::size_t i = 0; i < size; ++i) {
        oss << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]);
    }
    return oss.str();
  }

  std::string AdvisorySpeedType_(uint8_t type)
  {
    std::string label = "";
    switch (type)
    {
    case 0: label = "none"; break;
    case 1: label = "greenwave"; break;
    case 2: label = "ecoDrive"; break;
    case 3: label = "transit"; break;
    default: label = "unknown"; break;
    }
    return label;
  }

  std::string MovementPhaseState_(uint8_t state)
  {
    std::string label = "";
    switch (state)
    {
    case 0: label = "unavailable"; break;
    case 1: label = "dark"; break;
    case 2: label = "stop-Then-Proceed"; break;
    case 3: label = "stop-And-Remain"; break;
    case 4: label = "pre-Movement"; break;
    case 5: label = "permissive-Movement-Allowed"; break;
    case 6: label = "protected-Movement-Allowed"; break;
    case 7: label = "permissive-clearance"; break;
    case 8: label = "protected-clearance"; break;
    case 9: label = "caution-Conflicting-Traffic"; break;
    default: label = "unknown"; break;
    }
    return label;
  }

};
}
#endif // APSRC_V2X_ROSBRIDGENL_H1