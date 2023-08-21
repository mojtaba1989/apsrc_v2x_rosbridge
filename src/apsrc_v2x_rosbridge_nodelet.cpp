#include <string>
#include <vector>

#include "apsrc_v2x_rosbridge/apsrc_v2x_rosbridge_nodelet.hpp"
#include "Ieee1609Dot2Data.h"
#include "MessageFrame.h"
#include "apsrc_v2x_rosbridge/BasicSafetyMessage.h"

namespace apsrc_v2x_rosbridge
{

ApsrcV2xRosBridgeNl::ApsrcV2xRosBridgeNl()
{
}

ApsrcV2xRosBridgeNl::~ApsrcV2xRosBridgeNl()
{
  if(udp_server_running_){
    udp_server_.stop();
    udp_server_thread_.join();
  }
};

void ApsrcV2xRosBridgeNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  bsm_pub_ = nh_.advertise<apsrc_v2x_rosbridge::BasicSafetyMessage>("/v2x/BasicSafetyMessage", 10, true);
  spat_pub_ = nh_.advertise<apsrc_v2x_rosbridge::BasicSafetyMessage>("/v2x/SPaT", 10, true);
  map_pub_ = nh_.advertise<apsrc_v2x_rosbridge::BasicSafetyMessage>("/v2x/MAP", 10, true);

  if (startServer()){
    udp_server_running_ = true;
    udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
  } else {
    udp_server_running_ = false;
    ros::requestShutdown();
  }
}

void ApsrcV2xRosBridgeNl::loadParams()
{
  pnh_.param<std::string>("/v2x_rosbridge/server_ip", server_ip_, "127.0.0.1");
  pnh_.param("/v2x_rosbridge/server_port", server_port_, 1551); 
  ROS_INFO("Parameters Loaded");
}

bool ApsrcV2xRosBridgeNl::startServer()
{
  AS::Network::ReturnStatuses status = udp_server_.open(server_ip_, server_port_);
  if (status != AS::Network::ReturnStatuses::OK){
    ROS_ERROR("Could not start UDP server: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  } else {
    ROS_INFO("UDP server started at %s (%s)", server_ip_.c_str(), std::to_string(server_port_).c_str());
    udp_server_.registerReceiveHandler(std::bind(&ApsrcV2xRosBridgeNl::handleServerResponse, this, std::placeholders::_1));
    return true;
  }
}

std::vector<uint8_t> ApsrcV2xRosBridgeNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  std::vector <uint8_t> returned_msg = {}; 
  ieee1609_data_ = 0;
  ieee1609_rval_t_ = oer_decode(0, 
                                 &asn_DEF_Ieee1609Dot2Data,
                                 (void **)&ieee1609_data_, 
                                 (void **)&received_payload, 
                                 sizeof(received_payload));

  if (ieee1609_rval_t_.code != RC_OK){
    ROS_WARN("Broken IEEE1609.2 encoding at byte %ld", (long)ieee1609_rval_t_.consumed);
    return returned_msg;
  }
  
  j2735_rval_t_ = uper_decode(0, 
                              &asn_DEF_MessageFrame, 
                              (void **)&j2735_data_, 
                              ieee1609_data_->content->choice.signedData->tbsData->payload->data->content->choice.unsecuredData.buf, 
                              ieee1609_data_->content->choice.signedData->tbsData->payload->data->content->choice.unsecuredData.size, 0, 0);

  if (j2735_rval_t_.code != RC_OK){
    ROS_WARN("Broken J2735 encoding at byte %ld", (long)j2735_rval_t_.consumed);
    return returned_msg;
  }

  switch (j2735_data_->messageId)
  {
  case 20:
    if (!ApsrcV2xRosBridgeNl::BasicSafetyMessagePublisher(j2735_data_)){
      ROS_WARN("Failed to publish received Basic Safety Message...");
    }
    break;
  case 19:
    if (!ApsrcV2xRosBridgeNl::SPaTPublisher(j2735_data_)){
      ROS_WARN("Failed to publish received SPaT...");
    }
    break;
  case 18:
    if (!ApsrcV2xRosBridgeNl::MapPublisher(j2735_data_)){
      ROS_WARN("Failed to publish received MAP...");
    }
    break;
  default:
    ROS_WARN("Failed to identify J2735 message...");
    break;
  }
  return returned_msg;
}

bool ApsrcV2xRosBridgeNl::BasicSafetyMessagePublisher(const MessageFrame_t *j2735_data)
{
  apsrc_v2x_rosbridge::BasicSafetyMessage msg = {};
  msg.messageId = j2735_data->messageId;
  ApsrcV2xRosBridgeNl::bsm_pub_.publish(msg);
  return true;
}




} //namespace apsrc_v2x_rosbridge
PLUGINLIB_EXPORT_CLASS(apsrc_v2x_rosbridge::ApsrcV2xRosBridgeNl,
                      nodelet::Nodelet);