#include <string>
#include <vector>

#include "apsrc_v2x_rosbridge/apsrc_v2x_rosbridge_nodelet.hpp"

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

  bsm_pub_ = nh_.advertise<std_msgs::Header>("v2x/BasicSafetyMessage", 10, true);
  spat_pub_ = nh_.advertise<std_msgs::Header>("v2x/SPaT", 10, true);
  map_pub_ = nh_.advertise<std_msgs::Header>("v2x/MAP", 10, true);

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

bool S


} //namespace apsrc_v2x_rosbridge
PLUGINLIB_EXPORT_CLASS(apsrc_v2x_rosbridge::ApsrcV2xRosBridgeNl, nodelet::Nodelet);