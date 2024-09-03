#include <string>
#include <vector>

#include "apsrc_v2x_rosbridge/apsrc_v2x_rosbridge_nodelet.hpp"
#include "MessageFrame.h"
#include "Ieee1609Dot2Data.h"
#include "asn_SEQUENCE_OF.h"
#include "geodesy/utm.h"

#include "apsrc_msgs/BasicSafetyMessage.h"
#include "apsrc_msgs/MapData.h"
#include "apsrc_msgs/NodeXY.h"
#include "apsrc_msgs/GenericLane.h"

#include "apsrc_msgs/SignalPhaseAndTiming.h"
#include "apsrc_msgs/IntersectionState.h"
#include "apsrc_msgs/MovementState.h"
#include "apsrc_msgs/AdvisorySpeed.h"

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

  bsm_pub_        = nh_.advertise<apsrc_msgs::BasicSafetyMessage>("/v2x/BasicSafetyMessage", 10, true);
  spat_pub_       = nh_.advertise<apsrc_msgs::SignalPhaseAndTiming>("/v2x/SPaT", 10, true);
  map_pub_        = nh_.advertise<apsrc_msgs::MapData>("/v2x/MapData", 10, true);

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
  pnh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
  pnh_.param("server_port", server_port_, 1551); 
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
  ieee1609_data_ = {};
  memset(buffer_, 0, sizeof(buffer_));
  std::vector <uint8_t> returned_msg = {};
  if (received_payload.size() <= 1024) {
    std::copy(received_payload.begin(),
              received_payload.end(), 
              buffer_);
  } else {
    ROS_WARN("Received UDP is larger than buffer! (max size 1024 bytes)");
    return returned_msg;
  }

  ieee1609_rval_t_ = oer_decode(0, 
                                &asn_DEF_Ieee1609Dot2Data,
                                (void **)&ieee1609_data_, 
                                (void **)& buffer_, 
                                received_payload.size());

  if (ieee1609_rval_t_.code != RC_OK){
    ROS_WARN("Broken IEEE1609.2 encoding at byte %ld", (long)ieee1609_rval_t_.consumed);
    return returned_msg;
  }

  j2735_data_ = {};
  uint8_t *j2735_ptr;
  size_t j2735_size;
  switch (ieee1609_data_->content->present)
  {
  case Ieee1609Dot2Content_PR::Ieee1609Dot2Content_PR_signedData:
    j2735_ptr = ieee1609_data_->content->choice.signedData->tbsData->payload->data->content->choice.unsecuredData.buf;
    j2735_size = ieee1609_data_->content->choice.signedData->tbsData->payload->data->content->choice.unsecuredData.size;
    break;

  case Ieee1609Dot2Content_PR::Ieee1609Dot2Content_PR_unsecuredData:
    j2735_ptr = ieee1609_data_->content->choice.unsecuredData.buf;
    j2735_size = ieee1609_data_->content->choice.unsecuredData.size;
    break;
  default:
    return returned_msg;
  }

  j2735_rval_t_ = uper_decode(0, &asn_DEF_MessageFrame, (void **)&j2735_data_, j2735_ptr, j2735_size, 0, 0);

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
  case 27:
    if (!ApsrcV2xRosBridgeNl::RsaPublisher(j2735_data_)){
      ROS_WARN("Failed to publish received RSA...");
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
  apsrc_msgs::BasicSafetyMessage msg = {};
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.messageId = j2735_data->messageId;
  msg.value = "BasicSafetyMessage";

  // Core
  msg.BSMCore.msgCnt = j2735_data->value.choice.BasicSafetyMessage.coreData.msgCnt;
  msg.BSMCore.ID = ApsrcV2xRosBridgeNl::bufferToHex(j2735_data->value.choice.BasicSafetyMessage.coreData.id.buf, 4);
  msg.BSMCore.secMark = j2735_data->value.choice.BasicSafetyMessage.coreData.secMark;
  msg.BSMCore.Latitude = j2735_data->value.choice.BasicSafetyMessage.coreData.lat * 1e-7;
  msg.BSMCore.Longitude = j2735_data->value.choice.BasicSafetyMessage.coreData.Long * 1e-7;
  msg.BSMCore.Elevation = j2735_data->value.choice.BasicSafetyMessage.coreData.elev * 1e-1;
  msg.BSMCore.PositionAccuracy = j2735_data->value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor * 0.05;
  msg.BSMCore.OrientationAccuracy = j2735_data->value.choice.BasicSafetyMessage.coreData.accuracy.orientation * 0.05; //need to be fixed
  switch (j2735_data->value.choice.BasicSafetyMessage.coreData.transmission)
  {
  case 0:
    msg.BSMCore.TransmistionState = "Neutral";
    break;
  case 1:
    msg.BSMCore.TransmistionState = "Park";
    break;
  case 2:
    msg.BSMCore.TransmistionState = "Drive";
    break;
  case 3:
    msg.BSMCore.TransmistionState = "Reverse";
    break;
  default:
    msg.BSMCore.TransmistionState = "unavailable";
    break;
  }
  msg.BSMCore.Speed = j2735_data->value.choice.BasicSafetyMessage.coreData.speed * 0.02;
  msg.BSMCore.Speed = j2735_data->value.choice.BasicSafetyMessage.coreData.speed * 0.02;
  msg.BSMCore.Heading = j2735_data->value.choice.BasicSafetyMessage.coreData.heading * 0.0125;
  msg.BSMCore.SteeringWheelAngle = j2735_data->value.choice.BasicSafetyMessage.coreData.heading * 1.5;
  msg.BSMCore.AccelerationSet4Way.latitude = j2735_data->value.choice.BasicSafetyMessage.coreData.accelSet.lat * 0.01;
  msg.BSMCore.AccelerationSet4Way.longitude = j2735_data->value.choice.BasicSafetyMessage.coreData.accelSet.Long * 0.01;
  msg.BSMCore.AccelerationSet4Way.vertical = j2735_data->value.choice.BasicSafetyMessage.coreData.accelSet.vert * 0.02;
  msg.BSMCore.AccelerationSet4Way.yaw = j2735_data->value.choice.BasicSafetyMessage.coreData.accelSet.yaw * 0.01;
  if (j2735_data->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[0] == 0){
    msg.BSMCore.BrakeSystemStatus.BrakeAppliedStatus.available = true;
    msg.BSMCore.BrakeSystemStatus.BrakeAppliedStatus.LeftFront = j2735_data->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[1] ? true:false;
    msg.BSMCore.BrakeSystemStatus.BrakeAppliedStatus.LeftRear = j2735_data->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[2] ? true:false;
    msg.BSMCore.BrakeSystemStatus.BrakeAppliedStatus.RightFront = j2735_data->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[3] ? true:false;
    msg.BSMCore.BrakeSystemStatus.BrakeAppliedStatus.RightRear = j2735_data->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[4] ? true:false;
  }
  msg.BSMCore.VehicleSize.VehicleLength = j2735_data->value.choice.BasicSafetyMessage.coreData.size.length * 0.01;
  msg.BSMCore.VehicleSize.VehicleWidth = j2735_data->value.choice.BasicSafetyMessage.coreData.size.width * 0.01;

  // Part II
  /*
  code
  */

  ApsrcV2xRosBridgeNl::bsm_pub_.publish(msg);
  return true;
}

bool ApsrcV2xRosBridgeNl::MapPublisher(const MessageFrame_t *j2735_data)
{
  apsrc_msgs::MapData msg = {};

  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.messageId = j2735_data->messageId;
  msg.value = "MapData";
  msg.msgIssueRevision = j2735_data->value.choice.MapData.msgIssueRevision;

  IntersectionGeometry_t intersection_geometry = **j2735_data->value.choice.MapData.intersections->list.array;
  msg.Intersections.IntersectionGeometry.id = intersection_geometry.id.id;
  msg.Intersections.IntersectionGeometry.revision = intersection_geometry.revision;
  msg.Intersections.IntersectionGeometry.refPoint.latitude = intersection_geometry.refPoint.lat * 1e-7;
  msg.Intersections.IntersectionGeometry.refPoint.longitude = intersection_geometry.refPoint.Long * 1e-7;
  

  const asn_anonymous_sequence_ *list = _A_CSEQUENCE_FROM_VOID(&(intersection_geometry.laneSet));
  apsrc_msgs::IntersectionGeometry& x = msg.Intersections.IntersectionGeometry;
  x.laneSet.resize(list->count, {});
  GenericLane_t GenericLane[list->count];
  for (int i = 0; i < list->count; ++i) {
    void *memb_ptr = list->array[i];
    std::memcpy(&GenericLane[i], memb_ptr, sizeof(GenericLane[0]));
    x.laneSet[i].laneID = GenericLane[i].laneID;
    x.laneSet[i].laneAttributes.directionalUse.ingressPath = GenericLane[i].laneAttributes.directionalUse.buf[0] ? true:false;
    x.laneSet[i].laneAttributes.directionalUse.egressPath = GenericLane[i].laneAttributes.directionalUse.buf[1] ? true:false;
    
    const asn_anonymous_sequence_ *child_Node_list = _A_CSEQUENCE_FROM_VOID(&(GenericLane[i].nodeList.choice.nodes));
    if (child_Node_list == nullptr){
      continue;
    }

    apsrc_msgs::NodeList& y = x.laneSet[i].nodeList;
    x.laneSet[i].nodeList.NodeSetXY.resize(child_Node_list->count, {});
    NodeXY_t NodeXY[child_Node_list->count];
    for (int j = 0; j < child_Node_list->count; ++j){
      void *chid_ptr = child_Node_list->array[j];
      std::memcpy(&NodeXY[j], chid_ptr, sizeof(NodeXY[0]));
      y.NodeSetXY[j].delta.X = NodeXY[j].delta.choice.node_XY6.x * 0.1;
      y.NodeSetXY[j].delta.y = NodeXY[j].delta.choice.node_XY6.y * 0.1;
    }
    
    const asn_anonymous_sequence_ *child_To_list = _A_CSEQUENCE_FROM_VOID(&(GenericLane[i].connectsTo->list.array));
    if (child_To_list == nullptr){
      continue;
    }

    apsrc_msgs::ConnectsTo& z = x.laneSet[i].connectsTo;
    x.laneSet[i].connectsTo.ConnectsToList.resize(child_To_list->count, {});
    Connection_t Connection[child_To_list->count];
    for (int k = 0; k < child_To_list->count; ++k){
      void *chid_ptr_2 = child_To_list->array[k];
      std::memcpy(&Connection[k], chid_ptr_2, sizeof(Connection[0]));
      z.ConnectsToList[k].ConnectingLane.lane = Connection[k].connectingLane.lane;
      z.ConnectsToList[k].SignalGroup.SignalGroupID = *Connection[k].signalGroup;
    }
  }
  
  ApsrcV2xRosBridgeNl::map_pub_.publish(msg);
  return true;
}

bool ApsrcV2xRosBridgeNl::SPaTPublisher(const MessageFrame_t *j2735_data)
{
  apsrc_msgs::SignalPhaseAndTiming msg = {};

  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.value = "SPaT";
  msg.messageId = j2735_data->messageId;

  const asn_anonymous_sequence_ *intersection_list = _A_CSEQUENCE_FROM_VOID(&(j2735_data->value.choice.SPAT.intersections));
  msg.intersections.intersectionState.resize(intersection_list->count, {});
  IntersectionState_t intersection_state[intersection_list->count] = {};
  for(int i = 0; i < intersection_list->count; ++i){
    void *intersection_ptr = intersection_list->array[i];
    std::memcpy(&intersection_state[i], intersection_ptr, sizeof(intersection_state[0]));
    apsrc_msgs::IntersectionState& x = msg.intersections.intersectionState[i];
    x.messageId = intersection_state[i].id.id;
    x.revision = intersection_state[i].revision;
    std::memcpy(&x.status, &intersection_state[i].status, 13);
    
    const asn_anonymous_sequence_ *states_list = _A_CSEQUENCE_FROM_VOID(&(intersection_state[i].states));
    x.states.movementState.resize(states_list->count, {});
    MovementState_t movement_state[states_list->count] = {};
    for(int j = 0; j < states_list->count; ++j){
      void *ms_ptr = states_list->array[j];
      std::memcpy(&movement_state[j], ms_ptr, sizeof(movement_state[0]));
      x.states.movementState[j].signalGroup = movement_state[j].signalGroup;

      const asn_anonymous_sequence_ *movement_event_list = _A_CSEQUENCE_FROM_VOID(&(movement_state[j].state_time_speed));
      x.states.movementState[j].stateTimeSpeed.resize(movement_event_list->count, {});
      MovementEvent_t movement_event[movement_event_list->count] = {};
      for (int k = 0; k < movement_event_list->count; ++k){
        void *mv_ptr = movement_event_list->array[k];
        std::memcpy(&movement_event[k], mv_ptr, sizeof(movement_event[0]));
        x.states.movementState[j].stateTimeSpeed[k].eventState.state = movement_event[k].eventState;
        x.states.movementState[j].stateTimeSpeed[k].eventState.label = ApsrcV2xRosBridgeNl::MovementPhaseState_(movement_event[k].eventState);
        x.states.movementState[j].stateTimeSpeed[k].minEndTime = movement_event[k].timing->minEndTime * 0.1;

        const asn_anonymous_sequence_ *advisory_speed_list = _A_CSEQUENCE_FROM_VOID(&(movement_event[k].speeds->list.array));
        if (advisory_speed_list !=nullptr){
          x.states.movementState[j].stateTimeSpeed[k].speeds.resize(advisory_speed_list->count, {});
          AdvisorySpeed_t advisory_speed[advisory_speed_list->count] = {};
          for(int l = 0; l < advisory_speed_list->count; ++l){
            void *as_ptr = advisory_speed_list->array[l];
            std::memcpy(&advisory_speed[l], as_ptr, sizeof(advisory_speed[0]));
            apsrc_msgs::AdvisorySpeed& y = x.states.movementState[j].stateTimeSpeed[k].speeds[l];
            y.type.type = advisory_speed[l].type;
            y.type.label = ApsrcV2xRosBridgeNl::AdvisorySpeedType_(advisory_speed[l].type);
            y.speed = *advisory_speed[l].speed * 0.1;
            y.distance = *advisory_speed[l].distance;
          }
        }    
      }
    }
  }
  ApsrcV2xRosBridgeNl::spat_pub_.publish(msg);
  return true;
}

bool ApsrcV2xRosBridgeNl::RsaPublisher(const MessageFrame_t *j2735_data)
{
  apsrc_msgs::BasicSafetyMessage msg = {};
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.messageId = j2735_data->messageId;
  msg.value = "RoadSideAlert";

  msg.BSMCore.Latitude = j2735_data->value.choice.RoadSideAlert.position->lat * 1e-7;
  msg.BSMCore.Longitude = j2735_data->value.choice.RoadSideAlert.position->Long * 1e-7;

  ApsrcV2xRosBridgeNl::bsm_pub_.publish(msg);
  return true;
}


} //namespace apsrc_v2x_rosbridge
PLUGINLIB_EXPORT_CLASS(apsrc_v2x_rosbridge::ApsrcV2xRosBridgeNl,
                      nodelet::Nodelet);