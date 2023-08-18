#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apsrc_v2x_rosbridge_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "apsrc_v2x_rosbridge/apsrc_v2x_rosbridge_nodelet", remap, nargv);
  ros::spin();
  return 0;
}