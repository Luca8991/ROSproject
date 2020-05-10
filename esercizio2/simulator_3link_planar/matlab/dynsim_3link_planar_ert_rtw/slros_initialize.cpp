#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "dynsim_3link_planar";

// For Block dynsim_3link_planar/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray> Sub_dynsim_3link_planar_16;

// For Block dynsim_3link_planar/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_3link_planar_sensor_msgs_JointState> Pub_dynsim_3link_planar_22;

// For Block dynsim_3link_planar/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock> Pub_dynsim_3link_planar_50;

// For Block dynsim_3link_planar/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_3link_planar_sensor_msgs_JointState> Pub_dynsim_3link_planar_124;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_112;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_113;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_117;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_118;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_125;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_126;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

