#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block dynsim_3link_planar/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray> Sub_dynsim_3link_planar_16;

// For Block dynsim_3link_planar/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_3link_planar_sensor_msgs_JointState> Pub_dynsim_3link_planar_22;

// For Block dynsim_3link_planar/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock> Pub_dynsim_3link_planar_50;

// For Block dynsim_3link_planar/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_3link_planar_sensor_msgs_JointState> Pub_dynsim_3link_planar_124;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_112;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_113;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_117;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_118;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_125;

// For Block dynsim_3link_planar/2link_planar robot dynamic model/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_3link_planar_126;

void slros_node_init(int argc, char** argv);

#endif
