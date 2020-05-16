/*
 * kinsim_3link_planar_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "kinsim_3link_planar".
 *
 * Model version              : 1.118
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C++ source code generated on : Sun May 10 23:20:12 2020
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_kinsim_3link_planar_types_h_
#define RTW_HEADER_kinsim_3link_planar_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_std_msgs_String_

typedef struct {
  uint8_T Data[128];
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_kinsim_3link_planar_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_kinsim_3link_planar_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_kinsim_3link_planar_ros_time_Time Stamp;
} SL_Bus_kinsim_3link_planar_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_sensor_msgs_JointState_

typedef struct {
  SL_Bus_kinsim_3link_planar_std_msgs_String Name[16];
  SL_Bus_ROSVariableLengthArrayInfo Name_SL_Info;
  real_T Position[128];
  SL_Bus_ROSVariableLengthArrayInfo Position_SL_Info;
  real_T Velocity[128];
  SL_Bus_ROSVariableLengthArrayInfo Velocity_SL_Info;
  real_T Effort[128];
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;
  SL_Bus_kinsim_3link_planar_std_msgs_Header Header;
} SL_Bus_kinsim_3link_planar_sensor_msgs_JointState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_rosgraph_msgs_Clock_

typedef struct {
  SL_Bus_kinsim_3link_planar_ros_time_Time Clock_;
} SL_Bus_kinsim_3link_planar_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_ros_time_Duration_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_ros_time_Duration_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_kinsim_3link_planar_ros_time_Duration;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_JointTrajectoryPoint_1herr0_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_3link_planar_JointTrajectoryPoint_1herr0_

typedef struct {
  real_T Positions[128];
  SL_Bus_ROSVariableLengthArrayInfo Positions_SL_Info;
  real_T Velocities[128];
  SL_Bus_ROSVariableLengthArrayInfo Velocities_SL_Info;
  real_T Accelerations[128];
  SL_Bus_ROSVariableLengthArrayInfo Accelerations_SL_Info;
  real_T Effort[128];
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;
  SL_Bus_kinsim_3link_planar_ros_time_Duration TimeFromStart;
} SL_Bus_kinsim_3link_planar_JointTrajectoryPoint_1herr0;

#endif

#ifndef struct_tag_rkSooZHJZnr3Dpfu1LNqfH
#define struct_tag_rkSooZHJZnr3Dpfu1LNqfH

struct tag_rkSooZHJZnr3Dpfu1LNqfH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_rkSooZHJZnr3Dpfu1LNqfH*/

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct tag_rkSooZHJZnr3Dpfu1LNqfH ros_slros_internal_block_Publ_T;

#endif                               /*typedef_ros_slros_internal_block_Publ_T*/

#ifndef struct_tag_9SewJ4y3IXNs5GrZti8qkG
#define struct_tag_9SewJ4y3IXNs5GrZti8qkG

struct tag_9SewJ4y3IXNs5GrZti8qkG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_9SewJ4y3IXNs5GrZti8qkG*/

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct tag_9SewJ4y3IXNs5GrZti8qkG ros_slros_internal_block_Subs_T;

#endif                               /*typedef_ros_slros_internal_block_Subs_T*/

#ifndef struct_tag_KSdGoEc2IyOHz4CLi4rcCD
#define struct_tag_KSdGoEc2IyOHz4CLi4rcCD

struct tag_KSdGoEc2IyOHz4CLi4rcCD
{
  int32_T __dummy;
};

#endif                                 /*struct_tag_KSdGoEc2IyOHz4CLi4rcCD*/

#ifndef typedef_e_robotics_slcore_internal_bl_T
#define typedef_e_robotics_slcore_internal_bl_T

typedef struct tag_KSdGoEc2IyOHz4CLi4rcCD e_robotics_slcore_internal_bl_T;

#endif                               /*typedef_e_robotics_slcore_internal_bl_T*/

#ifndef struct_tag_PzhaB0v2Sx4ikuHWZx5WUB
#define struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

struct tag_PzhaB0v2Sx4ikuHWZx5WUB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                 /*struct_tag_PzhaB0v2Sx4ikuHWZx5WUB*/

#ifndef typedef_ros_slros_internal_block_GetP_T
#define typedef_ros_slros_internal_block_GetP_T

typedef struct tag_PzhaB0v2Sx4ikuHWZx5WUB ros_slros_internal_block_GetP_T;

#endif                               /*typedef_ros_slros_internal_block_GetP_T*/

/* Parameters (default storage) */
typedef struct P_kinsim_3link_planar_T_ P_kinsim_3link_planar_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_kinsim_3link_planar_T RT_MODEL_kinsim_3link_planar_T;

#endif                             /* RTW_HEADER_kinsim_3link_planar_types_h_ */
