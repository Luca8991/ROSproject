/*
 * dynsim_3link_planar_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "dynsim_3link_planar".
 *
 * Model version              : 1.121
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C++ source code generated on : Sun May 10 23:27:55 2020
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_dynsim_3link_planar_types_h_
#define RTW_HEADER_dynsim_3link_planar_types_h_
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

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_String_

typedef struct {
  uint8_T Data[128];
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_dynsim_3link_planar_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_dynsim_3link_planar_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_dynsim_3link_planar_ros_time_Time Stamp;
} SL_Bus_dynsim_3link_planar_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_sensor_msgs_JointState_

typedef struct {
  SL_Bus_dynsim_3link_planar_std_msgs_String Name[16];
  SL_Bus_ROSVariableLengthArrayInfo Name_SL_Info;
  real_T Position[128];
  SL_Bus_ROSVariableLengthArrayInfo Position_SL_Info;
  real_T Velocity[128];
  SL_Bus_ROSVariableLengthArrayInfo Velocity_SL_Info;
  real_T Effort[128];
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;
  SL_Bus_dynsim_3link_planar_std_msgs_Header Header;
} SL_Bus_dynsim_3link_planar_sensor_msgs_JointState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock_

typedef struct {
  SL_Bus_dynsim_3link_planar_ros_time_Time Clock_;
} SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension_

typedef struct {
  uint8_T Label[128];
  SL_Bus_ROSVariableLengthArrayInfo Label_SL_Info;
  uint32_T Size;
  uint32_T Stride;
} SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayLayout_

typedef struct {
  uint32_T DataOffset;
  SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension Dim[16];
  SL_Bus_ROSVariableLengthArrayInfo Dim_SL_Info;
} SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayLayout;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray_

typedef struct {
  real_T Data[128];
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
  SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayLayout Layout;
} SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_UKWjnfyEqJkOH7wgOaFOrC_
#define DEFINED_TYPEDEF_FOR_struct_UKWjnfyEqJkOH7wgOaFOrC_

typedef struct {
  real_T NameLength;
  uint8_T Name[12];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[3];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
} struct_UKWjnfyEqJkOH7wgOaFOrC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_0MUKHDZLECrZN0NMjSzIh_
#define DEFINED_TYPEDEF_FOR_struct_0MUKHDZLECrZN0NMjSzIh_

typedef struct {
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[13];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
} struct_0MUKHDZLECrZN0NMjSzIh;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_R50nRNqeXJGDv1TawD6FcD_
#define DEFINED_TYPEDEF_FOR_struct_R50nRNqeXJGDv1TawD6FcD_

typedef struct {
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[6];
  real_T VelocityDoFMap[6];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[6];
  struct_UKWjnfyEqJkOH7wgOaFOrC Bodies[4];
  struct_0MUKHDZLECrZN0NMjSzIh Joints[4];
} struct_R50nRNqeXJGDv1TawD6FcD;

#endif

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

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_char_T*/

#ifndef typedef_emxArray_char_T_dynsim_3link__T
#define typedef_emxArray_char_T_dynsim_3link__T

typedef struct emxArray_char_T emxArray_char_T_dynsim_3link__T;

#endif                               /*typedef_emxArray_char_T_dynsim_3link__T*/

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T_dynsim_3link__T
#define typedef_emxArray_real_T_dynsim_3link__T

typedef struct emxArray_real_T emxArray_real_T_dynsim_3link__T;

#endif                               /*typedef_emxArray_real_T_dynsim_3link__T*/

#ifndef struct_tag_s0LhUkJiqaA04TI4t533xfH
#define struct_tag_s0LhUkJiqaA04TI4t533xfH

struct tag_s0LhUkJiqaA04TI4t533xfH
{
  real_T NameLength;
  uint8_T Name[12];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[3];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 /*struct_tag_s0LhUkJiqaA04TI4t533xfH*/

#ifndef typedef_s0LhUkJiqaA04TI4t533xfH_dynsi_T
#define typedef_s0LhUkJiqaA04TI4t533xfH_dynsi_T

typedef struct tag_s0LhUkJiqaA04TI4t533xfH s0LhUkJiqaA04TI4t533xfH_dynsi_T;

#endif                               /*typedef_s0LhUkJiqaA04TI4t533xfH_dynsi_T*/

#ifndef struct_tag_skkEmNT0tN2ls5oE414CacG
#define struct_tag_skkEmNT0tN2ls5oE414CacG

struct tag_skkEmNT0tN2ls5oE414CacG
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[13];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 /*struct_tag_skkEmNT0tN2ls5oE414CacG*/

#ifndef typedef_skkEmNT0tN2ls5oE414CacG_dynsi_T
#define typedef_skkEmNT0tN2ls5oE414CacG_dynsi_T

typedef struct tag_skkEmNT0tN2ls5oE414CacG skkEmNT0tN2ls5oE414CacG_dynsi_T;

#endif                               /*typedef_skkEmNT0tN2ls5oE414CacG_dynsi_T*/

#ifndef struct_tag_pGgszObO16I6TGXaEMnuXB
#define struct_tag_pGgszObO16I6TGXaEMnuXB

struct tag_pGgszObO16I6TGXaEMnuXB
{
  real_T f1[36];
};

#endif                                 /*struct_tag_pGgszObO16I6TGXaEMnuXB*/

#ifndef typedef_f_cell_wrap_dynsim_3link_plan_T
#define typedef_f_cell_wrap_dynsim_3link_plan_T

typedef struct tag_pGgszObO16I6TGXaEMnuXB f_cell_wrap_dynsim_3link_plan_T;

#endif                               /*typedef_f_cell_wrap_dynsim_3link_plan_T*/

#ifndef struct_tag_QsLFVUgAtzQUBY1V99co0D
#define struct_tag_QsLFVUgAtzQUBY1V99co0D

struct tag_QsLFVUgAtzQUBY1V99co0D
{
  emxArray_char_T_dynsim_3link__T *Type;
  emxArray_real_T_dynsim_3link__T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 /*struct_tag_QsLFVUgAtzQUBY1V99co0D*/

#ifndef typedef_c_rigidBodyJoint_dynsim_3link_T
#define typedef_c_rigidBodyJoint_dynsim_3link_T

typedef struct tag_QsLFVUgAtzQUBY1V99co0D c_rigidBodyJoint_dynsim_3link_T;

#endif                               /*typedef_c_rigidBodyJoint_dynsim_3link_T*/

#ifndef struct_tag_IYTryndM9hCl2aQvRVOEpC
#define struct_tag_IYTryndM9hCl2aQvRVOEpC

struct tag_IYTryndM9hCl2aQvRVOEpC
{
  real_T Index;
  c_rigidBodyJoint_dynsim_3link_T JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
};

#endif                                 /*struct_tag_IYTryndM9hCl2aQvRVOEpC*/

#ifndef typedef_n_robotics_manip_internal_Rig_T
#define typedef_n_robotics_manip_internal_Rig_T

typedef struct tag_IYTryndM9hCl2aQvRVOEpC n_robotics_manip_internal_Rig_T;

#endif                               /*typedef_n_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_Y04n94zUMq8rhLX3OxwtfD
#define struct_tag_Y04n94zUMq8rhLX3OxwtfD

struct tag_Y04n94zUMq8rhLX3OxwtfD
{
  c_rigidBodyJoint_dynsim_3link_T JointInternal;
};

#endif                                 /*struct_tag_Y04n94zUMq8rhLX3OxwtfD*/

#ifndef typedef_o_robotics_manip_internal_Rig_T
#define typedef_o_robotics_manip_internal_Rig_T

typedef struct tag_Y04n94zUMq8rhLX3OxwtfD o_robotics_manip_internal_Rig_T;

#endif                               /*typedef_o_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_dcOwaIw5scRYPCPlMVVDiF
#define struct_tag_dcOwaIw5scRYPCPlMVVDiF

struct tag_dcOwaIw5scRYPCPlMVVDiF
{
  real_T NumBodies;
  o_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  n_robotics_manip_internal_Rig_T *Bodies[3];
  real_T VelocityNumber;
  real_T PositionDoFMap[6];
  real_T VelocityDoFMap[6];
};

#endif                                 /*struct_tag_dcOwaIw5scRYPCPlMVVDiF*/

#ifndef typedef_p_robotics_manip_internal_Rig_T
#define typedef_p_robotics_manip_internal_Rig_T

typedef struct tag_dcOwaIw5scRYPCPlMVVDiF p_robotics_manip_internal_Rig_T;

#endif                               /*typedef_p_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_2xCNoiC3ymIvCs3oXr0EMB
#define struct_tag_2xCNoiC3ymIvCs3oXr0EMB

struct tag_2xCNoiC3ymIvCs3oXr0EMB
{
  int32_T isInitialized;
  p_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 /*struct_tag_2xCNoiC3ymIvCs3oXr0EMB*/

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_2xCNoiC3ymIvCs3oXr0EMB robotics_slmanip_internal_blo_T;

#endif                               /*typedef_robotics_slmanip_internal_blo_T*/

#ifndef struct_emxArray_tag_pGgszObO16I6TGXaEM
#define struct_emxArray_tag_pGgszObO16I6TGXaEM

struct emxArray_tag_pGgszObO16I6TGXaEM
{
  f_cell_wrap_dynsim_3link_plan_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                /*struct_emxArray_tag_pGgszObO16I6TGXaEM*/

#ifndef typedef_emxArray_f_cell_wrap_dynsim_3_T
#define typedef_emxArray_f_cell_wrap_dynsim_3_T

typedef struct emxArray_tag_pGgszObO16I6TGXaEM emxArray_f_cell_wrap_dynsim_3_T;

#endif                               /*typedef_emxArray_f_cell_wrap_dynsim_3_T*/

/* Parameters (default storage) */
typedef struct P_dynsim_3link_planar_T_ P_dynsim_3link_planar_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_dynsim_3link_planar_T RT_MODEL_dynsim_3link_planar_T;

#endif                             /* RTW_HEADER_dynsim_3link_planar_types_h_ */
