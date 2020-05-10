/*
 * dynsim_3link_planar.h
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

#ifndef RTW_HEADER_dynsim_3link_planar_h_
#define RTW_HEADER_dynsim_3link_planar_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "dynsim_3link_planar_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  SL_Bus_dynsim_3link_planar_sensor_msgs_JointState msg_g;
                                     /* '<Root>/Assign to CartesianState msg' */
  SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray In1;/* '<S14>/In1' */
  SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T b_varargout_2_Data[128];
  real_T b_I[36];
  real_T R[36];
  real_T X[36];
  real_T T[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_m[16];
  real_T dv[16];
  real_T TJ_c[16];
  real_T obj_k[16];
  real_T R_c[9];
  real_T R_b[9];
  real_T dv1[9];
  real_T R_p[9];
  real_T tempR[9];
  real_T R_cv[9];
  real_T tempR_f[9];
  real_T dv2[9];
  real_T dv3[9];
  real_T R_g[9];
  real_T R_g1[9];
  real_T a0[6];
  real_T y[6];
  real_T X_m[6];
  real_T b_I_n[6];
  int8_T msubspace_data[36];
  char_T cv[34];
  char_T cv1[33];
  real_T result_data[4];
  char_T cv2[32];
  real_T TmpSignalConversionAtPositionIn[3];/* '<S1>/Subsystem' */
  real_T TmpSignalConversionAtVelocityIn[3];/* '<S1>/Subsystem' */
  real_T Velocity[3];                  /* '<S1>/Velocity' */
  real_T MATLABSystem[3];              /* '<S12>/MATLAB System' */
  real_T q_data[3];
  real_T v[3];
  real_T q_data_p[3];
  real_T v_l[3];
  char_T cv3[18];
  SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock msg_l;/* '<Root>/Assign to Time msg' */
  char_T cv4[14];
  int32_T nonFixedIndices_data[3];
  int32_T ii_data[3];
  char_T b[9];
  char_T b_j[9];
  char_T b_d[8];
  char_T b_g[8];
  char_T b_l[8];
  real_T vNum;
  real_T k;
  real_T j;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d6;
  real_T d7;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_d;
  real_T tempR_tmp_dy;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T nb_b;
  real_T vNum_n;
  real_T pid;
  real_T s;
  real_T p_idx_1;
  real_T b_idx_0_b;
  real_T b_idx_1_l;
  real_T b_h;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  char_T b_b[5];
  char_T b_da[5];
  char_T b_e[5];
  char_T b_bj[5];
  int32_T n;
  int32_T iend;
  int32_T j_j;
  int32_T u0;
  int32_T i;
  int32_T vNum_idx_0_tmp;
  int32_T MATLABSystem_tmp;
  int32_T b_k;
  int32_T p;
  int32_T b_k_f;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T q_size_tmp;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_a;
  int32_T b_kstr_j;
  int32_T b_i;
  int32_T f;
  int32_T cb;
  int32_T idx;
  int32_T n_j;
  int32_T nm1d2;
  int32_T m_o;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset_n;
  int32_T loop_ub;
  int32_T q_size_i;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T coffset_tmp;
  int32_T kstr_o;
  int32_T b_kstr_n;
  int32_T obj_tmp_m;
  int32_T obj_tmp_tmp_c;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp_m;
  int32_T X_tmp_m3;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_j;
  int32_T newNumel_h;
  int32_T i_c;
  int32_T newNumel_c;
  int32_T i_p;
  int32_T i_p5;
  int32_T i_a;
  int32_T i_e;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_l;
  boolean_T mask[3];
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_o;
  boolean_T b_bool_o2;
  boolean_T b_bool_i;
  boolean_T b_bool_f;
} B_dynsim_3link_planar_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  n_robotics_manip_internal_Rig_T gobj_1;/* '<S12>/MATLAB System' */
  n_robotics_manip_internal_Rig_T gobj_2;/* '<S12>/MATLAB System' */
  n_robotics_manip_internal_Rig_T gobj_3;/* '<S12>/MATLAB System' */
  n_robotics_manip_internal_Rig_T gobj_4;/* '<S12>/MATLAB System' */
  n_robotics_manip_internal_Rig_T gobj_5;/* '<S12>/MATLAB System' */
  n_robotics_manip_internal_Rig_T gobj_6;/* '<S12>/MATLAB System' */
  robotics_slmanip_internal_blo_T obj; /* '<S12>/MATLAB System' */
  ros_slros_internal_block_GetP_T obj_p;/* '<S13>/Get Parameter' */
  ros_slros_internal_block_GetP_T obj_n;/* '<S13>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_nf;/* '<S13>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_h;/* '<S13>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_b;/* '<S13>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_e;/* '<S13>/Get Parameter5' */
  ros_slros_internal_block_Publ_T obj_a;/* '<S10>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_f;/* '<S9>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_nr;/* '<S8>/SinkBlock' */
  ros_slros_internal_block_Subs_T obj_pp;/* '<S11>/SourceBlock' */
  int_T Position_IWORK;                /* '<S1>/Position' */
  int_T Velocity_IWORK;                /* '<S1>/Velocity' */
  boolean_T objisempty;                /* '<S11>/SourceBlock' */
  boolean_T objisempty_i;              /* '<S10>/SinkBlock' */
  boolean_T objisempty_m;              /* '<S9>/SinkBlock' */
  boolean_T objisempty_e;              /* '<S8>/SinkBlock' */
  boolean_T objisempty_ev;             /* '<S13>/Get Parameter' */
  boolean_T objisempty_k;              /* '<S13>/Get Parameter1' */
  boolean_T objisempty_iu;             /* '<S13>/Get Parameter4' */
  boolean_T objisempty_kw;             /* '<S13>/Get Parameter2' */
  boolean_T objisempty_b;              /* '<S13>/Get Parameter3' */
  boolean_T objisempty_l;              /* '<S13>/Get Parameter5' */
  boolean_T objisempty_j;              /* '<S12>/MATLAB System' */
} DW_dynsim_3link_planar_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Position_CSTATE[3];           /* '<S1>/Position' */
  real_T Velocity_CSTATE[3];           /* '<S1>/Velocity' */
} X_dynsim_3link_planar_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Position_CSTATE[3];           /* '<S1>/Position' */
  real_T Velocity_CSTATE[3];           /* '<S1>/Velocity' */
} XDot_dynsim_3link_planar_T;

/* State disabled  */
typedef struct {
  boolean_T Position_CSTATE[3];        /* '<S1>/Position' */
  boolean_T Velocity_CSTATE[3];        /* '<S1>/Velocity' */
} XDis_dynsim_3link_planar_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (default storage) */
struct P_dynsim_3link_planar_T_ {
  SL_Bus_dynsim_3link_planar_sensor_msgs_JointState Constant_Value;/* Computed Parameter: Constant_Value
                                                                    * Referenced by: '<S5>/Constant'
                                                                    */
  SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray Out1_Y0;/* Computed Parameter: Out1_Y0
                                                                 * Referenced by: '<S14>/Out1'
                                                                 */
  SL_Bus_dynsim_3link_planar_std_msgs_Float64MultiArray Constant_Value_h;/* Computed Parameter: Constant_Value_h
                                                                      * Referenced by: '<S11>/Constant'
                                                                      */
  SL_Bus_dynsim_3link_planar_rosgraph_msgs_Clock Constant_Value_o;/* Computed Parameter: Constant_Value_o
                                                                   * Referenced by: '<S6>/Constant'
                                                                   */
  real_T Constant_Value_f[18];         /* Expression: zeros(6,3)
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Constant_Value_oa[3];         /* Expression: [L1,L2,L3]
                                        * Referenced by: '<Root>/Constant'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_dynsim_3link_planar_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dynsim_3link_planar_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[6];
  real_T odeF[3][6];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_dynsim_3link_planar_T dynsim_3link_planar_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern B_dynsim_3link_planar_T dynsim_3link_planar_B;

#ifdef __cplusplus

}
#endif

/* Continuous states (default storage) */
extern X_dynsim_3link_planar_T dynsim_3link_planar_X;

/* Block states (default storage) */
extern DW_dynsim_3link_planar_T dynsim_3link_planar_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void dynsim_3link_planar_initialize(void);
  extern void dynsim_3link_planar_step(void);
  extern void dynsim_3link_planar_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_dynsim_3link_planar_T *const dynsim_3link_planar_M;

#ifdef __cplusplus

}
#endif

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/Reshape' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'dynsim_3link_planar'
 * '<S1>'   : 'dynsim_3link_planar/2link_planar robot dynamic model'
 * '<S2>'   : 'dynsim_3link_planar/Assign to CartesianState msg'
 * '<S3>'   : 'dynsim_3link_planar/Assign to JointState msg'
 * '<S4>'   : 'dynsim_3link_planar/Assign to Time msg'
 * '<S5>'   : 'dynsim_3link_planar/JointState'
 * '<S6>'   : 'dynsim_3link_planar/JointState1'
 * '<S7>'   : 'dynsim_3link_planar/MATLAB Function'
 * '<S8>'   : 'dynsim_3link_planar/Publish'
 * '<S9>'   : 'dynsim_3link_planar/Publish1'
 * '<S10>'  : 'dynsim_3link_planar/Publish2'
 * '<S11>'  : 'dynsim_3link_planar/Subscribe'
 * '<S12>'  : 'dynsim_3link_planar/2link_planar robot dynamic model/Forward Dynamics'
 * '<S13>'  : 'dynsim_3link_planar/2link_planar robot dynamic model/Subsystem'
 * '<S14>'  : 'dynsim_3link_planar/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_dynsim_3link_planar_h_ */
