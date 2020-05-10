/*
 * dynsim_3link_planar.cpp
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

#include "dynsim_3link_planar.h"
#include "dynsim_3link_planar_private.h"

/* Block signals (default storage) */
B_dynsim_3link_planar_T dynsim_3link_planar_B;

/* Continuous states */
X_dynsim_3link_planar_T dynsim_3link_planar_X;

/* Block states (default storage) */
DW_dynsim_3link_planar_T dynsim_3link_planar_DW;

/* Real-time model */
RT_MODEL_dynsim_3link_planar_T dynsim_3link_planar_M_ =
  RT_MODEL_dynsim_3link_planar_T();
RT_MODEL_dynsim_3link_planar_T *const dynsim_3link_planar_M =
  &dynsim_3link_planar_M_;

/* Forward declaration for local functions */
static void dynsim_3link_pla_emxInit_char_T(emxArray_char_T_dynsim_3link__T
  **pEmxArray, int32_T numDimensions);
static void dynsim_3link_pla_emxInit_real_T(emxArray_real_T_dynsim_3link__T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_3link_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_3link__T
  *emxArray, int32_T oldNumel);
static void dynsim_3link_pla_emxFree_char_T(emxArray_char_T_dynsim_3link__T
  **pEmxArray);
static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_3link__T
  *emxArray, int32_T oldNumel);
static n_robotics_manip_internal_Rig_T *dynsim_3lin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim_3l_RigidBody_RigidBody_j
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim_3_RigidBody_RigidBody_jr
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_jro
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2);
static void dynsim_3link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0);
static void dynsim_3lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  **pEmxArray, int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_3link_T *obj, real_T ax[3]);
static void dynsim_3link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_j(const
  c_rigidBodyJoint_dynsim_3link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_3link_T *obj, real_T T[16]);
static void dynsim_3link_planar_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynsim_3lin_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynsim_3link_pla_emxFree_real_T(emxArray_real_T_dynsim_3link__T
  **pEmxArray);
static void dynsim_3lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[3], emxArray_real_T_dynsim_3link__T *H,
  emxArray_real_T_dynsim_3link__T *lambda);
static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[3], const real_T qdot[3], const real_T fext[18], real_T
  tau[3]);
static void matlabCodegenHandle_m_jroq24mq5(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_3link_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void matlabCodegenHandle_matl_jroq24(ros_slros_internal_block_Publ_T *obj);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  dynsim_3link_planar_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  dynsim_3link_planar_step();
  dynsim_3link_planar_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  dynsim_3link_planar_step();
  dynsim_3link_planar_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void dynsim_3link_pla_emxInit_char_T(emxArray_char_T_dynsim_3link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_dynsim_3link__T *emxArray;
  *pEmxArray = (emxArray_char_T_dynsim_3link__T *)malloc(sizeof
    (emxArray_char_T_dynsim_3link__T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_3link_planar_B.i_p5 = 0; dynsim_3link_planar_B.i_p5 <
       numDimensions; dynsim_3link_planar_B.i_p5++) {
    emxArray->size[dynsim_3link_planar_B.i_p5] = 0;
  }
}

static void dynsim_3link_pla_emxInit_real_T(emxArray_real_T_dynsim_3link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_dynsim_3link__T *emxArray;
  *pEmxArray = (emxArray_real_T_dynsim_3link__T *)malloc(sizeof
    (emxArray_real_T_dynsim_3link__T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_3link_planar_B.i_a = 0; dynsim_3link_planar_B.i_a < numDimensions;
       dynsim_3link_planar_B.i_a++) {
    emxArray->size[dynsim_3link_planar_B.i_a] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_3link_T
  *pStruct)
{
  dynsim_3link_pla_emxInit_char_T(&pStruct->Type, 2);
  dynsim_3link_pla_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_3link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_3link_planar_B.newNumel_h = 1;
  for (dynsim_3link_planar_B.i_c = 0; dynsim_3link_planar_B.i_c <
       emxArray->numDimensions; dynsim_3link_planar_B.i_c++) {
    dynsim_3link_planar_B.newNumel_h *= emxArray->size[dynsim_3link_planar_B.i_c];
  }

  if (dynsim_3link_planar_B.newNumel_h > emxArray->allocatedSize) {
    dynsim_3link_planar_B.i_c = emxArray->allocatedSize;
    if (dynsim_3link_planar_B.i_c < 16) {
      dynsim_3link_planar_B.i_c = 16;
    }

    while (dynsim_3link_planar_B.i_c < dynsim_3link_planar_B.newNumel_h) {
      if (dynsim_3link_planar_B.i_c > 1073741823) {
        dynsim_3link_planar_B.i_c = MAX_int32_T;
      } else {
        dynsim_3link_planar_B.i_c <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_3link_planar_B.i_c), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynsim_3link_planar_B.i_c;
    emxArray->canFreeData = true;
  }
}

static void dynsim_3link_pla_emxFree_char_T(emxArray_char_T_dynsim_3link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_dynsim_3link__T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_dynsim_3link__T *)NULL;
  }
}

static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_3link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_3link_planar_B.newNumel = 1;
  for (dynsim_3link_planar_B.i_j = 0; dynsim_3link_planar_B.i_j <
       emxArray->numDimensions; dynsim_3link_planar_B.i_j++) {
    dynsim_3link_planar_B.newNumel *= emxArray->size[dynsim_3link_planar_B.i_j];
  }

  if (dynsim_3link_planar_B.newNumel > emxArray->allocatedSize) {
    dynsim_3link_planar_B.i_j = emxArray->allocatedSize;
    if (dynsim_3link_planar_B.i_j < 16) {
      dynsim_3link_planar_B.i_j = 16;
    }

    while (dynsim_3link_planar_B.i_j < dynsim_3link_planar_B.newNumel) {
      if (dynsim_3link_planar_B.i_j > 1073741823) {
        dynsim_3link_planar_B.i_j = MAX_int32_T;
      } else {
        dynsim_3link_planar_B.i_j <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_3link_planar_B.i_j), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynsim_3link_planar_B.i_j;
    emxArray->canFreeData = true;
  }
}

static n_robotics_manip_internal_Rig_T *dynsim_3lin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_3link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.67,
    0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.67, -0.0, 1.0, 0.0, 0.0, 0.0, -0.0, 2.0,
    0.0, 0.0, -0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 2.0 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.1, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_3link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_3link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim_3l_RigidBody_RigidBody_j
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_3link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.335,
    0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.335, -0.0, 0.5, 0.0, 0.0, 0.0, -0.0, 1.0,
    0.0, 0.0, -0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_3link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_3link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim_3_RigidBody_RigidBody_jr
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_3link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.335,
    0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.335, -0.0, 0.5, 0.0, 0.0, 0.0, -0.0, 1.0,
    0.0, 0.0, -0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_3link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_3link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_jro
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_dynsim_3link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_3link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      dynsim_3link_planar_B.b_j[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              dynsim_3link_planar_B.b_j[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_3link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_3link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_3link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_3link_planar_B.msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      dynsim_3link_planar_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2)
{
  p_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[6] = { 1, 2, 3, 1, 2, 3 };

  b_obj = obj;
  obj->Bodies[0] = dynsim_3lin_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = dynsim_3l_RigidBody_RigidBody_j(iobj_2);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = dynsim_3_RigidBody_RigidBody_jr(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->NumBodies = 3.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->VelocityNumber = 3.0;
  for (i = 0; i < 6; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 6; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  dynsim__RigidBody_RigidBody_jro(&obj->Base);
  return b_obj;
}

static void dynsim_3link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_dynsim_3link_planar_16.getLatestMessage
    (&dynsim_3link_planar_B.b_varargout_2);
  memcpy(&varargout_2_Data[0], &dynsim_3link_planar_B.b_varargout_2.Data[0],
         sizeof(real_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    dynsim_3link_planar_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    dynsim_3link_planar_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    dynsim_3link_planar_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &dynsim_3link_planar_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    dynsim_3link_planar_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    dynsim_3link_planar_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void dynsim_3lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_3_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_3_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_3_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_3link_plan_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_3link_planar_B.i_e = 0; dynsim_3link_planar_B.i_e < numDimensions;
       dynsim_3link_planar_B.i_e++) {
    emxArray->size[dynsim_3link_planar_B.i_e] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_3link_planar_B.newNumel_c = 1;
  for (dynsim_3link_planar_B.i_p = 0; dynsim_3link_planar_B.i_p <
       emxArray->numDimensions; dynsim_3link_planar_B.i_p++) {
    dynsim_3link_planar_B.newNumel_c *= emxArray->size[dynsim_3link_planar_B.i_p];
  }

  if (dynsim_3link_planar_B.newNumel_c > emxArray->allocatedSize) {
    dynsim_3link_planar_B.i_p = emxArray->allocatedSize;
    if (dynsim_3link_planar_B.i_p < 16) {
      dynsim_3link_planar_B.i_p = 16;
    }

    while (dynsim_3link_planar_B.i_p < dynsim_3link_planar_B.newNumel_c) {
      if (dynsim_3link_planar_B.i_p > 1073741823) {
        dynsim_3link_planar_B.i_p = MAX_int32_T;
      } else {
        dynsim_3link_planar_B.i_p <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_3link_planar_B.i_p), sizeof
                     (f_cell_wrap_dynsim_3link_plan_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_3link_plan_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_3link_plan_T *)newData;
    emxArray->allocatedSize = dynsim_3link_planar_B.i_p;
    emxArray->canFreeData = true;
  }
}

static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_3link_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_3link_planar_B.b_kstr_j = 0; dynsim_3link_planar_B.b_kstr_j < 8;
       dynsim_3link_planar_B.b_kstr_j++) {
    dynsim_3link_planar_B.b_g[dynsim_3link_planar_B.b_kstr_j] =
      tmp[dynsim_3link_planar_B.b_kstr_j];
  }

  dynsim_3link_planar_B.b_bool_o2 = false;
  if (obj->Type->size[1] == 8) {
    dynsim_3link_planar_B.b_kstr_j = 1;
    do {
      exitg1 = 0;
      if (dynsim_3link_planar_B.b_kstr_j - 1 < 8) {
        dynsim_3link_planar_B.kstr_a = dynsim_3link_planar_B.b_kstr_j - 1;
        if (obj->Type->data[dynsim_3link_planar_B.kstr_a] !=
            dynsim_3link_planar_B.b_g[dynsim_3link_planar_B.kstr_a]) {
          exitg1 = 1;
        } else {
          dynsim_3link_planar_B.b_kstr_j++;
        }
      } else {
        dynsim_3link_planar_B.b_bool_o2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_3link_planar_B.b_bool_o2) {
    guard1 = true;
  } else {
    for (dynsim_3link_planar_B.b_kstr_j = 0; dynsim_3link_planar_B.b_kstr_j < 9;
         dynsim_3link_planar_B.b_kstr_j++) {
      dynsim_3link_planar_B.b[dynsim_3link_planar_B.b_kstr_j] =
        tmp_0[dynsim_3link_planar_B.b_kstr_j];
    }

    dynsim_3link_planar_B.b_bool_o2 = false;
    if (obj->Type->size[1] == 9) {
      dynsim_3link_planar_B.b_kstr_j = 1;
      do {
        exitg1 = 0;
        if (dynsim_3link_planar_B.b_kstr_j - 1 < 9) {
          dynsim_3link_planar_B.kstr_a = dynsim_3link_planar_B.b_kstr_j - 1;
          if (obj->Type->data[dynsim_3link_planar_B.kstr_a] !=
              dynsim_3link_planar_B.b[dynsim_3link_planar_B.kstr_a]) {
            exitg1 = 1;
          } else {
            dynsim_3link_planar_B.b_kstr_j++;
          }
        } else {
          dynsim_3link_planar_B.b_bool_o2 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_3link_planar_B.b_bool_o2) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void dynsim_3link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyT_j(const
  c_rigidBodyJoint_dynsim_3link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 5;
       dynsim_3link_planar_B.b_kstr++) {
    dynsim_3link_planar_B.b_da[dynsim_3link_planar_B.b_kstr] =
      tmp[dynsim_3link_planar_B.b_kstr];
  }

  dynsim_3link_planar_B.b_bool_o = false;
  if (obj->Type->size[1] == 5) {
    dynsim_3link_planar_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynsim_3link_planar_B.b_kstr - 1 < 5) {
        dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr - 1;
        if (obj->Type->data[dynsim_3link_planar_B.kstr] !=
            dynsim_3link_planar_B.b_da[dynsim_3link_planar_B.kstr]) {
          exitg1 = 1;
        } else {
          dynsim_3link_planar_B.b_kstr++;
        }
      } else {
        dynsim_3link_planar_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_3link_planar_B.b_bool_o) {
    dynsim_3link_planar_B.b_kstr = 0;
  } else {
    for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 8;
         dynsim_3link_planar_B.b_kstr++) {
      dynsim_3link_planar_B.b_d[dynsim_3link_planar_B.b_kstr] =
        tmp_0[dynsim_3link_planar_B.b_kstr];
    }

    dynsim_3link_planar_B.b_bool_o = false;
    if (obj->Type->size[1] == 8) {
      dynsim_3link_planar_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (dynsim_3link_planar_B.b_kstr - 1 < 8) {
          dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr - 1;
          if (obj->Type->data[dynsim_3link_planar_B.kstr] !=
              dynsim_3link_planar_B.b_d[dynsim_3link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_3link_planar_B.b_kstr++;
          }
        } else {
          dynsim_3link_planar_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_3link_planar_B.b_bool_o) {
      dynsim_3link_planar_B.b_kstr = 1;
    } else {
      dynsim_3link_planar_B.b_kstr = -1;
    }
  }

  switch (dynsim_3link_planar_B.b_kstr) {
   case 0:
    memset(&dynsim_3link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynsim_3link_planar_B.TJ[0] = 1.0;
    dynsim_3link_planar_B.TJ[5] = 1.0;
    dynsim_3link_planar_B.TJ[10] = 1.0;
    dynsim_3link_planar_B.TJ[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_3link_planar_B.v);
    dynsim_3link_planar_B.result_data[0] = dynsim_3link_planar_B.v[0];
    dynsim_3link_planar_B.result_data[1] = dynsim_3link_planar_B.v[1];
    dynsim_3link_planar_B.result_data[2] = dynsim_3link_planar_B.v[2];
    if (0 <= (*q_size != 0) - 1) {
      dynsim_3link_planar_B.result_data[3] = q_data[0];
    }

    dynsim_3link_planar_B.cth = 1.0 / sqrt((dynsim_3link_planar_B.result_data[0]
      * dynsim_3link_planar_B.result_data[0] +
      dynsim_3link_planar_B.result_data[1] * dynsim_3link_planar_B.result_data[1])
      + dynsim_3link_planar_B.result_data[2] *
      dynsim_3link_planar_B.result_data[2]);
    dynsim_3link_planar_B.v[0] = dynsim_3link_planar_B.result_data[0] *
      dynsim_3link_planar_B.cth;
    dynsim_3link_planar_B.v[1] = dynsim_3link_planar_B.result_data[1] *
      dynsim_3link_planar_B.cth;
    dynsim_3link_planar_B.v[2] = dynsim_3link_planar_B.result_data[2] *
      dynsim_3link_planar_B.cth;
    dynsim_3link_planar_B.cth = cos(dynsim_3link_planar_B.result_data[3]);
    dynsim_3link_planar_B.sth = sin(dynsim_3link_planar_B.result_data[3]);
    dynsim_3link_planar_B.tempR_tmp = dynsim_3link_planar_B.v[1] *
      dynsim_3link_planar_B.v[0] * (1.0 - dynsim_3link_planar_B.cth);
    dynsim_3link_planar_B.tempR_tmp_d = dynsim_3link_planar_B.v[2] *
      dynsim_3link_planar_B.sth;
    dynsim_3link_planar_B.tempR_tmp_dy = dynsim_3link_planar_B.v[2] *
      dynsim_3link_planar_B.v[0] * (1.0 - dynsim_3link_planar_B.cth);
    dynsim_3link_planar_B.tempR_tmp_l = dynsim_3link_planar_B.v[1] *
      dynsim_3link_planar_B.sth;
    dynsim_3link_planar_B.tempR_tmp_o = dynsim_3link_planar_B.v[2] *
      dynsim_3link_planar_B.v[1] * (1.0 - dynsim_3link_planar_B.cth);
    dynsim_3link_planar_B.sth *= dynsim_3link_planar_B.v[0];
    dynsim_3link_planar_cat(dynsim_3link_planar_B.v[0] *
      dynsim_3link_planar_B.v[0] * (1.0 - dynsim_3link_planar_B.cth) +
      dynsim_3link_planar_B.cth, dynsim_3link_planar_B.tempR_tmp -
      dynsim_3link_planar_B.tempR_tmp_d, dynsim_3link_planar_B.tempR_tmp_dy +
      dynsim_3link_planar_B.tempR_tmp_l, dynsim_3link_planar_B.tempR_tmp +
      dynsim_3link_planar_B.tempR_tmp_d, dynsim_3link_planar_B.v[1] *
      dynsim_3link_planar_B.v[1] * (1.0 - dynsim_3link_planar_B.cth) +
      dynsim_3link_planar_B.cth, dynsim_3link_planar_B.tempR_tmp_o -
      dynsim_3link_planar_B.sth, dynsim_3link_planar_B.tempR_tmp_dy -
      dynsim_3link_planar_B.tempR_tmp_l, dynsim_3link_planar_B.tempR_tmp_o +
      dynsim_3link_planar_B.sth, dynsim_3link_planar_B.v[2] *
      dynsim_3link_planar_B.v[2] * (1.0 - dynsim_3link_planar_B.cth) +
      dynsim_3link_planar_B.cth, dynsim_3link_planar_B.tempR);
    for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 3;
         dynsim_3link_planar_B.b_kstr++) {
      dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr + 1;
      dynsim_3link_planar_B.R_p[dynsim_3link_planar_B.kstr - 1] =
        dynsim_3link_planar_B.tempR[(dynsim_3link_planar_B.kstr - 1) * 3];
      dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr + 1;
      dynsim_3link_planar_B.R_p[dynsim_3link_planar_B.kstr + 2] =
        dynsim_3link_planar_B.tempR[(dynsim_3link_planar_B.kstr - 1) * 3 + 1];
      dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr + 1;
      dynsim_3link_planar_B.R_p[dynsim_3link_planar_B.kstr + 5] =
        dynsim_3link_planar_B.tempR[(dynsim_3link_planar_B.kstr - 1) * 3 + 2];
    }

    memset(&dynsim_3link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 3;
         dynsim_3link_planar_B.b_kstr++) {
      dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr << 2;
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr] =
        dynsim_3link_planar_B.R_p[3 * dynsim_3link_planar_B.b_kstr];
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr + 1] =
        dynsim_3link_planar_B.R_p[3 * dynsim_3link_planar_B.b_kstr + 1];
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr + 2] =
        dynsim_3link_planar_B.R_p[3 * dynsim_3link_planar_B.b_kstr + 2];
    }

    dynsim_3link_planar_B.TJ[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_3link_planar_B.v);
    memset(&dynsim_3link_planar_B.tempR[0], 0, 9U * sizeof(real_T));
    dynsim_3link_planar_B.tempR[0] = 1.0;
    dynsim_3link_planar_B.tempR[4] = 1.0;
    dynsim_3link_planar_B.tempR[8] = 1.0;
    for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 3;
         dynsim_3link_planar_B.b_kstr++) {
      dynsim_3link_planar_B.kstr = dynsim_3link_planar_B.b_kstr << 2;
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr] =
        dynsim_3link_planar_B.tempR[3 * dynsim_3link_planar_B.b_kstr];
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr + 1] =
        dynsim_3link_planar_B.tempR[3 * dynsim_3link_planar_B.b_kstr + 1];
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.kstr + 2] =
        dynsim_3link_planar_B.tempR[3 * dynsim_3link_planar_B.b_kstr + 2];
      dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.b_kstr + 12] =
        dynsim_3link_planar_B.v[dynsim_3link_planar_B.b_kstr] * q_data[0];
    }

    dynsim_3link_planar_B.TJ[3] = 0.0;
    dynsim_3link_planar_B.TJ[7] = 0.0;
    dynsim_3link_planar_B.TJ[11] = 0.0;
    dynsim_3link_planar_B.TJ[15] = 1.0;
    break;
  }

  for (dynsim_3link_planar_B.b_kstr = 0; dynsim_3link_planar_B.b_kstr < 4;
       dynsim_3link_planar_B.b_kstr++) {
    for (dynsim_3link_planar_B.kstr = 0; dynsim_3link_planar_B.kstr < 4;
         dynsim_3link_planar_B.kstr++) {
      dynsim_3link_planar_B.obj_tmp_tmp = dynsim_3link_planar_B.kstr << 2;
      dynsim_3link_planar_B.obj_tmp = dynsim_3link_planar_B.b_kstr +
        dynsim_3link_planar_B.obj_tmp_tmp;
      dynsim_3link_planar_B.obj[dynsim_3link_planar_B.obj_tmp] = 0.0;
      dynsim_3link_planar_B.obj[dynsim_3link_planar_B.obj_tmp] +=
        dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.obj_tmp_tmp] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr];
      dynsim_3link_planar_B.obj[dynsim_3link_planar_B.obj_tmp] +=
        dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr + 4];
      dynsim_3link_planar_B.obj[dynsim_3link_planar_B.obj_tmp] +=
        dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr + 8];
      dynsim_3link_planar_B.obj[dynsim_3link_planar_B.obj_tmp] +=
        dynsim_3link_planar_B.TJ[dynsim_3link_planar_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr + 12];
    }

    for (dynsim_3link_planar_B.kstr = 0; dynsim_3link_planar_B.kstr < 4;
         dynsim_3link_planar_B.kstr++) {
      dynsim_3link_planar_B.obj_tmp_tmp = dynsim_3link_planar_B.kstr << 2;
      dynsim_3link_planar_B.obj_tmp = dynsim_3link_planar_B.b_kstr +
        dynsim_3link_planar_B.obj_tmp_tmp;
      T[dynsim_3link_planar_B.obj_tmp] = 0.0;
      T[dynsim_3link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp] *
        dynsim_3link_planar_B.obj[dynsim_3link_planar_B.b_kstr];
      T[dynsim_3link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp + 1] *
        dynsim_3link_planar_B.obj[dynsim_3link_planar_B.b_kstr + 4];
      T[dynsim_3link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp + 2] *
        dynsim_3link_planar_B.obj[dynsim_3link_planar_B.b_kstr + 8];
      T[dynsim_3link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp + 3] *
        dynsim_3link_planar_B.obj[dynsim_3link_planar_B.b_kstr + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_3link_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 5;
       dynsim_3link_planar_B.b_kstr_n++) {
    dynsim_3link_planar_B.b_bj[dynsim_3link_planar_B.b_kstr_n] =
      tmp[dynsim_3link_planar_B.b_kstr_n];
  }

  dynsim_3link_planar_B.b_bool_f = false;
  if (obj->Type->size[1] == 5) {
    dynsim_3link_planar_B.b_kstr_n = 1;
    do {
      exitg1 = 0;
      if (dynsim_3link_planar_B.b_kstr_n - 1 < 5) {
        dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n - 1;
        if (obj->Type->data[dynsim_3link_planar_B.kstr_o] !=
            dynsim_3link_planar_B.b_bj[dynsim_3link_planar_B.kstr_o]) {
          exitg1 = 1;
        } else {
          dynsim_3link_planar_B.b_kstr_n++;
        }
      } else {
        dynsim_3link_planar_B.b_bool_f = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_3link_planar_B.b_bool_f) {
    dynsim_3link_planar_B.b_kstr_n = 0;
  } else {
    for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 8;
         dynsim_3link_planar_B.b_kstr_n++) {
      dynsim_3link_planar_B.b_l[dynsim_3link_planar_B.b_kstr_n] =
        tmp_0[dynsim_3link_planar_B.b_kstr_n];
    }

    dynsim_3link_planar_B.b_bool_f = false;
    if (obj->Type->size[1] == 8) {
      dynsim_3link_planar_B.b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (dynsim_3link_planar_B.b_kstr_n - 1 < 8) {
          dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n - 1;
          if (obj->Type->data[dynsim_3link_planar_B.kstr_o] !=
              dynsim_3link_planar_B.b_l[dynsim_3link_planar_B.kstr_o]) {
            exitg1 = 1;
          } else {
            dynsim_3link_planar_B.b_kstr_n++;
          }
        } else {
          dynsim_3link_planar_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_3link_planar_B.b_bool_f) {
      dynsim_3link_planar_B.b_kstr_n = 1;
    } else {
      dynsim_3link_planar_B.b_kstr_n = -1;
    }
  }

  switch (dynsim_3link_planar_B.b_kstr_n) {
   case 0:
    memset(&dynsim_3link_planar_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    dynsim_3link_planar_B.TJ_c[0] = 1.0;
    dynsim_3link_planar_B.TJ_c[5] = 1.0;
    dynsim_3link_planar_B.TJ_c[10] = 1.0;
    dynsim_3link_planar_B.TJ_c[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_3link_planar_B.v_l);
    dynsim_3link_planar_B.axang_idx_0 = dynsim_3link_planar_B.v_l[0];
    dynsim_3link_planar_B.axang_idx_1 = dynsim_3link_planar_B.v_l[1];
    dynsim_3link_planar_B.axang_idx_2 = dynsim_3link_planar_B.v_l[2];
    dynsim_3link_planar_B.b_h = 1.0 / sqrt((dynsim_3link_planar_B.axang_idx_0 *
      dynsim_3link_planar_B.axang_idx_0 + dynsim_3link_planar_B.axang_idx_1 *
      dynsim_3link_planar_B.axang_idx_1) + dynsim_3link_planar_B.axang_idx_2 *
      dynsim_3link_planar_B.axang_idx_2);
    dynsim_3link_planar_B.v_l[0] = dynsim_3link_planar_B.axang_idx_0 *
      dynsim_3link_planar_B.b_h;
    dynsim_3link_planar_B.v_l[1] = dynsim_3link_planar_B.axang_idx_1 *
      dynsim_3link_planar_B.b_h;
    dynsim_3link_planar_B.v_l[2] = dynsim_3link_planar_B.axang_idx_2 *
      dynsim_3link_planar_B.b_h;
    dynsim_3link_planar_B.axang_idx_0 = dynsim_3link_planar_B.v_l[1] *
      dynsim_3link_planar_B.v_l[0] * 0.0;
    dynsim_3link_planar_B.axang_idx_1 = dynsim_3link_planar_B.v_l[2] *
      dynsim_3link_planar_B.v_l[0] * 0.0;
    dynsim_3link_planar_B.axang_idx_2 = dynsim_3link_planar_B.v_l[2] *
      dynsim_3link_planar_B.v_l[1] * 0.0;
    dynsim_3link_planar_cat(dynsim_3link_planar_B.v_l[0] *
      dynsim_3link_planar_B.v_l[0] * 0.0 + 1.0,
      dynsim_3link_planar_B.axang_idx_0 - dynsim_3link_planar_B.v_l[2] * 0.0,
      dynsim_3link_planar_B.axang_idx_1 + dynsim_3link_planar_B.v_l[1] * 0.0,
      dynsim_3link_planar_B.axang_idx_0 + dynsim_3link_planar_B.v_l[2] * 0.0,
      dynsim_3link_planar_B.v_l[1] * dynsim_3link_planar_B.v_l[1] * 0.0 + 1.0,
      dynsim_3link_planar_B.axang_idx_2 - dynsim_3link_planar_B.v_l[0] * 0.0,
      dynsim_3link_planar_B.axang_idx_1 - dynsim_3link_planar_B.v_l[1] * 0.0,
      dynsim_3link_planar_B.axang_idx_2 + dynsim_3link_planar_B.v_l[0] * 0.0,
      dynsim_3link_planar_B.v_l[2] * dynsim_3link_planar_B.v_l[2] * 0.0 + 1.0,
      dynsim_3link_planar_B.tempR_f);
    for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 3;
         dynsim_3link_planar_B.b_kstr_n++) {
      dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n + 1;
      dynsim_3link_planar_B.R_cv[dynsim_3link_planar_B.kstr_o - 1] =
        dynsim_3link_planar_B.tempR_f[(dynsim_3link_planar_B.kstr_o - 1) * 3];
      dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n + 1;
      dynsim_3link_planar_B.R_cv[dynsim_3link_planar_B.kstr_o + 2] =
        dynsim_3link_planar_B.tempR_f[(dynsim_3link_planar_B.kstr_o - 1) * 3 + 1];
      dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n + 1;
      dynsim_3link_planar_B.R_cv[dynsim_3link_planar_B.kstr_o + 5] =
        dynsim_3link_planar_B.tempR_f[(dynsim_3link_planar_B.kstr_o - 1) * 3 + 2];
    }

    memset(&dynsim_3link_planar_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 3;
         dynsim_3link_planar_B.b_kstr_n++) {
      dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n << 2;
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o] =
        dynsim_3link_planar_B.R_cv[3 * dynsim_3link_planar_B.b_kstr_n];
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o + 1] =
        dynsim_3link_planar_B.R_cv[3 * dynsim_3link_planar_B.b_kstr_n + 1];
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o + 2] =
        dynsim_3link_planar_B.R_cv[3 * dynsim_3link_planar_B.b_kstr_n + 2];
    }

    dynsim_3link_planar_B.TJ_c[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_3link_planar_B.v_l);
    memset(&dynsim_3link_planar_B.tempR_f[0], 0, 9U * sizeof(real_T));
    dynsim_3link_planar_B.tempR_f[0] = 1.0;
    dynsim_3link_planar_B.tempR_f[4] = 1.0;
    dynsim_3link_planar_B.tempR_f[8] = 1.0;
    for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 3;
         dynsim_3link_planar_B.b_kstr_n++) {
      dynsim_3link_planar_B.kstr_o = dynsim_3link_planar_B.b_kstr_n << 2;
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o] =
        dynsim_3link_planar_B.tempR_f[3 * dynsim_3link_planar_B.b_kstr_n];
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o + 1] =
        dynsim_3link_planar_B.tempR_f[3 * dynsim_3link_planar_B.b_kstr_n + 1];
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.kstr_o + 2] =
        dynsim_3link_planar_B.tempR_f[3 * dynsim_3link_planar_B.b_kstr_n + 2];
      dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.b_kstr_n + 12] =
        dynsim_3link_planar_B.v_l[dynsim_3link_planar_B.b_kstr_n] * 0.0;
    }

    dynsim_3link_planar_B.TJ_c[3] = 0.0;
    dynsim_3link_planar_B.TJ_c[7] = 0.0;
    dynsim_3link_planar_B.TJ_c[11] = 0.0;
    dynsim_3link_planar_B.TJ_c[15] = 1.0;
    break;
  }

  for (dynsim_3link_planar_B.b_kstr_n = 0; dynsim_3link_planar_B.b_kstr_n < 4;
       dynsim_3link_planar_B.b_kstr_n++) {
    for (dynsim_3link_planar_B.kstr_o = 0; dynsim_3link_planar_B.kstr_o < 4;
         dynsim_3link_planar_B.kstr_o++) {
      dynsim_3link_planar_B.obj_tmp_tmp_c = dynsim_3link_planar_B.kstr_o << 2;
      dynsim_3link_planar_B.obj_tmp_m = dynsim_3link_planar_B.b_kstr_n +
        dynsim_3link_planar_B.obj_tmp_tmp_c;
      dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.obj_tmp_m] = 0.0;
      dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.obj_tmp_m] +=
        dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.obj_tmp_tmp_c] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr_n];
      dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.obj_tmp_m] +=
        dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.obj_tmp_tmp_c + 1] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr_n + 4];
      dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.obj_tmp_m] +=
        dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.obj_tmp_tmp_c + 2] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr_n + 8];
      dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.obj_tmp_m] +=
        dynsim_3link_planar_B.TJ_c[dynsim_3link_planar_B.obj_tmp_tmp_c + 3] *
        obj->JointToParentTransform[dynsim_3link_planar_B.b_kstr_n + 12];
    }

    for (dynsim_3link_planar_B.kstr_o = 0; dynsim_3link_planar_B.kstr_o < 4;
         dynsim_3link_planar_B.kstr_o++) {
      dynsim_3link_planar_B.obj_tmp_tmp_c = dynsim_3link_planar_B.kstr_o << 2;
      dynsim_3link_planar_B.obj_tmp_m = dynsim_3link_planar_B.b_kstr_n +
        dynsim_3link_planar_B.obj_tmp_tmp_c;
      T[dynsim_3link_planar_B.obj_tmp_m] = 0.0;
      T[dynsim_3link_planar_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp_c] *
        dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.b_kstr_n];
      T[dynsim_3link_planar_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp_c + 1] *
        dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.b_kstr_n + 4];
      T[dynsim_3link_planar_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp_c + 2] *
        dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.b_kstr_n + 8];
      T[dynsim_3link_planar_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynsim_3link_planar_B.obj_tmp_tmp_c + 3] *
        dynsim_3link_planar_B.obj_k[dynsim_3link_planar_B.b_kstr_n + 12];
    }
  }
}

static void dynsim_3link_planar_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynsim_3link_planar_B.i3 = 0; dynsim_3link_planar_B.i3 < 3;
       dynsim_3link_planar_B.i3++) {
    dynsim_3link_planar_B.R_g[3 * dynsim_3link_planar_B.i3] =
      T[dynsim_3link_planar_B.i3];
    dynsim_3link_planar_B.R_g[3 * dynsim_3link_planar_B.i3 + 1] =
      T[dynsim_3link_planar_B.i3 + 4];
    dynsim_3link_planar_B.R_g[3 * dynsim_3link_planar_B.i3 + 2] =
      T[dynsim_3link_planar_B.i3 + 8];
  }

  for (dynsim_3link_planar_B.i3 = 0; dynsim_3link_planar_B.i3 < 9;
       dynsim_3link_planar_B.i3++) {
    dynsim_3link_planar_B.R_g1[dynsim_3link_planar_B.i3] =
      -dynsim_3link_planar_B.R_g[dynsim_3link_planar_B.i3];
  }

  for (dynsim_3link_planar_B.i3 = 0; dynsim_3link_planar_B.i3 < 3;
       dynsim_3link_planar_B.i3++) {
    dynsim_3link_planar_B.Tinv_tmp = dynsim_3link_planar_B.i3 << 2;
    Tinv[dynsim_3link_planar_B.Tinv_tmp] = dynsim_3link_planar_B.R_g[3 *
      dynsim_3link_planar_B.i3];
    Tinv[dynsim_3link_planar_B.Tinv_tmp + 1] = dynsim_3link_planar_B.R_g[3 *
      dynsim_3link_planar_B.i3 + 1];
    Tinv[dynsim_3link_planar_B.Tinv_tmp + 2] = dynsim_3link_planar_B.R_g[3 *
      dynsim_3link_planar_B.i3 + 2];
    Tinv[dynsim_3link_planar_B.i3 + 12] =
      dynsim_3link_planar_B.R_g1[dynsim_3link_planar_B.i3 + 6] * T[14] +
      (dynsim_3link_planar_B.R_g1[dynsim_3link_planar_B.i3 + 3] * T[13] +
       dynsim_3link_planar_B.R_g1[dynsim_3link_planar_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void dynsim_3lin_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  dynsim_3link_planar_B.dv2[0] = 0.0;
  dynsim_3link_planar_B.dv2[3] = -T[14];
  dynsim_3link_planar_B.dv2[6] = T[13];
  dynsim_3link_planar_B.dv2[1] = T[14];
  dynsim_3link_planar_B.dv2[4] = 0.0;
  dynsim_3link_planar_B.dv2[7] = -T[12];
  dynsim_3link_planar_B.dv2[2] = -T[13];
  dynsim_3link_planar_B.dv2[5] = T[12];
  dynsim_3link_planar_B.dv2[8] = 0.0;
  for (dynsim_3link_planar_B.i1 = 0; dynsim_3link_planar_B.i1 < 3;
       dynsim_3link_planar_B.i1++) {
    for (dynsim_3link_planar_B.X_tmp_m = 0; dynsim_3link_planar_B.X_tmp_m < 3;
         dynsim_3link_planar_B.X_tmp_m++) {
      dynsim_3link_planar_B.X_tmp_m3 = dynsim_3link_planar_B.i1 + 3 *
        dynsim_3link_planar_B.X_tmp_m;
      dynsim_3link_planar_B.dv3[dynsim_3link_planar_B.X_tmp_m3] = 0.0;
      dynsim_3link_planar_B.i2 = dynsim_3link_planar_B.X_tmp_m << 2;
      dynsim_3link_planar_B.dv3[dynsim_3link_planar_B.X_tmp_m3] +=
        T[dynsim_3link_planar_B.i2] *
        dynsim_3link_planar_B.dv2[dynsim_3link_planar_B.i1];
      dynsim_3link_planar_B.dv3[dynsim_3link_planar_B.X_tmp_m3] +=
        T[dynsim_3link_planar_B.i2 + 1] *
        dynsim_3link_planar_B.dv2[dynsim_3link_planar_B.i1 + 3];
      dynsim_3link_planar_B.dv3[dynsim_3link_planar_B.X_tmp_m3] +=
        T[dynsim_3link_planar_B.i2 + 2] *
        dynsim_3link_planar_B.dv2[dynsim_3link_planar_B.i1 + 6];
      X[dynsim_3link_planar_B.X_tmp_m + 6 * dynsim_3link_planar_B.i1] = T
        [(dynsim_3link_planar_B.i1 << 2) + dynsim_3link_planar_B.X_tmp_m];
      X[dynsim_3link_planar_B.X_tmp_m + 6 * (dynsim_3link_planar_B.i1 + 3)] =
        0.0;
    }
  }

  for (dynsim_3link_planar_B.i1 = 0; dynsim_3link_planar_B.i1 < 3;
       dynsim_3link_planar_B.i1++) {
    X[6 * dynsim_3link_planar_B.i1 + 3] = dynsim_3link_planar_B.dv3[3 *
      dynsim_3link_planar_B.i1];
    dynsim_3link_planar_B.X_tmp_m = dynsim_3link_planar_B.i1 << 2;
    dynsim_3link_planar_B.X_tmp_m3 = 6 * (dynsim_3link_planar_B.i1 + 3);
    X[dynsim_3link_planar_B.X_tmp_m3 + 3] = T[dynsim_3link_planar_B.X_tmp_m];
    X[6 * dynsim_3link_planar_B.i1 + 4] = dynsim_3link_planar_B.dv3[3 *
      dynsim_3link_planar_B.i1 + 1];
    X[dynsim_3link_planar_B.X_tmp_m3 + 4] = T[dynsim_3link_planar_B.X_tmp_m + 1];
    X[6 * dynsim_3link_planar_B.i1 + 5] = dynsim_3link_planar_B.dv3[3 *
      dynsim_3link_planar_B.i1 + 2];
    X[dynsim_3link_planar_B.X_tmp_m3 + 5] = T[dynsim_3link_planar_B.X_tmp_m + 2];
  }
}

static void dynsim_3link_pla_emxFree_real_T(emxArray_real_T_dynsim_3link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_dynsim_3link__T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_dynsim_3link__T *)NULL;
  }
}

static void dynsim_3lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_3_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_3_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_3link_plan_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_3_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[3], emxArray_real_T_dynsim_3link__T *H,
  emxArray_real_T_dynsim_3link__T *lambda)
{
  emxArray_f_cell_wrap_dynsim_3_T *Ic;
  emxArray_f_cell_wrap_dynsim_3_T *X;
  emxArray_real_T_dynsim_3link__T *lambda_;
  emxArray_real_T_dynsim_3link__T *Si;
  emxArray_real_T_dynsim_3link__T *Fi;
  emxArray_real_T_dynsim_3link__T *Hji;
  emxArray_real_T_dynsim_3link__T *s;
  n_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_3link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynsim_3link_planar_B.nb_b = robot->NumBodies;
  dynsim_3link_planar_B.vNum_n = robot->VelocityNumber;
  dynsim_3link_planar_B.f = H->size[0] * H->size[1];
  dynsim_3link_planar_B.b_i = static_cast<int32_T>(dynsim_3link_planar_B.vNum_n);
  H->size[0] = dynsim_3link_planar_B.b_i;
  H->size[1] = dynsim_3link_planar_B.b_i;
  dynsim_emxEnsureCapacity_real_T(H, dynsim_3link_planar_B.f);
  dynsim_3link_planar_B.loop_ub = dynsim_3link_planar_B.b_i *
    dynsim_3link_planar_B.b_i - 1;
  for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
       dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
    H->data[dynsim_3link_planar_B.f] = 0.0;
  }

  dynsim_3link_pla_emxInit_real_T(&lambda_, 2);
  dynsim_3link_planar_B.f = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynsim_3link_planar_B.nm1d2 = static_cast<int32_T>(dynsim_3link_planar_B.nb_b);
  lambda_->size[1] = dynsim_3link_planar_B.nm1d2;
  dynsim_emxEnsureCapacity_real_T(lambda_, dynsim_3link_planar_B.f);
  dynsim_3link_planar_B.idx = dynsim_3link_planar_B.nm1d2 - 1;
  for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
       dynsim_3link_planar_B.idx; dynsim_3link_planar_B.f++) {
    lambda_->data[dynsim_3link_planar_B.f] = 0.0;
  }

  dynsim_3link_planar_B.f = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynsim_3link_planar_B.b_i;
  dynsim_emxEnsureCapacity_real_T(lambda, dynsim_3link_planar_B.f);
  dynsim_3link_planar_B.loop_ub = dynsim_3link_planar_B.b_i - 1;
  for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
       dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
    lambda->data[dynsim_3link_planar_B.f] = 0.0;
  }

  dynsim_3lin_emxInit_f_cell_wrap(&Ic, 2);
  dynsim_3lin_emxInit_f_cell_wrap(&X, 2);
  dynsim_3link_planar_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynsim_3link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(Ic, dynsim_3link_planar_B.f);
  dynsim_3link_planar_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_3link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_3link_planar_B.f);
  for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <=
       dynsim_3link_planar_B.idx; dynsim_3link_planar_B.b_i++) {
    for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 36;
         dynsim_3link_planar_B.f++) {
      Ic->data[dynsim_3link_planar_B.b_i].f1[dynsim_3link_planar_B.f] =
        robot->Bodies[dynsim_3link_planar_B.b_i]->
        SpatialInertia[dynsim_3link_planar_B.f];
    }

    dynsim_3link_planar_B.vNum_n = robot->
      PositionDoFMap[dynsim_3link_planar_B.b_i];
    dynsim_3link_planar_B.p_idx_1 = robot->
      PositionDoFMap[dynsim_3link_planar_B.b_i + 3];
    if (dynsim_3link_planar_B.p_idx_1 < dynsim_3link_planar_B.vNum_n) {
      obj = robot->Bodies[dynsim_3link_planar_B.b_i];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_3link_planar_B.T_m);
    } else {
      if (dynsim_3link_planar_B.vNum_n > dynsim_3link_planar_B.p_idx_1) {
        dynsim_3link_planar_B.nm1d2 = 0;
        dynsim_3link_planar_B.f = -1;
      } else {
        dynsim_3link_planar_B.nm1d2 = static_cast<int32_T>
          (dynsim_3link_planar_B.vNum_n) - 1;
        dynsim_3link_planar_B.f = static_cast<int32_T>
          (dynsim_3link_planar_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynsim_3link_planar_B.b_i];
      dynsim_3link_planar_B.loop_ub = dynsim_3link_planar_B.f -
        dynsim_3link_planar_B.nm1d2;
      dynsim_3link_planar_B.q_size_i = dynsim_3link_planar_B.loop_ub + 1;
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
           dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
        dynsim_3link_planar_B.q_data_p[dynsim_3link_planar_B.f] =
          q[dynsim_3link_planar_B.nm1d2 + dynsim_3link_planar_B.f];
      }

      rigidBodyJoint_transformBodyT_j(&obj->JointInternal,
        dynsim_3link_planar_B.q_data_p, &dynsim_3link_planar_B.q_size_i,
        dynsim_3link_planar_B.T_m);
    }

    dynsim_3link_planar_tforminv(dynsim_3link_planar_B.T_m,
      dynsim_3link_planar_B.dv);
    dynsim_3lin_tformToSpatialXform(dynsim_3link_planar_B.dv, X->
      data[dynsim_3link_planar_B.b_i].f1);
  }

  dynsim_3link_planar_B.nm1d2 = static_cast<int32_T>(((-1.0 -
    dynsim_3link_planar_B.nb_b) + 1.0) / -1.0) - 1;
  dynsim_3link_pla_emxInit_real_T(&Si, 2);
  dynsim_3link_pla_emxInit_real_T(&Fi, 2);
  dynsim_3link_pla_emxInit_real_T(&Hji, 2);
  dynsim_3link_pla_emxInit_char_T(&a, 2);
  for (dynsim_3link_planar_B.idx = 0; dynsim_3link_planar_B.idx <=
       dynsim_3link_planar_B.nm1d2; dynsim_3link_planar_B.idx++) {
    dynsim_3link_planar_B.n_j = static_cast<int32_T>(dynsim_3link_planar_B.nb_b
      + -static_cast<real_T>(dynsim_3link_planar_B.idx));
    dynsim_3link_planar_B.pid_tmp = dynsim_3link_planar_B.n_j - 1;
    dynsim_3link_planar_B.pid = robot->Bodies[dynsim_3link_planar_B.pid_tmp]
      ->ParentIndex;
    dynsim_3link_planar_B.vNum_n = robot->
      VelocityDoFMap[dynsim_3link_planar_B.n_j - 1];
    dynsim_3link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_3link_planar_B.n_j + 2];
    if (dynsim_3link_planar_B.pid > 0.0) {
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
           dynsim_3link_planar_B.f++) {
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i < 6;
             dynsim_3link_planar_B.b_i++) {
          dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 6 *
            dynsim_3link_planar_B.b_i;
          dynsim_3link_planar_B.X[dynsim_3link_planar_B.X_tmp] = 0.0;
          for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <
               6; dynsim_3link_planar_B.loop_ub++) {
            dynsim_3link_planar_B.X[dynsim_3link_planar_B.X_tmp] += X->
              data[dynsim_3link_planar_B.pid_tmp].f1[6 * dynsim_3link_planar_B.f
              + dynsim_3link_planar_B.loop_ub] * Ic->
              data[dynsim_3link_planar_B.pid_tmp].f1[6 *
              dynsim_3link_planar_B.b_i + dynsim_3link_planar_B.loop_ub];
          }
        }
      }

      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
           dynsim_3link_planar_B.f++) {
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i < 6;
             dynsim_3link_planar_B.b_i++) {
          dynsim_3link_planar_B.b_idx_0_b = 0.0;
          for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <
               6; dynsim_3link_planar_B.loop_ub++) {
            dynsim_3link_planar_B.b_idx_0_b += dynsim_3link_planar_B.X[6 *
              dynsim_3link_planar_B.loop_ub + dynsim_3link_planar_B.f] * X->
              data[dynsim_3link_planar_B.pid_tmp].f1[6 *
              dynsim_3link_planar_B.b_i + dynsim_3link_planar_B.loop_ub];
          }

          dynsim_3link_planar_B.loop_ub = 6 * dynsim_3link_planar_B.b_i +
            dynsim_3link_planar_B.f;
          Ic->data[static_cast<int32_T>(dynsim_3link_planar_B.pid) - 1]
            .f1[dynsim_3link_planar_B.loop_ub] +=
            dynsim_3link_planar_B.b_idx_0_b;
        }
      }

      lambda_->data[dynsim_3link_planar_B.pid_tmp] = dynsim_3link_planar_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[dynsim_3link_planar_B.pid_tmp] > 0.0))
      {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[dynsim_3link_planar_B.pid_tmp]) - 1];
        dynsim_3link_planar_B.f = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        dynsim_emxEnsureCapacity_char_T(a, dynsim_3link_planar_B.f);
        dynsim_3link_planar_B.loop_ub = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
             dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
          a->data[dynsim_3link_planar_B.f] = obj->JointInternal.Type->
            data[dynsim_3link_planar_B.f];
        }

        for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 5;
             dynsim_3link_planar_B.f++) {
          dynsim_3link_planar_B.b_e[dynsim_3link_planar_B.f] =
            tmp[dynsim_3link_planar_B.f];
        }

        dynsim_3link_planar_B.b_bool_i = false;
        if (a->size[1] == 5) {
          dynsim_3link_planar_B.f = 1;
          do {
            exitg2 = 0;
            if (dynsim_3link_planar_B.f - 1 < 5) {
              dynsim_3link_planar_B.b_i = dynsim_3link_planar_B.f - 1;
              if (a->data[dynsim_3link_planar_B.b_i] !=
                  dynsim_3link_planar_B.b_e[dynsim_3link_planar_B.b_i]) {
                exitg2 = 1;
              } else {
                dynsim_3link_planar_B.f++;
              }
            } else {
              dynsim_3link_planar_B.b_bool_i = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynsim_3link_planar_B.b_bool_i) {
          lambda_->data[dynsim_3link_planar_B.pid_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[dynsim_3link_planar_B.pid_tmp]) -
            1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynsim_3link_planar_B.b_idx_0_b = robot->
      VelocityDoFMap[dynsim_3link_planar_B.n_j - 1];
    dynsim_3link_planar_B.b_idx_1_l = robot->
      VelocityDoFMap[dynsim_3link_planar_B.n_j + 2];
    if (dynsim_3link_planar_B.b_idx_0_b <= dynsim_3link_planar_B.b_idx_1_l) {
      obj = robot->Bodies[dynsim_3link_planar_B.pid_tmp];
      dynsim_3link_planar_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_3link_planar_B.f);
      dynsim_3link_planar_B.loop_ub = obj->JointInternal.MotionSubspace->size[0]
        * obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
           dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
        Si->data[dynsim_3link_planar_B.f] = obj->
          JointInternal.MotionSubspace->data[dynsim_3link_planar_B.f];
      }

      dynsim_3link_planar_B.n_j = Si->size[1] - 1;
      dynsim_3link_planar_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_3link_planar_B.f);
      for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <=
           dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.loop_ub++) {
        dynsim_3link_planar_B.coffset_tmp = dynsim_3link_planar_B.loop_ub * 6 -
          1;
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i < 6;
             dynsim_3link_planar_B.b_i++) {
          dynsim_3link_planar_B.s = 0.0;
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
               dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.s += Ic->data[dynsim_3link_planar_B.pid_tmp].
              f1[dynsim_3link_planar_B.f * 6 + dynsim_3link_planar_B.b_i] *
              Si->data[(dynsim_3link_planar_B.coffset_tmp +
                        dynsim_3link_planar_B.f) + 1];
          }

          Fi->data[(dynsim_3link_planar_B.coffset_tmp +
                    dynsim_3link_planar_B.b_i) + 1] = dynsim_3link_planar_B.s;
        }
      }

      if (dynsim_3link_planar_B.vNum_n > dynsim_3link_planar_B.p_idx_1) {
        dynsim_3link_planar_B.coffset_tmp = 0;
        dynsim_3link_planar_B.cb = 0;
      } else {
        dynsim_3link_planar_B.coffset_tmp = static_cast<int32_T>
          (dynsim_3link_planar_B.vNum_n) - 1;
        dynsim_3link_planar_B.cb = dynsim_3link_planar_B.coffset_tmp;
      }

      dynsim_3link_planar_B.m_o = Si->size[1];
      dynsim_3link_planar_B.n_j = Fi->size[1] - 1;
      dynsim_3link_planar_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Hji, dynsim_3link_planar_B.f);
      for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <=
           dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.loop_ub++) {
        dynsim_3link_planar_B.coffset = dynsim_3link_planar_B.loop_ub *
          dynsim_3link_planar_B.m_o - 1;
        dynsim_3link_planar_B.boffset = dynsim_3link_planar_B.loop_ub * 6 - 1;
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <
             dynsim_3link_planar_B.m_o; dynsim_3link_planar_B.b_i++) {
          dynsim_3link_planar_B.aoffset_n = dynsim_3link_planar_B.b_i * 6 - 1;
          dynsim_3link_planar_B.s = 0.0;
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
               dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 1;
            dynsim_3link_planar_B.s += Si->data[dynsim_3link_planar_B.aoffset_n
              + dynsim_3link_planar_B.X_tmp] * Fi->
              data[dynsim_3link_planar_B.boffset + dynsim_3link_planar_B.X_tmp];
          }

          Hji->data[(dynsim_3link_planar_B.coffset + dynsim_3link_planar_B.b_i)
            + 1] = dynsim_3link_planar_B.s;
        }
      }

      dynsim_3link_planar_B.loop_ub = Hji->size[1];
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <
           dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
        dynsim_3link_planar_B.n_j = Hji->size[0];
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <
             dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.b_i++) {
          H->data[(dynsim_3link_planar_B.coffset_tmp + dynsim_3link_planar_B.b_i)
            + H->size[0] * (dynsim_3link_planar_B.cb + dynsim_3link_planar_B.f)]
            = Hji->data[Hji->size[0] * dynsim_3link_planar_B.f +
            dynsim_3link_planar_B.b_i];
        }
      }

      dynsim_3link_planar_B.n_j = Fi->size[1];
      dynsim_3link_planar_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_3link_planar_B.f);
      dynsim_3link_planar_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
           dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
        Si->data[dynsim_3link_planar_B.f] = Fi->data[dynsim_3link_planar_B.f];
      }

      dynsim_3link_planar_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynsim_3link_planar_B.n_j;
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_3link_planar_B.f);
      for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <
           dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.loop_ub++) {
        dynsim_3link_planar_B.coffset_tmp = dynsim_3link_planar_B.loop_ub * 6 -
          1;
        for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i < 6;
             dynsim_3link_planar_B.b_i++) {
          dynsim_3link_planar_B.aoffset_n = dynsim_3link_planar_B.b_i * 6 - 1;
          dynsim_3link_planar_B.s = 0.0;
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
               dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 1;
            dynsim_3link_planar_B.s += X->data[dynsim_3link_planar_B.pid_tmp]
              .f1[dynsim_3link_planar_B.aoffset_n + dynsim_3link_planar_B.X_tmp]
              * Si->data[dynsim_3link_planar_B.coffset_tmp +
              dynsim_3link_planar_B.X_tmp];
          }

          Fi->data[(dynsim_3link_planar_B.coffset_tmp +
                    dynsim_3link_planar_B.b_i) + 1] = dynsim_3link_planar_B.s;
        }
      }

      while (dynsim_3link_planar_B.pid > 0.0) {
        dynsim_3link_planar_B.b_i = static_cast<int32_T>
          (dynsim_3link_planar_B.pid);
        dynsim_3link_planar_B.pid_tmp = dynsim_3link_planar_B.b_i - 1;
        obj = robot->Bodies[dynsim_3link_planar_B.pid_tmp];
        dynsim_3link_planar_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_3link_planar_B.f);
        dynsim_3link_planar_B.loop_ub = obj->JointInternal.MotionSubspace->size
          [0] * obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
             dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
          Si->data[dynsim_3link_planar_B.f] = obj->
            JointInternal.MotionSubspace->data[dynsim_3link_planar_B.f];
        }

        dynsim_3link_planar_B.b_idx_0_b = robot->
          VelocityDoFMap[dynsim_3link_planar_B.b_i - 1];
        dynsim_3link_planar_B.b_idx_1_l = robot->
          VelocityDoFMap[dynsim_3link_planar_B.b_i + 2];
        if (dynsim_3link_planar_B.b_idx_0_b <= dynsim_3link_planar_B.b_idx_1_l)
        {
          dynsim_3link_planar_B.m_o = Si->size[1];
          dynsim_3link_planar_B.n_j = Fi->size[1] - 1;
          dynsim_3link_planar_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynsim_emxEnsureCapacity_real_T(Hji, dynsim_3link_planar_B.f);
          for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <=
               dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.loop_ub++) {
            dynsim_3link_planar_B.coffset = dynsim_3link_planar_B.loop_ub *
              dynsim_3link_planar_B.m_o - 1;
            dynsim_3link_planar_B.boffset = dynsim_3link_planar_B.loop_ub * 6 -
              1;
            for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <
                 dynsim_3link_planar_B.m_o; dynsim_3link_planar_B.b_i++) {
              dynsim_3link_planar_B.aoffset_n = dynsim_3link_planar_B.b_i * 6 -
                1;
              dynsim_3link_planar_B.s = 0.0;
              for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
                   dynsim_3link_planar_B.f++) {
                dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 1;
                dynsim_3link_planar_B.s += Si->
                  data[dynsim_3link_planar_B.aoffset_n +
                  dynsim_3link_planar_B.X_tmp] * Fi->
                  data[dynsim_3link_planar_B.boffset +
                  dynsim_3link_planar_B.X_tmp];
              }

              Hji->data[(dynsim_3link_planar_B.coffset +
                         dynsim_3link_planar_B.b_i) + 1] =
                dynsim_3link_planar_B.s;
            }
          }

          if (dynsim_3link_planar_B.b_idx_0_b > dynsim_3link_planar_B.b_idx_1_l)
          {
            dynsim_3link_planar_B.X_tmp = 0;
          } else {
            dynsim_3link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_3link_planar_B.b_idx_0_b) - 1;
          }

          if (dynsim_3link_planar_B.vNum_n > dynsim_3link_planar_B.p_idx_1) {
            dynsim_3link_planar_B.coffset_tmp = 0;
          } else {
            dynsim_3link_planar_B.coffset_tmp = static_cast<int32_T>
              (dynsim_3link_planar_B.vNum_n) - 1;
          }

          dynsim_3link_planar_B.loop_ub = Hji->size[1];
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <
               dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.n_j = Hji->size[0];
            for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <
                 dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.b_i++) {
              H->data[(dynsim_3link_planar_B.X_tmp + dynsim_3link_planar_B.b_i)
                + H->size[0] * (dynsim_3link_planar_B.coffset_tmp +
                                dynsim_3link_planar_B.f)] = Hji->data[Hji->size
                [0] * dynsim_3link_planar_B.f + dynsim_3link_planar_B.b_i];
            }
          }

          if (dynsim_3link_planar_B.vNum_n > dynsim_3link_planar_B.p_idx_1) {
            dynsim_3link_planar_B.X_tmp = 0;
          } else {
            dynsim_3link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_3link_planar_B.vNum_n) - 1;
          }

          if (dynsim_3link_planar_B.b_idx_0_b > dynsim_3link_planar_B.b_idx_1_l)
          {
            dynsim_3link_planar_B.coffset_tmp = 0;
          } else {
            dynsim_3link_planar_B.coffset_tmp = static_cast<int32_T>
              (dynsim_3link_planar_B.b_idx_0_b) - 1;
          }

          dynsim_3link_planar_B.loop_ub = Hji->size[0];
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <
               dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.n_j = Hji->size[1];
            for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i <
                 dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.b_i++) {
              H->data[(dynsim_3link_planar_B.X_tmp + dynsim_3link_planar_B.b_i)
                + H->size[0] * (dynsim_3link_planar_B.coffset_tmp +
                                dynsim_3link_planar_B.f)] = Hji->data[Hji->size
                [0] * dynsim_3link_planar_B.b_i + dynsim_3link_planar_B.f];
            }
          }
        }

        dynsim_3link_planar_B.n_j = Fi->size[1];
        dynsim_3link_planar_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_3link_planar_B.f);
        dynsim_3link_planar_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
        for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
             dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
          Si->data[dynsim_3link_planar_B.f] = Fi->data[dynsim_3link_planar_B.f];
        }

        dynsim_3link_planar_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynsim_3link_planar_B.n_j;
        dynsim_emxEnsureCapacity_real_T(Fi, dynsim_3link_planar_B.f);
        for (dynsim_3link_planar_B.loop_ub = 0; dynsim_3link_planar_B.loop_ub <
             dynsim_3link_planar_B.n_j; dynsim_3link_planar_B.loop_ub++) {
          dynsim_3link_planar_B.coffset_tmp = dynsim_3link_planar_B.loop_ub * 6
            - 1;
          for (dynsim_3link_planar_B.b_i = 0; dynsim_3link_planar_B.b_i < 6;
               dynsim_3link_planar_B.b_i++) {
            dynsim_3link_planar_B.aoffset_n = dynsim_3link_planar_B.b_i * 6 - 1;
            dynsim_3link_planar_B.s = 0.0;
            for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f < 6;
                 dynsim_3link_planar_B.f++) {
              dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 1;
              dynsim_3link_planar_B.s += X->data[dynsim_3link_planar_B.pid_tmp].
                f1[dynsim_3link_planar_B.aoffset_n + dynsim_3link_planar_B.X_tmp]
                * Si->data[dynsim_3link_planar_B.coffset_tmp +
                dynsim_3link_planar_B.X_tmp];
            }

            Fi->data[(dynsim_3link_planar_B.coffset_tmp +
                      dynsim_3link_planar_B.b_i) + 1] = dynsim_3link_planar_B.s;
          }
        }

        dynsim_3link_planar_B.pid = robot->Bodies[dynsim_3link_planar_B.pid_tmp
          ]->ParentIndex;
      }
    }
  }

  dynsim_3link_pla_emxFree_char_T(&a);
  dynsim_3link_pla_emxFree_real_T(&Hji);
  dynsim_3link_pla_emxFree_real_T(&Fi);
  dynsim_3link_pla_emxFree_real_T(&Si);
  dynsim_3lin_emxFree_f_cell_wrap(&X);
  dynsim_3lin_emxFree_f_cell_wrap(&Ic);
  dynsim_3link_planar_B.mask[0] = (robot->VelocityDoFMap[0] <=
    robot->VelocityDoFMap[3]);
  dynsim_3link_planar_B.mask[1] = (robot->VelocityDoFMap[1] <=
    robot->VelocityDoFMap[4]);
  dynsim_3link_planar_B.mask[2] = (robot->VelocityDoFMap[2] <=
    robot->VelocityDoFMap[5]);
  dynsim_3link_planar_B.idx = 0;
  dynsim_3link_planar_B.f = 1;
  exitg1 = false;
  while ((!exitg1) && (dynsim_3link_planar_B.f - 1 < 3)) {
    if (dynsim_3link_planar_B.mask[dynsim_3link_planar_B.f - 1]) {
      dynsim_3link_planar_B.idx++;
      dynsim_3link_planar_B.ii_data[dynsim_3link_planar_B.idx - 1] =
        dynsim_3link_planar_B.f;
      if (dynsim_3link_planar_B.idx >= 3) {
        exitg1 = true;
      } else {
        dynsim_3link_planar_B.f++;
      }
    } else {
      dynsim_3link_planar_B.f++;
    }
  }

  if (1 > dynsim_3link_planar_B.idx) {
    dynsim_3link_planar_B.idx = 0;
  }

  for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <
       dynsim_3link_planar_B.idx; dynsim_3link_planar_B.f++) {
    dynsim_3link_planar_B.nonFixedIndices_data[dynsim_3link_planar_B.f] =
      dynsim_3link_planar_B.ii_data[dynsim_3link_planar_B.f];
  }

  dynsim_3link_planar_B.b_i = dynsim_3link_planar_B.idx - 1;
  dynsim_3link_pla_emxInit_real_T(&s, 2);
  for (dynsim_3link_planar_B.idx = 0; dynsim_3link_planar_B.idx <=
       dynsim_3link_planar_B.b_i; dynsim_3link_planar_B.idx++) {
    dynsim_3link_planar_B.vNum_n = robot->
      VelocityDoFMap[dynsim_3link_planar_B.nonFixedIndices_data[dynsim_3link_planar_B.idx]
      - 1];
    dynsim_3link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_3link_planar_B.nonFixedIndices_data[dynsim_3link_planar_B.idx]
      + 2];
    if (rtIsNaN(dynsim_3link_planar_B.vNum_n) || rtIsNaN
        (dynsim_3link_planar_B.p_idx_1)) {
      dynsim_3link_planar_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_3link_planar_B.f);
      s->data[0] = (rtNaN);
    } else if (dynsim_3link_planar_B.p_idx_1 < dynsim_3link_planar_B.vNum_n) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynsim_3link_planar_B.vNum_n) || rtIsInf
                (dynsim_3link_planar_B.p_idx_1)) &&
               (dynsim_3link_planar_B.vNum_n == dynsim_3link_planar_B.p_idx_1))
    {
      dynsim_3link_planar_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_3link_planar_B.f);
      s->data[0] = (rtNaN);
    } else if (floor(dynsim_3link_planar_B.vNum_n) ==
               dynsim_3link_planar_B.vNum_n) {
      dynsim_3link_planar_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynsim_3link_planar_B.loop_ub = static_cast<int32_T>(floor
        (dynsim_3link_planar_B.p_idx_1 - dynsim_3link_planar_B.vNum_n));
      s->size[1] = dynsim_3link_planar_B.loop_ub + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_3link_planar_B.f);
      for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
           dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
        s->data[dynsim_3link_planar_B.f] = dynsim_3link_planar_B.vNum_n +
          static_cast<real_T>(dynsim_3link_planar_B.f);
      }
    } else {
      dynsim_3link_planar_B.nb_b = floor((dynsim_3link_planar_B.p_idx_1 -
        dynsim_3link_planar_B.vNum_n) + 0.5);
      dynsim_3link_planar_B.pid = dynsim_3link_planar_B.vNum_n +
        dynsim_3link_planar_B.nb_b;
      dynsim_3link_planar_B.b_idx_0_b = dynsim_3link_planar_B.pid -
        dynsim_3link_planar_B.p_idx_1;
      dynsim_3link_planar_B.b_idx_1_l = fabs(dynsim_3link_planar_B.vNum_n);
      dynsim_3link_planar_B.s = fabs(dynsim_3link_planar_B.p_idx_1);
      if ((dynsim_3link_planar_B.b_idx_1_l > dynsim_3link_planar_B.s) || rtIsNaN
          (dynsim_3link_planar_B.s)) {
        dynsim_3link_planar_B.s = dynsim_3link_planar_B.b_idx_1_l;
      }

      if (fabs(dynsim_3link_planar_B.b_idx_0_b) < 4.4408920985006262E-16 *
          dynsim_3link_planar_B.s) {
        dynsim_3link_planar_B.nb_b++;
        dynsim_3link_planar_B.pid = dynsim_3link_planar_B.p_idx_1;
      } else if (dynsim_3link_planar_B.b_idx_0_b > 0.0) {
        dynsim_3link_planar_B.pid = (dynsim_3link_planar_B.nb_b - 1.0) +
          dynsim_3link_planar_B.vNum_n;
      } else {
        dynsim_3link_planar_B.nb_b++;
      }

      if (dynsim_3link_planar_B.nb_b >= 0.0) {
        dynsim_3link_planar_B.f = static_cast<int32_T>
          (dynsim_3link_planar_B.nb_b);
      } else {
        dynsim_3link_planar_B.f = 0;
      }

      dynsim_3link_planar_B.n_j = dynsim_3link_planar_B.f - 1;
      dynsim_3link_planar_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynsim_3link_planar_B.n_j + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_3link_planar_B.f);
      if (dynsim_3link_planar_B.n_j + 1 > 0) {
        s->data[0] = dynsim_3link_planar_B.vNum_n;
        if (dynsim_3link_planar_B.n_j + 1 > 1) {
          s->data[dynsim_3link_planar_B.n_j] = dynsim_3link_planar_B.pid;
          dynsim_3link_planar_B.nm1d2 = ((dynsim_3link_planar_B.n_j < 0) +
            dynsim_3link_planar_B.n_j) >> 1;
          dynsim_3link_planar_B.loop_ub = dynsim_3link_planar_B.nm1d2 - 2;
          for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <=
               dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
            dynsim_3link_planar_B.X_tmp = dynsim_3link_planar_B.f + 1;
            s->data[dynsim_3link_planar_B.X_tmp] = dynsim_3link_planar_B.vNum_n
              + static_cast<real_T>(dynsim_3link_planar_B.X_tmp);
            s->data[dynsim_3link_planar_B.n_j - dynsim_3link_planar_B.X_tmp] =
              dynsim_3link_planar_B.pid - static_cast<real_T>
              (dynsim_3link_planar_B.X_tmp);
          }

          if (dynsim_3link_planar_B.nm1d2 << 1 == dynsim_3link_planar_B.n_j) {
            s->data[dynsim_3link_planar_B.nm1d2] = (dynsim_3link_planar_B.vNum_n
              + dynsim_3link_planar_B.pid) / 2.0;
          } else {
            s->data[dynsim_3link_planar_B.nm1d2] = dynsim_3link_planar_B.vNum_n
              + static_cast<real_T>(dynsim_3link_planar_B.nm1d2);
            s->data[dynsim_3link_planar_B.nm1d2 + 1] = dynsim_3link_planar_B.pid
              - static_cast<real_T>(dynsim_3link_planar_B.nm1d2);
          }
        }
      }
    }

    if (dynsim_3link_planar_B.vNum_n > dynsim_3link_planar_B.p_idx_1) {
      dynsim_3link_planar_B.nm1d2 = 0;
    } else {
      dynsim_3link_planar_B.nm1d2 = static_cast<int32_T>
        (dynsim_3link_planar_B.vNum_n) - 1;
    }

    dynsim_3link_planar_B.loop_ub = s->size[1];
    for (dynsim_3link_planar_B.f = 0; dynsim_3link_planar_B.f <
         dynsim_3link_planar_B.loop_ub; dynsim_3link_planar_B.f++) {
      lambda->data[dynsim_3link_planar_B.nm1d2 + dynsim_3link_planar_B.f] =
        s->data[dynsim_3link_planar_B.f] - 1.0;
    }

    if (lambda_->
        data[dynsim_3link_planar_B.nonFixedIndices_data[dynsim_3link_planar_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(dynsim_3link_planar_B.vNum_n) - 1] = 0.0;
    } else {
      dynsim_3link_planar_B.f = static_cast<int32_T>(lambda_->
        data[dynsim_3link_planar_B.nonFixedIndices_data[dynsim_3link_planar_B.idx]
        - 1]);
      dynsim_3link_planar_B.b_idx_1_l = robot->
        VelocityDoFMap[dynsim_3link_planar_B.f + 2];
      lambda->data[static_cast<int32_T>(dynsim_3link_planar_B.vNum_n) - 1] =
        dynsim_3link_planar_B.b_idx_1_l;
    }
  }

  dynsim_3link_pla_emxFree_real_T(&s);
  dynsim_3link_pla_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[3], const real_T qdot[3], const real_T fext[18], real_T
  tau[3])
{
  emxArray_f_cell_wrap_dynsim_3_T *X;
  emxArray_f_cell_wrap_dynsim_3_T *Xtree;
  emxArray_real_T_dynsim_3link__T *vJ;
  emxArray_real_T_dynsim_3link__T *vB;
  emxArray_real_T_dynsim_3link__T *aB;
  emxArray_real_T_dynsim_3link__T *f;
  emxArray_real_T_dynsim_3link__T *S;
  emxArray_real_T_dynsim_3link__T *taui;
  n_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_3link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  dynsim_3link_planar_B.a0[0] = 0.0;
  dynsim_3link_planar_B.a0[1] = 0.0;
  dynsim_3link_planar_B.a0[2] = 0.0;
  dynsim_3link_planar_B.a0[3] = -robot->Gravity[0];
  dynsim_3link_planar_B.a0[4] = -robot->Gravity[1];
  dynsim_3link_planar_B.a0[5] = -robot->Gravity[2];
  dynsim_3link_pla_emxInit_real_T(&vJ, 2);
  dynsim_3link_planar_B.nb = robot->NumBodies;
  dynsim_3link_planar_B.b_k_f = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynsim_3link_planar_B.unnamed_idx_1 = static_cast<int32_T>
    (dynsim_3link_planar_B.nb);
  vJ->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vJ, dynsim_3link_planar_B.b_k_f);
  dynsim_3link_planar_B.loop_ub_tmp = 6 * dynsim_3link_planar_B.unnamed_idx_1 -
    1;
  for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
       dynsim_3link_planar_B.loop_ub_tmp; dynsim_3link_planar_B.b_k_f++) {
    vJ->data[dynsim_3link_planar_B.b_k_f] = 0.0;
  }

  dynsim_3link_pla_emxInit_real_T(&vB, 2);
  dynsim_3link_planar_B.b_k_f = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vB, dynsim_3link_planar_B.b_k_f);
  for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
       dynsim_3link_planar_B.loop_ub_tmp; dynsim_3link_planar_B.b_k_f++) {
    vB->data[dynsim_3link_planar_B.b_k_f] = 0.0;
  }

  dynsim_3link_pla_emxInit_real_T(&aB, 2);
  dynsim_3link_planar_B.b_k_f = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(aB, dynsim_3link_planar_B.b_k_f);
  for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
       dynsim_3link_planar_B.loop_ub_tmp; dynsim_3link_planar_B.b_k_f++) {
    aB->data[dynsim_3link_planar_B.b_k_f] = 0.0;
  }

  tau[0] = 0.0;
  tau[1] = 0.0;
  tau[2] = 0.0;
  dynsim_3lin_emxInit_f_cell_wrap(&X, 2);
  dynsim_3lin_emxInit_f_cell_wrap(&Xtree, 2);
  dynsim_3link_planar_B.loop_ub_tmp = dynsim_3link_planar_B.unnamed_idx_1 - 1;
  dynsim_3link_planar_B.b_k_f = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(Xtree, dynsim_3link_planar_B.b_k_f);
  dynsim_3link_planar_B.b_k_f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_3link_planar_B.b_k_f);
  for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k <=
       dynsim_3link_planar_B.loop_ub_tmp; dynsim_3link_planar_B.b_k++) {
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 36;
         dynsim_3link_planar_B.b_k_f++) {
      Xtree->data[dynsim_3link_planar_B.b_k].f1[dynsim_3link_planar_B.b_k_f] =
        0.0;
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
         dynsim_3link_planar_B.b_k_f++) {
      Xtree->data[dynsim_3link_planar_B.b_k].f1[dynsim_3link_planar_B.b_k_f + 6 *
        dynsim_3link_planar_B.b_k_f] = 1.0;
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 36;
         dynsim_3link_planar_B.b_k_f++) {
      X->data[dynsim_3link_planar_B.b_k].f1[dynsim_3link_planar_B.b_k_f] = 0.0;
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
         dynsim_3link_planar_B.b_k_f++) {
      X->data[dynsim_3link_planar_B.b_k].f1[dynsim_3link_planar_B.b_k_f + 6 *
        dynsim_3link_planar_B.b_k_f] = 1.0;
    }
  }

  dynsim_3link_pla_emxInit_real_T(&f, 2);
  dynsim_3link_planar_B.b_k_f = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynsim_3link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(f, dynsim_3link_planar_B.b_k_f);
  dynsim_3link_pla_emxInit_real_T(&S, 2);
  if (0 <= dynsim_3link_planar_B.loop_ub_tmp) {
    dynsim_3link_planar_B.dv1[0] = 0.0;
    dynsim_3link_planar_B.dv1[4] = 0.0;
    dynsim_3link_planar_B.dv1[8] = 0.0;
  }

  for (dynsim_3link_planar_B.unnamed_idx_1 = 0;
       dynsim_3link_planar_B.unnamed_idx_1 <= dynsim_3link_planar_B.loop_ub_tmp;
       dynsim_3link_planar_B.unnamed_idx_1++) {
    obj = robot->Bodies[dynsim_3link_planar_B.unnamed_idx_1];
    dynsim_3link_planar_B.b_k_f = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynsim_emxEnsureCapacity_real_T(S, dynsim_3link_planar_B.b_k_f);
    dynsim_3link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
         dynsim_3link_planar_B.b_k; dynsim_3link_planar_B.b_k_f++) {
      S->data[dynsim_3link_planar_B.b_k_f] = obj->
        JointInternal.MotionSubspace->data[dynsim_3link_planar_B.b_k_f];
    }

    dynsim_3link_planar_B.a_idx_0 = robot->
      PositionDoFMap[dynsim_3link_planar_B.unnamed_idx_1];
    dynsim_3link_planar_B.a_idx_1 = robot->
      PositionDoFMap[dynsim_3link_planar_B.unnamed_idx_1 + 3];
    dynsim_3link_planar_B.b_idx_0 = robot->
      VelocityDoFMap[dynsim_3link_planar_B.unnamed_idx_1];
    dynsim_3link_planar_B.b_idx_1 = robot->
      VelocityDoFMap[dynsim_3link_planar_B.unnamed_idx_1 + 3];
    if (dynsim_3link_planar_B.a_idx_1 < dynsim_3link_planar_B.a_idx_0) {
      obj = robot->Bodies[dynsim_3link_planar_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_3link_planar_B.T);
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        vJ->data[dynsim_3link_planar_B.b_k_f + 6 *
          dynsim_3link_planar_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynsim_3link_planar_B.a_idx_0 > dynsim_3link_planar_B.a_idx_1) {
        dynsim_3link_planar_B.b_k = 0;
        dynsim_3link_planar_B.b_k_f = -1;
      } else {
        dynsim_3link_planar_B.b_k = static_cast<int32_T>
          (dynsim_3link_planar_B.a_idx_0) - 1;
        dynsim_3link_planar_B.b_k_f = static_cast<int32_T>
          (dynsim_3link_planar_B.a_idx_1) - 1;
      }

      if (dynsim_3link_planar_B.b_idx_0 > dynsim_3link_planar_B.b_idx_1) {
        dynsim_3link_planar_B.p = -1;
      } else {
        dynsim_3link_planar_B.p = static_cast<int32_T>
          (dynsim_3link_planar_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynsim_3link_planar_B.unnamed_idx_1];
      dynsim_3link_planar_B.q_size_tmp = dynsim_3link_planar_B.b_k_f -
        dynsim_3link_planar_B.b_k;
      dynsim_3link_planar_B.q_size = dynsim_3link_planar_B.q_size_tmp + 1;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
           dynsim_3link_planar_B.q_size_tmp; dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.q_data[dynsim_3link_planar_B.b_k_f] =
          q[dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f];
      }

      rigidBodyJoint_transformBodyT_j(&obj->JointInternal,
        dynsim_3link_planar_B.q_data, &dynsim_3link_planar_B.q_size,
        dynsim_3link_planar_B.T);
      dynsim_3link_planar_B.inner = S->size[1] - 1;
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
           dynsim_3link_planar_B.b_k++) {
        vJ->data[dynsim_3link_planar_B.b_k + 6 *
          dynsim_3link_planar_B.unnamed_idx_1] = 0.0;
      }

      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k <=
           dynsim_3link_planar_B.inner; dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.aoffset = dynsim_3link_planar_B.b_k * 6 - 1;
        for (dynsim_3link_planar_B.q_size_tmp = 0;
             dynsim_3link_planar_B.q_size_tmp < 6;
             dynsim_3link_planar_B.q_size_tmp++) {
          dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1
            + dynsim_3link_planar_B.q_size_tmp;
          vJ->data[dynsim_3link_planar_B.b_k_f] += S->data
            [(dynsim_3link_planar_B.aoffset + dynsim_3link_planar_B.q_size_tmp)
            + 1] * qdot[(dynsim_3link_planar_B.p + dynsim_3link_planar_B.b_k) +
            1];
        }
      }
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f] =
        dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f + 1] =
        dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k_f + 4];
      dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f + 2] =
        dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k_f + 8];
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 9;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.b_k_f] =
        -dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f];
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.b_k = dynsim_3link_planar_B.b_k_f << 2;
      dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k] =
        dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k + 1] =
        dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f + 1];
      dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k + 2] =
        dynsim_3link_planar_B.R_c[3 * dynsim_3link_planar_B.b_k_f + 2];
      dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k_f + 12] =
        dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.b_k_f + 6] *
        dynsim_3link_planar_B.T[14] +
        (dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.b_k_f + 3] *
         dynsim_3link_planar_B.T[13] +
         dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.b_k_f] *
         dynsim_3link_planar_B.T[12]);
    }

    dynsim_3link_planar_B.Tinv[3] = 0.0;
    dynsim_3link_planar_B.Tinv[7] = 0.0;
    dynsim_3link_planar_B.Tinv[11] = 0.0;
    dynsim_3link_planar_B.Tinv[15] = 1.0;
    dynsim_3link_planar_B.dv1[3] = -dynsim_3link_planar_B.Tinv[14];
    dynsim_3link_planar_B.dv1[6] = dynsim_3link_planar_B.Tinv[13];
    dynsim_3link_planar_B.dv1[1] = dynsim_3link_planar_B.Tinv[14];
    dynsim_3link_planar_B.dv1[7] = -dynsim_3link_planar_B.Tinv[12];
    dynsim_3link_planar_B.dv1[2] = -dynsim_3link_planar_B.Tinv[13];
    dynsim_3link_planar_B.dv1[5] = dynsim_3link_planar_B.Tinv[12];
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
         dynsim_3link_planar_B.b_k_f++) {
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 3;
           dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.q_size_tmp = dynsim_3link_planar_B.b_k_f + 3 *
          dynsim_3link_planar_B.b_k;
        dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.q_size_tmp] = 0.0;
        dynsim_3link_planar_B.p = dynsim_3link_planar_B.b_k << 2;
        dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.q_size_tmp] +=
          dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.p] *
          dynsim_3link_planar_B.dv1[dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.q_size_tmp] +=
          dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.p + 1] *
          dynsim_3link_planar_B.dv1[dynsim_3link_planar_B.b_k_f + 3];
        dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.q_size_tmp] +=
          dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.p + 2] *
          dynsim_3link_planar_B.dv1[dynsim_3link_planar_B.b_k_f + 6];
        X->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.b_k + 6 * dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.Tinv[(dynsim_3link_planar_B.b_k_f << 2) +
          dynsim_3link_planar_B.b_k];
        X->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.b_k + 6 * (dynsim_3link_planar_B.b_k_f + 3)]
          = 0.0;
      }
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
         dynsim_3link_planar_B.b_k_f++) {
      X->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_3link_planar_B.b_k_f + 3] = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.b_k = dynsim_3link_planar_B.b_k_f << 2;
      dynsim_3link_planar_B.q_size_tmp = 6 * (dynsim_3link_planar_B.b_k_f + 3);
      X->data[dynsim_3link_planar_B.unnamed_idx_1]
        .f1[dynsim_3link_planar_B.q_size_tmp + 3] =
        dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k];
      X->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_3link_planar_B.b_k_f + 4] = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f + 1];
      X->data[dynsim_3link_planar_B.unnamed_idx_1]
        .f1[dynsim_3link_planar_B.q_size_tmp + 4] =
        dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k + 1];
      X->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_3link_planar_B.b_k_f + 5] = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f + 2];
      X->data[dynsim_3link_planar_B.unnamed_idx_1]
        .f1[dynsim_3link_planar_B.q_size_tmp + 5] =
        dynsim_3link_planar_B.Tinv[dynsim_3link_planar_B.b_k + 2];
    }

    dynsim_3link_planar_B.a_idx_0 = robot->
      Bodies[dynsim_3link_planar_B.unnamed_idx_1]->ParentIndex;
    if (dynsim_3link_planar_B.a_idx_0 > 0.0) {
      dynsim_3link_planar_B.m = static_cast<int32_T>
        (dynsim_3link_planar_B.a_idx_0);
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.a_idx_1 = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 += vB->data[(dynsim_3link_planar_B.m - 1)
            * 6 + dynsim_3link_planar_B.b_k] * X->
            data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f];
        }

        dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f] = vJ->data[6 *
          dynsim_3link_planar_B.unnamed_idx_1 + dynsim_3link_planar_B.b_k_f] +
          dynsim_3link_planar_B.a_idx_1;
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        vB->data[dynsim_3link_planar_B.b_k_f + 6 *
          dynsim_3link_planar_B.unnamed_idx_1] =
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f];
      }

      dynsim_3link_planar_B.inner = S->size[1] - 1;
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
           dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k] = 0.0;
      }

      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k <=
           dynsim_3link_planar_B.inner; dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.aoffset = dynsim_3link_planar_B.b_k * 6 - 1;
        for (dynsim_3link_planar_B.q_size_tmp = 0;
             dynsim_3link_planar_B.q_size_tmp < 6;
             dynsim_3link_planar_B.q_size_tmp++) {
          dynsim_3link_planar_B.a_idx_1 = S->data[(dynsim_3link_planar_B.aoffset
            + dynsim_3link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_3link_planar_B.y[dynsim_3link_planar_B.q_size_tmp];
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.q_size_tmp] =
            dynsim_3link_planar_B.a_idx_1;
        }
      }

      dynsim_3link_planar_B.R_c[0] = 0.0;
      dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 2;
      dynsim_3link_planar_B.R_c[3] = -vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 1;
      dynsim_3link_planar_B.R_c[6] = vB->data[dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R_c[1] = vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.R_c[4] = 0.0;
      dynsim_3link_planar_B.R_c[7] = -vB->data[6 *
        dynsim_3link_planar_B.unnamed_idx_1];
      dynsim_3link_planar_B.R_c[2] = -vB->data[dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R_c[5] = vB->data[6 *
        dynsim_3link_planar_B.unnamed_idx_1];
      dynsim_3link_planar_B.R_c[8] = 0.0;
      dynsim_3link_planar_B.R[3] = 0.0;
      dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 5;
      dynsim_3link_planar_B.R[9] = -vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 4;
      dynsim_3link_planar_B.R[15] = vB->data[dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R[4] = vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.R[10] = 0.0;
      dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 3;
      dynsim_3link_planar_B.R[16] = -vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.R[5] = -vB->data[dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R[11] = vB->data[dynsim_3link_planar_B.b_k];
      dynsim_3link_planar_B.R[17] = 0.0;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
          dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.b_k = 6 * (dynsim_3link_planar_B.b_k_f + 3);
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k] = 0.0;
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 3] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
          dynsim_3link_planar_B.b_k_f + 1];
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 1] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 1] = 0.0;
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 4] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
          dynsim_3link_planar_B.b_k_f + 2];
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 2] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 2] = 0.0;
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 5] =
          dynsim_3link_planar_B.a_idx_1;
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.a_idx_1 = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 += aB->data[(dynsim_3link_planar_B.m - 1)
            * 6 + dynsim_3link_planar_B.b_k] * X->
            data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f];
        }

        dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.a_idx_1 +
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f];
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f] = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R[6 *
            dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f] * vJ->data
            [6 * dynsim_3link_planar_B.unnamed_idx_1 + dynsim_3link_planar_B.b_k]
            + dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f];
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f] =
            dynsim_3link_planar_B.a_idx_1;
        }

        aB->data[dynsim_3link_planar_B.b_k_f + 6 *
          dynsim_3link_planar_B.unnamed_idx_1] =
          dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k_f] +
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f];
      }

      dynsim_3link_planar_B.R_c[0] = 0.0;
      dynsim_3link_planar_B.R_c[3] = -dynsim_3link_planar_B.T[14];
      dynsim_3link_planar_B.R_c[6] = dynsim_3link_planar_B.T[13];
      dynsim_3link_planar_B.R_c[1] = dynsim_3link_planar_B.T[14];
      dynsim_3link_planar_B.R_c[4] = 0.0;
      dynsim_3link_planar_B.R_c[7] = -dynsim_3link_planar_B.T[12];
      dynsim_3link_planar_B.R_c[2] = -dynsim_3link_planar_B.T[13];
      dynsim_3link_planar_B.R_c[5] = dynsim_3link_planar_B.T[12];
      dynsim_3link_planar_B.R_c[8] = 0.0;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
           dynsim_3link_planar_B.b_k_f++) {
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 3;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.q_size_tmp = dynsim_3link_planar_B.b_k_f + 3 *
            dynsim_3link_planar_B.b_k;
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] = 0.0;
          dynsim_3link_planar_B.p = dynsim_3link_planar_B.b_k << 2;
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f];
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p + 1] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f + 3];
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p + 2] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f + 6];
          dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 6 *
            dynsim_3link_planar_B.b_k_f] = dynsim_3link_planar_B.T
            [(dynsim_3link_planar_B.b_k_f << 2) + dynsim_3link_planar_B.b_k];
          dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 6 *
            (dynsim_3link_planar_B.b_k_f + 3)] = 0.0;
        }
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 3] =
          dynsim_3link_planar_B.R_b[3 * dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.b_k = dynsim_3link_planar_B.b_k_f << 2;
        dynsim_3link_planar_B.q_size_tmp = 6 * (dynsim_3link_planar_B.b_k_f + 3);
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.q_size_tmp + 3] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k];
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 4] =
          dynsim_3link_planar_B.R_b[3 * dynsim_3link_planar_B.b_k_f + 1];
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.q_size_tmp + 4] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k + 1];
        dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 5] =
          dynsim_3link_planar_B.R_b[3 * dynsim_3link_planar_B.b_k_f + 2];
        dynsim_3link_planar_B.R[dynsim_3link_planar_B.q_size_tmp + 5] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k + 2];
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.p = dynsim_3link_planar_B.b_k_f + 6 *
            dynsim_3link_planar_B.b_k;
          dynsim_3link_planar_B.b_I[dynsim_3link_planar_B.p] = 0.0;
          for (dynsim_3link_planar_B.q_size_tmp = 0;
               dynsim_3link_planar_B.q_size_tmp < 6;
               dynsim_3link_planar_B.q_size_tmp++) {
            dynsim_3link_planar_B.b_I[dynsim_3link_planar_B.p] += Xtree->data[
              static_cast<int32_T>(dynsim_3link_planar_B.a_idx_0) - 1].f1[6 *
              dynsim_3link_planar_B.q_size_tmp + dynsim_3link_planar_B.b_k_f] *
              dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k +
              dynsim_3link_planar_B.q_size_tmp];
          }
        }
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 36;
           dynsim_3link_planar_B.b_k_f++) {
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.b_I[dynsim_3link_planar_B.b_k_f];
      }
    } else {
      dynsim_3link_planar_B.inner = S->size[1] - 1;
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
           dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1 +
          dynsim_3link_planar_B.b_k;
        vB->data[dynsim_3link_planar_B.b_k_f] = vJ->
          data[dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k] = 0.0;
      }

      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k <=
           dynsim_3link_planar_B.inner; dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.aoffset = dynsim_3link_planar_B.b_k * 6 - 1;
        for (dynsim_3link_planar_B.q_size_tmp = 0;
             dynsim_3link_planar_B.q_size_tmp < 6;
             dynsim_3link_planar_B.q_size_tmp++) {
          dynsim_3link_planar_B.a_idx_1 = S->data[(dynsim_3link_planar_B.aoffset
            + dynsim_3link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_3link_planar_B.y[dynsim_3link_planar_B.q_size_tmp];
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.q_size_tmp] =
            dynsim_3link_planar_B.a_idx_1;
        }
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.a_idx_1 = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 += X->
            data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f] *
            dynsim_3link_planar_B.a0[dynsim_3link_planar_B.b_k];
        }

        aB->data[dynsim_3link_planar_B.b_k_f + 6 *
          dynsim_3link_planar_B.unnamed_idx_1] = dynsim_3link_planar_B.a_idx_1 +
          dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f];
      }

      dynsim_3link_planar_B.R_c[0] = 0.0;
      dynsim_3link_planar_B.R_c[3] = -dynsim_3link_planar_B.T[14];
      dynsim_3link_planar_B.R_c[6] = dynsim_3link_planar_B.T[13];
      dynsim_3link_planar_B.R_c[1] = dynsim_3link_planar_B.T[14];
      dynsim_3link_planar_B.R_c[4] = 0.0;
      dynsim_3link_planar_B.R_c[7] = -dynsim_3link_planar_B.T[12];
      dynsim_3link_planar_B.R_c[2] = -dynsim_3link_planar_B.T[13];
      dynsim_3link_planar_B.R_c[5] = dynsim_3link_planar_B.T[12];
      dynsim_3link_planar_B.R_c[8] = 0.0;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
           dynsim_3link_planar_B.b_k_f++) {
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 3;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.q_size_tmp = dynsim_3link_planar_B.b_k_f + 3 *
            dynsim_3link_planar_B.b_k;
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] = 0.0;
          dynsim_3link_planar_B.p = dynsim_3link_planar_B.b_k << 2;
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f];
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p + 1] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f + 3];
          dynsim_3link_planar_B.R_b[dynsim_3link_planar_B.q_size_tmp] +=
            dynsim_3link_planar_B.T[dynsim_3link_planar_B.p + 2] *
            dynsim_3link_planar_B.R_c[dynsim_3link_planar_B.b_k_f + 6];
          Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
            .f1[dynsim_3link_planar_B.b_k + 6 * dynsim_3link_planar_B.b_k_f] =
            dynsim_3link_planar_B.T[(dynsim_3link_planar_B.b_k_f << 2) +
            dynsim_3link_planar_B.b_k];
          Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
            .f1[dynsim_3link_planar_B.b_k + 6 * (dynsim_3link_planar_B.b_k_f + 3)]
            = 0.0;
        }
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
           dynsim_3link_planar_B.b_k_f++) {
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_3link_planar_B.b_k_f + 3] = dynsim_3link_planar_B.R_b[3 *
          dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.b_k = dynsim_3link_planar_B.b_k_f << 2;
        dynsim_3link_planar_B.q_size_tmp = 6 * (dynsim_3link_planar_B.b_k_f + 3);
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.q_size_tmp + 3] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k];
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_3link_planar_B.b_k_f + 4] = dynsim_3link_planar_B.R_b[3 *
          dynsim_3link_planar_B.b_k_f + 1];
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.q_size_tmp + 4] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k + 1];
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_3link_planar_B.b_k_f + 5] = dynsim_3link_planar_B.R_b[3 *
          dynsim_3link_planar_B.b_k_f + 2];
        Xtree->data[dynsim_3link_planar_B.unnamed_idx_1]
          .f1[dynsim_3link_planar_B.q_size_tmp + 5] =
          dynsim_3link_planar_B.T[dynsim_3link_planar_B.b_k + 2];
      }
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 36;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.b_I[dynsim_3link_planar_B.b_k_f] = robot->
        Bodies[dynsim_3link_planar_B.unnamed_idx_1]->
        SpatialInertia[dynsim_3link_planar_B.b_k_f];
    }

    dynsim_3link_planar_B.R_c[0] = 0.0;
    dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 2;
    dynsim_3link_planar_B.R_c[3] = -vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 1;
    dynsim_3link_planar_B.R_c[6] = vB->data[dynsim_3link_planar_B.b_k_f];
    dynsim_3link_planar_B.R_c[1] = vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.R_c[4] = 0.0;
    dynsim_3link_planar_B.R_c[7] = -vB->data[6 *
      dynsim_3link_planar_B.unnamed_idx_1];
    dynsim_3link_planar_B.R_c[2] = -vB->data[dynsim_3link_planar_B.b_k_f];
    dynsim_3link_planar_B.R_c[5] = vB->data[6 *
      dynsim_3link_planar_B.unnamed_idx_1];
    dynsim_3link_planar_B.R_c[8] = 0.0;
    dynsim_3link_planar_B.R[18] = 0.0;
    dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 5;
    dynsim_3link_planar_B.R[24] = -vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.b_k_f = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 4;
    dynsim_3link_planar_B.R[30] = vB->data[dynsim_3link_planar_B.b_k_f];
    dynsim_3link_planar_B.R[19] = vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.R[25] = 0.0;
    dynsim_3link_planar_B.b_k = 6 * dynsim_3link_planar_B.unnamed_idx_1 + 3;
    dynsim_3link_planar_B.R[31] = -vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.R[20] = -vB->data[dynsim_3link_planar_B.b_k_f];
    dynsim_3link_planar_B.R[26] = vB->data[dynsim_3link_planar_B.b_k];
    dynsim_3link_planar_B.R[32] = 0.0;
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 3;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f];
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f] =
        dynsim_3link_planar_B.a_idx_1;
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 3] = 0.0;
      dynsim_3link_planar_B.b_k = 6 * (dynsim_3link_planar_B.b_k_f + 3);
      dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 3] =
        dynsim_3link_planar_B.a_idx_1;
      dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f + 1];
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 1] =
        dynsim_3link_planar_B.a_idx_1;
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 4] = 0.0;
      dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 4] =
        dynsim_3link_planar_B.a_idx_1;
      dynsim_3link_planar_B.a_idx_1 = dynsim_3link_planar_B.R_c[3 *
        dynsim_3link_planar_B.b_k_f + 2];
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 2] =
        dynsim_3link_planar_B.a_idx_1;
      dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k_f + 5] = 0.0;
      dynsim_3link_planar_B.R[dynsim_3link_planar_B.b_k + 5] =
        dynsim_3link_planar_B.a_idx_1;
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k_f] = 0.0;
      dynsim_3link_planar_B.b_I_n[dynsim_3link_planar_B.b_k_f] = 0.0;
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
           dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.a_idx_0 = dynsim_3link_planar_B.b_I[6 *
          dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.q_size_tmp = 6 *
          dynsim_3link_planar_B.unnamed_idx_1 + dynsim_3link_planar_B.b_k;
        dynsim_3link_planar_B.a_idx_1 = vB->
          data[dynsim_3link_planar_B.q_size_tmp] * dynsim_3link_planar_B.a_idx_0
          + dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.a_idx_0 = aB->
          data[dynsim_3link_planar_B.q_size_tmp] * dynsim_3link_planar_B.a_idx_0
          + dynsim_3link_planar_B.b_I_n[dynsim_3link_planar_B.b_k_f];
        dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.a_idx_1;
        dynsim_3link_planar_B.b_I_n[dynsim_3link_planar_B.b_k_f] =
          dynsim_3link_planar_B.a_idx_0;
      }
    }

    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f] = 0.0;
      dynsim_3link_planar_B.a_idx_1 = 0.0;
      for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
           dynsim_3link_planar_B.b_k++) {
        dynsim_3link_planar_B.a_idx_1 += Xtree->
          data[dynsim_3link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_3link_planar_B.b_k_f + dynsim_3link_planar_B.b_k] * fext[6 *
          dynsim_3link_planar_B.unnamed_idx_1 + dynsim_3link_planar_B.b_k];
        dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f] +=
          dynsim_3link_planar_B.R[6 * dynsim_3link_planar_B.b_k +
          dynsim_3link_planar_B.b_k_f] *
          dynsim_3link_planar_B.X_m[dynsim_3link_planar_B.b_k];
      }

      f->data[dynsim_3link_planar_B.b_k_f + 6 *
        dynsim_3link_planar_B.unnamed_idx_1] =
        (dynsim_3link_planar_B.b_I_n[dynsim_3link_planar_B.b_k_f] +
         dynsim_3link_planar_B.y[dynsim_3link_planar_B.b_k_f]) -
        dynsim_3link_planar_B.a_idx_1;
    }
  }

  dynsim_3link_pla_emxFree_real_T(&aB);
  dynsim_3link_pla_emxFree_real_T(&vB);
  dynsim_3link_pla_emxFree_real_T(&vJ);
  dynsim_3lin_emxFree_f_cell_wrap(&Xtree);
  dynsim_3link_planar_B.q_size_tmp = static_cast<int32_T>(((-1.0 -
    dynsim_3link_planar_B.nb) + 1.0) / -1.0) - 1;
  dynsim_3link_pla_emxInit_real_T(&taui, 1);
  dynsim_3link_pla_emxInit_char_T(&a, 2);
  if (0 <= dynsim_3link_planar_B.q_size_tmp) {
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 5;
         dynsim_3link_planar_B.b_k_f++) {
      dynsim_3link_planar_B.b_b[dynsim_3link_planar_B.b_k_f] =
        tmp[dynsim_3link_planar_B.b_k_f];
    }
  }

  for (dynsim_3link_planar_B.loop_ub_tmp = 0; dynsim_3link_planar_B.loop_ub_tmp <=
       dynsim_3link_planar_B.q_size_tmp; dynsim_3link_planar_B.loop_ub_tmp++) {
    dynsim_3link_planar_B.a_idx_0 = dynsim_3link_planar_B.nb +
      -static_cast<real_T>(dynsim_3link_planar_B.loop_ub_tmp);
    dynsim_3link_planar_B.p = static_cast<int32_T>(dynsim_3link_planar_B.a_idx_0);
    dynsim_3link_planar_B.inner = dynsim_3link_planar_B.p - 1;
    obj = robot->Bodies[dynsim_3link_planar_B.inner];
    dynsim_3link_planar_B.b_k_f = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(a, dynsim_3link_planar_B.b_k_f);
    dynsim_3link_planar_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
         dynsim_3link_planar_B.b_k; dynsim_3link_planar_B.b_k_f++) {
      a->data[dynsim_3link_planar_B.b_k_f] = obj->JointInternal.Type->
        data[dynsim_3link_planar_B.b_k_f];
    }

    dynsim_3link_planar_B.b_bool = false;
    if (a->size[1] == 5) {
      dynsim_3link_planar_B.b_k_f = 1;
      do {
        exitg1 = 0;
        if (dynsim_3link_planar_B.b_k_f - 1 < 5) {
          dynsim_3link_planar_B.unnamed_idx_1 = dynsim_3link_planar_B.b_k_f - 1;
          if (a->data[dynsim_3link_planar_B.unnamed_idx_1] !=
              dynsim_3link_planar_B.b_b[dynsim_3link_planar_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynsim_3link_planar_B.b_k_f++;
          }
        } else {
          dynsim_3link_planar_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynsim_3link_planar_B.b_bool) {
      obj = robot->Bodies[dynsim_3link_planar_B.inner];
      dynsim_3link_planar_B.b_k_f = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(S, dynsim_3link_planar_B.b_k_f);
      dynsim_3link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <=
           dynsim_3link_planar_B.b_k; dynsim_3link_planar_B.b_k_f++) {
        S->data[dynsim_3link_planar_B.b_k_f] = obj->
          JointInternal.MotionSubspace->data[dynsim_3link_planar_B.b_k_f];
      }

      dynsim_3link_planar_B.m = S->size[1] - 1;
      dynsim_3link_planar_B.b_k_f = taui->size[0];
      taui->size[0] = S->size[1];
      dynsim_emxEnsureCapacity_real_T(taui, dynsim_3link_planar_B.b_k_f);
      for (dynsim_3link_planar_B.unnamed_idx_1 = 0;
           dynsim_3link_planar_B.unnamed_idx_1 <= dynsim_3link_planar_B.m;
           dynsim_3link_planar_B.unnamed_idx_1++) {
        dynsim_3link_planar_B.aoffset = dynsim_3link_planar_B.unnamed_idx_1 * 6
          - 1;
        dynsim_3link_planar_B.a_idx_1 = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 += f->data[(static_cast<int32_T>
            (dynsim_3link_planar_B.a_idx_0) - 1) * 6 + dynsim_3link_planar_B.b_k]
            * S->data[(dynsim_3link_planar_B.aoffset + dynsim_3link_planar_B.b_k)
            + 1];
        }

        taui->data[dynsim_3link_planar_B.unnamed_idx_1] =
          dynsim_3link_planar_B.a_idx_1;
      }

      dynsim_3link_planar_B.b_idx_0 = robot->
        VelocityDoFMap[dynsim_3link_planar_B.p - 1];
      dynsim_3link_planar_B.b_idx_1 = robot->
        VelocityDoFMap[dynsim_3link_planar_B.p + 2];
      if (dynsim_3link_planar_B.b_idx_0 > dynsim_3link_planar_B.b_idx_1) {
        dynsim_3link_planar_B.b_k = 0;
        dynsim_3link_planar_B.b_k_f = 0;
      } else {
        dynsim_3link_planar_B.b_k = static_cast<int32_T>
          (dynsim_3link_planar_B.b_idx_0) - 1;
        dynsim_3link_planar_B.b_k_f = static_cast<int32_T>
          (dynsim_3link_planar_B.b_idx_1);
      }

      dynsim_3link_planar_B.unnamed_idx_1 = dynsim_3link_planar_B.b_k_f -
        dynsim_3link_planar_B.b_k;
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f <
           dynsim_3link_planar_B.unnamed_idx_1; dynsim_3link_planar_B.b_k_f++) {
        tau[dynsim_3link_planar_B.b_k + dynsim_3link_planar_B.b_k_f] =
          taui->data[dynsim_3link_planar_B.b_k_f];
      }
    }

    dynsim_3link_planar_B.a_idx_0 = robot->Bodies[dynsim_3link_planar_B.inner]
      ->ParentIndex;
    if (dynsim_3link_planar_B.a_idx_0 > 0.0) {
      dynsim_3link_planar_B.m = static_cast<int32_T>
        (dynsim_3link_planar_B.a_idx_0);
      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        dynsim_3link_planar_B.a_idx_1 = 0.0;
        for (dynsim_3link_planar_B.b_k = 0; dynsim_3link_planar_B.b_k < 6;
             dynsim_3link_planar_B.b_k++) {
          dynsim_3link_planar_B.a_idx_1 += f->data[(dynsim_3link_planar_B.p - 1)
            * 6 + dynsim_3link_planar_B.b_k] * X->
            data[dynsim_3link_planar_B.inner].f1[6 * dynsim_3link_planar_B.b_k_f
            + dynsim_3link_planar_B.b_k];
        }

        dynsim_3link_planar_B.a0[dynsim_3link_planar_B.b_k_f] = f->data
          [(dynsim_3link_planar_B.m - 1) * 6 + dynsim_3link_planar_B.b_k_f] +
          dynsim_3link_planar_B.a_idx_1;
      }

      for (dynsim_3link_planar_B.b_k_f = 0; dynsim_3link_planar_B.b_k_f < 6;
           dynsim_3link_planar_B.b_k_f++) {
        f->data[dynsim_3link_planar_B.b_k_f + 6 * (dynsim_3link_planar_B.m - 1)]
          = dynsim_3link_planar_B.a0[dynsim_3link_planar_B.b_k_f];
      }
    }
  }

  dynsim_3link_pla_emxFree_char_T(&a);
  dynsim_3link_pla_emxFree_real_T(&taui);
  dynsim_3link_pla_emxFree_real_T(&S);
  dynsim_3link_pla_emxFree_real_T(&f);
  dynsim_3lin_emxFree_f_cell_wrap(&X);
}

static void matlabCodegenHandle_m_jroq24mq5(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_3link_T
  *pStruct)
{
  dynsim_3link_pla_emxFree_char_T(&pStruct->Type);
  dynsim_3link_pla_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matl_jroq24(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void dynsim_3link_planar_step(void)
{
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_dynsim_3link__T *L;
  emxArray_real_T_dynsim_3link__T *lambda;
  emxArray_real_T_dynsim_3link__T *H;
  emxArray_real_T_dynsim_3link__T *tmp;
  static const char_T b[9] = { 'a', 'r', 'm', '_', 'j', 'o', 'i', 'n', 't' };

  static const char_T c[13] = { 'f', 'o', 'r', 'e', 'a', 'r', 'm', '_', 'j', 'o',
    'i', 'n', 't' };

  static const char_T d[11] = { 'w', 'r', 'i', 's', 't', '_', 'j', 'o', 'i', 'n',
    't' };

  if (rtmIsMajorTimeStep(dynsim_3link_planar_M)) {
    /* set solver stop time */
    if (!(dynsim_3link_planar_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&dynsim_3link_planar_M->solverInfo,
                            ((dynsim_3link_planar_M->Timing.clockTickH0 + 1) *
        dynsim_3link_planar_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&dynsim_3link_planar_M->solverInfo,
                            ((dynsim_3link_planar_M->Timing.clockTick0 + 1) *
        dynsim_3link_planar_M->Timing.stepSize0 +
        dynsim_3link_planar_M->Timing.clockTickH0 *
        dynsim_3link_planar_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(dynsim_3link_planar_M)) {
    dynsim_3link_planar_M->Timing.t[0] = rtsiGetT
      (&dynsim_3link_planar_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(dynsim_3link_planar_M)) {
    /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
    /* MATLABSystem: '<S11>/SourceBlock' incorporates:
     *  Inport: '<S14>/In1'
     */
    dynsim_3link_pl_SystemCore_step(&dynsim_3link_planar_B.b_varargout_1,
      dynsim_3link_planar_B.b_varargout_2_Data,
      &dynsim_3link_planar_B.b_varargout_2_Data_SL_Info_Curr,
      &dynsim_3link_planar_B.b_varargout_2_Data_SL_Info_Rece,
      &dynsim_3link_planar_B.b_varargout_2_Layout_DataOffset,
      dynsim_3link_planar_B.b_varargout_2_Layout_Dim,
      &dynsim_3link_planar_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynsim_3link_planar_B.b_varargout_2_Layout_Dim_SL_I_l);

    /* Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S14>/Enable'
     */
    if (dynsim_3link_planar_B.b_varargout_1) {
      memcpy(&dynsim_3link_planar_B.In1.Data[0],
             &dynsim_3link_planar_B.b_varargout_2_Data[0], sizeof(real_T) << 7U);
      dynsim_3link_planar_B.In1.Data_SL_Info.CurrentLength =
        dynsim_3link_planar_B.b_varargout_2_Data_SL_Info_Curr;
      dynsim_3link_planar_B.In1.Data_SL_Info.ReceivedLength =
        dynsim_3link_planar_B.b_varargout_2_Data_SL_Info_Rece;
      dynsim_3link_planar_B.In1.Layout.DataOffset =
        dynsim_3link_planar_B.b_varargout_2_Layout_DataOffset;
      memcpy(&dynsim_3link_planar_B.In1.Layout.Dim[0],
             &dynsim_3link_planar_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_dynsim_3link_planar_std_msgs_MultiArrayDimension) << 4U);
      dynsim_3link_planar_B.In1.Layout.Dim_SL_Info.CurrentLength =
        dynsim_3link_planar_B.b_varargout_2_Layout_Dim_SL_Inf;
      dynsim_3link_planar_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        dynsim_3link_planar_B.b_varargout_2_Layout_Dim_SL_I_l;
    }

    /* End of MATLABSystem: '<S11>/SourceBlock' */
    /* End of Outputs for SubSystem: '<S11>/Enabled Subsystem' */
    /* End of Outputs for SubSystem: '<Root>/Subscribe' */
  }

  /* SignalConversion generated from: '<S1>/Position' incorporates:
   *  MATLABSystem: '<S13>/Get Parameter'
   *  MATLABSystem: '<S13>/Get Parameter1'
   *  MATLABSystem: '<S13>/Get Parameter4'
   */
  ParamGet_dynsim_3link_planar_112.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[0]);
  ParamGet_dynsim_3link_planar_113.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[1]);
  ParamGet_dynsim_3link_planar_125.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[2]);

  /* Integrator: '<S1>/Position' */
  if (dynsim_3link_planar_DW.Position_IWORK != 0) {
    dynsim_3link_planar_X.Position_CSTATE[0] =
      dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[0];
    dynsim_3link_planar_X.Position_CSTATE[1] =
      dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[1];
    dynsim_3link_planar_X.Position_CSTATE[2] =
      dynsim_3link_planar_B.TmpSignalConversionAtPositionIn[2];
  }

  /* SignalConversion generated from: '<S1>/Velocity' incorporates:
   *  MATLABSystem: '<S13>/Get Parameter2'
   *  MATLABSystem: '<S13>/Get Parameter3'
   *  MATLABSystem: '<S13>/Get Parameter5'
   */
  ParamGet_dynsim_3link_planar_117.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[0]);
  ParamGet_dynsim_3link_planar_118.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[1]);
  ParamGet_dynsim_3link_planar_126.get_parameter
    (&dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[2]);

  /* Integrator: '<S1>/Velocity' */
  if (dynsim_3link_planar_DW.Velocity_IWORK != 0) {
    dynsim_3link_planar_X.Velocity_CSTATE[0] =
      dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[0];
    dynsim_3link_planar_X.Velocity_CSTATE[1] =
      dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[1];
    dynsim_3link_planar_X.Velocity_CSTATE[2] =
      dynsim_3link_planar_B.TmpSignalConversionAtVelocityIn[2];
  }

  dynsim_3link_planar_B.Velocity[0] = dynsim_3link_planar_X.Velocity_CSTATE[0];
  dynsim_3link_planar_B.Velocity[1] = dynsim_3link_planar_X.Velocity_CSTATE[1];
  dynsim_3link_planar_B.Velocity[2] = dynsim_3link_planar_X.Velocity_CSTATE[2];

  /* End of Integrator: '<S1>/Velocity' */
  dynsim_3link_pla_emxInit_real_T(&L, 2);
  dynsim_3link_pla_emxInit_real_T(&lambda, 2);
  dynsim_3link_pla_emxInit_real_T(&tmp, 1);

  /* MATLABSystem: '<S12>/MATLAB System' incorporates:
   *  Constant: '<S1>/Constant'
   *  Integrator: '<S1>/Position'
   */
  obj = &dynsim_3link_planar_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynsim_3link_planar_DW.obj.TreeInternal,
    dynsim_3link_planar_X.Position_CSTATE, L, lambda);
  dynsim_3link_planar_B.vNum = obj->TreeInternal.VelocityNumber;
  dynsim_3link_planar_B.vNum_idx_0_tmp = static_cast<int32_T>
    (dynsim_3link_planar_B.vNum);
  dynsim_3link_planar_B.j_j = tmp->size[0];
  tmp->size[0] = dynsim_3link_planar_B.vNum_idx_0_tmp;
  dynsim_emxEnsureCapacity_real_T(tmp, dynsim_3link_planar_B.j_j);
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j <
       dynsim_3link_planar_B.vNum_idx_0_tmp; dynsim_3link_planar_B.j_j++) {
    tmp->data[dynsim_3link_planar_B.j_j] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal,
    dynsim_3link_planar_X.Position_CSTATE, dynsim_3link_planar_B.Velocity,
    dynsim_3link_planar_P.Constant_Value_f, dynsim_3link_planar_B.MATLABSystem);
  dynsim_3link_pla_emxFree_real_T(&tmp);

  /* MATLABSystem: '<S12>/MATLAB System' */
  dynsim_3link_planar_B.MATLABSystem[0] = dynsim_3link_planar_B.In1.Data[1] -
    dynsim_3link_planar_B.MATLABSystem[0];
  dynsim_3link_planar_B.MATLABSystem[1] = dynsim_3link_planar_B.In1.Data[2] -
    dynsim_3link_planar_B.MATLABSystem[1];
  dynsim_3link_planar_B.MATLABSystem[2] = dynsim_3link_planar_B.In1.Data[3] -
    dynsim_3link_planar_B.MATLABSystem[2];
  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynsim_3link_planar_B.iend = 0;
  } else {
    dynsim_3link_planar_B.u0 = L->size[0];
    dynsim_3link_planar_B.iend = L->size[1];
    if (dynsim_3link_planar_B.u0 > dynsim_3link_planar_B.iend) {
      dynsim_3link_planar_B.iend = dynsim_3link_planar_B.u0;
    }
  }

  dynsim_3link_pla_emxInit_real_T(&H, 2);

  /* MATLABSystem: '<S12>/MATLAB System' */
  dynsim_3link_planar_B.j_j = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynsim_emxEnsureCapacity_real_T(H, dynsim_3link_planar_B.j_j);
  dynsim_3link_planar_B.u0 = L->size[0] * L->size[1] - 1;
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j <=
       dynsim_3link_planar_B.u0; dynsim_3link_planar_B.j_j++) {
    H->data[dynsim_3link_planar_B.j_j] = L->data[dynsim_3link_planar_B.j_j];
  }

  dynsim_3link_planar_B.n = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynsim_3link_planar_B.iend)) + 1.0) / -1.0) - 1;
  for (dynsim_3link_planar_B.u0 = 0; dynsim_3link_planar_B.u0 <=
       dynsim_3link_planar_B.n; dynsim_3link_planar_B.u0++) {
    dynsim_3link_planar_B.j = static_cast<real_T>(dynsim_3link_planar_B.iend) +
      -static_cast<real_T>(dynsim_3link_planar_B.u0);
    dynsim_3link_planar_B.j_j = static_cast<int32_T>(dynsim_3link_planar_B.j);
    dynsim_3link_planar_B.MATLABSystem_tmp = dynsim_3link_planar_B.j_j - 1;
    H->data[(static_cast<int32_T>(dynsim_3link_planar_B.j) + H->size[0] * (
              static_cast<int32_T>(dynsim_3link_planar_B.j) - 1)) - 1] = sqrt
      (H->data[(dynsim_3link_planar_B.MATLABSystem_tmp * H->size[0] +
                dynsim_3link_planar_B.j_j) - 1]);
    dynsim_3link_planar_B.k = lambda->
      data[dynsim_3link_planar_B.MATLABSystem_tmp];
    while (dynsim_3link_planar_B.k > 0.0) {
      dynsim_3link_planar_B.i = static_cast<int32_T>(dynsim_3link_planar_B.k) -
        1;
      H->data[(static_cast<int32_T>(dynsim_3link_planar_B.j) + H->size[0] * (
                static_cast<int32_T>(dynsim_3link_planar_B.k) - 1)) - 1] =
        H->data[(dynsim_3link_planar_B.i * H->size[0] +
                 dynsim_3link_planar_B.j_j) - 1] / H->data[((static_cast<int32_T>
        (dynsim_3link_planar_B.j) - 1) * H->size[0] + static_cast<int32_T>
        (dynsim_3link_planar_B.j)) - 1];
      dynsim_3link_planar_B.k = lambda->data[dynsim_3link_planar_B.i];
    }

    dynsim_3link_planar_B.k = lambda->
      data[dynsim_3link_planar_B.MATLABSystem_tmp];
    while (dynsim_3link_planar_B.k > 0.0) {
      dynsim_3link_planar_B.j = dynsim_3link_planar_B.k;
      while (dynsim_3link_planar_B.j > 0.0) {
        dynsim_3link_planar_B.MATLABSystem_tmp = static_cast<int32_T>
          (dynsim_3link_planar_B.j) - 1;
        H->data[(static_cast<int32_T>(dynsim_3link_planar_B.k) + H->size[0] * (
                  static_cast<int32_T>(dynsim_3link_planar_B.j) - 1)) - 1] =
          H->data[(dynsim_3link_planar_B.MATLABSystem_tmp * H->size[0] +
                   static_cast<int32_T>(dynsim_3link_planar_B.k)) - 1] - H->
          data[((static_cast<int32_T>(dynsim_3link_planar_B.k) - 1) * H->size[0]
                + dynsim_3link_planar_B.j_j) - 1] * H->data
          [((static_cast<int32_T>(dynsim_3link_planar_B.j) - 1) * H->size[0] +
            dynsim_3link_planar_B.j_j) - 1];
        dynsim_3link_planar_B.j = lambda->
          data[dynsim_3link_planar_B.MATLABSystem_tmp];
      }

      dynsim_3link_planar_B.k = lambda->data[static_cast<int32_T>
        (dynsim_3link_planar_B.k) - 1];
    }
  }

  dynsim_3link_planar_B.j_j = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynsim_emxEnsureCapacity_real_T(L, dynsim_3link_planar_B.j_j);
  dynsim_3link_planar_B.u0 = H->size[0] * H->size[1] - 1;
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j <=
       dynsim_3link_planar_B.u0; dynsim_3link_planar_B.j_j++) {
    L->data[dynsim_3link_planar_B.j_j] = H->data[dynsim_3link_planar_B.j_j];
  }

  dynsim_3link_planar_B.n = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynsim_3link_planar_B.iend = 0;
    for (dynsim_3link_planar_B.j_j = 2; dynsim_3link_planar_B.j_j <=
         dynsim_3link_planar_B.n; dynsim_3link_planar_B.j_j++) {
      for (dynsim_3link_planar_B.u0 = 0; dynsim_3link_planar_B.u0 <=
           dynsim_3link_planar_B.iend; dynsim_3link_planar_B.u0++) {
        L->data[dynsim_3link_planar_B.u0 + L->size[0] *
          (dynsim_3link_planar_B.j_j - 1)] = 0.0;
      }

      if (dynsim_3link_planar_B.iend + 1 < H->size[0]) {
        dynsim_3link_planar_B.iend++;
      }
    }
  }

  dynsim_3link_pla_emxFree_real_T(&H);

  /* MATLABSystem: '<S12>/MATLAB System' */
  dynsim_3link_planar_B.n = static_cast<int32_T>(((-1.0 -
    dynsim_3link_planar_B.vNum) + 1.0) / -1.0) - 1;
  for (dynsim_3link_planar_B.u0 = 0; dynsim_3link_planar_B.u0 <=
       dynsim_3link_planar_B.n; dynsim_3link_planar_B.u0++) {
    dynsim_3link_planar_B.iend = static_cast<int32_T>(dynsim_3link_planar_B.vNum
      + -static_cast<real_T>(dynsim_3link_planar_B.u0));
    dynsim_3link_planar_B.j_j = dynsim_3link_planar_B.iend - 1;
    dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.j_j] /= L->data
      [(dynsim_3link_planar_B.j_j * L->size[0] + dynsim_3link_planar_B.iend) - 1];
    dynsim_3link_planar_B.j = lambda->data[dynsim_3link_planar_B.j_j];
    while (dynsim_3link_planar_B.j > 0.0) {
      dynsim_3link_planar_B.MATLABSystem_tmp = static_cast<int32_T>
        (dynsim_3link_planar_B.j) - 1;
      dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.MATLABSystem_tmp]
        -= L->data[(dynsim_3link_planar_B.MATLABSystem_tmp * L->size[0] +
                    dynsim_3link_planar_B.iend) - 1] *
        dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.j_j];
      dynsim_3link_planar_B.j = lambda->
        data[dynsim_3link_planar_B.MATLABSystem_tmp];
    }
  }

  dynsim_3link_planar_B.vNum_idx_0_tmp--;
  for (dynsim_3link_planar_B.u0 = 0; dynsim_3link_planar_B.u0 <=
       dynsim_3link_planar_B.vNum_idx_0_tmp; dynsim_3link_planar_B.u0++) {
    dynsim_3link_planar_B.j = lambda->data[dynsim_3link_planar_B.u0];
    while (dynsim_3link_planar_B.j > 0.0) {
      dynsim_3link_planar_B.j_j = static_cast<int32_T>(dynsim_3link_planar_B.j)
        - 1;
      dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.u0] -= L->
        data[dynsim_3link_planar_B.j_j * L->size[0] + dynsim_3link_planar_B.u0] *
        dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.j_j];
      dynsim_3link_planar_B.j = lambda->data[dynsim_3link_planar_B.j_j];
    }

    dynsim_3link_planar_B.MATLABSystem[dynsim_3link_planar_B.u0] /= L->data
      [L->size[0] * dynsim_3link_planar_B.u0 + dynsim_3link_planar_B.u0];
  }

  dynsim_3link_pla_emxFree_real_T(&lambda);
  dynsim_3link_pla_emxFree_real_T(&L);

  /* Clock: '<Root>/Clock1' incorporates:
   *  Clock: '<Root>/Clock'
   */
  dynsim_3link_planar_B.vNum = dynsim_3link_planar_M->Timing.t[0];

  /* MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
   *  Clock: '<Root>/Clock1'
   *  Constant: '<S5>/Constant'
   */
  dynsim_3link_planar_B.msg_g = dynsim_3link_planar_P.Constant_Value;
  if (dynsim_3link_planar_B.vNum < 0.0) {
    /* MATLAB Function: '<Root>/Assign to JointState msg' */
    dynsim_3link_planar_B.k = ceil(dynsim_3link_planar_B.vNum);
  } else {
    /* MATLAB Function: '<Root>/Assign to JointState msg' */
    dynsim_3link_planar_B.k = floor(dynsim_3link_planar_B.vNum);
  }

  dynsim_3link_planar_B.msg_g.Header.Stamp.Sec = dynsim_3link_planar_B.k;
  dynsim_3link_planar_B.j = (dynsim_3link_planar_B.vNum -
    dynsim_3link_planar_B.k) * 1.0E+9;
  if (dynsim_3link_planar_B.j < 0.0) {
    /* MATLAB Function: '<Root>/Assign to JointState msg' */
    dynsim_3link_planar_B.j = ceil(dynsim_3link_planar_B.j);
  } else {
    /* MATLAB Function: '<Root>/Assign to JointState msg' */
    dynsim_3link_planar_B.j = floor(dynsim_3link_planar_B.j);
  }

  dynsim_3link_planar_B.msg_g.Header.Stamp.Nsec = dynsim_3link_planar_B.j;
  dynsim_3link_planar_B.msg_g.Name_SL_Info.CurrentLength = 2U;
  dynsim_3link_planar_B.msg_g.Position_SL_Info.CurrentLength = 2U;
  dynsim_3link_planar_B.msg_g.Velocity_SL_Info.CurrentLength = 2U;
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j < 9;
       dynsim_3link_planar_B.j_j++) {
    dynsim_3link_planar_B.msg_g.Name[0].Data[dynsim_3link_planar_B.j_j] =
      static_cast<uint8_T>(b[dynsim_3link_planar_B.j_j]);
  }

  dynsim_3link_planar_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 9U;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Integrator: '<S1>/Position'
   */
  dynsim_3link_planar_B.d1 = cos(dynsim_3link_planar_X.Position_CSTATE[0]);
  dynsim_3link_planar_B.d4 = dynsim_3link_planar_X.Position_CSTATE[0] +
    dynsim_3link_planar_X.Position_CSTATE[1];
  dynsim_3link_planar_B.d2 = cos(dynsim_3link_planar_B.d4);
  dynsim_3link_planar_B.d5 = dynsim_3link_planar_B.d4 +
    dynsim_3link_planar_X.Position_CSTATE[2];
  dynsim_3link_planar_B.d3 = cos(dynsim_3link_planar_B.d5);

  /* MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
   *  Constant: '<Root>/Constant'
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  dynsim_3link_planar_B.msg_g.Position[0] =
    (dynsim_3link_planar_P.Constant_Value_oa[0] * dynsim_3link_planar_B.d1 +
     dynsim_3link_planar_P.Constant_Value_oa[1] * dynsim_3link_planar_B.d2) +
    dynsim_3link_planar_P.Constant_Value_oa[2] * dynsim_3link_planar_B.d3;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Constant: '<Root>/Constant'
   *  Integrator: '<S1>/Position'
   */
  dynsim_3link_planar_B.d = sin(dynsim_3link_planar_X.Position_CSTATE[0]);
  dynsim_3link_planar_B.d4 = sin(dynsim_3link_planar_B.d4);
  dynsim_3link_planar_B.d5 = sin(dynsim_3link_planar_B.d5);
  dynsim_3link_planar_B.d6 = dynsim_3link_planar_B.Velocity[0] +
    dynsim_3link_planar_B.Velocity[1];
  dynsim_3link_planar_B.d7 = dynsim_3link_planar_B.d6 *
    dynsim_3link_planar_P.Constant_Value_oa[1];
  dynsim_3link_planar_B.d6 = (dynsim_3link_planar_B.d6 +
    dynsim_3link_planar_B.Velocity[2]) *
    dynsim_3link_planar_P.Constant_Value_oa[2];

  /* MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
   *  Constant: '<Root>/Constant'
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  dynsim_3link_planar_B.msg_g.Velocity[0] =
    (-dynsim_3link_planar_P.Constant_Value_oa[0] *
     dynsim_3link_planar_B.Velocity[0] * dynsim_3link_planar_B.d -
     dynsim_3link_planar_B.d7 * dynsim_3link_planar_B.d4) -
    dynsim_3link_planar_B.d6 * dynsim_3link_planar_B.d5;
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j < 13;
       dynsim_3link_planar_B.j_j++) {
    dynsim_3link_planar_B.msg_g.Name[1].Data[dynsim_3link_planar_B.j_j] =
      static_cast<uint8_T>(c[dynsim_3link_planar_B.j_j]);
  }

  dynsim_3link_planar_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 13U;
  dynsim_3link_planar_B.msg_g.Position[1] =
    (dynsim_3link_planar_P.Constant_Value_oa[0] * dynsim_3link_planar_B.d +
     dynsim_3link_planar_P.Constant_Value_oa[1] * dynsim_3link_planar_B.d4) +
    dynsim_3link_planar_P.Constant_Value_oa[2] * dynsim_3link_planar_B.d5;
  dynsim_3link_planar_B.msg_g.Velocity[1] = (dynsim_3link_planar_B.d7 *
    dynsim_3link_planar_B.d2 + dynsim_3link_planar_P.Constant_Value_oa[0] *
    dynsim_3link_planar_B.Velocity[0] * dynsim_3link_planar_B.d1) +
    dynsim_3link_planar_B.d6 * dynsim_3link_planar_B.d3;

  /* Outputs for Atomic SubSystem: '<Root>/Publish2' */
  /* MATLABSystem: '<S10>/SinkBlock' */
  Pub_dynsim_3link_planar_124.publish(&dynsim_3link_planar_B.msg_g);

  /* End of Outputs for SubSystem: '<Root>/Publish2' */

  /* MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
   *  Constant: '<S5>/Constant'
   *  Integrator: '<S1>/Position'
   */
  dynsim_3link_planar_B.msg_g = dynsim_3link_planar_P.Constant_Value;
  dynsim_3link_planar_B.msg_g.Header.Stamp.Sec = dynsim_3link_planar_B.k;
  dynsim_3link_planar_B.msg_g.Header.Stamp.Nsec = dynsim_3link_planar_B.j;
  dynsim_3link_planar_B.msg_g.Name_SL_Info.CurrentLength = 3U;
  dynsim_3link_planar_B.msg_g.Position_SL_Info.CurrentLength = 3U;
  dynsim_3link_planar_B.msg_g.Velocity_SL_Info.CurrentLength = 3U;
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j < 9;
       dynsim_3link_planar_B.j_j++) {
    dynsim_3link_planar_B.msg_g.Name[0].Data[dynsim_3link_planar_B.j_j] =
      static_cast<uint8_T>(b[dynsim_3link_planar_B.j_j]);
  }

  dynsim_3link_planar_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 9U;
  dynsim_3link_planar_B.msg_g.Position[0] =
    dynsim_3link_planar_X.Position_CSTATE[0];
  dynsim_3link_planar_B.msg_g.Velocity[0] = dynsim_3link_planar_B.Velocity[0];
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j < 13;
       dynsim_3link_planar_B.j_j++) {
    dynsim_3link_planar_B.msg_g.Name[1].Data[dynsim_3link_planar_B.j_j] =
      static_cast<uint8_T>(c[dynsim_3link_planar_B.j_j]);
  }

  dynsim_3link_planar_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 13U;
  dynsim_3link_planar_B.msg_g.Position[1] =
    dynsim_3link_planar_X.Position_CSTATE[1];
  dynsim_3link_planar_B.msg_g.Velocity[1] = dynsim_3link_planar_B.Velocity[1];
  for (dynsim_3link_planar_B.j_j = 0; dynsim_3link_planar_B.j_j < 11;
       dynsim_3link_planar_B.j_j++) {
    dynsim_3link_planar_B.msg_g.Name[2].Data[dynsim_3link_planar_B.j_j] =
      static_cast<uint8_T>(d[dynsim_3link_planar_B.j_j]);
  }

  dynsim_3link_planar_B.msg_g.Name[2].Data_SL_Info.CurrentLength = 11U;
  dynsim_3link_planar_B.msg_g.Position[2] =
    dynsim_3link_planar_X.Position_CSTATE[2];
  dynsim_3link_planar_B.msg_g.Velocity[2] = dynsim_3link_planar_B.Velocity[2];

  /* Outputs for Atomic SubSystem: '<Root>/Publish' */
  /* MATLABSystem: '<S8>/SinkBlock' */
  Pub_dynsim_3link_planar_22.publish(&dynsim_3link_planar_B.msg_g);

  /* End of Outputs for SubSystem: '<Root>/Publish' */

  /* MATLAB Function: '<Root>/Assign to Time msg' */
  if (dynsim_3link_planar_B.vNum < 0.0) {
    dynsim_3link_planar_B.k = ceil(dynsim_3link_planar_B.vNum);
  } else {
    dynsim_3link_planar_B.k = floor(dynsim_3link_planar_B.vNum);
  }

  dynsim_3link_planar_B.msg_l.Clock_.Sec = dynsim_3link_planar_B.k;
  dynsim_3link_planar_B.j = (dynsim_3link_planar_B.vNum -
    dynsim_3link_planar_B.k) * 1.0E+9;
  if (dynsim_3link_planar_B.j < 0.0) {
    dynsim_3link_planar_B.msg_l.Clock_.Nsec = ceil(dynsim_3link_planar_B.j);
  } else {
    dynsim_3link_planar_B.msg_l.Clock_.Nsec = floor(dynsim_3link_planar_B.j);
  }

  /* End of MATLAB Function: '<Root>/Assign to Time msg' */

  /* Outputs for Atomic SubSystem: '<Root>/Publish1' */
  /* MATLABSystem: '<S9>/SinkBlock' */
  Pub_dynsim_3link_planar_50.publish(&dynsim_3link_planar_B.msg_l);

  /* End of Outputs for SubSystem: '<Root>/Publish1' */
  if (rtmIsMajorTimeStep(dynsim_3link_planar_M)) {
    /* Update for Integrator: '<S1>/Position' */
    dynsim_3link_planar_DW.Position_IWORK = 0;

    /* Update for Integrator: '<S1>/Velocity' */
    dynsim_3link_planar_DW.Velocity_IWORK = 0;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(dynsim_3link_planar_M)) {
    rt_ertODEUpdateContinuousStates(&dynsim_3link_planar_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++dynsim_3link_planar_M->Timing.clockTick0)) {
      ++dynsim_3link_planar_M->Timing.clockTickH0;
    }

    dynsim_3link_planar_M->Timing.t[0] = rtsiGetSolverStopTime
      (&dynsim_3link_planar_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      dynsim_3link_planar_M->Timing.clockTick1++;
      if (!dynsim_3link_planar_M->Timing.clockTick1) {
        dynsim_3link_planar_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void dynsim_3link_planar_derivatives(void)
{
  XDot_dynsim_3link_planar_T *_rtXdot;
  _rtXdot = ((XDot_dynsim_3link_planar_T *) dynsim_3link_planar_M->derivs);

  /* Derivatives for Integrator: '<S1>/Position' */
  _rtXdot->Position_CSTATE[0] = dynsim_3link_planar_B.Velocity[0];

  /* Derivatives for Integrator: '<S1>/Velocity' */
  _rtXdot->Velocity_CSTATE[0] = dynsim_3link_planar_B.MATLABSystem[0];

  /* Derivatives for Integrator: '<S1>/Position' */
  _rtXdot->Position_CSTATE[1] = dynsim_3link_planar_B.Velocity[1];

  /* Derivatives for Integrator: '<S1>/Velocity' */
  _rtXdot->Velocity_CSTATE[1] = dynsim_3link_planar_B.MATLABSystem[1];

  /* Derivatives for Integrator: '<S1>/Position' */
  _rtXdot->Position_CSTATE[2] = dynsim_3link_planar_B.Velocity[2];

  /* Derivatives for Integrator: '<S1>/Velocity' */
  _rtXdot->Velocity_CSTATE[2] = dynsim_3link_planar_B.MATLABSystem[2];
}

/* Model initialize function */
void dynsim_3link_planar_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&dynsim_3link_planar_M->solverInfo,
                          &dynsim_3link_planar_M->Timing.simTimeStep);
    rtsiSetTPtr(&dynsim_3link_planar_M->solverInfo, &rtmGetTPtr
                (dynsim_3link_planar_M));
    rtsiSetStepSizePtr(&dynsim_3link_planar_M->solverInfo,
                       &dynsim_3link_planar_M->Timing.stepSize0);
    rtsiSetdXPtr(&dynsim_3link_planar_M->solverInfo,
                 &dynsim_3link_planar_M->derivs);
    rtsiSetContStatesPtr(&dynsim_3link_planar_M->solverInfo, (real_T **)
                         &dynsim_3link_planar_M->contStates);
    rtsiSetNumContStatesPtr(&dynsim_3link_planar_M->solverInfo,
      &dynsim_3link_planar_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dynsim_3link_planar_M->solverInfo,
      &dynsim_3link_planar_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dynsim_3link_planar_M->solverInfo,
      &dynsim_3link_planar_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dynsim_3link_planar_M->solverInfo,
      &dynsim_3link_planar_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dynsim_3link_planar_M->solverInfo,
                          (&rtmGetErrorStatus(dynsim_3link_planar_M)));
    rtsiSetRTModelPtr(&dynsim_3link_planar_M->solverInfo, dynsim_3link_planar_M);
  }

  rtsiSetSimTimeStep(&dynsim_3link_planar_M->solverInfo, MAJOR_TIME_STEP);
  dynsim_3link_planar_M->intgData.y = dynsim_3link_planar_M->odeY;
  dynsim_3link_planar_M->intgData.f[0] = dynsim_3link_planar_M->odeF[0];
  dynsim_3link_planar_M->intgData.f[1] = dynsim_3link_planar_M->odeF[1];
  dynsim_3link_planar_M->intgData.f[2] = dynsim_3link_planar_M->odeF[2];
  dynsim_3link_planar_M->contStates = ((X_dynsim_3link_planar_T *)
    &dynsim_3link_planar_X);
  rtsiSetSolverData(&dynsim_3link_planar_M->solverInfo, static_cast<void *>
                    (&dynsim_3link_planar_M->intgData));
  rtsiSetSolverName(&dynsim_3link_planar_M->solverInfo,"ode3");
  rtmSetTPtr(dynsim_3link_planar_M, &dynsim_3link_planar_M->Timing.tArray[0]);
  dynsim_3link_planar_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(dynsim_3link_planar_M, 1);

  /* block I/O */
  (void) memset((static_cast<void *>(&dynsim_3link_planar_B)), 0,
                sizeof(B_dynsim_3link_planar_T));

  /* states (continuous) */
  {
    (void) memset(static_cast<void *>(&dynsim_3link_planar_X), 0,
                  sizeof(X_dynsim_3link_planar_T));
  }

  /* states (dwork) */
  (void) memset(static_cast<void *>(&dynsim_3link_planar_DW), 0,
                sizeof(DW_dynsim_3link_planar_T));

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '1',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_2[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '2',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_3[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '3',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_4[32] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', 'v',
      '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_5[33] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q',
      'v', '2', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_6[33] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', '3', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q',
      'v', '3', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_7[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_8[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_9[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S11>/SourceBlock' */
    dynsim_3link_planar_DW.obj_pp.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty = true;
    dynsim_3link_planar_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynsim_3link_planar_B.cv4[i] = tmp_0[i];
    }

    dynsim_3link_planar_B.cv4[13] = '\x00';
    Sub_dynsim_3link_planar_16.createSubscriber(dynsim_3link_planar_B.cv4, 1);
    dynsim_3link_planar_DW.obj_pp.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S11>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */

    /* Start for MATLABSystem: '<S13>/Get Parameter' */
    dynsim_3link_planar_DW.obj_p.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_ev = true;
    dynsim_3link_planar_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_3link_planar_B.cv2[i] = tmp_1[i];
    }

    dynsim_3link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_3link_planar_112.initialize(dynsim_3link_planar_B.cv2);
    ParamGet_dynsim_3link_planar_112.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_112.set_initial_value(0.78539816339744828);
    dynsim_3link_planar_DW.obj_p.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter' */

    /* Start for MATLABSystem: '<S13>/Get Parameter1' */
    dynsim_3link_planar_DW.obj_n.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_k = true;
    dynsim_3link_planar_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_3link_planar_B.cv2[i] = tmp_2[i];
    }

    dynsim_3link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_3link_planar_113.initialize(dynsim_3link_planar_B.cv2);
    ParamGet_dynsim_3link_planar_113.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_113.set_initial_value(-1.5707963267948966);
    dynsim_3link_planar_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter1' */

    /* Start for MATLABSystem: '<S13>/Get Parameter4' */
    dynsim_3link_planar_DW.obj_nf.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_iu = true;
    dynsim_3link_planar_DW.obj_nf.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_3link_planar_B.cv2[i] = tmp_3[i];
    }

    dynsim_3link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_3link_planar_125.initialize(dynsim_3link_planar_B.cv2);
    ParamGet_dynsim_3link_planar_125.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_125.set_initial_value(0.0);
    dynsim_3link_planar_DW.obj_nf.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter4' */

    /* Start for MATLABSystem: '<S13>/Get Parameter2' */
    dynsim_3link_planar_DW.obj_h.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_kw = true;
    dynsim_3link_planar_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      dynsim_3link_planar_B.cv1[i] = tmp_4[i];
    }

    dynsim_3link_planar_B.cv1[32] = '\x00';
    ParamGet_dynsim_3link_planar_117.initialize(dynsim_3link_planar_B.cv1);
    ParamGet_dynsim_3link_planar_117.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_117.set_initial_value(0.0);
    dynsim_3link_planar_DW.obj_h.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter2' */

    /* Start for MATLABSystem: '<S13>/Get Parameter3' */
    dynsim_3link_planar_DW.obj_b.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_b = true;
    dynsim_3link_planar_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      dynsim_3link_planar_B.cv[i] = tmp_5[i];
    }

    dynsim_3link_planar_B.cv[33] = '\x00';
    ParamGet_dynsim_3link_planar_118.initialize(dynsim_3link_planar_B.cv);
    ParamGet_dynsim_3link_planar_118.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_118.set_initial_value(0.0);
    dynsim_3link_planar_DW.obj_b.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter3' */

    /* Start for MATLABSystem: '<S13>/Get Parameter5' */
    dynsim_3link_planar_DW.obj_e.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_l = true;
    dynsim_3link_planar_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      dynsim_3link_planar_B.cv[i] = tmp_6[i];
    }

    dynsim_3link_planar_B.cv[33] = '\x00';
    ParamGet_dynsim_3link_planar_126.initialize(dynsim_3link_planar_B.cv);
    ParamGet_dynsim_3link_planar_126.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_3link_planar_126.set_initial_value(0.0);
    dynsim_3link_planar_DW.obj_e.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/Get Parameter5' */
    emxInitStruct_robotics_slmanip_(&dynsim_3link_planar_DW.obj);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_1);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_6);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_5);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_4);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_3);
    emxInitStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_2);

    /* Start for MATLABSystem: '<S12>/MATLAB System' */
    dynsim_3link_planar_DW.obj.isInitialized = 0;
    dynsim_3link_planar_DW.objisempty_j = true;
    dynsim_3link_planar_DW.obj.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynsim_3link_planar_DW.obj.TreeInternal,
      &dynsim_3link_planar_DW.gobj_2, &dynsim_3link_planar_DW.gobj_4,
      &dynsim_3link_planar_DW.gobj_3);

    /* Start for Atomic SubSystem: '<Root>/Publish2' */
    /* Start for MATLABSystem: '<S10>/SinkBlock' */
    dynsim_3link_planar_DW.obj_a.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_i = true;
    dynsim_3link_planar_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      dynsim_3link_planar_B.cv3[i] = tmp_7[i];
    }

    dynsim_3link_planar_B.cv3[17] = '\x00';
    Pub_dynsim_3link_planar_124.createPublisher(dynsim_3link_planar_B.cv3, 1);
    dynsim_3link_planar_DW.obj_a.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S10>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish2' */

    /* Start for Atomic SubSystem: '<Root>/Publish' */
    /* Start for MATLABSystem: '<S8>/SinkBlock' */
    dynsim_3link_planar_DW.obj_nr.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_e = true;
    dynsim_3link_planar_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynsim_3link_planar_B.cv4[i] = tmp_8[i];
    }

    dynsim_3link_planar_B.cv4[13] = '\x00';
    Pub_dynsim_3link_planar_22.createPublisher(dynsim_3link_planar_B.cv4, 1);
    dynsim_3link_planar_DW.obj_nr.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S8>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish' */

    /* Start for Atomic SubSystem: '<Root>/Publish1' */
    /* Start for MATLABSystem: '<S9>/SinkBlock' */
    dynsim_3link_planar_DW.obj_f.matlabCodegenIsDeleted = false;
    dynsim_3link_planar_DW.objisempty_m = true;
    dynsim_3link_planar_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_9[i];
    }

    tmp[6] = '\x00';
    Pub_dynsim_3link_planar_50.createPublisher(tmp, 1);
    dynsim_3link_planar_DW.obj_f.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S9>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish1' */
  }

  /* InitializeConditions for Integrator: '<S1>/Position' incorporates:
   *  Integrator: '<S1>/Velocity'
   */
  if (rtmIsFirstInitCond(dynsim_3link_planar_M)) {
    dynsim_3link_planar_X.Position_CSTATE[0] = 0.0;
    dynsim_3link_planar_X.Position_CSTATE[1] = 0.0;
    dynsim_3link_planar_X.Position_CSTATE[2] = 0.0;
    dynsim_3link_planar_X.Velocity_CSTATE[0] = 0.0;
    dynsim_3link_planar_X.Velocity_CSTATE[1] = 0.0;
    dynsim_3link_planar_X.Velocity_CSTATE[2] = 0.0;
  }

  dynsim_3link_planar_DW.Position_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S1>/Position' */

  /* InitializeConditions for Integrator: '<S1>/Velocity' */
  dynsim_3link_planar_DW.Velocity_IWORK = 1;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S14>/Out1' */
  dynsim_3link_planar_B.In1 = dynsim_3link_planar_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<Root>/Subscribe' */

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(dynsim_3link_planar_M)) {
    rtmSetFirstInitCond(dynsim_3link_planar_M, 0);
  }
}

/* Model terminate function */
void dynsim_3link_planar_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Terminate for MATLABSystem: '<S11>/SourceBlock' */
  matlabCodegenHandle_m_jroq24mq5(&dynsim_3link_planar_DW.obj_pp);

  /* End of Terminate for SubSystem: '<Root>/Subscribe' */

  /* Terminate for MATLABSystem: '<S13>/Get Parameter' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_p);

  /* Terminate for MATLABSystem: '<S13>/Get Parameter1' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_n);

  /* Terminate for MATLABSystem: '<S13>/Get Parameter4' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_nf);

  /* Terminate for MATLABSystem: '<S13>/Get Parameter2' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_h);

  /* Terminate for MATLABSystem: '<S13>/Get Parameter3' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_b);

  /* Terminate for MATLABSystem: '<S13>/Get Parameter5' */
  matlabCodegenHandle_matlabCodeg(&dynsim_3link_planar_DW.obj_e);
  emxFreeStruct_robotics_slmanip_(&dynsim_3link_planar_DW.obj);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_1);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_6);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_5);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_4);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_3);
  emxFreeStruct_n_robotics_manip_(&dynsim_3link_planar_DW.gobj_2);

  /* Terminate for Atomic SubSystem: '<Root>/Publish2' */
  /* Terminate for MATLABSystem: '<S10>/SinkBlock' */
  matlabCodegenHandle_matl_jroq24(&dynsim_3link_planar_DW.obj_a);

  /* End of Terminate for SubSystem: '<Root>/Publish2' */

  /* Terminate for Atomic SubSystem: '<Root>/Publish' */
  /* Terminate for MATLABSystem: '<S8>/SinkBlock' */
  matlabCodegenHandle_matl_jroq24(&dynsim_3link_planar_DW.obj_nr);

  /* End of Terminate for SubSystem: '<Root>/Publish' */

  /* Terminate for Atomic SubSystem: '<Root>/Publish1' */
  /* Terminate for MATLABSystem: '<S9>/SinkBlock' */
  matlabCodegenHandle_matl_jroq24(&dynsim_3link_planar_DW.obj_f);

  /* End of Terminate for SubSystem: '<Root>/Publish1' */
}
