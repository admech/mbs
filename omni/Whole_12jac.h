/* Jacobians */
static const REAL_ATTRIBUTE dummyREAL_ATTRIBUTE = omc_dummyRealAttribute;
/* Jacobian Variables */
#if defined(__cplusplus)
extern "C" {
#endif
  #define Whole_INDEX_JAC_A 3
  int Whole_functionJacA_column(void* data, threadData_t *threadData);
  int Whole_initialAnalyticJacobianA(void* data, threadData_t *threadData);
#if defined(__cplusplus)
}
#endif
/* A */
#define $Ppart_state_xSeedA data->simulationInfo->analyticJacobians[3].seedVars[0]
#define $Ppart_subPart_state_ySeedA data->simulationInfo->analyticJacobians[3].seedVars[1]

#if defined(__cplusplus)
extern "C" {
#endif
  #define Whole_INDEX_JAC_B 2
  int Whole_functionJacB_column(void* data, threadData_t *threadData);
  int Whole_initialAnalyticJacobianB(void* data, threadData_t *threadData);
#if defined(__cplusplus)
}
#endif
/* B */

#if defined(__cplusplus)
extern "C" {
#endif
  #define Whole_INDEX_JAC_C 1
  int Whole_functionJacC_column(void* data, threadData_t *threadData);
  int Whole_initialAnalyticJacobianC(void* data, threadData_t *threadData);
#if defined(__cplusplus)
}
#endif
/* C */

#if defined(__cplusplus)
extern "C" {
#endif
  #define Whole_INDEX_JAC_D 0
  int Whole_functionJacD_column(void* data, threadData_t *threadData);
  int Whole_initialAnalyticJacobianD(void* data, threadData_t *threadData);
#if defined(__cplusplus)
}
#endif
/* D */


