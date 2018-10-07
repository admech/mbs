/* Algebraic */
#include "Whole_model.h"

#ifdef __cplusplus
extern "C" {
#endif


/* forwarded equations */
extern void Whole_eqFunction_7(DATA* data, threadData_t *threadData);

static void functionAlg_system0(DATA *data, threadData_t *threadData)
{
  Whole_eqFunction_7(data, threadData);
}
/* for continuous time variables */
int Whole_functionAlgebraics(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  
  data->simulationInfo->callStatistics.functionAlgebraics++;
  
  functionAlg_system0(data, threadData);

  Whole_function_savePreSynchronous(data, threadData);
  
  TRACE_POP
  return 0;
}

#ifdef __cplusplus
}
#endif
