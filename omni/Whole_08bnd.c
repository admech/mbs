/* update bound parameters and variable attributes (start, nominal, min, max) */
#include "Whole_model.h"
#if defined(__cplusplus)
extern "C" {
#endif

int Whole_updateBoundVariableAttributes(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  /* min ******************************************************** */
  
  infoStreamPrint(LOG_INIT, 1, "updating min-values");
  if (ACTIVE_STREAM(LOG_INIT)) messageClose(LOG_INIT);
  
  /* max ******************************************************** */
  
  infoStreamPrint(LOG_INIT, 1, "updating max-values");
  if (ACTIVE_STREAM(LOG_INIT)) messageClose(LOG_INIT);
  
  /* nominal **************************************************** */
  
  infoStreamPrint(LOG_INIT, 1, "updating nominal-values");
  if (ACTIVE_STREAM(LOG_INIT)) messageClose(LOG_INIT);
  
  /* start ****************************************************** */
  infoStreamPrint(LOG_INIT, 1, "updating primary start-values");
  if (ACTIVE_STREAM(LOG_INIT)) messageClose(LOG_INIT);
  
  TRACE_POP
  return 0;
}


/*
 equation index: 9
 type: SIMPLE_ASSIGN
 constraint._part._subPart._b = partState.subPart.b
 */
void Whole_eqFunction_9(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,9};
  data->simulationInfo->realParameter[1] = data->simulationInfo->realParameter[6];
  TRACE_POP
}

/*
 equation index: 10
 type: SIMPLE_ASSIGN
 constraint._part._a = partState.a
 */
void Whole_eqFunction_10(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,10};
  data->simulationInfo->realParameter[0] = data->simulationInfo->realParameter[5];
  TRACE_POP
}

/*
 equation index: 11
 type: SIMPLE_ASSIGN
 part._state._subPart._b = partState.subPart.b
 */
void Whole_eqFunction_11(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,11};
  data->simulationInfo->realParameter[3] = data->simulationInfo->realParameter[6];
  TRACE_POP
}

/*
 equation index: 12
 type: SIMPLE_ASSIGN
 part._subPart._state._b = part.state.subPart.b
 */
void Whole_eqFunction_12(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,12};
  data->simulationInfo->realParameter[4] = data->simulationInfo->realParameter[3];
  TRACE_POP
}

/*
 equation index: 13
 type: SIMPLE_ASSIGN
 part._state._a = partState.a
 */
void Whole_eqFunction_13(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,13};
  data->simulationInfo->realParameter[2] = data->simulationInfo->realParameter[5];
  TRACE_POP
}
int Whole_updateBoundParameters(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  Whole_eqFunction_9(data, threadData);

  Whole_eqFunction_10(data, threadData);

  Whole_eqFunction_11(data, threadData);

  Whole_eqFunction_12(data, threadData);

  Whole_eqFunction_13(data, threadData);
  
  TRACE_POP
  return 0;
}

#if defined(__cplusplus)
}
#endif

