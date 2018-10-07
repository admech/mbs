/* Initialization */
#include "Whole_model.h"
#include "Whole_11mix.h"
#include "Whole_12jac.h"
#if defined(__cplusplus)
extern "C" {
#endif

void Whole_functionInitialEquations_0(DATA *data, threadData_t *threadData);


/*
 equation index: 1
 type: SIMPLE_ASSIGN
 part._subPart._state._y = $START.part.subPart.state.y
 */
void Whole_eqFunction_1(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,1};
  data->localData[0]->realVars[1] /* part._subPart._state._y STATE(1) */ = data->modelData->realVarsData[1].attribute /* part._subPart._state._y STATE(1) */.start;
  TRACE_POP
}

/*
 equation index: 2
 type: SIMPLE_ASSIGN
 part._state._x = $START.part.state.x
 */
void Whole_eqFunction_2(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,2};
  data->localData[0]->realVars[0] /* part._state._x STATE(1) */ = data->modelData->realVarsData[0].attribute /* part._state._x STATE(1) */.start;
  TRACE_POP
}

/*
 equation index: 3
 type: SIMPLE_ASSIGN
 partState._y = 1.0 - part.state.x
 */
void Whole_eqFunction_3(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,3};
  data->localData[0]->realVars[4] /* partState._y variable */ = 1.0 - data->localData[0]->realVars[0] /* part._state._x STATE(1) */;
  TRACE_POP
}

/*
 equation index: 4
 type: SIMPLE_ASSIGN
 der(part._state._x) = part.state.a * part.state.x
 */
void Whole_eqFunction_4(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,4};
  data->localData[0]->realVars[2] /* der(part._state._x) STATE_DER */ = (data->simulationInfo->realParameter[2]) * (data->localData[0]->realVars[0] /* part._state._x STATE(1) */);
  TRACE_POP
}

/*
 equation index: 5
 type: SIMPLE_ASSIGN
 der(part._subPart._state._y) = part.subPart.state.b
 */
void Whole_eqFunction_5(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,5};
  data->localData[0]->realVars[3] /* der(part._subPart._state._y) STATE_DER */ = data->simulationInfo->realParameter[4];
  TRACE_POP
}
void Whole_functionInitialEquations_0(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  Whole_eqFunction_1(data, threadData);
  Whole_eqFunction_2(data, threadData);
  Whole_eqFunction_3(data, threadData);
  Whole_eqFunction_4(data, threadData);
  Whole_eqFunction_5(data, threadData);
  TRACE_POP
}


int Whole_functionInitialEquations(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  data->simulationInfo->discreteCall = 1;
  Whole_functionInitialEquations_0(data, threadData);
  data->simulationInfo->discreteCall = 0;
  
  TRACE_POP
  return 0;
}


int Whole_functionInitialEquations_lambda0(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  data->simulationInfo->discreteCall = 1;
  data->simulationInfo->discreteCall = 0;
  
  TRACE_POP
  return 0;
}
int Whole_functionRemovedInitialEquations(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int *equationIndexes = NULL;
  double res = 0.0;

  
  TRACE_POP
  return 0;
}


#if defined(__cplusplus)
}
#endif

