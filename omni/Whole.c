/* Main Simulation File */
#include "Whole_model.h"

#define prefixedName_performSimulation Whole_performSimulation
#define prefixedName_updateContinuousSystem Whole_updateContinuousSystem
#include <simulation/solver/perform_simulation.c>

#define prefixedName_performQSSSimulation Whole_performQSSSimulation
#include <simulation/solver/perform_qss_simulation.c>

/* dummy VARINFO and FILEINFO */
const FILE_INFO dummyFILE_INFO = omc_dummyFileInfo;
const VAR_INFO dummyVAR_INFO = omc_dummyVarInfo;
#if defined(__cplusplus)
extern "C" {
#endif

int Whole_input_function(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}

int Whole_input_function_init(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}

int Whole_input_function_updateStartValues(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}

int Whole_inputNames(DATA *data, char ** names){
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}

int Whole_output_function(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}


/*
 equation index: 6
 type: SIMPLE_ASSIGN
 der(part._state._x) = part.state.a * part.state.x
 */
void Whole_eqFunction_6(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,6};
  data->localData[0]->realVars[2] /* der(part._state._x) STATE_DER */ = (data->simulationInfo->realParameter[2]) * (data->localData[0]->realVars[0] /* part._state._x STATE(1) */);
  TRACE_POP
}
/*
 equation index: 7
 type: SIMPLE_ASSIGN
 partState._y = 1.0 - part.state.x
 */
void Whole_eqFunction_7(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,7};
  data->localData[0]->realVars[4] /* partState._y variable */ = 1.0 - data->localData[0]->realVars[0] /* part._state._x STATE(1) */;
  TRACE_POP
}
/*
 equation index: 8
 type: SIMPLE_ASSIGN
 der(part._subPart._state._y) = part.subPart.state.b
 */
void Whole_eqFunction_8(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  const int equationIndexes[2] = {1,8};
  data->localData[0]->realVars[3] /* der(part._subPart._state._y) STATE_DER */ = data->simulationInfo->realParameter[4];
  TRACE_POP
}


int Whole_functionDAE(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH
  int equationIndexes[1] = {0};
  
  data->simulationInfo->needToIterate = 0;
  data->simulationInfo->discreteCall = 1;
  Whole_functionLocalKnownVars(data, threadData);
  Whole_eqFunction_6(data, threadData);

  Whole_eqFunction_7(data, threadData);

  Whole_eqFunction_8(data, threadData);
  data->simulationInfo->discreteCall = 0;
  
  TRACE_POP
  return 0;
}


int Whole_functionLocalKnownVars(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  TRACE_POP
  return 0;
}


/* forwarded equations */
extern void Whole_eqFunction_8(DATA* data, threadData_t *threadData);
extern void Whole_eqFunction_6(DATA* data, threadData_t *threadData);

static void functionODE_system0(DATA *data, threadData_t *threadData)
{
  Whole_eqFunction_8(data, threadData);

  Whole_eqFunction_6(data, threadData);
}

int Whole_functionODE(DATA *data, threadData_t *threadData)
{
  TRACE_PUSH

  
  data->simulationInfo->callStatistics.functionODE++;
  
  Whole_functionLocalKnownVars(data, threadData);
  functionODE_system0(data, threadData);

  
  TRACE_POP
  return 0;
}

#ifdef FMU_EXPERIMENTAL
#endif
/* forward the main in the simulation runtime */
extern int _main_SimulationRuntime(int argc, char**argv, DATA *data, threadData_t *threadData);

#include "Whole_12jac.h"
#include "Whole_13opt.h"

struct OpenModelicaGeneratedFunctionCallbacks Whole_callback = {
   (int (*)(DATA *, threadData_t *, void *)) Whole_performSimulation,
   (int (*)(DATA *, threadData_t *, void *)) Whole_performQSSSimulation,
   Whole_updateContinuousSystem,
   Whole_callExternalObjectDestructors,
   NULL,
   NULL,
   NULL,
   #if !defined(OMC_NO_STATESELECTION)
   Whole_initializeStateSets,
   #else
   NULL,
   #endif
   Whole_initializeDAEmodeData,
   Whole_functionODE,
   Whole_functionAlgebraics,
   Whole_functionDAE,
   Whole_functionLocalKnownVars,
   Whole_input_function,
   Whole_input_function_init,
   Whole_input_function_updateStartValues,
   Whole_output_function,
   Whole_function_storeDelayed,
   Whole_updateBoundVariableAttributes,
   Whole_functionInitialEquations,
   0, /* useHomotopy - 0: no homotopy or local homotopy, 1: global homotopy, 2: new global homotopy approach */
   Whole_functionInitialEquations_lambda0,
   Whole_functionRemovedInitialEquations,
   Whole_updateBoundParameters,
   Whole_checkForAsserts,
   Whole_function_ZeroCrossingsEquations,
   Whole_function_ZeroCrossings,
   Whole_function_updateRelations,
   Whole_checkForDiscreteChanges,
   Whole_zeroCrossingDescription,
   Whole_relationDescription,
   Whole_function_initSample,
   Whole_INDEX_JAC_A,
   Whole_INDEX_JAC_B,
   Whole_INDEX_JAC_C,
   Whole_INDEX_JAC_D,
   Whole_initialAnalyticJacobianA,
   Whole_initialAnalyticJacobianB,
   Whole_initialAnalyticJacobianC,
   Whole_initialAnalyticJacobianD,
   Whole_functionJacA_column,
   Whole_functionJacB_column,
   Whole_functionJacC_column,
   Whole_functionJacD_column,
   Whole_linear_model_frame,
   Whole_linear_model_datarecovery_frame,
   Whole_mayer,
   Whole_lagrange,
   Whole_pickUpBoundsForInputsInOptimization,
   Whole_setInputData,
   Whole_getTimeGrid,
   Whole_symbolicInlineSystem,
   Whole_function_initSynchronous,
   Whole_function_updateSynchronous,
   Whole_function_equationsSynchronous,
   NULL,
   #ifdef FMU_EXPERIMENTAL
   Whole_functionODE_Partial,
   Whole_functionFMIJacobian,
   #endif
   Whole_inputNames
};

void Whole_setupDataStruc(DATA *data, threadData_t *threadData)
{
  assertStreamPrint(threadData,0!=data, "Error while initialize Data");
  data->callback = &Whole_callback;
  data->modelData->modelName = "Whole";
  data->modelData->modelFilePrefix = "Whole";
  data->modelData->resultFileName = NULL;
  data->modelData->modelDir = "";
  data->modelData->modelGUID = "{107cb06b-9cdc-453c-9571-7eb03873028f}";
  #if defined(OPENMODELICA_XML_FROM_FILE_AT_RUNTIME)
  data->modelData->initXMLData = NULL;
  data->modelData->modelDataXml.infoXMLData = NULL;
  #else
  #if defined(_MSC_VER) /* handle joke compilers */
  {
  /* for MSVC we encode a string like char x[] = {'a', 'b', 'c', '\0'} */
  /* because the string constant limit is 65535 bytes */
  static const char contents_init[] =
    #include "Whole_init.c"
    ;
  static const char contents_info[] =
    #include "Whole_info.c"
    ;
    data->modelData->initXMLData = contents_init;
    data->modelData->modelDataXml.infoXMLData = contents_info;
  }
  #else /* handle real compilers */
  data->modelData->initXMLData =
  #include "Whole_init.c"
    ;
  data->modelData->modelDataXml.infoXMLData =
  #include "Whole_info.c"
    ;
  #endif /* defined(_MSC_VER) */
  #endif /* defined(OPENMODELICA_XML_FROM_FILE_AT_RUNTIME) */
  
  data->modelData->nStates = 2;
  data->modelData->nVariablesReal = 5;
  data->modelData->nDiscreteReal = 0;
  data->modelData->nVariablesInteger = 0;
  data->modelData->nVariablesBoolean = 0;
  data->modelData->nVariablesString = 0;
  data->modelData->nParametersReal = 7;
  data->modelData->nParametersInteger = 0;
  data->modelData->nParametersBoolean = 0;
  data->modelData->nParametersString = 0;
  data->modelData->nInputVars = 0;
  data->modelData->nOutputVars = 0;
  
  data->modelData->nAliasReal = 7;
  data->modelData->nAliasInteger = 0;
  data->modelData->nAliasBoolean = 0;
  data->modelData->nAliasString = 0;
  
  data->modelData->nZeroCrossings = 0;
  data->modelData->nSamples = 0;
  data->modelData->nRelations = 0;
  data->modelData->nMathEvents = 0;
  data->modelData->nExtObjs = 0;
  data->modelData->modelDataXml.fileName = "Whole_info.json";
  data->modelData->modelDataXml.modelInfoXmlLength = 0;
  data->modelData->modelDataXml.nFunctions = 4;
  data->modelData->modelDataXml.nProfileBlocks = 0;
  data->modelData->modelDataXml.nEquations = 14;
  data->modelData->nMixedSystems = 0;
  data->modelData->nLinearSystems = 0;
  data->modelData->nNonLinearSystems = 0;
  data->modelData->nStateSets = 0;
  data->modelData->nJacobians = 4;
  data->modelData->nOptimizeConstraints = 0;
  data->modelData->nOptimizeFinalConstraints = 0;
  
  data->modelData->nDelayExpressions = 0;
  
  data->modelData->nClocks = 0;
  data->modelData->nSubClocks = 0;
  
  data->modelData->nSensitivityVars = 0;
  data->modelData->nSensitivityParamVars = 0;
}

#ifdef __cplusplus
}
#endif

static int rml_execution_failed()
{
  fflush(NULL);
  fprintf(stderr, "Execution failed!\n");
  fflush(NULL);
  return 1;
}

#if defined(threadData)
#undef threadData
#endif
/* call the simulation runtime main from our main! */
int main(int argc, char**argv)
{
  int res;
  DATA data;
  MODEL_DATA modelData;
  SIMULATION_INFO simInfo;
  data.modelData = &modelData;
  data.simulationInfo = &simInfo;
  measure_time_flag = 0;
  compiledInDAEMode = 0;
  compiledWithSymSolver = 0;
  MMC_INIT(0);
  omc_alloc_interface.init();
  {
    MMC_TRY_TOP()
  
    MMC_TRY_STACK()
  
    Whole_setupDataStruc(&data, threadData);
    res = _main_SimulationRuntime(argc, argv, &data, threadData);
    
    MMC_ELSE()
    rml_execution_failed();
    fprintf(stderr, "Stack overflow detected and was not caught.\nSend us a bug report at https://trac.openmodelica.org/OpenModelica/newticket\n    Include the following trace:\n");
    printStacktraceMessages();
    fflush(NULL);
    return 1;
    MMC_CATCH_STACK()
    
    MMC_CATCH_TOP(return rml_execution_failed());
  }

  fflush(NULL);
  EXIT(res);
  return res;
}

