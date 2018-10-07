#ifndef Whole__H
#define Whole__H
#include "meta/meta_modelica.h"
#include "util/modelica.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "simulation/simulation_runtime.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct SubPartState_s {
  modelica_real _b;
  modelica_real _y;
} SubPartState;
typedef base_array_t SubPartState_array;
extern struct record_description SubPartState__desc;

typedef struct PartState_s {
  modelica_real _a;
  SubPartState _subPart;
  modelica_real _x;
  modelica_real _y;
} PartState;
typedef base_array_t PartState_array;
extern struct record_description PartState__desc;

typedef SubPartState SubPartState$partState$subPart;
typedef base_array_t SubPartState$partState$subPart_array;
extern struct record_description SubPartState$partState$subPart__desc;

typedef PartState PartState$partState;
typedef base_array_t PartState$partState_array;
extern struct record_description PartState$partState__desc;

DLLExport
PartState omc_PartState (threadData_t *threadData, modelica_real omc_a, modelica_real omc_x, modelica_real omc_y, SubPartState omc_subPart);

DLLExport
modelica_metatype boxptr_PartState(threadData_t *threadData, modelica_metatype _a, modelica_metatype _x, modelica_metatype _y, modelica_metatype _subPart);
static const MMC_DEFSTRUCTLIT(boxvar_lit_PartState,2,0) {(void*) boxptr_PartState,0}};
#define boxvar_PartState MMC_REFSTRUCTLIT(boxvar_lit_PartState)


DLLExport
PartState$partState omc_PartState$partState (threadData_t *threadData, modelica_real omc_a, modelica_real omc_x, modelica_real omc_y, SubPartState omc_subPart);

DLLExport
modelica_metatype boxptr_PartState$partState(threadData_t *threadData, modelica_metatype _a, modelica_metatype _x, modelica_metatype _y, modelica_metatype _subPart);
static const MMC_DEFSTRUCTLIT(boxvar_lit_PartState$partState,2,0) {(void*) boxptr_PartState$partState,0}};
#define boxvar_PartState$partState MMC_REFSTRUCTLIT(boxvar_lit_PartState$partState)


DLLExport
SubPartState omc_SubPartState (threadData_t *threadData, modelica_real omc_b, modelica_real omc_y);

DLLExport
modelica_metatype boxptr_SubPartState(threadData_t *threadData, modelica_metatype _b, modelica_metatype _y);
static const MMC_DEFSTRUCTLIT(boxvar_lit_SubPartState,2,0) {(void*) boxptr_SubPartState,0}};
#define boxvar_SubPartState MMC_REFSTRUCTLIT(boxvar_lit_SubPartState)


DLLExport
SubPartState$partState$subPart omc_SubPartState$partState$subPart (threadData_t *threadData, modelica_real omc_b, modelica_real omc_y);

DLLExport
modelica_metatype boxptr_SubPartState$partState$subPart(threadData_t *threadData, modelica_metatype _b, modelica_metatype _y);
static const MMC_DEFSTRUCTLIT(boxvar_lit_SubPartState$partState$subPart,2,0) {(void*) boxptr_SubPartState$partState$subPart,0}};
#define boxvar_SubPartState$partState$subPart MMC_REFSTRUCTLIT(boxvar_lit_SubPartState$partState$subPart)
#include "Whole_model.h"


#ifdef __cplusplus
}
#endif
#endif

