#include "Whole_functions.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "Whole_includes.h"


PartState omc_PartState(threadData_t *threadData, modelica_real omc_a, modelica_real omc_x, modelica_real omc_y, SubPartState omc_subPart)
{
  PartState tmp1;
  tmp1._a = omc_a;
  tmp1._x = omc_x;
  tmp1._y = omc_y;
  tmp1._subPart = omc_subPart;
  return tmp1;
}

modelica_metatype boxptr_PartState(threadData_t *threadData, modelica_metatype _a, modelica_metatype _x, modelica_metatype _y, modelica_metatype _subPart)
{
  return mmc_mk_box5(3, &PartState__desc, _a, _x, _y, _subPart);
}

PartState$partState omc_PartState$partState(threadData_t *threadData, modelica_real omc_a, modelica_real omc_x, modelica_real omc_y, SubPartState omc_subPart)
{
  PartState$partState tmp1;
  tmp1._a = omc_a;
  tmp1._x = omc_x;
  tmp1._y = omc_y;
  tmp1._subPart = omc_subPart;
  return tmp1;
}

modelica_metatype boxptr_PartState$partState(threadData_t *threadData, modelica_metatype _a, modelica_metatype _x, modelica_metatype _y, modelica_metatype _subPart)
{
  return mmc_mk_box5(3, &PartState$partState__desc, _a, _x, _y, _subPart);
}

SubPartState omc_SubPartState(threadData_t *threadData, modelica_real omc_b, modelica_real omc_y)
{
  SubPartState tmp1;
  tmp1._b = omc_b;
  tmp1._y = omc_y;
  return tmp1;
}

modelica_metatype boxptr_SubPartState(threadData_t *threadData, modelica_metatype _b, modelica_metatype _y)
{
  return mmc_mk_box3(3, &SubPartState__desc, _b, _y);
}

SubPartState$partState$subPart omc_SubPartState$partState$subPart(threadData_t *threadData, modelica_real omc_b, modelica_real omc_y)
{
  SubPartState$partState$subPart tmp1;
  tmp1._b = omc_b;
  tmp1._y = omc_y;
  return tmp1;
}

modelica_metatype boxptr_SubPartState$partState$subPart(threadData_t *threadData, modelica_metatype _b, modelica_metatype _y)
{
  return mmc_mk_box3(3, &SubPartState$partState$subPart__desc, _b, _y);
}

#ifdef __cplusplus
}
#endif
