/* Linearization */
#include "Whole_model.h"
#if defined(__cplusplus)
extern "C" {
#endif

const char *Whole_linear_model_frame()
{
  return "model linear_Whole\n  parameter Integer n = 2; // states\n  parameter Integer k = 0; // top-level inputs\n  parameter Integer l = 0; // top-level outputs\n"
  "  parameter Real x0[2] = {%s};\n"
  "  parameter Real u0[0] = {%s};\n"
  "  parameter Real A[2,2] = [%s];\n"
  "  parameter Real B[2,0] = zeros(2,0);%s\n"
  "  parameter Real C[0,2] = zeros(0,2);%s\n"
  "  parameter Real D[0,0] = zeros(0,0);%s\n"
  "  Real x[2](start=x0);\n"
  "  input Real u[0];\n"
  "  output Real y[0];\n"
  "\n  Real 'x_part.state.x' = x[1];\nReal 'x_part.subPart.state.y' = x[2];\n\n"
  "equation\n  der(x) = A * x + B * u;\n  y = C * x + D * u;\nend linear_Whole;\n";
}
const char *Whole_linear_model_datarecovery_frame()
{
  return "model linear_Whole\n  parameter Integer n = 2; // states\n  parameter Integer k = 0; // top-level inputs\n  parameter Integer l = 0; // top-level outputs\n  parameter Integer nz = 1; // data recovery variables\n"
  "  parameter Real x0[2] = {%s};\n"
  "  parameter Real u0[0] = {%s};\n"
  "  parameter Real z0[1] = {%s};\n"
  "  parameter Real A[2,2] = [%s];\n"
  "  parameter Real B[2,0] = zeros(2,0);%s\n"
  "  parameter Real C[0,2] = zeros(0,2);%s\n"
  "  parameter Real D[0,0] = zeros(0,0);%s\n"
  "  parameter Real Cz[1,2] = [%s];\n"
  "  parameter Real Dz[1,0] = zeros(1,0);%s\n"
  "  Real x[2](start=x0);\n"
  "  input Real u[0];\n"
  "  output Real y[0];\n"
  "  output Real z[1];\n"
  "\nReal 'x_part.state.x' = x[1];\nReal 'x_part.subPart.state.y' = x[2];\nReal 'z_partState.y' = z[1];\n\n"
  "equation\n  der(x) = A * x + B * u;\n  y = C * x + D * u;\n  z = Cz * x + Dz * u;\nend linear_Whole;\n";
}
#if defined(__cplusplus)
}
#endif

