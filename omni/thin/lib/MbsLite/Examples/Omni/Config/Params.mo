within MbsLite.Examples.Omni.Config;

record Params

  parameter String   name;

  parameter Integer  NWheels;
  parameter Integer  nRollers;
  parameter Real     mecanumAngle;

  parameter Real     platformRadius;
  parameter Real     wheelRadius;

  parameter Real     platformMass;
  parameter Real     wheelHubMass;
  parameter Real     rollerMass;
  parameter Real     fractionOfRollerMassInWholeWheel;

  // below params to be calculated in a factory function

  parameter Real     rollerHalfAngle;
  parameter Real     wheelHubRadius;
  parameter Real     rollerRadiusForMoi;
  parameter Real     rollerLengthForMoi;

  parameter Real     platformAxialMoi;
  parameter Real     platformOrthogonalMoi;
  parameter Real     wheelHubAxialMoi;
  parameter Real     wheelHubOrthogonalMoi;
  parameter Real     rollerAxialMoi;
  parameter Real     rollerOrthogonalMoi;

end Params;
