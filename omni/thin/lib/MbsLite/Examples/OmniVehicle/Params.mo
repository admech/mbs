within MbsLite.Examples.OmniVehicle;

record Params

  parameter String   name(fixed = false);

  parameter Integer  NWheels(fixed = false);
  parameter Integer  nRollers(fixed = false);
  parameter Real     mecanumAngle(fixed = false);

  parameter Real     platformRadius(fixed = false);
  parameter Real     wheelRadius(fixed = false);

  parameter Real     platformMass(fixed = false);
  parameter Real     wheelHubMass(fixed = false);
  parameter Real     rollerMass(fixed = false);

  // below params to be calculated in a factory function

  parameter Real     rollerHalfAngle(fixed = false);
  parameter Real     wheelHubRadius(fixed = false);
  parameter Real     rollerRadius(fixed = false);
  parameter Real     rollerLength(fixed = false);

  parameter Real     platformAxialMoi(fixed = false);
  parameter Real     platformOrthogonalMoi(fixed = false);
  parameter Real     wheelHubAxialMoi(fixed = false);
  parameter Real     wheelHubOrthogonalMoi(fixed = false);
  parameter Real     rollerAxialMoi(fixed = false);
  parameter Real     rollerOrthogonalMoi(fixed = false);

end Params;
