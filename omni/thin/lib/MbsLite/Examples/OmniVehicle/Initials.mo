within MbsLite.Examples.OmniVehicle;

record Initials

  parameter String   name(fixed = false);

  parameter Real     omega(fixed = false);
  parameter Real     vAbs(fixed = false);
  parameter Real     vDirAngle(fixed = false) "angle between inertial X axis and vehicle's X axis counter-clockwise-positive looking from the end of Z axis";

  parameter Real[3] vVec(each fixed = false);
  parameter Real[3] omegaVec(each fixed = false);

end Initials;
