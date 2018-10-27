within MbsLite.Examples.Omni.Config;

record Initials

  parameter String   name;
  parameter Params   params;

  parameter Real     omega;
  parameter Real     vAbs;
  parameter Real     vDirAngle "angle between inertial X axis and vehicle's X axis counter-clockwise-positive looking from the end of Z axis";

  parameter Real[3] vVec;
  parameter Real[3] omegaVec;

  parameter Real[FIXME_N_WHEELS]  wheelAxialOmegas;
  parameter Real[FIXME_N_WHEELS]  firstRollerAxialOmegas;

end Initials;
