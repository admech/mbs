within MbsLite.Examples.Omni.Config;

record OmniVehicleParams

  parameter Params          params;
  parameter Initials        initials;
  parameter FrictionParams  frictionParams;

  parameter Real[3]  gravity;
  parameter Real[4]  platformQuaternion;

  // below params to be calculated in a factory function

  parameter Real[3]     platformCenter;
  parameter Real[:, 3]  wheelCenters;
  parameter Real[:, 3]  wheelAxisDirections;
  parameter Real[:, 4]  wheelQuaternions;

  parameter Initials[:] wheelInitials;

end OmniVehicleParams;

